package main

import (
	"github.com/martinmarsh/nmea-mux"
	"fmt"
	"strconv"
	"time"
)

func main() {
	mux := nmea_mux.NewMux()

	// for default config.yaml in current folder
	// optional parameters define folder, filename, format, "config as a string
	if err := mux.LoadConfig(); err == nil {
		mux.Run()        // Run the virtual devices / go tasks
		ExternalFeatures(mux)
		fmt.Println("waiting to stop")
		mux.WaitToStop() // Wait for ever?
		fmt.Println("stopped")
	}else{
		fmt.Println(err)
	}

}

type nmeaData struct{
	esp_heading 	float32
	compass_heading float32
	desired_heading float32
	head_correction float32
	heading			float32
	esp_data_ok 	bool
	cp_data_ok 		bool
	auto_on			bool
	mode 			string
	status			string
	gain			float32
	pd				float32
	pi 				float32
	last_error		float32
	integral		float32
	last_mode		string
	waypt_id		string
    xte				float32
	waypt_bearing	float32
	waypt_set		bool
	autohelm_channels		map[string](chan string)
	handcontrol_channels	map[string](chan string)
}


func ExternalFeatures(mux *nmea_mux.NmeaMux){

	nmea_data := nmeaData{}
	nmea_data.last_error = 0
	nmea_data.integral = 0
	nmea_data.last_mode = "0"
	nmea_data.head_correction = 0
	nmea_data.autohelm_channels = make(map[string](chan string))
	nmea_data.handcontrol_channels = make(map[string](chan string))
	nmea_data.auto_on = false
	nmea_data.xte = 0
	nmea_data.waypt_id = ""
	nmea_data.waypt_bearing = 0
	nmea_data.waypt_set = false

	all_channels := &mux.Channels

	// external autohelm setup
	config, is_set := mux.Config.Values["auto_helm"]

	if is_set {
		nmea_data.pd, _ = getFloat32(100.0, config["pd"][0], 0, true)
		nmea_data.pi, _ = getFloat32(100.0, config["pi"][0], 0, true)
		nmea_data.gain, _ = getFloat32(100.0, config["gain"][0], 0, true)

		for _, v := range(config["outputs"]){
			nmea_data.autohelm_channels[v] = (*all_channels)[v]
		}

		go process_autohelm(&nmea_data, mux)
	}

	// external handcontrol setup

	config, is_set = mux.Config.Values["hand_controller"]
	
	if is_set {
		for _, v := range(config["outputs"]){
			nmea_data.handcontrol_channels[v] = (*all_channels)[v]
		}

		go process_handcontoller(&nmea_data, mux)
	}
}


func process_autohelm(d *nmeaData, mux *nmea_mux.NmeaMux){

	helm_ticker := time.NewTicker(500 * time.Millisecond)

	for {
		<- helm_ticker.C
		computeHelm(d, mux)
	}
}	

func process_handcontoller(d *nmeaData, mux *nmea_mux.NmeaMux){

	hc_ticker := time.NewTicker(1500 * time.Millisecond)

	for {
		fmt.Println("handcontroller waiting for ticker")
		<- hc_ticker.C
		fmt.Println("handcontroller got ticker")
		updateController(d, mux)
		fmt.Println("handcontroller has returned")
	}
}	



func computeHelm(d *nmeaData, mux *nmea_mux.NmeaMux ){
	getNmeaData(d, mux)
	pd := d.pd/120
	if(pd > 1){
		pd = 1
	}
	

	if d.cp_data_ok {
		d.heading = d.compass_heading
	}
	
	if d.esp_data_ok {
		if d.mode == "1" {
			d.auto_on = true
		} else {
			d.auto_on = false
		}	
		if d.status == "3333" {
			if d.cp_data_ok && d.head_correction == 0 {
				d.head_correction = relative180(d.esp_heading - d.compass_heading)
			}
			//d.heading = relative360(d.esp_heading - d.head_correction)
		}
	} else {
		d.head_correction = 0
	}

	//fmt.Printf("data.mode: %s \n", d.mode)
	//fmt.Printf("data.pi: %4.2f \n", d.pi)

	if d.auto_on && (d.cp_data_ok || d.esp_data_ok) {
		course_error:= relative180(d.heading - d.desired_heading)
		//fmt.Printf("error: %5.1f  d.gain: %.0f\n", course_error, d.gain)
		change := (course_error - d.last_error)
		d.last_error += change/20
		
		d.integral +=  course_error *  d.pi/30000.0;
		if (d.integral > 15.0){
			d.integral = 15.0;
		} else if (d.integral < -15.0){
			d.integral = -15.0;
		}

		helm_pos := (course_error + d.integral - change*pd) * d.gain * 3
		// fmt.Printf("helm: %5.1f \n", helm_pos)
		str, err := moveTo(helm_pos, mux)
		//fmt.Printf("Send 0183: %s\n", str)
		if err == nil {
			for _, v := range(d.autohelm_channels){
				v <- str
			}
		}	
	}
}


func updateController(d *nmeaData, mux *nmea_mux.NmeaMux ){
	//getNmeaData(d, mux)
	handle := mux.Processors["main_processor"].GetNmeaHandle()
	handle.Nmea_mu.Lock()           // must lock and unlock on function end
	defer handle.Nmea_mu.Unlock()

	str, err :=handle.Nmea.WriteSentencePrefixVar("PX","xs3", "hc_")

	if err == nil {
		for _, v := range(d.handcontrol_channels){
			v <- str
		}
	}
}

func  relative180(dif float32) float32{
	if dif < -180.0 {
		dif += 360.0
	}
	if dif > 180.0 {
		dif -= 360.0
	}
	return dif
}

/*
func  relative360(dif float32) float32{
	if dif < 0 {
		dif += 360.0
	}
	if dif > 360.0{
		dif -= 360.0
	}
	return dif
}
*/

func moveTo(position float32, mux *nmea_mux.NmeaMux )(string, error){
	handle := mux.Processors["main_processor"].GetNmeaHandle()
	data := make(map[string]string)
	data["helm_helm"] = fmt.Sprintf("%.0f", position)
	data["helm_reset"] = "0"
	mux.Processors["main_processor"].PutData(data)
	handle.Nmea_mu.Lock()           // must lock and unlock on function end
	defer handle.Nmea_mu.Unlock()
	str, err :=handle.Nmea.WriteSentencePrefixVar("PX","xs2", "helm_")
	return str, err
}



func getNmeaData(d *nmeaData, mux *nmea_mux.NmeaMux ){
	//Note only one process must call this routine or a deadly embrace will occur - 
	ok := true
	//handle := mux.Processors["main_processor"].GetNmeaHandle()
	//handle.Nmea_mu.Lock()           // must lock and unlock on function end
	//defer handle.Nmea_mu.Unlock()
	data := mux.Processors["main_processor"].GetData("esp_")
	new_data := make(map[string]string)

	//fmt.Printf("Nmea Data: %s \n", data)
	d.mode = data["esp_mode"]
	d.status = data["esp_compass_status"]
	d.gain, ok =  getFloat32(d.gain, data["esp_auto_gain"], 0, ok)
	d.pi, ok = getFloat32(d.pi, data["esp_auto_pi"], 0, ok)
	d.pd, ok = getFloat32(d.pd, data["esp_auto_pd"], 0, ok)

	d.esp_heading, ok = getFloat32(d.esp_heading, data["esp_hdm"], -3, ok)	
	d.desired_heading, ok = getFloat32(d.desired_heading, data["esp_set_hdm"], -3, ok)
	d.esp_data_ok = ok

	cp_data := mux.Processors["main_processor"].GetData("cp_")
	d.compass_heading, d.cp_data_ok = getFloat32(d.compass_heading,cp_data["cp_hdm"], -3, true)

	
	new_data["hc_auto_gain"] = strconv.FormatFloat(float64(d.gain),'f',0,32)
	new_data["hc_auto_pi"] = strconv.FormatFloat(float64(d.pi),'f',0,32)
	new_data["hc_auto_pd"] = strconv.FormatFloat(float64(d.pd),'f',0,32)
	new_data["hc_head"] = strconv.FormatFloat(float64(d.compass_heading),'f',0,32)
	new_data["hc_head_to_way"] = data["hc_head"] 
	
	d.waypt_set = false
	d.waypt_id, ok = data["ray_waypt_id"]
	if ok && len(d.waypt_id) > 0 {
		d.waypt_set = true
		xte_str, xte_valid := data["ray_xte"]
		if xte_valid {
			xte, valid_xte := getFloat32(0, xte_str[1:], -1, true)
			if valid_xte {
				if xte_str[0] == 'L'{
					d.xte = -xte
				} else {
					d.xte = xte
				}
			}
	
		} else {
			d.xte = 0 
		}

		d.waypt_bearing, _ = getFloat32(d.compass_heading, data["ray_bearing_position_to_waypt"], -3, true)
		new_data["hc_head_to_way"] = strconv.FormatFloat(float64(d.waypt_bearing),'f',0,32)
	} 
	mux.Processors["main_processor"].PutData(new_data)
}

func getFloat32(current float32, data string, end int, e bool) (float32, bool){
	l := len(data) + end
	
	if l > 0 {
		//fmt.Printf("conv %s end %d\n", data[:l], end )
		f64, err := strconv.ParseFloat(data[:l], 32)
		if err != nil{
			return current, false
		}
		return float32(f64), e
	} 
	return current, false
}
