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
		go start(mux)
		mux.WaitToStop() // Wait for ever?

	}else{
		fmt.Println(err)
	}

}

type nmeaData struct{
	esp_heading 	float32
	compass_heading float32
	desired_heading float32
	esp_data_ok 	bool
	cp_data_ok 		bool
	mode 			string
	status			string
	gain			float32
	pd				float32
	pi 				float32
	last_error		float32
	integral		float32
	last_mode		string

}

func start(mux *nmea_mux.NmeaMux){
	nmea_data := nmeaData{}
	nmea_data.last_error = 0
	nmea_data.integral = 0
	nmea_data.pd = 100
	nmea_data.pi = 100
	nmea_data.gain = 100
	nmea_data.last_mode = "0"
	helm_ticker := time.NewTicker(500 * time.Millisecond)

	to_helm := mux.channels["to_helm"]
	
	fmt.Printf("Mux: %s \n", mux)

	for {
		select {
	    case str := <- to_helm:
			fmt.Printf("to helm %s \n", str)
		case <-helm_ticker.C:
			computeHelm(&nmea_data, mux)
		}
	}
}	


func computeHelm(d *nmeaData, mux *nmea_mux.NmeaMux ){
	getNmeaData(d, mux)
	d.pd = d.pd/120
	if(d.pd > 1){
		d.pd = 1
	}

	fmt.Printf("data: %s \n", d)
	fmt.Printf("data.mode: %s \n", d.mode)
	fmt.Printf("data.pi: %4.2f \n", d.pi)
	

	if d.mode == "1" {
		if d.esp_data_ok {
			error := relative180(d.esp_heading - d.desired_heading)
			
			change := (error - d.last_error)
			d.last_error += change/20
			
			d.integral +=  error *  d.pi/30000.0;
			if (d.integral > 15.0){
				d.integral = 15.0;
			} else if (d.integral < -15.0){
				d.integral = -15.0;
			}

			helm_pos := (error + d.integral - change*d.pd) * d.gain * 3
			fmt.Printf("helm: %5.1f \n", helm_pos)
			to_helm <- helm_pos

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

func getNmeaData(d *nmeaData, mux *nmea_mux.NmeaMux ){
	ok := true
	handle := mux.Processors["main_processor"].GetNmeaHandle()
	handle.Nmea_mu.Lock()           // must lock and unlock on function end
	defer handle.Nmea_mu.Unlock()
	data := handle.Nmea.GetMap()

	fmt.Printf("Nmea Data: %s \n", data)
	d.mode = data["esp_mode"]
	d.status = data["esp_compass_status"]
	d.gain, ok =  getFloat32(d.gain, data["esp_auto_gain"], 0, ok)
	d.pi, ok = getFloat32(d.pi, data["esp_auto_pi"], 0, ok)
	d.pd, ok = getFloat32(d.pd, data["esp_auto_pd"], 0, ok)
	d.esp_heading, ok = getFloat32(d.esp_heading, data["esp_hdm"], -2, ok)	
	d.desired_heading, ok = getFloat32(d.desired_heading, data["esp_set_hdm"], -2, ok)
	d.esp_data_ok = ok
	d.compass_heading, d.cp_data_ok = getFloat32(d.compass_heading, data["cp_hdm"], -2, true)	

}

func getFloat32(current float32, data string, end int, e bool) (float32, bool){
	l := len(data) + end
	
	if l > 0 {
		f64, err := strconv.ParseFloat(data[:l], 32)
		if err != nil{
			return current, false
		}
		return float32(f64), e
	} 
	return current, false
}
