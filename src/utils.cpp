#include "utils.h"

void log_data(double dt, bool failsafe, bool bypass, double pitch, double roll, double angular_rate[3], double current_quaternion[4], double target_quaternion[4], double error_quaternion[4], double throttle_percent, double elevator_ang, double rudder_ang, double aileron_ang, bool header, File& dataFile) {
  
  if (dataFile) {
    
    if (header) {
      // Create header information
      String header_str = "dt, flight_mode, pitch, roll, ang_rate_x, ang_rate_y, ang_rate_z, ";
      header_str += "quat_s2b_w, quat_s2b_x, quat_s2b_y, quat_s2b_z, ";
      header_str += "quat_s2t_w, quat_s2t_x, quat_s2t_y, quat_s2t_z, ";
      header_str += "quat_b2t_w, quat_b2t_x, quat_b2t_y, quat_b2t_z, ";
      header_str += "throttle_percent, elevator_ang, rudder_ang, aileron_ang";
      dataFile.println(header_str);
    }    
    else {
      // Create data information
      int mode = (failsafe) ? 0 : ((bypass) ? 1 : 2);
      
      String data_str = "";
      data_str += String(dt) + ", ";
      data_str += String(mode) + ", ";
      data_str += String(pitch * 180 / PI) + ", ";
      data_str += String(roll * 180 / PI) + ", ";
      data_str += String(angular_rate[0] * 180 / PI) + ", ";
      data_str += String(angular_rate[1] * 180 / PI) + ", ";
      data_str += String(angular_rate[2] * 180 / PI) + ", ";
      data_str += String(current_quaternion[0]) + ", ";
      data_str += String(current_quaternion[1]) + ", ";
      data_str += String(current_quaternion[2]) + ", ";
      data_str += String(current_quaternion[3]) + ", ";
      data_str += String(target_quaternion[0]) + ", ";
      data_str += String(target_quaternion[1]) + ", ";
      data_str += String(target_quaternion[2]) + ", ";
      data_str += String(target_quaternion[3]) + ", ";
      data_str += String(error_quaternion[0]) + ", ";
      data_str += String(error_quaternion[1]) + ", ";
      data_str += String(error_quaternion[2]) + ", ";
      data_str += String(error_quaternion[3]) + ", ";
      data_str += String(throttle_percent) + ", ";
      data_str += String(elevator_ang) + ", ";
      data_str += String(rudder_ang) + ", ";
      data_str += String(aileron_ang);
      
      dataFile.println(data_str);
    }
    
  }
  else{
    Serial.println("SD error");
  }
}

void flash_led(int pin, int repetitions, int period){
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(pin, OUTPUT);
  for(int i = 0; i < repetitions; i++){
    digitalWrite(pin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(period);                         // wait for 50 ms
    digitalWrite(pin, LOW);    // turn the LED off by making the voltage LOW
    delay(period);                         // wait for 50 ms 
  }
}