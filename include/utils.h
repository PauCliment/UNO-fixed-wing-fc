#ifndef UTILS_H
#define UTILS_H

// Include libraries

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>

// Define regular functions

void log_data(double dt, bool failsafe, bool bypass, double pitch, double roll, double angular_rate[3], double current_quaternion[4], double target_quaternion[4], double error_quaternion[4], double throttle_percent, double elevator_ang, double rudder_ang, double aileron_ang, bool header, File& dataFile);
void flash_led(int pin, int repetitions, int period);

// Define template functions

template <typename Tval, typename Tlim>
Tval saturate(Tval value, Tlim min, Tlim max){
  // Function to saturate a given input 
  value = (value > max) ? max : value;
  value = (value < min) ? min : value;
  return value;
}

template <typename Xres, typename Xin, typename Xout>
Xres map_Generic(Xres x, Xin in_min, Xin in_max, Xout out_min, Xout out_max) {

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // UTILS_H