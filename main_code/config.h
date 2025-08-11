#ifndef CONFIG_H
#define CONFIG_H

// Include libraries
#include <stdint.h>

// Specify control gains
const double Kp[3] = {50.0, 75.0, 0.0};
const double Ki[3] = {100.0, 150.0, 0.0};
const double Kd[3] = {4.0, 5.0, 8.0};

// Specify name of the data file
const char *fileName = "data.txt";
const double sd_sampling = 0.25;

// Define output pins
const uint8_t elevator_pin = 3;
const uint8_t throttle_pin = 5;
const uint8_t aileron_pin  = 6;
const uint8_t rudder_pin   = 9;
const uint8_t chipSelect   = 10;
const uint8_t statusLED    = 7;

// Declare MPU 6050 addresses
const uint8_t MPU_6050     = 0x68;
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t CONFIG       = 0x1A;
const uint8_t GYRO_CONFIG  = 0x1B;
const uint8_t ACC_CONFIG   = 0x1C;
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t GYRO_XOUT_H  = 0x43;

// Define scaling values of the IMU sensors
const double acc_scaling  = 8192.f;    // 2^16 / 8 (16 bits max value / +-4 g)
const double gyro_scaling = 7509.8724; // 2^16 / 500 * 180 / pi (16 bits max value / +-250º/s in rad/s)

// Define uncertainty matrices of the MPU6050 sensors for the roll axis
double Q_roll[2][2] = {{0.001, 0.0}, {0.0, 0.003}};
double P_roll[2][2] = {{1.f, 0.0}, {0.0, 1.f}};
double R_roll = 30.f;

// Define uncertainty matrices of the MPU6050 sensors for the pitch axis
double Q_pitch[2][2] = {{0.001, 0.0}, {0.0, 0.003}};
double P_pitch[2][2] = {{1.f, 0.0}, {0.0, 1.f}};
double R_pitch = 30.f;

// Define minimum and maximum SBUS values
const uint16_t min_sbus = 180;
const uint16_t max_sbus = 1800;
const uint16_t mid_sbus = 990;
const uint16_t deadband_sbus = 10;

// Define maximum absolute rotation rates for processing the RC inputs
const double max_pitch_rate = 150 * M_PI / 180.0;
const double max_roll_rate = 100 * M_PI / 180.0;
const double max_yaw_rate = 100 * M_PI / 180.0;

// Declare min and max pulse length for servo PWM
const uint16_t min_servo = 600;
const uint16_t max_servo = 2300;
const uint16_t min_esc = 1000;
const uint16_t max_esc = 2000;

// Declare min and max angle values for servos and throttle (180deg max range)
const int16_t min_elevator = -30;
const int16_t max_elevator = 30;
const int16_t min_throttle = 0;
const int16_t max_throttle = 180;
const int16_t min_aileron = -25;
const int16_t max_aileron = 25;
const int16_t min_rudder = -20;
const int16_t max_rudder = 20;

// Define MPU configuration function
int MPU_config(){
  // Initialize temporary error and error flag
  int error_temp = 0;
  int error_flag = 0;
  
  // Write reset to low
  Wire.beginTransmission(MPU_6050);
  Wire.write(PWR_MGMT_1); // Write to the power management 1 adress
  Wire.write(0x00); // Set DEVICE_RESET (bit 7) to 0 - don't reset MPU 6050 ??
  error_temp = Wire.endTransmission(true);
  error_flag = (error_temp != 0) ? error_temp : error_flag;

  // Set filter bandwidth values to ~ 96Hz
  Wire.beginTransmission(MPU_6050);
  Wire.write(CONFIG); // Write to the gyro config 1 adress
  Wire.write(0x02); // Set DLPF_CFG to 02 (acc - 94Hz @ 3ms, gyro - 98Hz @ 2.8ms)
  error_temp = Wire.endTransmission(true);
  error_flag = (error_temp != 0) ? error_temp : error_flag;

  // Set gyroscope configuration to +- 250 deg/s
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_CONFIG); // Write to the gyro config adress
  Wire.write(0x00); // Set FS_SEL to 00 (+-250deg/s)
  error_temp = Wire.endTransmission(true);
  error_flag = (error_temp != 0) ? error_temp : error_flag;

  // Set accelerometer configuration to +- 4g
  Wire.beginTransmission(MPU_6050);
  Wire.write(ACC_CONFIG); // Write to the acc config adress
  Wire.write(0x08); // Set AFS_SEL to 01 (+-4g)
  error_temp = Wire.endTransmission(true);
  error_flag = (error_temp != 0) ? error_temp : error_flag;

  return error_flag;
}

#endif // CONFIG_H
