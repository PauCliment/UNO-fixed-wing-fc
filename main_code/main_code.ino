// Include libraries to be used
#include <Servo.h>
#include <Wire.h>
#include <SBUS.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <stdint.h>

// Include custom header files
#include "config.h"
#include "utils.h"
#include "algebra.h"
#include "quaternions.h"
#include "estimation.h"
#include "control.h"

// Initialize error flag of MPU 6050
int error = 1;

// Initialize SBUS objects for reading and writing on RX0
SBUS sbus(Serial);

// Declare control action
double control_action[3] = {0.0, 0.0, 0.0};

// Initialize servo objects
Servo elevator, throttle, aileron, rudder;

// Initialize Time management variables
double delta_time = 0.0;
unsigned long gyro_time = 0;

// Initialize data file object
File dataFile;
double sd_timer = 0.0;

// Declare global state variables
bool failsafe_mode = false;
bool manual_bypass = true;

// Declare the RX commands to be read from the receiver
unsigned int RX_cmd[8] = {0};
/*
0 -> Throttle
1 -> Ailerons
2 -> Elevator
3 -> Rudder
4 -> Large Left Toggle switch
5 -> Large Right Toggle switch
*/

// Initialize the accelerometer and gyroscope readings
double ACC_values[3];
double GYRO_values[3];

// Initialize the control surface angle values
double elevator_angle = 0.0;
double throttle_angle = 0.0;
double aileron_angle = 0.0;
double rudder_angle = 0.0;

// Initialize variables for Kalman Filter
double X_roll[2] = {0.0, 0.0};
double X_pitch[2] = {0.0, 0.0};

// Declare attitude, target and error quaternion, as well as angular body rates
double quaternion_f2b[4] = {1.0, 0.0, 0.0, 0.0};
double quaternion_f2t[4] = {1.0, 0.0, 0.0, 0.0};
double quaternion_t2b[4] = {1.0, 0.0, 0.0, 0.0};
double quaternion_t2b_int[4] = {0.0, 0.0, 0.0, 0.0};
double angular_rate[3] = {0.0, 0.0, 0.0};

// Declare target pitch and roll Euler angles, as well as target angular rate
double pitch_tg = 0.0;
double roll_tg = 0.0;
double yaw_rate_tg = 0.0;

// Begin setup
void setup() {
  // Initialize USB serial communications
  //Serial.begin(9600);

  // Perform IMU setup
  Wire.begin();
  //Wire.setClock(40000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  //Wire.setWireTimeout(3000, true); //timeout value in uSec
      
  error = 1;
  while (error != 0){
    //Serial.println("MPU error");
    error = MPU_config();
  }

  // Attach PWM control pins
  elevator.attach(elevator_pin, min_servo, max_servo);
  throttle.attach(throttle_pin, min_esc, max_esc);
  aileron.attach(aileron_pin, min_servo, max_servo);
  rudder.attach(rudder_pin, min_servo, max_servo);

  // Use manual processing for sbus
  sbus.begin(false);
  
  // Begin SD card communications
  /*
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("card initialized.");

  // Write header to data file
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {

    // Log header to CSV
    //log_data(delta_time, failsafe_mode, manual_bypass, X_pitch[0], X_roll[0], angular_rate, quaternion_f2b, quaternion_f2t, quaternion_t2b, throttle_angle / 0.18, elevator_angle, rudder_angle, aileron_angle, false, dataFile);
    //delay(50);
    dataFile.println("Timestamp, SensorValue1, SensorValue2");
    // Close the file after writing the data
    dataFile.close();

  } else {
    Serial.print("Error opening SD card \n");
  }
  */

  // Flash LED to indicate MPU connection steps performed  
  flash_led(statusLED, 5, 50);
  flash_led(LED_BUILTIN, 5, 50);

}

void loop () {

  // Begin transmission with the IMU over the Accelerometer register
  Wire.beginTransmission(MPU_6050);
  Wire.write(ACCEL_XOUT_H);
  error = Wire.endTransmission(false);
  if (error != 0) {
    //Serial.println("MPU error");
    MPU_config();
    digitalWrite(statusLED, HIGH);
  }
  else {
    digitalWrite(statusLED, LOW);
  }

  // Extract accelerometer data
  Wire.requestFrom(MPU_6050, (uint8_t)6, (uint8_t)true); // Extract the 6 following registers where all the acc data is stored
  int16_t ACCEL_OUT_X = Wire.read() << 8 | Wire.read();
  int16_t ACCEL_OUT_Y = Wire.read() << 8 | Wire.read();
  int16_t ACCEL_OUT_Z = Wire.read() << 8 | Wire.read();

  // Begin transmission with the IMU over the Accelerometer register
  Wire.beginTransmission(MPU_6050);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(true);

  // Store time in microseconds right before requesting gyroscope data
  delta_time = (double)(micros() - gyro_time) / 1000000.0;
  gyro_time = micros();
  sd_timer += delta_time;

  // Extrac gyroscope data
  Wire.requestFrom(MPU_6050, (uint8_t)6, (uint8_t)true); // Extract the 6 following registers where all the acc data is stored
  int16_t GYRO_OUT_X = Wire.read() << 8 | Wire.read();
  int16_t GYRO_OUT_Y = Wire.read() << 8 | Wire.read();
  int16_t GYRO_OUT_Z = Wire.read() << 8 | Wire.read();

  // Scale accelerometer data
  ACC_values[0] = - double(ACCEL_OUT_Y) / acc_scaling;
  ACC_values[1] = - double(ACCEL_OUT_X) / acc_scaling;
  ACC_values[2] = + double(ACCEL_OUT_Z) / acc_scaling;

  // Scale gyroscope data
  GYRO_values[0] = + double(GYRO_OUT_Y) / gyro_scaling;
  GYRO_values[1] = + double(GYRO_OUT_X) / gyro_scaling;
  GYRO_values[2] = - double(GYRO_OUT_Z) / gyro_scaling;
  
  // Perform Kalman Filtering
  double Y_pitch = pitch_Accelerometer(ACC_values[0], ACC_values[1], ACC_values[2]);
  double Y_roll = roll_Accelerometer(ACC_values[1], ACC_values[2]);
  kalman_filter(delta_time, GYRO_values[0], Q_roll, R_roll, Y_roll, X_roll, P_roll);
  kalman_filter(delta_time, GYRO_values[1], Q_pitch, R_pitch, Y_pitch, X_pitch, P_pitch);

  // Remove bias from angular rate estimation
  angular_rate[0] = GYRO_values[0] - X_roll[1];
  angular_rate[1] = GYRO_values[1] - X_pitch[1];
  angular_rate[2] = GYRO_values[2];

  // Call SBUS
  sbus.process();

  // If receiver is connected

  // If RX detects failsafe, set mode to failsafe
  if(sbus.getFailsafeStatus() == SBUS_FAILSAFE_ACTIVE){
    failsafe_mode = true;

    // Set servo commands to neutral and throttle to zero
    rudder_angle = map(mid_sbus, min_sbus, max_sbus, -90.0, 90.0);
    elevator_angle = map(mid_sbus, min_sbus, max_sbus, -90.0, 90.0);
    aileron_angle = map(mid_sbus, min_sbus, max_sbus, -90.0, 90.0);
    throttle_angle = 0.0;
  }

  else{

    // Read all channel data
    for(int ch_counter = 0; ch_counter < 8; ch_counter ++){
      
      // Read the data and set to middle values for deadband
      unsigned int data_ch = sbus.getChannel(ch_counter + 1);
      RX_cmd[ch_counter] = constrain(data_ch, min_sbus, max_sbus);
      if(data_ch < mid_sbus + deadband_sbus && data_ch > mid_sbus - deadband_sbus)
        RX_cmd[ch_counter] = mid_sbus;
    }

    // Check manual bypass switch
    manual_bypass = (RX_cmd[4] > mid_sbus) ? false : true;

    // Check attitude reset switch
    if (RX_cmd[5] > mid_sbus){
      // Set target attitude to zero
      pitch_tg = 0.0;
      roll_tg = 0.0;

      // Set quaternion error integral to zero
      for(int i = 0; i < 4; i++){
        quaternion_t2b_int[i] = 0.0;
      }
    }

    // Set commands to the servos
    if(manual_bypass == true || error != 0){
      // Set quaternion error integral to zero
      for(int i = 0; i < 4; i++){
        quaternion_t2b_int[i] = 0.0;
      }

      // Set servo commands to direct manual values
      rudder_angle = map(RX_cmd[3], min_sbus, max_sbus, min_rudder, max_rudder);
      elevator_angle = map(RX_cmd[2], min_sbus, max_sbus, min_elevator, max_elevator);
      aileron_angle = map(RX_cmd[1], min_sbus, max_sbus, min_aileron, max_aileron);

      // Set current attitude as target attitude to avoid abrupt changes into stabilized mode
      pitch_tg = X_pitch[0];
      roll_tg = X_roll[0];
      yaw_rate_tg = 0.0;
    }
    
    else{

      // Update the target pitch and roll
      pitch_tg -= map_Generic((double)RX_cmd[2], min_sbus, max_sbus, -max_pitch_rate, max_pitch_rate) * delta_time;
      roll_tg += map_Generic((double)RX_cmd[1], min_sbus, max_sbus, -max_roll_rate, max_roll_rate) * delta_time;
      yaw_rate_tg = map_Generic((double)RX_cmd[3], min_sbus, max_sbus, -max_yaw_rate, max_yaw_rate);
    
      // Update the current and target quaternion
      euler2quat(0.0, X_pitch[0], X_roll[0], quaternion_f2b);
      euler2quat(0.0, pitch_tg, roll_tg, quaternion_f2t);

      // Compute the quaternion error
      quaternion_error(quaternion_f2t, quaternion_f2b, quaternion_t2b);

      // Update quaternion error integral
      for(int i = 0; i < 4; i++){
        quaternion_t2b_int[i] += quaternion_t2b[i] * delta_time;
      }

      // Apply quaternion PID
      quaternionPID(quaternion_t2b, quaternion_t2b_int, angular_rate, yaw_rate_tg, Kp, Ki, Kd, control_action);

      // Set servo commands to stabilized values with control loop
      elevator_angle = - saturate(control_action[1], (double) min_elevator, (double) max_elevator);
      aileron_angle = saturate(control_action[0], (double) min_aileron, (double) max_aileron);
      rudder_angle = saturate(control_action[2], (double) min_rudder, (double) max_rudder);
    }

    // Set command to the throttle
    throttle_angle = map(RX_cmd[0], min_sbus, max_sbus, 0.0, 180.0);

    // Saturate command values
    throttle_angle = saturate(throttle_angle, (double) min_throttle, (double) max_throttle);

  }

  // Send servo commands through PWM
  elevator.write(- elevator_angle + 90.0);
  throttle.write(throttle_angle);
  aileron.write(- aileron_angle + 90.0);
  rudder.write(- rudder_angle + 90.0);

  // Write data to SD card
  /*
  if (sd_timer > sd_sampling){

    // Reset SD timer
    sd_timer = 0.0;

    // Write to data file
    //dataFile = SD.open(fileName, FILE_WRITE);
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile) {

      // Log header to CSV
      //log_data(delta_time, failsafe_mode, manual_bypass, X_pitch[0], X_roll[0], angular_rate, quaternion_f2b, quaternion_f2t, quaternion_t2b, throttle_angle / 0.18, elevator_angle, rudder_angle, aileron_angle, false, dataFile);
      delay(50);
      dataFile.println("Timestamp, SensorValue1, SensorValue2");

      // Close the file after writing the data
      dataFile.close();

    } else {
      Serial.print("Error opening SD card \n");
    }
  }
  */
}