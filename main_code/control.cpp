#include "control.h"

void quaternionPID(double err_quaternion[4], double int_quaternion[4], double ang_rate[3], double ref_rate_yaw, const double Kp[3], const double Ki[3], const double Kd[3], double* output){
  // Non-linear quaternion PID function

  // Declare signs of quaternions (account for unwinding)
  int att_sign, int_sign;

  // Declare target angular rate vector
  double ref_yaw_vector[3] = {0.0, 0.0, ref_rate_yaw};

  // Declare P, I and D terms
  double P_term, I_term, D_term;

  // Apply PID equation to each axis: Roll - Pitch - Yaw
  for(int i = 0; i < 3; i++){
    att_sign = sign(err_quaternion[0]);
    int_sign = sign(int_quaternion[0]);
    P_term = att_sign * Kp[i] * err_quaternion[i + 1];
    I_term = int_sign * Ki[i] * int_quaternion[i + 1];
    D_term = - Kd[i] * (ref_yaw_vector[i] - ang_rate[i]);
    output[i] = - (P_term + I_term + D_term);
  }
}