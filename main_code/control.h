#ifndef CONTROL_H
#define CONTROL_H

// Include libraries
#include <math.h>
#include "algebra.h"

// Define the various functions to be used
void quaternionPID(double err_quaternion[4], double int_quaternion[4], double ang_rate[3], double ref_rate_yaw, const double Kp[3], const double Ki[3], const double Kd[3], double* output);

#endif // CONTROL_H