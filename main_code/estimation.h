#ifndef ESTIMATION_H
#define ESTIMATION_H

// Include libraries
#include <math.h>

// Include other source files
#include "algebra.h"
#include "estimation.tpp"

template <typename T>
T pitch_Accelerometer(T AccX, T AccY, T AccZ);

template <typename T>
T roll_Accelerometer(T AccY, T AccZ);

template <typename T>
void kalman_filter(T delta_time, T W_gyro, T Q[2][2], T R, T Y, T X_estimated[2], T P_estimated[2][2]);

template <typename T>
void euler2quat(T yaw, T pitch, T roll, T* output);

#endif // ESTIMATION_H