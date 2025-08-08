#ifndef ESTIMATION_TPP
#define ESTIMATION_TPP

#include "estimation.h"

template <typename T>
T pitch_Accelerometer(T AccX, T AccY, T AccZ) {
    return std::atan2(-AccX, std::sqrt(AccY * AccY + AccZ * AccZ));
}

template <typename T>
T roll_Accelerometer(T AccY, T AccZ) {
    return std::atan2(AccY, AccZ);
}

template <typename T>
void kalman_filter(T delta_time, T W_gyro, T Q[2][2], T R, T Y, T X_estimated[2], T P_estimated[2][2]){

  // Store temporary X previous
  T X_previous[2];
  for(int i = 0; i < 2; i++){
    X_previous[i] = X_estimated[i];
  }
  
  // Store temporary P previous
  T P_previous[2][2];
  for(int i = 0; i < 2; i++){
    for(int j = 0; j < 2; j++){
      P_previous[i][j] = P_estimated[i][j];
    }
  }

  // Construct Kalman filter model matrices
  T A[2][2] = {{1.0, -delta_time}, {0.0, 1.0}};
  T B[2] = {delta_time, 0.0};
  T C[2] = {1.0, 0.0};

  // Perform prediction - X_predicted = A * X_previous + B * W_angle;
  T AX_prev[2];
  matrix_multiply((T*) A, (T*) X_previous, 2, 2, 1, (T*) AX_prev);
  T BW_ang[2] = {B[0] * W_gyro, B[1] * W_gyro};
  T X_predicted[2] = {AX_prev[0] + BW_ang[0], AX_prev[1] + BW_ang[1]};

  // Propagate the error covariance matrix - P_predicted = A * P_previous * A' + Q;
  T A_transposed[2][2];
  matrix_transpose((T*) A, 2, 2, (T*) A_transposed);
  T AP[2][2];
  matrix_multiply((T*) A, (T*) P_previous, 2, 2, 2, (T*) AP);
  T APAt[2][2];
  matrix_multiply((T*) AP, (T*) A_transposed, 2, 2, 2, (T*) APAt);
  T P_predicted[2][2];
  matrix_sum((T*) APAt, (T*) Q, 2, 2, (T*) P_predicted);  

  // Compute the Kalman Gain - /K_gain = P_predicted * C' / (C * P_predicted * C' + R);
  T PCt[2];
  matrix_multiply((T*) P_predicted, (T*) C, 2, 2, 1, (T*) PCt);
  T CPCtR = dot_product((T*) C, (T*) PCt, 2) + R;
  T K_gain[2] = {PCt[0] / CPCtR, PCt[1] / CPCtR};

  // Update estimate with measurement - X_estimated = X_predicted + K_gain * (Y - C * X_predicted);
  T YmCX = Y - dot_product((T*) C, (T*) X_predicted, 2);
  T K_prod[2] = {K_gain[0] * YmCX, K_gain[1] * YmCX};
  matrix_sum((T*) X_predicted, (T*) K_prod, 2, 1, (T*) X_estimated);

  // Update the error covariance - P_estimated = (eye(size(P_previous)) - K_gain * C) * P_predicted;
  T Identity_2x2[2][2] = {{1.f, 0.f}, {0.f, 1.f}};
  T mKC[2][2];
  for(int i = 0; i < 2; i++){
    for(int j = 0; j < 2; j++){
      mKC[i][j] = - K_gain[i] * C[j];
    }
  }
  T ImKC[2][2];
  matrix_sum((T*) Identity_2x2, (T*) mKC, 2, 2, (T*) ImKC);
  matrix_multiply((T*) ImKC, (T*) P_predicted, 2, 2, 2, (T*) P_estimated);

}

template <typename T>
void euler2quat(T yaw, T pitch, T roll, T* output) {

  T cr = std::cos(roll * 0.5);
  T sr = std::sin(roll * 0.5);
  T cp = std::cos(pitch * 0.5);
  T sp = std::sin(pitch * 0.5);
  T cy = std::cos(yaw * 0.5);
  T sy = std::sin(yaw * 0.5);

  output[0] = cr * cp * cy + sr * sp * sy;
  output[1] = sr * cp * cy - cr * sp * sy;
  output[2] = cr * sp * cy + sr * cp * sy;
  output[3] = cr * cp * sy - sr * sp * cy;

}

#endif // ESTIMATION_TPP