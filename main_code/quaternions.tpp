#ifndef QUATERNIONS_TPP
#define QUATERNIONS_TPP

#include "quaternions.h"

template <typename T>
void quaternion_product(T input_p[4], T input_q[4], T* output){
    // Function for computing the product between two quaternions
    // Input arrays must have a size of 4

    // Extract the real parts of the quaternions
    T pw = input_p[0];
    T qw = input_q[0];

    // Extract the imaginary parts of the quaternions
    T pv[3] = {input_p[1], input_p[2], input_p[3]};
    T qv[3] = {input_q[1], input_q[2], input_q[3]};

    // Compute the dot product of the imaginary parts of the input quaternions
    T dot_product_pqv = 0;
    for(int i = 0; i < 3; i++)
        dot_product_pqv = dot_product_pqv + pv[i] * qv[i];

    // Compute elements of the quaternion
    output[0] = pw * qw - dot_product_pqv;
    output[1] = pw * qv[0] + qw * pv[0] + pv[1] * qv[2] - pv[2] * qv[1];
    output[2] = pw * qv[1] + qw * pv[1] + pv[2] * qv[0] - pv[0] * qv[2];
    output[3] = pw * qv[2] + qw * pv[2] + pv[0] * qv[1] - pv[1] * qv[0];
}

template <typename T>
void normalize_quaternion(T* output){
    // Function for normalizing the attitude quaternion after numerical integration
    // The output must enter as a 1D array of length 4

    // If scalar part is negative, multiply by -1
    T sign = 1;    
    if(output[0] < 0)
        sign = - sign;

    for(int i = 0; i < 4; i++)
        output[i] *= sign;
    
    // Normalize the array (of size 4 <--> quaternion)
    normalize_vector(output, 4);
    
}

template <typename T>
void quaternion_error(T q_desired[4], T q_actual[4], T* output){
    // Function for computing the error between two quaternions
    // Input arrays must have a size of 4

    // Normalize both quaternions
    normalize_quaternion(q_desired);
    normalize_quaternion(q_actual);

    // Extract the real parts of the quaternions
    T q_desired_inverse[4];
    q_desired_inverse[0] = + q_desired[0];
    q_desired_inverse[1] = - q_desired[1];
    q_desired_inverse[2] = - q_desired[2];
    q_desired_inverse[3] = - q_desired[3];

    // Apply the error formula
    quaternion_product(q_desired_inverse, q_actual, output);

    normalize_quaternion(output);
}

#endif // QUATERNIONS_TPP