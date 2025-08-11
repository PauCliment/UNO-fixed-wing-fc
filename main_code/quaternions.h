// Declare the header guards

#ifndef QUATERNIONS_H
#define QUATERNIONS_H

// Include libraries

#include <math.h>

// Include other source files

#include "algebra.h"
#include "quaternions.tpp"

// Declare and define functions

template <typename T>
void quaternion_product(T input_p[4], T input_q[4], T* output);

template <typename T>
void normalize_quaternion(T* output);

template <typename T>
void quaternion_error(T q_desired[4], T q_actual[4], T* output);

#endif // QUATERNION_FUNCTIONS_H