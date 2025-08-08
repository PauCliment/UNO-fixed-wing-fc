#ifndef ALGEBRA_H
#define ALGEBRA_H

#include <math.h>

#include "algebra.tpp"

template <typename T>
T sign(T number);
template <typename T>
T dot_product(T *vec1, T *vec2, int size);

template <typename T>
T vector_norm(T *vector, int size);

template <typename T>
void matrix_sum(T* mat1, T* mat2, int rows, int cols, T* result);

template <typename T>
void matrix_multiply(T *mat1, T *mat2, int rows1, int cols1, int cols2, T *result);

template <typename T>
void matrix_transpose(T *mat, int rows, int cols, T *result);

template <typename T>
void normalize_vector(T *vector, int size);

#endif // ALGEBRA_H