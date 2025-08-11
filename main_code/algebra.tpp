#ifndef ALGEBRA_TPP
#define ALGEBRA_TPP

#include "algebra.h"

template <typename T>
T sign(T number) {
    return (number > 0.0) - (number < 0.0);
}

template <typename T>
T dot_product(T *vec1, T *vec2, int size){
  // Perform dot product

  T result = 0.0;
  for(int i = 0; i < size; i++)
    result += vec1[i] * vec2[i];
  
  return result;
}

template <typename T>
T vector_norm(T *vector, int size){
    // Function to find the norm of a given vector
    // Compute the array's norm
    T norm = 0.0;
    T norm_squared = 0.0;
    for(int i = 0; i < size; i++)
        norm_squared += pow(vector[i], 2);
    norm = sqrt(norm_squared);

    return norm;
}

template <typename T>
void matrix_sum(T* mat1, T* mat2, int rows, int cols, T* result) {
  // Perform matrix sum
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      int idx = i * cols + j;
      result[idx] = mat1[idx] + mat2[idx];
    }
  }
}

template <typename T>
void matrix_multiply(T *mat1, T *mat2, int rows1, int cols1, int cols2, T *result) {
    // Perform matrix multiplication
    for (int i = 0; i < rows1; ++i) {
        for (int j = 0; j < cols2; ++j) {
            T sum = 0.0;
            for (int k = 0; k < cols1; ++k) {
                sum += mat1[i * cols1 + k] * mat2[k * cols2 + j];
            }
            result[i * cols2 + j] = sum;
        }
    }
}

template <typename T>
void matrix_transpose(T *mat, int rows, int cols, T *result) {
    // Perform matrix transpose
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[j * rows + i] = mat[i * cols + j];
        }
    }
}

template <typename T>
void normalize_vector(T *vector, int size){
    // Function to normalize a given vector
    // The input-output array should be a 1 dimensional float array

    // Compute the array's norm
    T norm = vector_norm(vector, size);

    // Divide individual elements by the array's norm
    for(int i = 0; i < size; i++)
        vector[i] /= norm;
}

#endif // ALGEBRA_TPP