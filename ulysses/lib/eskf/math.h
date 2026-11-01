#ifndef ULYSSES_LIB_ESKF_MATH_H
#define ULYSSES_LIB_ESKF_MATH_H

// w, x, y, z
void quat_multiplication(float *srcA[4], float *srcB[4], float *dest[4]);
void quat_normalization_in_place(float *src[4]);



#endif // ULYSSES_LIB_ESKF_MATH_H
