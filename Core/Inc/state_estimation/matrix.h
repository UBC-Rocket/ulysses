#ifndef MATRIX_H
#define MATRIX_H

#define N 3

void normalize(float q[4]);

int inverse(float a[N][N], float inverse[N][N]);

void transpose3x4_to_4x3(const float A[3][4], float AT[4][3]);

void transpose4x4(const float A[4][4], float AT[4][4]);


#endif