#include "ulysses/lib/eskf/eskf.h"
#include "math.h"

void quat_multiplication(float *srcA[4], float *srcB[4], float *dest[4]) {
    *dest[0] = (*srcA[0]) * (*srcB[0]) - (*srcA[1]) * (*srcB[1]) - (*srcA[2]) * (*srcB[2]) - (*srcA[3]) * (*srcB[3]);
    *dest[1] = (*srcA[0]) * (*srcB[1]) + (*srcA[1]) * (*srcB[0]) + (*srcA[2]) * (*srcB[3]) + (*srcA[3]) * (*srcB[2]);
    *dest[2] = (*srcA[0]) * (*srcB[2]) - (*srcA[1]) * (*srcB[3]) + (*srcA[2]) * (*srcB[0]) + (*srcA[3]) * (*srcB[1]);
    *dest[3] = (*srcA[0]) * (*srcB[3]) + (*srcA[1]) * (*srcB[2]) - (*srcA[2]) * (*srcB[1]) + (*srcA[3]) * (*srcB[2]);
}

void quat_normalization_in_place(float *src[4]) {
    float norm = sqrt((*src[0]) * (*src[0]) + (*src[1]) * (*src[1]) + (*src[2]) * (*src[2]) + (*src[3]) * (*src[3]));

    *src[0] = *src[0] / norm;
    *src[1] = *src[1] / norm;
    *src[2] = *src[2] / norm;
    *src[3] = *src[3] / norm;
}
