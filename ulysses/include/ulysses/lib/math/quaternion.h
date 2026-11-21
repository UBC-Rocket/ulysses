#ifndef ULYSSES_LIB_MATH_QUATERNION
#define ULYSSES_LIB_MATH_QUATERNION

#include "ulysses/lib/assert.h"

#include <math.h>

typedef union quaternion {
    float q[4];

    struct {
        float w;
        float v[3];
    };
} quaternion_t;

#endif /* ULYSSES_LIB_MATH_QUATERNION */
