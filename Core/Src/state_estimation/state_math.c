#include "state_estimation/state.h"
#include <math.h>

void quaternion_to_rotation_matrix(const quaternion_t *q, rotation_matrix_t *out)
{
    if (!q || !out) return;

    const float w = q->w;
    const float x = q->x;
    const float y = q->y;
    const float z = q->z;

    const float ww = w * w;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    out->R[0][0] = ww + xx - yy - zz;
    out->R[0][1] = 2.0f * (xy - wz);
    out->R[0][2] = 2.0f * (xz + wy);

    out->R[1][0] = 2.0f * (xy + wz);
    out->R[1][1] = ww - xx + yy - zz;
    out->R[1][2] = 2.0f * (yz - wx);

    out->R[2][0] = 2.0f * (xz - wy);
    out->R[2][1] = 2.0f * (yz + wx);
    out->R[2][2] = ww - xx - yy + zz;
}

void rotation_matrix_vector_mul(const rotation_matrix_t *R,
                                const float v[3],
                                float out[3])
{
    if (!R || !v || !out) return;

    out[0] = R->R[0][0] * v[0] + R->R[0][1] * v[1] + R->R[0][2] * v[2];
    out[1] = R->R[1][0] * v[0] + R->R[1][1] * v[1] + R->R[1][2] * v[2];
    out[2] = R->R[2][0] * v[0] + R->R[2][1] * v[1] + R->R[2][2] * v[2];
}
