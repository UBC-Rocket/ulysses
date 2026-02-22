#ifndef STATE_H
#define STATE_H

#include <stdint.h>

typedef struct {
    float w;  // scalar component
    float x;  // vector x component
    float y;  // vector y component
    float z;  // vector z component
} quaternion_t;

typedef struct {
    // --- Translational state (navigation frame: NED or ENU, pick one and be consistent) ---
    float pos[3];     // position [m]      -> {x, y, z} in nav frame
    float vel[3];     // velocity [m/s]    -> derivative of position in nav frame

    // --- Rotational state (body frame) ---
    quaternion_t q_bn; // attitude quaternion: rotation from body to navigation frame
    float omega_b[3];  // angular velocity [rad/s], body frame {p, q, r}

    uint64_t u_s; // timestamp
} state_t;

typedef struct{
    float R[3][3];
} rotation_matrix_t;

/**
 * @brief Convert a unit quaternion to a 3x3 rotation matrix.
 *
 * The quaternion is assumed to follow the Hamilton convention (w + xi + yj + zk)
 * and describe the rotation from the body frame to the navigation frame.
 *
 * @param q   Pointer to input quaternion.
 * @param out Pointer to rotation matrix to fill.
 */
void quaternion_to_rotation_matrix(const quaternion_t *q, rotation_matrix_t *out);

/**
 * @brief Multiply a rotation matrix by a 3-vector.
 *
 * Computes out = R * v.
 *
 * @param R     Pointer to rotation matrix.
 * @param v     Pointer to vector (length 3).
 * @param out   Destination for result (length 3).
 */
void rotation_matrix_vector_mul(const rotation_matrix_t *R,
                                const float v[3],
                                float out[3]);

#endif
