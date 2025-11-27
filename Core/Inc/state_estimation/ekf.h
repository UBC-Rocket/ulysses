#ifndef EKF_H
#define EKF_H

#include "FreeRTOS.h"

#define STATE_DIM 4

typedef struct {
    float vals[4];            // real, i, j, k (or w, x, y, z)
    float covar[4][4];        // covariance matrix
    float process[4][4];      // process noise
    float measurement[3][3];  // measurement noise (of accel)
} quaternion_state;

typedef struct {
    float vals[3];            // x, y, z
    float covar[3][3];        // covariance matrix
    float process[3][3];      // process noise
    float measurement[3][3];  // measurement noise (of gps)
} position_state;

typedef struct {
    quaternion_state quaternion;  
    position_state position;                 
} EKF;

void get_state_x(float out[4]);

void init_ekf(
    float process_noise[STATE_DIM][STATE_DIM],
    float measurement_noise[3][3]
);

void tick_ekf(
    float deltaTime,
    float gyro[3],
    float accel[3]
);

/* Macro to multiply any given A (r1 x c1) by B (c1 x c2), producing (r1 x c2) */
#define MAT_MUL(A, B, C, r1, c1, c2)                          \
    do {                                                      \
        for (int i = 0; i < (r1); ++i) {                      \
            for (int j = 0; j < (c2); ++j) {                  \
                (C)[i][j] = 0;                                \
                for (int k = 0; k < (c1); ++k) {              \
                    (C)[i][j] += (A)[i][k] * (B)[k][j];       \
                }                                             \
            }                                                 \
        }                                                     \
    } while (0)

/**
 * @brief shared orientation data
 */
typedef struct {
    /** 
     * @brief Quaternion
     * @details Stored in the order [w, x, y, z]
     */
    float quat[4]; // real, i, j, k
    /** 
     * @brief Euler angle in degrees
     * @details Stored as [roll, pitch]
     */
    float euler[2]; // roll, pitch
} orientation_t;

/**
 * @brief returns orientation data in orientation_t
 */
void get_orientation(orientation_t v);

#endif