#ifndef EKF_H
#define EKF_H

#define STATE_DIM 4

typedef struct {
    float x[STATE_DIM];                    // state vector (just a quaternion for now)
    float covar[STATE_DIM][STATE_DIM];     // covariance matrix
    float process[STATE_DIM][STATE_DIM];   // process noise
    float measurement[3][3];               // measurement noise (of accel)
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

#endif