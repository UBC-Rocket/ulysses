#include "ekf.h"
#include <math.h>

static EKF ekf;

void init_ekf(
    float process_noise[STATE_DIM][STATE_DIM],
    float measurement_noise[3][3])
{
    // init
    ekf.x[0] = 1;
    ekf.x[1] = 0;
    ekf.x[2] = 0;
    ekf.x[3] = 0;

    // sets ekf.P to an identity matrix
    for (int i = 0; i < STATE_DIM; i++) for (int j = 0; j < STATE_DIM; j++) {
        if (i==j) {
            ekf.P[i][j] = 1;
            continue;
        }

        ekf.P[i][j] = 0;
    }

    // copy inputs
    for (int i = 0; i < STATE_DIM; i++) for (int j = 0; j < STATE_DIM; j++) ekf.Q[i][j] = process_noise[i][j];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ekf.R[i][j] = measurement_noise[i][j];
}

void tick_ekf(
    float deltaTime,
    float gyro[3],
    float accel[3]
)
{
    
}

#define N 3

/* finds inverse of matrix
    1 if successful, 0 if no inverse exists */
int inverse(float a[N][N], float inverse[N][N]) {
    // Initialize inverse as the identity matrix
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            inverse[i][j] = (i == j) ? 1.0 : 0.0;

    // Perform elementary row operations
    for (int i = 0; i < N; i++) {
        // Find the pivot element
        float pivot = a[i][i];
        if (fabs(pivot) < 1e-6) {
            // Find a row below with a non-zero element and swap
            int swap_row = -1;
            for (int r = i + 1; r < N; r++) {
                if (fabs(a[r][i]) > 1e-6) {
                    swap_row = r;
                    break;
                }
            }
            if (swap_row == -1) {
                return 0; // No inverse
            }

            // Swap rows in both a and inverse
            for (int c = 0; c < N; c++) {
                float temp = a[i][c];
                a[i][c] = a[swap_row][c];
                a[swap_row][c] = temp;

                temp = inverse[i][c];
                inverse[i][c] = inverse[swap_row][c];
                inverse[swap_row][c] = temp;
            }

            pivot = a[i][i];
        }

        // Normalize the pivot row
        for (int j = 0; j < N; j++) {
            a[i][j] /= pivot;
            inverse[i][j] /= pivot;
        }

        // Eliminate all other elements in column i
        for (int r = 0; r < N; r++) {
            if (r != i) {
                float factor = a[r][i];
                for (int c = 0; c < N; c++) {
                    a[r][c] -= factor * a[i][c];
                    inverse[r][c] -= factor * inverse[i][c];
                }
            }
        }
    }

    return 1; // Success
}

/* normalizes quaternion */
void normalize(float q[4]) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    
    for (int i = 0; i < 4; i++) q[i] = q[i] / norm;
}