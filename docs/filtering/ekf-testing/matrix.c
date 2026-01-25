#include "ekf.h"
#include <math.h>
#include <string.h>


/* normalizes quaternion */
void normalize(float q[4]) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    
    for (int i = 0; i < 4; i++) q[i] = q[i] / norm;
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

void transpose4x4(const float A[4][4], float AT[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            AT[j][i] = A[i][j];
}

void transpose3x4_to_4x3(const float A[3][4], float AT[4][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            AT[j][i] = A[i][j];
}

void transpose6x6(const float A[6][6], float AT[6][6])
{
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            AT[j][i] = A[i][j];
}

void transpose3x3(const float A[3][3], float AT[3][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            AT[j][i] = A[i][j];
}

void transpose3x6_to_6x3(const float A[3][6], float AT[6][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j)
            AT[j][i] = A[i][j];
}