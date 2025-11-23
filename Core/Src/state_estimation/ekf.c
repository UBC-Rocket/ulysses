#include "ekf.h"
#include <math.h>
#include <string.h>

/* ------------- helpers ---------------- */

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

/* ------------- EKF ---------------- */
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
            ekf.covar[i][j] = 1;
            continue;
        }

        ekf.covar[i][j] = 0;
    }

    // copy inputs
    for (int i = 0; i < STATE_DIM; i++) for (int j = 0; j < STATE_DIM; j++) ekf.process[i][j] = process_noise[i][j];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ekf.measurement[i][j] = measurement_noise[i][j];
}

void state_transition(
    float p[4], // previous state
    float next_state[4],
    float time_step,
    float g[3] // gyro data
)
{
    float P[4][1];
    for (int i = 0; i < 4; i++) P[i][0] = p[i];

    /*
    | 0   -gx   -gy  -gz  |
    | gx   0     gz  -gy  |
    | gy  -gz    0    gx  |
    | gz   gy   -gx   0   |
    */

    float B[4][4] = {{0, -g[0], -g[1], -g[2]},
                    {g[0], 0, g[2], -g[1]},
                    {g[1], -g[2], 0, g[0]},
                    {g[2], g[1], -g[0], 0}};


    float C[4][1];

    MAT_MUL(B, P, C, 4, 4, 1);

    for (int i = 0; i < 4; i++) C[i][0] = C[i][0] * (time_step / 2);

    for (int i = 0; i < 4; i++) {
        next_state[i] = p[i] + C[i][0];
    }

    normalize(next_state);
}

void get_state_jacobian(
    float g[3],
    float dT,
    float j[4][4]
)
{
    /*
    |  1    -gxdt   -gydt   -gzdt |
    | gxdt    1      gzdt   -gydt |
    | gydt  -gzdt     1      gxdt |
    | gzdt   gydt   -gxdt     1   |
    */
    float xdt = 0.5f * g[0] * dT;
    float ydt = 0.5f * g[1] * dT;
    float zdt = 0.5f * g[2] * dT;

    memcpy(j,
        (float [4][4]){
            {1,   -xdt, -ydt, -zdt},
            {xdt,  1,    zdt, -ydt},
            {ydt, -zdt,  1,    xdt},
            {zdt,  ydt, -xdt,  1  }
        },
        sizeof(float[4][4]));

    // j[0][0]=1;   j[0][1]=-xdt; j[0][2]=-ydt; j[0][3]=-zdt;
    // j[1][0]=xdt; j[1][1]=1;    j[1][2]=zdt;  j[1][3]=-ydt;
    // j[2][0]=ydt; j[2][1]=-zdt; j[2][2]=1;    j[2][3]=xdt;
    // j[3][0]=zdt; j[3][1]=ydt;  j[3][2]=-xdt; j[3][3]=1;

}

void predict_covar(
    float predicted_covar[4][4],
    float jacobian[4][4]
)
{
    float m1[4][4];

    float jacobian_transposed[4][4];
    transpose4x4(jacobian, jacobian_transposed);

    MAT_MUL(jacobian, ekf.covar, m1, 4, 4, 4);
    MAT_MUL(m1, jacobian_transposed, predicted_covar, 4, 4, 4);

    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) predicted_covar[i][j] += ekf.process[i][j];
}

void predict_accel_from_quat(const float q[4], float accel_pred[3])
{
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    // v_body = q * v_world * q_conjugate
    accel_pred[0] = 2.0f * (q1*q3 - q0*q2); // ax
    accel_pred[1] = 2.0f * (q2*q3 + q0*q1); // ay
    accel_pred[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3; // az
}

void get_h_jacobian(
    float q[4],
    float h_jacobian[3][4]
)
{
    float q0 = 2 * q[0];
    float q1 = 2 * q[1];
    float q2 = 2 * q[2];
    float q3 = 2 * q[3];

    memcpy(h_jacobian, 
        (float[3][4]) {
            {-q2, q3, -q0, q1},
            {q1, q0, q3, q2},
            {q0, -q1, -q2, q3}
        }, 
        sizeof(float[3][4]));
}

void tick_ekf(
    float deltaTime,
    float gyro[3],
    float accel[3]
)
{
    /* prediction step */
    float next_state[4];
    state_transition(ekf.x, next_state, deltaTime, gyro);

    float state_jacobian[4][4];
    get_state_jacobian(gyro, deltaTime, state_jacobian);

    float predicted_covar[4][4];

    predict_covar(predicted_covar, state_jacobian);

    /* update step */
    // find innovation
    float innovation[3][1];

    float predicted_accel[3];
    predict_accel_from_quat(next_state, predicted_accel);

    for (int i = 0; i < 3; i++) innovation[i][0] = accel[i] - predicted_accel[i]; // minus predicted gravity 

    // get new kalman gain ( KILL ME !!!)
    // half of the ram of ulysses will be dedicated to 4x4 matrices :thumbsup:
    float h_jacobian[3][4];
    get_h_jacobian(next_state, h_jacobian);

    float h_jacobian_t[4][3];
    transpose3x4_to_4x3(h_jacobian, h_jacobian_t);

    float mat1[4][3]; // p_{k, k-1} * H_k ^ T
    MAT_MUL(predicted_covar, h_jacobian_t, mat1, 4, 4, 3);

    float mat2[3][4]; // H_k * p_{k, k-1}
    MAT_MUL(h_jacobian,predicted_covar, mat2, 3, 4, 4);

    float mat3[3][3]; // mat2 * H_k ^ T
    MAT_MUL(mat2, h_jacobian_t, mat3, 3, 4, 3);

    // add measurement covariance
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) mat3[i][j] += ekf.measurement[i][j];

    float inv_mat3[3][3]; // self-explanatory
    int result = inverse(mat3, inv_mat3);

    if (!result) {
        /* matrix could not be inverted, cannot continue with ekf update (unlikely in typical conditions) */
        return;
    }
    
    float kalman_gain[4][3]; // mat1 * inv_mat3
    MAT_MUL(mat1, inv_mat3, kalman_gain, 4, 3, 3);

    // compute adjustment
    float adjustment[4][1];
    MAT_MUL(kalman_gain, innovation, adjustment, 4, 3, 1);

    for (int i = 0; i < 4; i++) ekf.x[i] = next_state[i] + adjustment[i][0];
    normalize(ekf.x);

    float KH[4][4];
    MAT_MUL(kalman_gain, h_jacobian, KH, 4, 3, 4);

    float I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    float I_minus_KH[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            I_minus_KH[i][j] = I[i][j] - KH[i][j];

    float new_p[4][4];
    MAT_MUL(I_minus_KH, predicted_covar, new_p, 4, 4, 4);

    // store it
    memcpy(ekf.covar, new_p, sizeof(new_p));
}

void get_state_x(float out[4])
{
    for (int i = 0; i < 4; i++)
        out[i] = ekf.x[i];
}
