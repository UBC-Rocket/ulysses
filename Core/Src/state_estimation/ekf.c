#include "ekf.h"
#include <math.h>
#include <string.h>
#include <matrix.h>
#include <quaternion.h>
#include <body.h>

static EKF ekf;

void predict_covar_orientation(
    float jacobian[4][4],
    float predicted_covar[4][4]
)
{
    float m1[4][4];

    float jacobian_transposed[4][4];
    transpose4x4(jacobian, jacobian_transposed);

    MAT_MUL(jacobian, ekf.quaternion.covar, m1, 4, 4, 4);
    MAT_MUL(m1, jacobian_transposed, predicted_covar, 4, 4, 4);

    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) predicted_covar[i][j] += ekf.quaternion.process[i][j];
}

void predict_covar_body(
    float jacobian[6][6],
    float predicted_covar[6][6]
)
{
    float m1[6][6];

    float jacobian_transposed[6][6];
    transpose6x6(jacobian, jacobian_transposed);

    MAT_MUL(jacobian, ekf.body.covar, m1, 6, 6, 6);
    MAT_MUL(m1, jacobian_transposed, predicted_covar, 6, 6, 6);

    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) predicted_covar[i][j] += ekf.body.process[i][j];
}

void init_ekf(
    float process_noise_quaternion[4][4],
    float measurement_noise_quaternion[3][3],
    float process_noise_body[6][6],
    float measurement_noise_body[3][3]
)
{
    // init
    ekf.quaternion.vals[0] = 1;
    ekf.quaternion.vals[1] = 0;
    ekf.quaternion.vals[2] = 0;
    ekf.quaternion.vals[3] = 0;

    for (int i = 0; i < 3; i++) ekf.body.position[i] = 0;
    for (int i = 0; i < 3; i++) ekf.body.velocity[i] = 0;

    // sets quat covar to an identity matrix
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
        if (i==j) {
            ekf.quaternion.covar[i][j] = 1;
            continue;
        }

        ekf.quaternion.covar[i][j] = 0;
    }

    // sets body covar to an identity matrix
    for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) {
        if (i==j) {
            ekf.body.covar[i][j] = 1;
            continue;
        }

        ekf.body.covar[i][j] = 0;
    }

    // copy inputs
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) ekf.quaternion.process[i][j] = process_noise_quaternion[i][j];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ekf.quaternion.measurement[i][j] = measurement_noise_quaternion[i][j];

    for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) ekf.body.process[i][j] = process_noise_body[i][j];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ekf.body.measurement[i][j] = measurement_noise_body[i][j];
}

void tick_ekf(
    float deltaTime,
    float gyro[3],
    float accel[3],
    float gps_pos[3]
)
{
    /* prediction step */
    state_transition_orientation(&ekf.quaternion, deltaTime, gyro);
    state_transition_body(&ekf.body, deltaTime, accel);

    float state_jacobian_quaternion[4][4];
    float state_jacobian_body[6][6];

    get_state_jacobian_orientation(gyro, deltaTime, state_jacobian_quaternion);
    get_state_jacobian_body(gyro, deltaTime, state_jacobian_body);

    float predicted_covar_quaternion[4][4];
    float predicted_covar_body[6][6];

    predict_covar_orientation(state_jacobian_quaternion, predicted_covar_quaternion);
    predict_covar_body(state_jacobian_body, predicted_covar_body);

    /* update step */
    // find innovation
    float innovation_quaternion[3][1];
    float innovation_body[3][1];

    float predicted_accel[3];
    predict_accel_from_quat(ekf.quaternion.vals, predicted_accel);

    for (int i = 0; i < 3; i++) innovation_quaternion[i][0] = accel[i] - predicted_accel[i]; // minus predicted gravity 

    // get new kalman gain ( KILL ME !!!)
    // half of the ram of ulysses will be dedicated to 4x4 matrices :thumbsup:
    float h_jacobian[3][4];
    get_h_jacobian_quaternion(ekf.quaternion.vals, h_jacobian);

    float h_jacobian_t[4][3];
    transpose3x4_to_4x3(h_jacobian, h_jacobian_t);

    float mat1[4][3]; // p_{k, k-1} * H_k ^ T
    MAT_MUL(predicted_covar_quaternion, h_jacobian_t, mat1, 4, 4, 3);

    float mat2[3][4]; // H_k * p_{k, k-1}
    MAT_MUL(h_jacobian, predicted_covar_quaternion, mat2, 3, 4, 4);

    float mat3[3][3]; // mat2 * H_k ^ T
    MAT_MUL(mat2, h_jacobian_t, mat3, 3, 4, 3);

    // add measurement covariance
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) mat3[i][j] += ekf.quaternion.measurement[i][j];

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
    MAT_MUL(kalman_gain, innovation_quaternion, adjustment, 4, 3, 1);

    for (int i = 0; i < 4; i++) ekf.quaternion.vals[i] = ekf.quaternion.vals[i] + adjustment[i][0];
    normalize(ekf.quaternion.vals);

    float KH[4][4];
    MAT_MUL(kalman_gain, h_jacobian, KH, 4, 3, 4);

    float I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    float I_minus_KH[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            I_minus_KH[i][j] = I[i][j] - KH[i][j];

    float new_p[4][4];
    MAT_MUL(I_minus_KH, predicted_covar_quaternion, new_p, 4, 4, 4);

    // store it
    memcpy(ekf.quaternion.covar, new_p, sizeof(new_p));
}

void get_state_x(float out[4])
{
    for (int i = 0; i < 4; i++)
        out[i] = ekf.quaternion.vals[i];
}
