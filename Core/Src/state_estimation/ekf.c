#include "ekf.h"
#include <debug/log.h>
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

    for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) predicted_covar[i][j] += ekf.body.process[i][j];
}

void init_ekf(
    float process_noise_quaternion[4][4],
    float measurement_noise_quaternion[3][3],
    float process_noise_body[6][6],
    float measurement_noise_body[3][3],
    float expected_g[3]
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

    for (int i = 0; i < 3; i++) ekf.expected_g[i] = expected_g[i];
}

void tick_ekf(
    float deltaTime,
    float gyro[3],
    float accel[3],
    float gps_pos[3]
)
{
    /* prediction step */
    float processing_quaternion[4];
    float processing_position[3];
    float processing_velocity[3];
    
    state_transition_orientation(&ekf.quaternion, deltaTime, gyro, processing_quaternion);

    // float transformed_accel[3];
    // transform_accel_data(accel, ekf.quaternion.vals, transformed_accel);
    state_transition_body(&ekf.body, deltaTime, accel, processing_position, processing_velocity);

    float state_jacobian_quaternion[4][4];
    float state_jacobian_body[6][6];

    get_state_jacobian_orientation(gyro, deltaTime, state_jacobian_quaternion);
    get_state_jacobian_body(deltaTime, state_jacobian_body);

    float predicted_covar_quaternion[4][4];
    float predicted_covar_body[6][6];

    predict_covar_orientation(state_jacobian_quaternion, predicted_covar_quaternion);
    predict_covar_body(state_jacobian_body, predicted_covar_body);

    /* update step */
    // find innovation
    float innovation_quaternion[3][1];
    float innovation_body[3][1];

    float predicted_accel[3];
    predict_accel_from_quat(processing_quaternion, predicted_accel, ekf.expected_g);
    //DLOG_PRINT("%f %f %f\n", predicted_accel[0], predicted_accel[1], predicted_accel[2]);

    for (int i = 0; i < 3; i++) innovation_quaternion[i][0] = accel[i] - predicted_accel[i]; // minus predicted gravity 

    for (int i = 0; i < 3; i++) innovation_body[i][0] = gps_pos[i] - processing_position[i];

    // get new kalman gain ( KILL ME !!!)
    // half of the ram of ulysses will be dedicated to 4x4 matrices :thumbsup:
    float h_jacobian_quaternion[3][4];
    float h_jacobian_body[3][6];
    get_h_jacobian_quaternion(processing_quaternion, ekf.expected_g, h_jacobian_quaternion);
    get_h_jacobian_body(h_jacobian_body);

    float h_jacobian_quaternion_t[4][3];
    float h_jacobian_body_t[6][3];
    transpose3x4_to_4x3(h_jacobian_quaternion, h_jacobian_quaternion_t);
    transpose3x6_to_6x3(h_jacobian_body, h_jacobian_body_t);

    // p_{k, k-1} * H_k ^ T
    float mat1_q[4][3]; 
    float mat1_b[6][3];
    MAT_MUL(predicted_covar_quaternion, h_jacobian_quaternion_t, mat1_q, 4, 4, 3);
    MAT_MUL(predicted_covar_body, h_jacobian_body_t, mat1_b, 6, 6, 3);

    // H_k * p_{k, k-1}
    float mat2_q[3][4]; 
    float mat2_b[3][6]; 
    MAT_MUL(h_jacobian_quaternion, predicted_covar_quaternion, mat2_q, 3, 4, 4);
    MAT_MUL(h_jacobian_body, predicted_covar_body, mat2_b, 3, 6, 6);

    // mat2 * H_k ^ T
    float mat3_q[3][3]; 
    float mat3_b[3][3]; 
    MAT_MUL(mat2_q, h_jacobian_quaternion_t, mat3_q, 3, 4, 3);
    MAT_MUL(mat2_b, h_jacobian_body_t, mat3_b, 3, 6, 3);

    // add measurement covariance
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) mat3_q[i][j] += ekf.quaternion.measurement[i][j];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) mat3_b[i][j] += ekf.body.measurement[i][j];
    
    float inv_mat3_q[3][3]; // self-explanatory
    float inv_mat3_b[3][3]; 
    int result_q = inverse(mat3_q, inv_mat3_q);
    int result_b = inverse(mat3_b, inv_mat3_b);

    if (!result_q || !result_b) {
        /* matrix could not be inverted, cannot continue with ekf update (unlikely in typical conditions) */
        return;
    }
    
    // mat1 * inv_mat3
    float kalman_gain_quaternion[4][3]; 
    float kalman_gain_body[6][3]; 
    MAT_MUL(mat1_q, inv_mat3_q, kalman_gain_quaternion, 4, 3, 3);
    MAT_MUL(mat1_b, inv_mat3_b, kalman_gain_body, 6, 3, 3);

    // compute adjustment
    float adjustment_quaternion[4][1];
    float adjustment_body[6][1];
    MAT_MUL(kalman_gain_quaternion, innovation_quaternion, adjustment_quaternion, 4, 3, 1);
    MAT_MUL(kalman_gain_body, innovation_body, adjustment_body, 6, 3, 1);

    for (int i = 0; i < 4; i++) ekf.quaternion.vals[i] = processing_quaternion[i] + adjustment_quaternion[i][0];
    normalize(ekf.quaternion.vals);

    for (int i = 0; i < 3; i++) ekf.body.position[i] = processing_position[i] + adjustment_body[i][0];
    for (int i = 0; i < 3; i++) ekf.body.velocity[i] = processing_velocity[i] + adjustment_body[i + 3][0];

    float KH_q[4][4];
    float KH_b[6][6];
    MAT_MUL(kalman_gain_quaternion, h_jacobian_quaternion, KH_q, 4, 3, 4);
    MAT_MUL(kalman_gain_body, h_jacobian_body, KH_b, 6, 3, 6);

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            if (i==j) KH_q[i][j] = 1 - KH_q[i][j];
            else  KH_q[i][j] = -KH_q[i][j];

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            if (i==j) KH_b[i][j] = 1 - KH_b[i][j];
            else  KH_b[i][j] = -KH_b[i][j];

    float new_covar_quaternion[4][4];
    MAT_MUL(KH_q, predicted_covar_quaternion, new_covar_quaternion, 4, 4, 4);

    float new_covar_body[6][6];
    MAT_MUL(KH_b, predicted_covar_body, new_covar_body, 6, 6, 6);

    // store it
    memcpy(ekf.quaternion.covar, new_covar_quaternion, sizeof(new_covar_quaternion));
    memcpy(ekf.body.covar, new_covar_body, sizeof(new_covar_body));
}

void get_state(float quaternion[4], float position[3], float velocity[3])
{
    for (int i = 0; i < 4; i++)
        quaternion[i] = ekf.quaternion.vals[i];

    for (int i = 0; i < 3; i++) {
        position[i] = ekf.body.position[i];
        velocity[i] = ekf.body.velocity[i];
    }
}
