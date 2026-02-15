#include "ekf.h"
#include "math.h"
#include "string.h"
#include "matrix.h"
#include "quaternion.h"
#include "body.h"
#include "state.h"
#include "debug/log.h"

static EKF ekf;
long long int mm = 0;

// ==========================================
// HELPER FUNCTIONS
// ==========================================

void predict_covar_orientation(float jacobian[4][4], float predicted_covar[4][4]) {
    float m1[4][4];
    float jacobian_transposed[4][4];
    
    transpose4x4(jacobian, jacobian_transposed);
    MAT_MUL(jacobian, ekf.quaternion.covar, m1, 4, 4, 4);
    MAT_MUL(m1, jacobian_transposed, predicted_covar, 4, 4, 4);

    for (int i = 0; i < 4; i++) 
        for (int j = 0; j < 4; j++) 
            predicted_covar[i][j] += ekf.quaternion.process[i][j];
}

void predict_covar_body(float jacobian[6][6], float predicted_covar[6][6]) {
    float m1[6][6];
    float jacobian_transposed[6][6];

    transpose6x6(jacobian, jacobian_transposed);
    MAT_MUL(jacobian, ekf.body.covar, m1, 6, 6, 6);
    MAT_MUL(m1, jacobian_transposed, predicted_covar, 6, 6, 6);

    for (int i = 0; i < 6; i++) 
        for (int j = 0; j < 6; j++) 
            predicted_covar[i][j] += ekf.body.process[i][j];
}

// ==========================================
// INITIALIZATION
// ==========================================

void init_ekf_orientation(
    float process_noise[4][4],
    float measurement_noise[3][3],
    float expected_g[3]
) {
    // Initialize quaternion to identity
    ekf.quaternion.vals[0] = 1;
    ekf.quaternion.vals[1] = 0;
    ekf.quaternion.vals[2] = 0;
    ekf.quaternion.vals[3] = 0;
    ekf.quaternion.index = 0;

    // Set Covariance to Identity
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf.quaternion.covar[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Copy Noise and Gravity
    for (int i = 0; i < 4; i++) 
        for (int j = 0; j < 4; j++) 
            ekf.quaternion.process[i][j] = process_noise[i][j];
            
    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            ekf.quaternion.measurement[i][j] = measurement_noise[i][j];

    for (int i = 0; i < 3; i++) 
        ekf.expected_g[i] = expected_g[i];
}

void init_ekf_body(
    float process_noise[6][6],
    float measurement_noise[3][3]
) {
    // Initialize state to 0
    for (int i = 0; i < 3; i++) ekf.body.position[i] = 0;
    for (int i = 0; i < 3; i++) ekf.body.velocity[i] = 0;
    ekf.body.index = 0;

    // Set Covariance to Identity
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf.body.covar[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Copy Noise
    for (int i = 0; i < 6; i++) 
        for (int j = 0; j < 6; j++) 
            ekf.body.process[i][j] = process_noise[i][j];

    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            ekf.body.measurement[i][j] = measurement_noise[i][j];
}

// Wrapper for legacy initialization if needed
void init_ekf(
    float process_noise_quaternion[4][4],
    float measurement_noise_quaternion[3][3],
    float process_noise_body[6][6],
    float measurement_noise_body[3][3],
    float expected_g[3]
) {
    init_ekf_orientation(process_noise_quaternion, measurement_noise_quaternion, expected_g);
    init_ekf_body(process_noise_body, measurement_noise_body);
}

// ==========================================
// UPDATE LOOPS
// ==========================================

void tick_ekf_orientation(float deltaTime, float gyro[3], float accel[3]) {
    ekf.quaternion.index += 1;

    // --- 1. PREDICTION ---
    float processing_quaternion[4];
    state_transition_orientation(&ekf.quaternion, deltaTime, gyro, processing_quaternion);

    float state_jacobian_quaternion[4][4];
    get_state_jacobian_orientation(gyro, deltaTime, state_jacobian_quaternion);

    float predicted_covar_quaternion[4][4];
    predict_covar_orientation(state_jacobian_quaternion, predicted_covar_quaternion);

    // --- 2. UPDATE (Correction) ---
    float innovation_quaternion[3][1];
    float predicted_accel[3];
    
    // Project current quaternion to expected gravity vector
    predict_accel_from_quat(processing_quaternion, predicted_accel, ekf.expected_g);

    // Normalize accel for innovation
    float a_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float a_normalized[3];
    if (a_norm > 0.1f) {
        for(int i=0; i<3; i++) a_normalized[i] = accel[i] / a_norm;
    } else {
        for(int i=0; i<3; i++) a_normalized[i] = ekf.expected_g[i];
    }

    // Calculate Innovation
    for (int i = 0; i < 3; i++) 
        innovation_quaternion[i][0] = a_normalized[i] - predicted_accel[i];

    // H Jacobian
    float h_jacobian_quaternion[3][4];
    get_h_jacobian_quaternion(processing_quaternion, ekf.expected_g, h_jacobian_quaternion);

    float h_jacobian_quaternion_t[4][3];
    transpose3x4_to_4x3(h_jacobian_quaternion, h_jacobian_quaternion_t);

    // Calculate S = H * P * H' + R
    float mat1_q[4][3]; 
    MAT_MUL(predicted_covar_quaternion, h_jacobian_quaternion_t, mat1_q, 4, 4, 3); // P * H'

    float mat2_q[3][4]; 
    MAT_MUL(h_jacobian_quaternion, predicted_covar_quaternion, mat2_q, 3, 4, 4); // H * P

    float mat3_q[3][3]; // S
    MAT_MUL(mat2_q, h_jacobian_quaternion_t, mat3_q, 3, 4, 3);

    // Add Measurement Noise (R)
    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            mat3_q[i][j] += ekf.quaternion.measurement[i][j];

    // Invert S
    float inv_mat3_q[3][3]; 
    if (!inverse(mat3_q, inv_mat3_q)) return; // Failed to invert

    // Calculate Kalman Gain: K = P * H' * S^-1
    float kalman_gain_quaternion[4][3]; 
    MAT_MUL(mat1_q, inv_mat3_q, kalman_gain_quaternion, 4, 3, 3);

    // Update State
    float adjustment_quaternion[4][1];
    MAT_MUL(kalman_gain_quaternion, innovation_quaternion, adjustment_quaternion, 4, 3, 1);

    for (int i = 0; i < 4; i++) 
        ekf.quaternion.vals[i] = processing_quaternion[i] + adjustment_quaternion[i][0];
    
    normalize(ekf.quaternion.vals);

    // Update Covariance: P = (I - K * H) * P
    if (ekf.quaternion.index <= UPDATE_COVAR) {
        return;
    }

    ekf.quaternion.index = 0;

    float KH_q[4][4];
    MAT_MUL(kalman_gain_quaternion, h_jacobian_quaternion, KH_q, 4, 3, 4);

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            KH_q[i][j] = (i == j) ? (1 - KH_q[i][j]) : (-KH_q[i][j]);

    float new_covar_quaternion[4][4];
    MAT_MUL(KH_q, predicted_covar_quaternion, new_covar_quaternion, 4, 4, 4);
    
    memcpy(ekf.quaternion.covar, new_covar_quaternion, sizeof(new_covar_quaternion));
}

void tick_ekf_body(float deltaTime, float accel[3], float gps_pos[3]) {
    ekf.body.index += 1;

    // --- 1. PREDICTION ---
    float processing_position[3];
    float processing_velocity[3];

    // Note: Ideally, accel should be transformed by ekf.quaternion.vals here if needed
    state_transition_body(&ekf.body, deltaTime, accel, processing_position, processing_velocity);

    float state_jacobian_body[6][6];
    get_state_jacobian_body(deltaTime, state_jacobian_body);

    float predicted_covar_body[6][6];
    predict_covar_body(state_jacobian_body, predicted_covar_body);

    // --- 2. UPDATE (Correction) ---
    float innovation_body[3][1];
    for (int i = 0; i < 3; i++) 
        innovation_body[i][0] = gps_pos[i] - processing_position[i];

    // H Jacobian
    float h_jacobian_body[3][6];
    get_h_jacobian_body(h_jacobian_body);

    float h_jacobian_body_t[6][3];
    transpose3x6_to_6x3(h_jacobian_body, h_jacobian_body_t);

    // Calculate S = H * P * H' + R
    float mat1_b[6][3];
    MAT_MUL(predicted_covar_body, h_jacobian_body_t, mat1_b, 6, 6, 3); // P * H'

    float mat2_b[3][6];
    MAT_MUL(h_jacobian_body, predicted_covar_body, mat2_b, 3, 6, 6); // H * P

    float mat3_b[3][3]; // S
    MAT_MUL(mat2_b, h_jacobian_body_t, mat3_b, 3, 6, 3);

    // Add Measurement Noise (R)
    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            mat3_b[i][j] += ekf.body.measurement[i][j];

    // Invert S
    float inv_mat3_b[3][3];
    if (!inverse(mat3_b, inv_mat3_b)) return; // Failed to invert

    // Calculate Kalman Gain: K = P * H' * S^-1
    float kalman_gain_body[6][3];
    MAT_MUL(mat1_b, inv_mat3_b, kalman_gain_body, 6, 3, 3);

    // Update State
    float adjustment_body[6][1];
    MAT_MUL(kalman_gain_body, innovation_body, adjustment_body, 6, 3, 1);

    for (int i = 0; i < 3; i++) ekf.body.position[i] = processing_position[i] + adjustment_body[i][0];
    for (int i = 0; i < 3; i++) ekf.body.velocity[i] = processing_velocity[i] + adjustment_body[i + 3][0];

    // Update Covariance: P = (I - K * H) * P
    if (ekf.body.index <= UPDATE_COVAR) {
        return;
    }

    ekf.body.index = 0;

    float KH_b[6][6];
    MAT_MUL(kalman_gain_body, h_jacobian_body, KH_b, 6, 3, 6);

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            KH_b[i][j] = (i == j) ? (1 - KH_b[i][j]) : (-KH_b[i][j]);

    float new_covar_body[6][6];
    MAT_MUL(KH_b, predicted_covar_body, new_covar_body, 6, 6, 6);

    memcpy(ekf.body.covar, new_covar_body, sizeof(new_covar_body));
}

// ==========================================
// GETTERS
// ==========================================

void get_state(float quaternion[4], float position[3], float velocity[3])
{
    for (int i = 0; i < 4; i++)
        quaternion[i] = ekf.quaternion.vals[i];

    for (int i = 0; i < 3; i++) {
        position[i] = ekf.body.position[i];
        velocity[i] = ekf.body.velocity[i];
    }
}