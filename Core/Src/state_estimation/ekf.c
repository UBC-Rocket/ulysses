#include "ekf.h"
#include "math.h"
#include "string.h"
#include "matrix.h"
#include "quaternion.h"
#include "body.h"
#include "state.h"
#include "debug/log.h"
#include "timestamp.h"

static EKF ekf;

#define EKF_DEBUG_LOG_PERIOD 50U
#define MAX_QUAT_VARIANCE 0.5f 
#define ACCEL_UPDATE_MIN_NORM_G 0.85f
#define ACCEL_UPDATE_MAX_NORM_G 1.15f
#define ACCEL_UPDATE_MAX_GYRO_NORM_RAD_S 0.6f

static uint32_t s_orientation_tick_count = 0;
static uint32_t s_body_tick_count = 0;

static uint8_t ekf_should_log(uint32_t tick_count)
{
    return (tick_count % EKF_DEBUG_LOG_PERIOD) == 0U;
}

static uint8_t ekf_accel_update_valid(float a_norm_g)
{
    return (a_norm_g >= ACCEL_UPDATE_MIN_NORM_G) &&
           (a_norm_g <= ACCEL_UPDATE_MAX_NORM_G);
}

static uint8_t ekf_gyro_update_valid(float g_norm_rad_s)
{
    /* During fast rotations, accel-based gravity direction is less reliable for correction. */
    return g_norm_rad_s <= ACCEL_UPDATE_MAX_GYRO_NORM_RAD_S;
}

static void quat_to_rpy_rad(const float q[4], float *roll, float *pitch, float *yaw)
{
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    const float sinr = 2.0f * (w * x + y * z);
    const float cosr = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr, cosr);

    const float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf((float)M_PI * 0.5f, sinp);
    } else {
        *pitch = asinf(sinp);
    }

    const float siny = 2.0f * (w * z + x * y);
    const float cosy = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny, cosy);
}

static void rpy_to_quat_rad(float roll, float pitch, float yaw, float q[4])
{
    const float hr = 0.5f * roll;
    const float hp = 0.5f * pitch;
    const float hy = 0.5f * yaw;

    const float cr = cosf(hr);
    const float sr = sinf(hr);
    const float cp = cosf(hp);
    const float sp = sinf(hp);
    const float cy = cosf(hy);
    const float sy = sinf(hy);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

 static void ekf_preserve_predicted_yaw(const float q_pred[4], float q_corr[4]) {
    // isolate yaw of quaternion
    float norm_corr = sqrtf(q_corr[0] * q_corr[0] + q_corr[3] * q_corr[3]);
    float twist_corr_inv[4] = { 1.0f, 0, 0, 0 };
    if (norm_corr > 1e-6f) {
        twist_corr_inv[0] = q_corr[0] / norm_corr;
        twist_corr_inv[3] = -q_corr[3] / norm_corr; // Conjugate
    }

    // swing_corr = q_corr * twist_corr_inv
    float swing_corr[4];
    quat_mult(q_corr, twist_corr_inv, swing_corr);

    float norm_pred = sqrtf(q_pred[0] * q_pred[0] + q_pred[3] * q_pred[3]);
    float twist_pred[4] = { 1.0f, 0, 0, 0 };
    if (norm_pred > 1e-6f) {
        twist_pred[0] = q_pred[0] / norm_pred;
        twist_pred[3] = q_pred[3] / norm_pred;
    }

    // swing_corr * twist_pred
    quat_mult(swing_corr, twist_pred, q_corr);
    normalize(q_corr);
 }

#define EKF_LOG(fmt, ...) \
    //DLOG_PRINT("[EKF %lu] " fmt "\r\n", (unsigned long)1, ##__VA_ARGS__)

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

/**
 * Sparse covariance prediction exploiting F = [[I, dt*I], [0, I]].
 *
 * P (6x6) is partitioned into 3x3 blocks:
 *   P = [[Ppp, Ppv],
 *        [Pvp, Pvv]]
 *
 * F*P*F' = [[Ppp + dt*(Ppv+Pvp) + dt²*Pvv,  Ppv + dt*Pvv],
 *           [Pvp + dt*Pvv,                    Pvv         ]]
 */
void predict_covar_body_sparse(float dt, float predicted_covar[6][6]) {
    const float dt2 = dt * dt;
    const float (*P)[6] = (const float (*)[6])ekf.body.covar;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float Ppp = P[i][j];
            float Ppv = P[i][j + 3];
            float Pvp = P[i + 3][j];
            float Pvv = P[i + 3][j + 3];

            /* Top-left: Ppp + dt*(Ppv+Pvp) + dt²*Pvv */
            predicted_covar[i][j] = Ppp + dt * (Ppv + Pvp) + dt2 * Pvv
                                    + ekf.body.process[i][j];

            /* Top-right: Ppv + dt*Pvv */
            predicted_covar[i][j + 3] = Ppv + dt * Pvv
                                         + ekf.body.process[i][j + 3];

            /* Bottom-left: Pvp + dt*Pvv */
            predicted_covar[i + 3][j] = Pvp + dt * Pvv
                                         + ekf.body.process[i + 3][j];

            /* Bottom-right: Pvv (unchanged) */
            predicted_covar[i + 3][j + 3] = Pvv
                                             + ekf.body.process[i + 3][j + 3];
        }
    }
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
    s_orientation_tick_count++;

    if (ekf_should_log(s_orientation_tick_count)) {
        //DLOG_PRINT("gyro=[%f, %f, %f]\n", gyro[0], gyro[1], gyro[2]);
    }

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

    // Normalize accel for innovation
    float a_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);

    const float g_norm = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);

    /* During strong linear acceleration or fast angular motion, gravity direction from
       accel is unreliable. Skip measurement correction and run gyro-only propagation. */
    if (!ekf_accel_update_valid(a_norm) || !ekf_gyro_update_valid(g_norm)) {
        memcpy(ekf.quaternion.vals, processing_quaternion, sizeof(processing_quaternion));
        memcpy(ekf.quaternion.covar, predicted_covar_quaternion, sizeof(predicted_covar_quaternion));
        return;
    }

    /* During strong linear acceleration, gravity direction from accel is unreliable.
       Skip measurement correction and run gyro-only propagation. */

    // Project current quaternion to expected gravity vector
    predict_accel_from_quat(processing_quaternion, predicted_accel, ekf.expected_g);

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

    // Calculate S = H * (P * H') + R  (reuse P*H' for Kalman gain)
    float mat1_q[4][3];
    MAT_MUL(predicted_covar_quaternion, h_jacobian_quaternion_t, mat1_q, 4, 4, 3); // P * H'

    float mat3_q[3][3]; // S = H * (P * H')
    MAT_MUL(h_jacobian_quaternion, mat1_q, mat3_q, 3, 4, 3);

    // Add Measurement Noise (R)
    for (int i = 0; i < 3; i++) 
        for (int j = 0; j < 3; j++) 
            mat3_q[i][j] += ekf.quaternion.measurement[i][j];

    // Invert S
    float inv_mat3_q[3][3]; 
    if (!inverse(mat3_q, inv_mat3_q)) {
        EKF_LOG("ori S inv fail");
        return; // Failed to invert
    }

    // Calculate Kalman Gain: K = P * H' * S^-1
    float kalman_gain_quaternion[4][3]; 
    MAT_MUL(mat1_q, inv_mat3_q, kalman_gain_quaternion, 4, 3, 3);

    // Update State
    float adjustment_quaternion[4][1];
    MAT_MUL(kalman_gain_quaternion, innovation_quaternion, adjustment_quaternion, 4, 3, 1);

    for (int i = 0; i < 4; i++) 
        ekf.quaternion.vals[i] = processing_quaternion[i] + adjustment_quaternion[i][0];
    
    normalize(ekf.quaternion.vals);
    ekf_preserve_predicted_yaw(processing_quaternion, ekf.quaternion.vals);

    // if (ekf_should_log(s_orientation_tick_count)) {
    //     EKF_LOG("ori q=%.3f %.3f %.3f %.3f",
    //             ekf.quaternion.vals[0], ekf.quaternion.vals[1],
    //             ekf.quaternion.vals[2], ekf.quaternion.vals[3]);
    // }

    // Update Covariance: P = (I - K * H) * P
    if (ekf.quaternion.index <= UPDATE_COVAR) {
        return;
    }

    ekf.quaternion.index = 0;

    // Joseph form: P = (I-KH)*P*(I-KH)' + K*R*K'
    float IKH[4][4];
    MAT_MUL(kalman_gain_quaternion, h_jacobian_quaternion, IKH, 4, 3, 4);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            IKH[i][j] = (i == j) ? (1.0f - IKH[i][j]) : (-IKH[i][j]);

    // (I-KH)*P
    float IKH_P[4][4];
    MAT_MUL(IKH, predicted_covar_quaternion, IKH_P, 4, 4, 4);

    // (I-KH)*P*(I-KH)'
    float IKH_t[4][4];
    transpose4x4(IKH, IKH_t);
    float new_covar_quaternion[4][4];
    MAT_MUL(IKH_P, IKH_t, new_covar_quaternion, 4, 4, 4);

    // K*R*K'
    float KR[4][3];
    MAT_MUL(kalman_gain_quaternion, ekf.quaternion.measurement, KR, 4, 3, 3);
    float KR_Kt[4][4];
    float K_t[3][4];
    transpose4x3_to_3x4(kalman_gain_quaternion, K_t);
    MAT_MUL(KR, K_t, KR_Kt, 4, 3, 4);

    // Sum
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            new_covar_quaternion[i][j] += KR_Kt[i][j];

    memcpy(ekf.quaternion.covar, new_covar_quaternion, sizeof(new_covar_quaternion));

    for (int i = 0; i < 4; i++) {
        if (ekf.quaternion.covar[i][i] > MAX_QUAT_VARIANCE) {
            ekf.quaternion.covar[i][i] = MAX_QUAT_VARIANCE;
        }
        
        // Optional: Symmetric enforcement to prevent drift-induced asymmetry
        for (int j = i + 1; j < 4; j++) {
            float avg = (ekf.quaternion.covar[i][j] + ekf.quaternion.covar[j][i]) * 0.5f;
            ekf.quaternion.covar[i][j] = ekf.quaternion.covar[j][i] = avg;
        }
    }
    // if (ekf_should_log(s_orientation_tick_count)) {
    //     EKF_LOG("ori cov p00=%.4f", ekf.quaternion.covar[0][0]);
    // }
}

/**
 * Body EKF update exploiting H = [I₃, 0₃] sparsity throughout.
 *
 * With H = [I, 0]:
 *   H*P*H' = P[0:3][0:3]  (top-left 3x3 block, zero multiplies)
 *   P*H'   = P[:,0:3]     (first 3 columns of P, zero multiplies)
 *   K      = P[:,0:3] * S⁻¹
 *   P_new  = P - K * P[0:3,:]  (since (I-KH)*P simplifies)
 */
void tick_ekf_body(float deltaTime, float accel[3], float gps_pos[3]) {
    ekf.body.index += 1;
    s_body_tick_count++;

    // if (ekf_should_log(s_body_tick_count)) {
    //     EKF_LOG("body dt=%.4f aZ=%.3f z=%.2f", deltaTime, accel[2], gps_pos[2]);
    // }

    /* --- 1. PREDICTION --- */
    float processing_position[3];
    float processing_velocity[3];
    state_transition_body(&ekf.body, deltaTime, accel,
                          processing_position, processing_velocity);

    float predicted_covar[6][6];
    predict_covar_body_sparse(deltaTime, predicted_covar);

    /* --- 2. UPDATE (Correction) --- */

    /* Innovation: y = z - H*x_pred = gps - position */
    float innovation[3];
    for (int i = 0; i < 3; i++)
        innovation[i] = gps_pos[i] - processing_position[i];

    /* S = H*P*H' + R = P[0:3][0:3] + R   (3x3, zero multiplies) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = predicted_covar[i][j] + ekf.body.measurement[i][j];

    float S_inv[3][3];
    if (!inverse(S, S_inv)) {
        EKF_LOG("body S inv fail");
        return;
    }

    /* K = P*H' * S⁻¹ = P[:,0:3] * S⁻¹   (6x3 * 3x3 = 54 MACs) */
    float K[6][3];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
                sum += predicted_covar[i][k] * S_inv[k][j];
            K[i][j] = sum;
        }

    /* State update: x += K * innovation */
    for (int i = 0; i < 3; i++) {
        float adj = K[i][0]*innovation[0] + K[i][1]*innovation[1] + K[i][2]*innovation[2];
        ekf.body.position[i] = processing_position[i] + adj;
    }
    for (int i = 0; i < 3; i++) {
        float adj = K[i+3][0]*innovation[0] + K[i+3][1]*innovation[1] + K[i+3][2]*innovation[2];
        ekf.body.velocity[i] = processing_velocity[i] + adj;
    }

    // if (ekf_should_log(s_body_tick_count)) {
    //     EKF_LOG("body pz=%.2f vz=%.2f", ekf.body.position[2], ekf.body.velocity[2]);
    // }

    /* Covariance update: P = (I - K*H) * P = P - K * P[0:3,:]
     * Since H=[I,0], K*H has non-zero only in first 3 columns,
     * so (I-KH)*P[i][j] = P[i][j] - sum_{k=0..2} K[i][k]*P[k][j] */
    if (ekf.body.index <= UPDATE_COVAR) {
        return;
    }
    ekf.body.index = 0;

    float new_covar[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) {
            float kp = K[i][0]*predicted_covar[0][j]
                     + K[i][1]*predicted_covar[1][j]
                     + K[i][2]*predicted_covar[2][j];
            new_covar[i][j] = predicted_covar[i][j] - kp;
        }

    memcpy(ekf.body.covar, new_covar, sizeof(new_covar));

    // if (ekf_should_log(s_body_tick_count)) {
    //     EKF_LOG("body cov p00=%.3f", ekf.body.covar[0][0]);
    // }
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
