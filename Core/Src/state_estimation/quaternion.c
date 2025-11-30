#include "state.h"
#include <ekf.h>
#include <math.h>
#include <string.h>
#include <matrix.h>

void state_transition_orientation(
    quaternion_state *state,
    float time_step,
    float g[3], // gyro data
    float out_q[4]
)
{
    float P[4][1];
    for (int i = 0; i < 4; i++) P[i][0] = state->vals[i];

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
        out_q[i] = state->vals[i] + C[i][0];
    }

    normalize(out_q);
}

void get_state_jacobian_orientation(
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

void predict_accel_from_quat(const float q[4], float accel_pred[3], float expected_g[3])
{
    rotation_matrix_t r;
    quaternion_t quat;
    quat.w = q[0]; quat.x = q[1]; quat.y = q[2]; quat.z = q[3];

    quaternion_to_rotation_matrix(&quat, &r);

    for (int i = 0; i < 3; i++) accel_pred[i] = 0;

    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) accel_pred[i] += r.R[j][i] * expected_g[j];
}

void get_h_jacobian_quaternion(float q[4], float eg[3], float H[3][4]) {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
    
    float gx = eg[0];
    float gy = eg[1];
    float gz = eg[2];

    // Derivatives of R^T * g
    // Where R^T is the rotation from World to Body
    
    // Row 0 (Gradient of a_x)
    H[0][0] = 2.0f * (w * gx + z * gy - y * gz); // d/dw
    H[0][1] = 2.0f * (x * gx + y * gy + z * gz); // d/dx
    H[0][2] = 2.0f * (-y * gx + x * gy - w * gz); // d/dy
    H[0][3] = 2.0f * (-z * gx + w * gy + x * gz); // d/dz

    // Row 1 (Gradient of a_y)
    H[1][0] = 2.0f * (-z * gx + w * gy + x * gz); // d/dw
    H[1][1] = 2.0f * (y * gx - x * gy + w * gz);  // d/dx
    H[1][2] = 2.0f * (x * gx + y * gy + z * gz);  // d/dy
    H[1][3] = 2.0f * (-w * gx - z * gy + y * gz); // d/dz

    // Row 2 (Gradient of a_z)
    H[2][0] = 2.0f * (y * gx - x * gy + w * gz);  // d/dw
    H[2][1] = 2.0f * (z * gx - w * gy + x * gz);  // d/dx
    H[2][2] = 2.0f * (w * gx + z * gy - y * gz);  // d/dy
    H[2][3] = 2.0f * (x * gx + y * gy + z * gz);  // d/dz
}