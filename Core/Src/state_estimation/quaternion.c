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

void get_h_jacobian_quaternion(
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