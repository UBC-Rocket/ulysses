#ifndef BODY_H
#define BODY_H

#include <ekf.h>


void transform_accel_data(
    float a[3], // raw_data
    float q[4], // orientation in quat
    float new_a[3]
);

void state_transition_body(
    body_state *state,
    float time_step,
    float a[3], // gyro data
    float out_p[3],
    float out_v[3]
);

void get_state_jacobian_body(
    float dT,
    float j[6][6]
);

void get_h_jacobian_body(
    float h_jacobian[3][6]
);

#endif