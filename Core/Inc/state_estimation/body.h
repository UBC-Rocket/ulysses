#ifndef BODY_H
#define BODY_H

#include <ekf.h>

void state_transition_body(
    body_state *state,
    float time_step,
    float a[3] // gyro data
);

void get_state_jacobian_body(
    float a[3],
    float dT,
    float j[6][6]
);

void get_h_jacobian_body(
    float h_jacobian[3][6]
);

#endif