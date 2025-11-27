#ifndef BODY_H
#define BODY_H

#include<ekf.h>

void state_transition_body(
    body_state *state,
    float time_step,
    float g[3] // gyro data
);

void get_state_jacobian_body(
    float g[3],
    float dT,
    float j[4][4]
);

void get_h_jacobian_body(
    float q[4],
    float h_jacobian[3][4]
);

#endif