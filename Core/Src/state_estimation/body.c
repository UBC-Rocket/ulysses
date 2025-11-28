#include <matrix.h>
#include <ekf.h>
#include <string.h>

void state_transition_body(
    body_state *state,
    float time_step,
    float a[3] // accel data
)
{
    for (int i = 0; i < 3; i++) {
        state->position[i] += + time_step * state->velocity[i]
                              + 0.5 * time_step * time_step * a[i];
    }

    for (int i = 0; i < 3; i++) {
        state->velocity[i] += time_step * a[i];
    }
}


void get_state_jacobian_body(
    float dT,
    float j[6][6]
)
{
    memcpy(j,
        (float[6][6]) {
            {1, 0, 0, dT, 0, 0},
            {0, 1, 0, 0, dT, 0},
            {0, 0, 1, 0, 0, dT},
            {0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0,1, 0},
            {0, 0, 0, 0, 0, 1}
        },
        (sizeof (float[6][6])));
}

void get_h_jacobian_body(
    float h_jacobian[3][6]
)
{
    memcpy(h_jacobian,
    (float[6][6]) {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},
    },
    (sizeof (float[6][6])));
}