#ifndef ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER
#define ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER

#include "ulysses/lib/controls/flight_config.h"
#include "ulysses/lib/eskf/eskf.h"

#include "ulysses/lib/math/quaternion.h"

typedef struct fc_ref {
    quaternion_t attitude_ref;
    float pos_ref[3];
    float vel_ref[3];
} fc_ref_t;

typedef struct fc_command {
    float theta_x_cmd;
    float theta_y_cmd;
    float thrust_top;
    float thrust_bottom;
} fc_command_t;

void flight_controller_init(const fc_config_t *config);
void flight_controller_step(const eskf_state_t *state, const fc_ref_t *ref, fc_command_t *cmd_out, float dt_s);

#endif /* ULYSSES_LIB_CONTROLS_FLIGHT_CONTROLLER */
