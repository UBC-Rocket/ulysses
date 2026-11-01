#include "ulysses/lib/eskf/eskf.h"

// current architecture has the firmware maintaining an eskf_t state and passing it in along with data
// we could use a static state and just have the firmware passing in sensor data since we're only running
// one eskf, will see 

void eskf_init(eskf_t *filter, const eskf_config_t *config) {
    null_state(&filter->state);
    copy_config(&filter->config, config);
}

// helpers
void copy_config(eskf_config_t *dest, eskf_config_t *src) {
    for (int i = 0; i < 3; i++) dest->gravity[i] = src->gravity[i];

    dest->accelerometer_noise_density = src->accelerometer_noise_density;
    dest->gyroscope_noise_density = src->gyroscope_noise_density;
    dest->accelerometer_bias_random_walk = src->accelerometer_bias_random_walk;
    dest->gyroscope_bias_random_walk = src->gyroscope_bias_random_walk;

    for (int i = 0; i < ESKF_ERROR_STATE_SIZE; i++) {
        dest->initial_variance[i] = dest->initial_variance[i];
    }
}

void null_state(eskf_state_t *state) {
    for (int i = 0; i < 3; i++) {
        state->position[i] = 0;
        state->velocity[i] = 0;
        state->accelerometer_bias[i] = 0;
        state->gyroscope_bias[i] = 0;
        state->orientation[i + 1] = 0;
    }

    state->orientation[0] = 1;
}

void eskf_get_state(const eskf_t *filter, eskf_state_t *state) {
    for (int i = 0; i < 3; i++) {
        state->position[i] = filter->state.position[i];
        state->velocity[i] = filter->state.velocity[i];
        state->accelerometer_bias[i] = filter->state.accelerometer_bias[i];
        state->gyroscope_bias[i] = filter->state.gyroscope_bias[i];
    }

    for (int i = 0; i < 4; i++) {
        state->orientation[i] = filter->state.orientation[i];
    }
}

void eskf_reset(eskf_t *filter) {
    null_state(&filter->state);
}
