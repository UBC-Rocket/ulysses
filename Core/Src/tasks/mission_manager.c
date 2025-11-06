#include <stdbool.h>
#include "cmsis_os2.h"
#include "state_exchange.h"
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"

void mission_manager_task_start(void *argument)
{
    flight_state_t flight_state = IDLE;
    state_exchange_publish_flight_state(flight_state);

    for (;;) {
        state_t current_state = {0};
        state_exchange_get_state(&current_state);

        if (flight_state != E_STOP) {
            rotation_matrix_t rotation = {0};
            quaternion_to_rotation_matrix(&current_state.q_bn, &rotation);

            static const float body_up[3] = {0.0f, 0.0f, 1.0f};
            float nav_body_up[3] = {0.0f, 0.0f, 0.0f};
            rotation_matrix_vector_mul(&rotation, body_up, nav_body_up);

            float dot_product = nav_body_up[2];

            if (dot_product < 0.0f) {
                flight_state = E_STOP;
            }
        }

        // TODO: implementet radio parsing

        // TODO: flight state transitions from radio message

        // TODO: Logging

        // TODO: Sd card driver

        // TODO: flash driver

        // TODO: radio logging


        state_exchange_publish_flight_state(flight_state);
        osDelay(10);
    }
}
