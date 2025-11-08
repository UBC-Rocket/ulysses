#include <stdbool.h>
#include "cmsis_os2.h"
#include "state_exchange.h"
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"
#include "SD_logging/log_service.h"

#ifdef DEBUG
#include "debug_uart.h"
#endif

static bool estop_event_logged = false;

void mission_manager_task_start(void *argument)
{
    flight_state_t flight_state = IDLE;
    state_exchange_publish_flight_state(flight_state);

    for (;;) {
        log_service_try_init();

        state_t current_state = {0};
        state_exchange_get_state(&current_state);

        if (flight_state != E_STOP) {
            estop_event_logged = false;
            rotation_matrix_t rotation = {0};
            quaternion_to_rotation_matrix(&current_state.q_bn, &rotation);

            static const float body_up[3] = {0.0f, 0.0f, 1.0f};
            float nav_body_up[3] = {0.0f, 0.0f, 0.0f};
            rotation_matrix_vector_mul(&rotation, body_up, nav_body_up);

            float dot_product = nav_body_up[2];

            if (dot_product < 0.0f) {
                flight_state = E_STOP;
#ifdef DEBUG
                static const char estop_message[] = "MissionManager: E-STOP (orientation)\r\n";
                debug_uart_write((const uint8_t *)estop_message, sizeof(estop_message) - 1U);
#endif
                log_service_log_event(LOG_EVENT_CODE_ESTOP,
                                      (uint16_t)flight_state,
                                      current_state.u_s);
                estop_event_logged = true;
            }
        } else if (!estop_event_logged) {
            log_service_log_event(LOG_EVENT_CODE_ESTOP,
                                  (uint16_t)flight_state,
                                  current_state.u_s);
            estop_event_logged = true;
        }

        // TODO: implementet radio parsing

        // TODO: flight state transitions from radio message

        log_service_log_state(&current_state, flight_state);
        log_service_periodic_flush();

        // TODO: flash driver

        // TODO: radio logging


        state_exchange_publish_flight_state(flight_state);
        osDelay(10);
    }
}
