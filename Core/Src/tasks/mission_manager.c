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
static flight_state_t last_logged_flight_state = IDLE;

void mission_manager_task_start(void *argument)
{
    flight_state_t flight_state = IDLE;
    state_exchange_publish_flight_state(flight_state);

    for (;;) {
        log_service_try_init();

        state_t current_state = {0};
        state_exchange_get_state(&current_state);

        handle_orientation_estop(&current_state, &flight_state);

        // TODO: implementet radio parsing

        // TODO: flight state transitions from radio message

        // TODO: radio logging

        log_service_periodic_flush();

        log_flight_state_if_changed(flight_state, current_state.u_s);

        state_exchange_publish_flight_state(flight_state);
        osDelay(10);
    }
}

static void handle_orientation_estop(const state_t *state, flight_state_t *flight_state)
{
    if (!state || !flight_state) return;

    if (*flight_state != E_STOP) {
        estop_event_logged = false;
        rotation_matrix_t rotation = {0};
        quaternion_to_rotation_matrix(&state->q_bn, &rotation);

        static const float body_up[3] = {0.0f, 0.0f, 1.0f};
        float nav_body_up[3] = {0.0f, 0.0f, 0.0f};
        rotation_matrix_vector_mul(&rotation, body_up, nav_body_up);

        if (nav_body_up[2] < 0.0f) {
            *flight_state = E_STOP;
#ifdef DEBUG
            static const char estop_message[] = "MissionManager: E-STOP (orientation)\r\n";
            debug_uart_write((const uint8_t *)estop_message, sizeof(estop_message) - 1U);
#endif
            log_service_log_event(LOG_EVENT_CODE_ESTOP,
                                  (uint16_t)(*flight_state),
                                  state->u_s);
            estop_event_logged = true;
        }
    } else if (!estop_event_logged) {
        log_service_log_event(LOG_EVENT_CODE_ESTOP,
                              (uint16_t)(*flight_state),
                              state->u_s);
        estop_event_logged = true;
    }
}

static void log_flight_state_if_changed(flight_state_t flight_state, uint32_t timestamp)
{
    if (flight_state == last_logged_flight_state) {
        return;
    }

    log_service_log_event(LOG_EVENT_CODE_FLIGHT_STATE,
                          (uint16_t)flight_state,
                          timestamp);
    last_logged_flight_state = flight_state;
}
