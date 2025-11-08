#include <stdbool.h>
#include "cmsis_os2.h"
#include "state_exchange.h"
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"
#include "SD_logging/log_writer.h"
#include "SD_logging/log_records.h"
#ifdef DEBUG
#include "debug_uart.h"
#endif

extern bool g_sd_card_initialized;

static bool sd_logger_initialized = false;
static bool estop_event_logged = false;
static uint32_t log_flush_counter = 0U;

static void mission_manager_try_init_logger(void);
static void mission_manager_log_state(const state_t *state, flight_state_t flight_state);
static void mission_manager_log_event(uint16_t event_code, uint16_t data, uint32_t timestamp_us);
static void mission_manager_periodic_flush(void);

#define LOG_EVENT_CODE_ESTOP 0x0001U

void mission_manager_task_start(void *argument)
{
    flight_state_t flight_state = IDLE;
    state_exchange_publish_flight_state(flight_state);

    for (;;) {
        mission_manager_try_init_logger();

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
                mission_manager_log_event(LOG_EVENT_CODE_ESTOP,
                                          (uint16_t)flight_state,
                                          current_state.u_s);
                estop_event_logged = true;
            }
        } else if (!estop_event_logged) {
            mission_manager_log_event(LOG_EVENT_CODE_ESTOP,
                                      (uint16_t)flight_state,
                                      current_state.u_s);
            estop_event_logged = true;
        }

        // TODO: implementet radio parsing

        // TODO: flight state transitions from radio message

        mission_manager_log_state(&current_state, flight_state);
        mission_manager_periodic_flush();

        // TODO: Sd card driver

        // TODO: flash driver

        // TODO: radio logging


        state_exchange_publish_flight_state(flight_state);
        osDelay(10);
    }
}

static void mission_manager_try_init_logger(void)
{
    if (sd_logger_initialized || !g_sd_card_initialized) {
        return;
    }

    if (log_writer_init()) {
        sd_logger_initialized = true;
    }
}

static void mission_manager_log_state(const state_t *state, flight_state_t flight_state)
{
    if (!sd_logger_initialized || !log_writer_ready() || state == NULL) {
        return;
    }

    log_record_state_snapshot_t snapshot = {
        .timestamp_us = state->u_s,
        .q_w = state->q_bn.w,
        .q_x = state->q_bn.x,
        .q_y = state->q_bn.y,
        .q_z = state->q_bn.z,
        .altitude_m = state->pos[2],
        .vel_n_mps = state->vel[0],
        .vel_e_mps = state->vel[1],
        .vel_d_mps = state->vel[2],
        .flight_state = (uint8_t)flight_state,
        .estop_active = (uint8_t)(flight_state == E_STOP),
        .reserved = 0U
    };

    log_writer_append_record(LOG_RECORD_TYPE_state_snapshot,
                             &snapshot,
                             sizeof(snapshot));
}

static void mission_manager_log_event(uint16_t event_code, uint16_t data, uint32_t timestamp_us)
{
    if (!sd_logger_initialized || !log_writer_ready()) {
        return;
    }

    log_record_event_t event = {
        .timestamp_us = timestamp_us,
        .event_code = event_code,
        .data_u16 = data
    };

    log_writer_append_record(LOG_RECORD_TYPE_event,
                             &event,
                             sizeof(event));
}

static void mission_manager_periodic_flush(void)
{
    if (!sd_logger_initialized || !log_writer_ready()) {
        return;
    }

    log_flush_counter++;
    if (log_flush_counter >= 100U) {
        log_writer_flush();
        log_flush_counter = 0U;
    }
}
