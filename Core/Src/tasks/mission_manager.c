#include <stdbool.h>
#include "cmsis_os2.h"
#include "state_exchange.h"
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"
#include "SD_logging/log_service.h"
#include "main.h"


static flight_state_t last_logged_flight_state = IDLE;
static bool flight_header_logged = false;
static uint32_t flight_magic = 0U;
static uint32_t flight_counter = 0U;

static void log_flight_header_if_ready(uint32_t timestamp_us);
static void log_flight_state_if_changed(flight_state_t flight_state, uint32_t timestamp);

void mission_manager_task_start(void *argument)
{
    flight_state_t flight_state = IDLE;
    state_exchange_publish_flight_state(flight_state);

    for (;;) {
        log_service_try_init();

        state_t current_state = {0};
        state_exchange_get_state(&current_state);

        log_flight_header_if_ready(current_state.u_s);

        // TODO: implementet radio parsing

        // TODO: flight state transitions from radio message

        // TODO: radio logging

        log_service_periodic_flush();

        log_flight_state_if_changed(flight_state, current_state.u_s);

        state_exchange_publish_flight_state(flight_state);
        osDelay(2);
    }
}

static uint32_t next_flight_magic(void)
{
    uint32_t seed = HAL_GetTick() ^ 0x5A5AA5A5U;
    if (seed == 0U) {
        seed = 0xA5A5A5A5U;
    }
    return seed;
}

static void log_flight_header_if_ready(uint32_t timestamp_us)
{
    if (flight_header_logged || !log_service_ready()) {
        return;
    }

    if (flight_magic == 0U) {
        flight_magic = next_flight_magic();
        flight_counter = HAL_GetTick();
    }

    log_service_log_flight_header(timestamp_us,
                                  flight_magic,
                                  flight_counter);
    flight_header_logged = true;
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
