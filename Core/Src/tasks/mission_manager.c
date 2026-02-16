#include <stdbool.h>
#include <string.h>
#include "printf/printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "state_exchange.h"
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"
#include "SD_logging/log_service.h"
#include "spi_drivers/gnss_radio_master.h"
#include "debug/log.h"
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

    const TickType_t timeout = pdMS_TO_TICKS(100);

    /* 1Hz dummy radio TX sender */
    uint32_t dummy_seq = 0;
    TickType_t last_dummy_tick = xTaskGetTickCount();

    /* Periodic GNSS stats */
    TickType_t last_stats_tick = xTaskGetTickCount();

    DLOG_PRINT("[MM] Task started\r\n");

    for (;;) {
        uint32_t flags = 0;
        xTaskNotifyWaitIndexed(0, 0, UINT32_MAX, &flags, timeout);

        log_service_try_init();

        if (flags & GNSS_RADIO_MSG_READY_FLAG) {
            uint8_t radio_msg[GNSS_RADIO_MESSAGE_MAX_LEN];
            while (gnss_radio_dequeue(radio_msg)) {
                /* Log first 16 bytes as hex */
                DLOG_PRINT("[RADIO RX] %02X%02X%02X%02X...\r\n",
                           radio_msg[0], radio_msg[1],
                           radio_msg[2], radio_msg[3]);
                // TODO: decode radio message
                // TODO: flight state transitions from radio message
            }
        }

        /* ── 1Hz Dummy Radio TX ── */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_dummy_tick) >= pdMS_TO_TICKS(1000)) {
            last_dummy_tick = now;

            char dummy_buf[48];
            int len = snprintf_(dummy_buf, sizeof(dummy_buf),
                               "ULYSSES TX #%lu t=%lu",
                               (unsigned long)dummy_seq,
                               (unsigned long)HAL_GetTick());
            if (len < 0) len = 0;
            dummy_seq++;

            bool sent = gnss_radio_send((const uint8_t *)dummy_buf,
                                        (uint16_t)len);
            DLOG_PRINT("[RADIO TX] #%lu %s\r\n",
                       (unsigned long)(dummy_seq - 1),
                       sent ? "OK" : "FAIL");
        }

        /* ── Periodic GNSS Stats (every 5s) ── */
        if ((now - last_stats_tick) >= pdMS_TO_TICKS(5000)) {
            last_stats_tick = now;
            uint32_t gps_cnt, radio_cnt, err_cnt;
            gnss_radio_get_stats(&gps_cnt, &radio_cnt, &err_cnt);
            DLOG_PRINT("[GNSS] g=%lu r=%lu e=%lu\r\n",
                       (unsigned long)gps_cnt,
                       (unsigned long)radio_cnt,
                       (unsigned long)err_cnt);
        }

        state_t current_state = {0};
        state_exchange_get_state(&current_state);

        log_flight_header_if_ready(current_state.u_s);
        log_service_periodic_flush();
        log_flight_state_if_changed(flight_state, current_state.u_s);
        state_exchange_publish_flight_state(flight_state);
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
