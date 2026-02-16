#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "cmsis_os2.h"
#include "main.h"
#include "state_estimation/state.h"
#include "state_exchange.h"
#include <stdbool.h>

#include "FreeRTOS.h"
#include "flight_command.pb.h"
#include "gnss_radio_master.h"
#include "rp/codec.h"
#include "task.h"
#include <stdint.h>

static flight_state_t last_logged_flight_state = IDLE;
static bool flight_header_logged = false;
static uint32_t flight_magic = 0U;
static uint32_t flight_counter = 0U;

static void log_flight_header_if_ready(uint32_t timestamp_us);
static void log_flight_state_if_changed(flight_state_t flight_state,
                                        uint32_t timestamp);

uint32_t flags = 0;

void mission_manager_task_start(void *argument) {
    flight_state_t flight_state = IDLE;
    state_exchange_publish_flight_state(flight_state);

    for (;;) {
        log_service_try_init();

        state_t current_state = {0};
        state_exchange_get_state(&current_state);

        log_flight_header_if_ready(current_state.u_s);

        xTaskNotifyWait(0, UINT32_MAX, &flags, 0);
        
        if (flags & GNSS_RADIO_MSG_READY_FLAG) {
            uint8_t spi_msg[GNSS_RADIO_MESSAGE_MAX_LEN];

            while (gnss_radio_dequeue(spi_msg)) {
                FlightCommand decoded = FlightCommand_init_zero;

                rp_packet_decode_result_t dec = rp_packet_decode(
                    spi_msg, 
                    GNSS_RADIO_MESSAGE_MAX_LEN,
                    FlightCommand_fields,  
                    &decoded
                );
                
                if (dec.status == RP_CODEC_OK) {
                    switch (decoded.type) {
                        case FlightCommand_CommandType_COMMAND_LAUNCH:
                            if (flight_state == IDLE) {
                                flight_state = RISE;
                            }
                            break;
                            
                        case FlightCommand_CommandType_COMMAND_ABORT:
                            flight_state = E_STOP;
                            break;
                            
                        case FlightCommand_CommandType_COMMAND_NONE:
                        default:
                            break;
                    }
                    
                    log_service_log_event(
                        LOG_EVENT_CODE_RADIO_RX,
                        (uint16_t)decoded.type,
                        current_state.u_s
                    );
                    
                } else if (dec.status == RP_CODEC_CHECKSUM_MISMATCH) {
                    log_service_log_event(
                        LOG_EVENT_CODE_RADIO_RX,
                        0xFFFF,
                        current_state.u_s
                    );
                }
            }
        }

        log_service_periodic_flush();
        log_flight_state_if_changed(flight_state, current_state.u_s);
        state_exchange_publish_flight_state(flight_state);
        
        osDelay(2);
    }
}

static uint32_t next_flight_magic(void) {
    uint32_t seed = HAL_GetTick() ^ 0x5A5AA5A5U;
    if (seed == 0U) {
        seed = 0xA5A5A5A5U;
    }
    return seed;
}

static void log_flight_header_if_ready(uint32_t timestamp_us) {
    if (flight_header_logged || !log_service_ready()) {
        return;
    }

    if (flight_magic == 0U) {
        flight_magic = next_flight_magic();
        flight_counter = HAL_GetTick();
    }

    log_service_log_flight_header(timestamp_us, flight_magic, flight_counter);
    flight_header_logged = true;
}

static void log_flight_state_if_changed(flight_state_t flight_state,
                                        uint32_t timestamp) {
    if (flight_state == last_logged_flight_state) {
        return;
    }

    log_service_log_event(LOG_EVENT_CODE_FLIGHT_STATE, (uint16_t)flight_state,
                          timestamp);
    last_logged_flight_state = flight_state;
}