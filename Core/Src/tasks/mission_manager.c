#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "cmsis_os2.h"
#include "main.h"
#include "state_estimation/state.h"
#include "state_exchange.h"
#include <stdbool.h>
// ---------------------Change-------------------------
#include "FreeRTOS.h"
#include "gnss_radio_master.h"
#include "task.h"
// include protobuff file (codec.h from rocket-protocol-lib repo)
// ----------------------------------------------------

static flight_state_t last_logged_flight_state = IDLE;
static bool flight_header_logged = false;
static uint32_t flight_magic = 0U;
static uint32_t flight_counter = 0U;

static void log_flight_header_if_ready(uint32_t timestamp_us);
static void log_flight_state_if_changed(flight_state_t flight_state,
                                        uint32_t timestamp);

// ----------------------Change------------------------
uint32_t flags = 0;
// ----------------------------------------------------

void mission_manager_task_start(void *argument) {
  flight_state_t flight_state = IDLE;
  state_exchange_publish_flight_state(flight_state);

  for (;;) {
    log_service_try_init();

    state_t current_state = {0};
    state_exchange_get_state(&current_state);

    log_flight_header_if_ready(current_state.u_s);

    // TODO: implementet radio parsing
    // ----------------------Change------------------------
    xTaskNotifyWait(0, UINT32_MAX, &flags, 0);
    if (flags & GNSS_RADIO_MSG_READY_FLAG) {
      // Drain all queued radio messages
      uint8_t spi_msg[GNSS_RADIO_MESSAGE_MAX_LEN];

      while (gnss_radio_dequeue(spi_msg)) {
        // Decode packet (COBS + CRC + protobuf)
        // NOTE: packet_size is fixed 256 here
        CodecTestData decoded = CodecTestData_init_zero;

        rp_packet_decode_result_t dec =
            rp_packet_decode(spi_msg, GNSS_RADIO_MESSAGE_MAX_LEN,
                             CodecTestData_fields, &decoded);

        if (dec.status != RP_CODEC_OK) {
          // Optional: log decode failure
          // log_service_log_event(LOG_EVENT_CODE_RADIO_DECODE_FAIL, dec.status,
          // current_state.u_s);
          continue;
        }
      }
    }
    // ----------------------------------------------------

    // TODO: flight state transitions from radio message

    // TODO: radio logging

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