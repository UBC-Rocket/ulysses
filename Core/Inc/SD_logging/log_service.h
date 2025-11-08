#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"

/**
 * @brief Shared event identifiers for log_service_log_event.
 */
#define LOG_EVENT_CODE_ESTOP 0x0001U

/**
 * @brief Perform one-time initialisation of the SD log writer if a card is present.
 */
void log_service_try_init(void);

/**
 * @brief Returns true if the logging backend is ready to accept records.
 */
bool log_service_ready(void);

/**
 * @brief Append the latest navigation state snapshot to the log.
 *
 * Safe to call from the mission manager loop; function is a no-op if logging is disabled.
 */
void log_service_log_state(const state_t *state, flight_state_t flight_state);

/**
 * @brief Append an event record to the log (e.g., estop, state changes).
 */
void log_service_log_event(uint16_t event_code, uint16_t data, uint32_t timestamp_us);

/**
 * @brief Periodic flush helper to limit data loss on power failure.
 */
void log_service_periodic_flush(void);

#endif /* LOG_SERVICE_H */
