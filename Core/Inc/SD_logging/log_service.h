#ifndef LOG_SERVICE_H
#define LOG_SERVICE_H

#include "mission_manager/mission_manager.h"
#include "state_estimation/state.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Shared event identifiers for log_service_log_event.
 */
#define LOG_EVENT_CODE_ESTOP 0x0001U
#define LOG_EVENT_CODE_FLIGHT_STATE 0x0002U
#define LOG_EVENT_CODE_RADIO_RX 0x0003U

/**
 * @brief Perform one-time initialisation of the SD log writer if a card is
 * present.
 */
void log_service_try_init(void);

/**
 * @brief Returns true if the logging backend is ready to accept records.
 */
bool log_service_ready(void);

/**
 * @brief Append the latest navigation state snapshot to the log.
 *
 * Safe to call from the mission manager loop; function is a no-op if logging is
 * disabled.
 */
void log_service_log_state(const state_t *state, flight_state_t flight_state);

void log_service_log_flight_header(uint32_t timestamp_us, uint32_t flight_magic,
                                   uint32_t flight_counter);

void log_service_log_accel_sample(uint32_t timestamp_us, float ax_mps2,
                                  float ay_mps2, float az_mps2);

void log_service_log_gyro_sample(uint32_t timestamp_us, float gx_rad_s,
                                 float gy_rad_s, float gz_rad_s);

void log_service_log_baro_sample(uint32_t timestamp_us, int32_t temp_centi,
                                 int32_t pressure_centi, uint32_t seq);

void log_service_log_baro2_sample(uint32_t timestamp_us, int32_t temp_centi,
                                  int32_t pressure_centi, uint32_t seq);

/**
 * @brief Append an event record to the log (e.g., estop, state changes).
 */
void log_service_log_event(uint16_t event_code, uint16_t data,
                           uint32_t timestamp_us);

/**
 * @brief Periodic flush helper to limit data loss on power failure.
 */
void log_service_periodic_flush(void);

void log_service_log_radio_telemetry(uint32_t timestamp_ms,
                                     float ang_rate_x_rps, float ang_rate_y_rps,
                                     float ang_rate_z_rps, float thrust_cmd,
                                     float gimbal_x_rad, float gimbal_y_rad);

void log_service_log_status(uint32_t timestamp_ms, uint32_t uptime_ms,
                            uint8_t flight_state, bool accel_ok, bool gyro_ok,
                            bool baro1_ok, bool baro2_ok, bool gps_connected,
                            uint32_t radio_tx_count, uint32_t radio_rx_count,
                            uint32_t cmd_rx_count);

void log_service_log_controls_output(uint32_t timestamp_us, float T_cmd,
                                     float theta_x_cmd, float theta_y_cmd,
                                     float esc_thrust_0, float esc_thrust_1,
                                     float servo_x_deg, float servo_y_deg,
                                     uint8_t flight_state, uint8_t flags);

#endif /* LOG_SERVICE_H */
