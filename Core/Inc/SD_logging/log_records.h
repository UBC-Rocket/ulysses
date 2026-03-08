#ifndef LOG_RECORDS_H
#define LOG_RECORDS_H

#include <stdint.h>

#define LOG_SCHEMA_VERSION 2U
#define LOG_RECORD_MAGIC 0xA5U

typedef enum {
  LOG_FRAME_FLAG_NONE = 0x00,
} log_frame_flags_t;

typedef struct __attribute__((packed)) {
  uint8_t magic;     ///< Always LOG_RECORD_MAGIC
  uint8_t type;      ///< log_record_type_t
  uint16_t length;   ///< Payload size in bytes (does not include header or CRC)
  uint16_t crc16;    ///< CRC-16 over header (magic/type/length) + payload
  uint16_t reserved; ///< Padding to keep header aligned on 32-bit boundary
} log_record_frame_t;

#define LOG_RECORD_FIELDS_FLIGHT_HEADER(FIELD)                                 \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(uint32_t, flight_magic)                                                \
  FIELD(uint32_t, flight_counter)

#define LOG_RECORD_FIELDS_ACCEL_SAMPLE(FIELD)                                  \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(float, ax_mps2)                                                        \
  FIELD(float, ay_mps2)                                                        \
  FIELD(float, az_mps2)

#define LOG_RECORD_FIELDS_GYRO_SAMPLE(FIELD)                                   \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(float, gx_rad_s)                                                       \
  FIELD(float, gy_rad_s)                                                       \
  FIELD(float, gz_rad_s)

#define LOG_RECORD_FIELDS_BARO_SAMPLE(FIELD)                                   \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(int32_t, temp_centi)                                                   \
  FIELD(int32_t, pressure_centi)                                               \
  FIELD(uint32_t, seq)

#define LOG_RECORD_FIELDS_BARO2_SAMPLE(FIELD)                                  \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(int32_t, temp_centi)                                                   \
  FIELD(int32_t, pressure_centi)                                               \
  FIELD(uint32_t, seq)

#define LOG_RECORD_FIELDS_STATE_SNAPSHOT(FIELD)                                \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(float, q_w)                                                            \
  FIELD(float, q_x)                                                            \
  FIELD(float, q_y)                                                            \
  FIELD(float, q_z)                                                            \
  FIELD(float, altitude_m)                                                     \
  FIELD(float, vel_n_mps)                                                      \
  FIELD(float, vel_e_mps)                                                      \
  FIELD(float, vel_d_mps)                                                      \
  FIELD(uint8_t, flight_state)                                                 \
  FIELD(uint8_t, estop_active)                                                 \
  FIELD(uint16_t, reserved)

#define LOG_RECORD_FIELDS_EVENT(FIELD)                                         \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(uint16_t, event_code)                                                  \
  FIELD(uint16_t, data_u16)

#define LOG_RECORD_FIELDS_RADIO_TELEMETRY(FIELD)                               \
  FIELD(uint32_t, timestamp_ms)                                                \
  FIELD(float, ang_rate_x_rps)                                                 \
  FIELD(float, ang_rate_y_rps)                                                 \
  FIELD(float, ang_rate_z_rps)                                                 \
  FIELD(float, thrust_cmd)                                                     \
  FIELD(float, gimbal_x_rad)                                                   \
  FIELD(float, gimbal_y_rad)

#define LOG_RECORD_FIELDS_STATUS(FIELD)                                        \
  FIELD(uint32_t, timestamp_ms)                                                \
  FIELD(uint32_t, uptime_ms)                                                   \
  FIELD(uint8_t, flight_state)                                                 \
  FIELD(uint8_t, accel_ok)                                                     \
  FIELD(uint8_t, gyro_ok)                                                      \
  FIELD(uint8_t, baro1_ok)                                                     \
  FIELD(uint8_t, baro2_ok)                                                     \
  FIELD(uint8_t, gps_connected)                                                \
  FIELD(uint16_t, reserved)                                                    \
  FIELD(uint32_t, radio_tx_count)                                              \
  FIELD(uint32_t, radio_rx_count)                                              \
  FIELD(uint32_t, cmd_rx_count)

#define LOG_RECORD_FIELDS_CONTROLS_OUTPUT(FIELD)                               \
  FIELD(uint32_t, timestamp_us)                                                \
  FIELD(float, T_cmd)                                                          \
  FIELD(float, theta_x_cmd)                                                    \
  FIELD(float, theta_y_cmd)                                                    \
  FIELD(float, esc_thrust_0)                                                   \
  FIELD(float, esc_thrust_1)                                                   \
  FIELD(float, servo_x_deg)                                                    \
  FIELD(float, servo_y_deg)                                                    \
  FIELD(uint8_t, flight_state)                                                 \
  FIELD(uint8_t, flags)                                                        \
  FIELD(uint16_t, reserved)

#define LOG_RECORD_LIST(APP)                                                   \
  APP(0x10, flight_header, LOG_RECORD_FIELDS_FLIGHT_HEADER)                    \
  APP(0x01, accel_sample, LOG_RECORD_FIELDS_ACCEL_SAMPLE)                      \
  APP(0x02, gyro_sample, LOG_RECORD_FIELDS_GYRO_SAMPLE)                        \
  APP(0x03, baro_sample, LOG_RECORD_FIELDS_BARO_SAMPLE)                        \
  APP(0x04, state_snapshot, LOG_RECORD_FIELDS_STATE_SNAPSHOT)                  \
  APP(0x05, event, LOG_RECORD_FIELDS_EVENT)                                    \
  APP(0x06, baro2_sample, LOG_RECORD_FIELDS_BARO2_SAMPLE)                      \
  APP(0x07, radio_telemetry, LOG_RECORD_FIELDS_RADIO_TELEMETRY)                \
  APP(0x08, status, LOG_RECORD_FIELDS_STATUS)                                  \
  APP(0x09, controls_output, LOG_RECORD_FIELDS_CONTROLS_OUTPUT)

#define DECLARE_ENUM(id, name, fields) LOG_RECORD_TYPE_##name = id,
typedef enum { LOG_RECORD_LIST(DECLARE_ENUM) } log_record_type_t;
#undef DECLARE_ENUM

#define DECLARE_FIELD(type, name) type name;
#define DECLARE_STRUCT(id, name, fields)                                       \
  typedef struct __attribute__((packed)) {                                     \
    fields(DECLARE_FIELD)                                                      \
  } log_record_##name##_t;
LOG_RECORD_LIST(DECLARE_STRUCT)
#undef DECLARE_STRUCT
#undef DECLARE_FIELD

#define DECLARE_SIZE(id, name, fields)                                         \
  enum { LOG_RECORD_##name##_SIZE = sizeof(log_record_##name##_t) };
LOG_RECORD_LIST(DECLARE_SIZE)
#undef DECLARE_SIZE

#endif /* LOG_RECORDS_H */
