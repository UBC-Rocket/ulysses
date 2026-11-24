#ifndef ULYSSES_LIB_DYNAMIXEL_PROTOCOL2_INSTRUCTIONS
#define ULYSSES_LIB_DYNAMIXEL_PROTOCOL2_INSTRUCTIONS

#include <stdint.h>

#include "ulysses/status.h"

struct dynamixel_protocol2_fast_sync_read_request {
    uint16_t address;
    uint16_t length;
    uint8_t id_count;
    const uint8_t *ids;
};

/**
 * @brief Serialize Fast Sync Read parameters for dynamixel_protocol2_encode_request().
 * @param request At least one device ID and a nonzero per-device read length
 * @param[out] parameters Serialized parameters
 * @param capacity Output capacity in bytes
 * @param[out] parameters_length Bytes written on success
 * @retval ULS_STATUS_OK Parameters written
 * @retval ULS_STATUS_NO_SPACE Output buffer is too small. Both outputs stay untouched.
 */
enum uls_status dynamixel_protocol2_serialize_fast_sync_read_request(
    const struct dynamixel_protocol2_fast_sync_read_request *request, uint8_t *restrict parameters,
    uint16_t capacity, uint16_t *parameters_length);

struct dynamixel_protocol2_sync_write_device {
    uint8_t id;
    const uint8_t *data;
};

struct dynamixel_protocol2_sync_write_request {
    uint16_t address;
    uint16_t length;
    uint8_t device_count;
    const struct dynamixel_protocol2_sync_write_device *devices;
};

/**
 * @brief Serialize Sync Write parameters for dynamixel_protocol2_encode_request().
 * @param request At least one device, all with the same nonzero number of data bytes
 * @param[out] parameters Serialized parameters
 * @param capacity Output capacity in bytes
 * @param[out] parameters_length Bytes written on success
 * @retval ULS_STATUS_OK Parameters written
 * @retval ULS_STATUS_NO_SPACE Output buffer is too small. Both outputs stay untouched.
 */
enum uls_status dynamixel_protocol2_serialize_sync_write_request(
    const struct dynamixel_protocol2_sync_write_request *request, uint8_t *restrict parameters,
    uint16_t capacity, uint16_t *parameters_length);

struct dynamixel_protocol2_ping_response {
    uint8_t error;
    uint16_t model;
    uint8_t firmware;
};

/**
 * @brief Deserialize a Ping reply.
 * @param parameters Payload from dynamixel_protocol2_decode_response()
 * @param parameters_length Number of decoded bytes
 * @param[out] response Filled on success
 * @retval ULS_STATUS_OK Reply deserialized
 * @retval ULS_STATUS_INVALID_INPUT Reply length is incorrect. The response stays untouched.
 */
enum uls_status
dynamixel_protocol2_deserialize_ping_response(const uint8_t *parameters, uint16_t parameters_length,
                                              struct dynamixel_protocol2_ping_response *response);

struct dynamixel_protocol2_fast_sync_read_response {
    uint8_t error;
    uint8_t id;
    const uint8_t *data;
};

/**
 * @brief Deserialize the next device from a combined Fast Sync Read reply.
 * @param parameters Remaining decoded payload from a reply with outer ID 0xFE
 * @param parameters_length Number of remaining decoded bytes
 * @param data_length Per-device data length from the serialized Fast Sync Read request
 * @param[out] response Filled on success. Its data points into the decoded buffer.
 * @param[out] bytes_consumed Bytes to advance on success, including any intermediate CRC
 * @retval ULS_STATUS_OK Device entry deserialized
 * @retval ULS_STATUS_INVALID_INPUT Remaining length does not match the entry layout. Both outputs
 *                                 stay untouched.
 */
enum uls_status dynamixel_protocol2_deserialize_fast_sync_read_response(
    const uint8_t *parameters, uint16_t parameters_length, uint16_t data_length,
    struct dynamixel_protocol2_fast_sync_read_response *response, uint16_t *bytes_consumed);

#endif // ULYSSES_LIB_DYNAMIXEL_PROTOCOL2_INSTRUCTIONS
