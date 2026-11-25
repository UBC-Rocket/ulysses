#include "ulysses/lib/dynamixel/protocol2_parameters.h"

#include <string.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/buffer.h"

// Instructions: https://emanual.robotis.com/docs/en/dxl/protocol2/#instruction-details

#define CRC_LENGTH (2)

enum uls_status dynamixel_protocol2_serialize_fast_sync_read_request(
    const struct dynamixel_protocol2_fast_sync_read_request *request, uint8_t *restrict parameters,
    uint16_t capacity, uint16_t *parameters_length)
{
    ULYSSES_ASSERT(request != NULL, "fast sync read request must not be NULL");
    ULYSSES_ASSERT(request->ids != NULL && request->id_count > 0,
                   "fast sync read request must include device IDs");
    ULYSSES_ASSERT(request->length > 0, "fast sync read length must be nonzero");
    ULYSSES_ASSERT(parameters != NULL, "parameters output must not be NULL");
    ULYSSES_ASSERT(parameters_length != NULL, "parameters length output must not be NULL");

    uint16_t length = 4 + request->id_count;
    if (capacity < length) {
        return ULS_STATUS_NO_SPACE;
    }

    buffer_write_uint16_le(parameters, request->address);
    buffer_write_uint16_le(&parameters[2], request->length);
    memcpy(&parameters[4], request->ids, request->id_count);
    *parameters_length = length;

    return ULS_STATUS_OK;
}

enum uls_status dynamixel_protocol2_serialize_sync_write_request(
    const struct dynamixel_protocol2_sync_write_request *request, uint8_t *restrict parameters,
    uint16_t capacity, uint16_t *parameters_length)
{
    ULYSSES_ASSERT(request != NULL, "sync write request must not be NULL");
    ULYSSES_ASSERT(request->devices != NULL && request->device_count > 0,
                   "sync write request must include devices");
    ULYSSES_ASSERT(request->length > 0, "sync write length must be nonzero");
    ULYSSES_ASSERT(parameters != NULL, "parameters output must not be NULL");
    ULYSSES_ASSERT(parameters_length != NULL, "parameters length output must not be NULL");

    if (capacity < 4 + request->device_count * (1 + request->length)) {
        return ULS_STATUS_NO_SPACE;
    }

    buffer_write_uint16_le(parameters, request->address);
    buffer_write_uint16_le(&parameters[2], request->length);
    uint16_t offset = 4;
    for (uint8_t i = 0; i < request->device_count; i++) {
        const struct dynamixel_protocol2_sync_write_device *device = &request->devices[i];

        ULYSSES_ASSERT(device->data != NULL, "sync write device data must not be NULL");
        parameters[offset++] = device->id;
        memcpy(&parameters[offset], device->data, request->length);
        offset += request->length;
    }
    *parameters_length = offset;

    return ULS_STATUS_OK;
}

enum uls_status
dynamixel_protocol2_deserialize_ping_response(const uint8_t *parameters, uint16_t parameters_length,
                                              struct dynamixel_protocol2_ping_response *response)
{
    ULYSSES_ASSERT(response != NULL, "ping response output must not be NULL");

    if (parameters_length != 4) {
        return ULS_STATUS_INVALID_INPUT;
    }

    response->error = parameters[0];
    response->model = buffer_read_uint16_le(&parameters[1]);
    response->firmware = parameters[3];

    return ULS_STATUS_OK;
}

enum uls_status dynamixel_protocol2_deserialize_fast_sync_read_response(
    const uint8_t *parameters, uint16_t parameters_length, uint16_t data_length,
    struct dynamixel_protocol2_fast_sync_read_response *response, uint16_t *consumed)
{
    ULYSSES_ASSERT(response != NULL, "fast sync read response output must not be NULL");
    ULYSSES_ASSERT(consumed != NULL, "bytes consumed output must not be NULL");

    // The packet decoder strips the final CRC, so the last device entry is two bytes shorter.
    if ((parameters_length + CRC_LENGTH) % (2 + data_length + CRC_LENGTH) != 0) {
        return ULS_STATUS_INVALID_INPUT;
    }

    uint16_t entry_length = 2 + data_length;
    response->error = parameters[0];
    response->id = parameters[1];
    response->data = &parameters[2];
    *consumed = entry_length + (parameters_length == entry_length ? 0 : CRC_LENGTH);

    return ULS_STATUS_OK;
}
