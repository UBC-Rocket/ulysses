#include <string.h>

#include "ulysses/lib/buffer.h"
#include "ulysses/lib/dynamixel/protocol2/packet.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#include "dev/servo/dynamixel/servo_dynamixel.h"
#include "dev/servo/dynamixel/servo_dynamixel_receive_status.h"

#define PRESENT_POSITION_ADDRESS (0x84)

static uint16_t prepare_transmit(const dev_servo_t *servo)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    const struct dynamixel_protocol2_fast_sync_read_request request = {
        .address = PRESENT_POSITION_ADDRESS,
        .length = 4,
        .id_count = config->count,
        .ids = config->ids,
    };

    uint16_t parameters_length;
    dynamixel_protocol2_serialize_fast_sync_read_request(
        &request, data->parameters, config->parameters_capacity, &parameters_length);

    uint32_t packet_size;
    dynamixel_protocol2_encode_request(
        DYNAMIXEL_PROTOCOL2_BROADCAST_ID, DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ, data->parameters,
        parameters_length, data->packet, config->packet_capacity, &packet_size);

    return packet_size;
}

static enum uls_status decode_positions(const dev_servo_t *servo)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    const uint16_t reply_length = dynamixel_protocol2_read_packet_length(data->packet);
    if (reply_length != dynamixel_protocol2_fast_sync_read_response_length(config->count, 4)) {
        return ULS_STATUS_INVALID_INPUT;
    }

    const uint32_t packet_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + reply_length;
    uint8_t id;
    uint16_t remaining;
    enum uls_status status = dynamixel_protocol2_decode_response(
        data->packet, packet_size, &id, data->parameters, config->parameters_capacity, &remaining);
    if (status != ULS_STATUS_OK) {
        return status;
    }
    if (id != DYNAMIXEL_PROTOCOL2_BROADCAST_ID) {
        return ULS_STATUS_INVALID_INPUT;
    }

    const uint8_t *entry = data->parameters;
    for (uint32_t i = 0; i < config->count; i++) {
        struct dynamixel_protocol2_fast_sync_read_response response;
        uint16_t consumed;
        status = dynamixel_protocol2_deserialize_fast_sync_read_response(entry, remaining, 4,
                                                                         &response, &consumed);
        if (status != ULS_STATUS_OK || response.id != config->ids[i]) {
            return ULS_STATUS_INVALID_INPUT;
        }
        if (response.error != 0) {
            return ULS_STATUS_ERROR;
        }

        const float position_ticks = buffer_read_int32_le(response.data);
        data->positions[i] = position_ticks / DYNAMIXEL_TICKS_PER_ROTATION;

        entry += consumed;
        remaining -= consumed;
    }

    return ULS_STATUS_OK;
}

static void complete_read(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        status = decode_positions(servo);
    }

    data->busy = false;
    data->read_cb(status, status == ULS_STATUS_OK ? data->positions : NULL, data->user_data);
}

static void on_transmit_finish(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;

    if (status == ULS_STATUS_OK) {
        status = dev_servo_dynamixel_receive_status_async(servo, complete_read);
    }

    if (status != ULS_STATUS_OK) {
        complete_read(status, servo);
    }
}

enum uls_status dev_servo_dynamixel_read_positions(dev_servo_t *servo, float *positions)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    const uint16_t length = prepare_transmit(servo);
    // TODO: Switch the direction GPIO to TX before transmitting.
    enum uls_status status =
        io_uart_transmit(config->uart, data->packet, length, DYNAMIXEL_UART_TIMEOUT_MS);
    if (status != ULS_STATUS_OK) {
        goto cleanup;
    }

    status = dev_servo_dynamixel_receive_status(servo);
    if (status != ULS_STATUS_OK) {
        goto cleanup;
    }

    status = decode_positions(servo);
    if (status != ULS_STATUS_OK) {
        goto cleanup;
    }

    memcpy(positions, data->positions, config->count * sizeof(*positions));

cleanup:
    data->busy = false;
    return status;
}

enum uls_status dev_servo_dynamixel_read_positions_async(dev_servo_t *servo, dev_servo_read_cb_t cb,
                                                         void *user_data)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    data->read_cb = cb;
    data->user_data = user_data;

    io_uart_set_transmit_cb(config->uart, on_transmit_finish, servo);

    // TODO: Switch the direction GPIO to TX before starting the transmit.
    enum uls_status status =
        io_uart_transmit_async(config->uart, data->packet, prepare_transmit(servo));
    if (status != ULS_STATUS_OK) {
        data->busy = false;
    }

    return status;
}
