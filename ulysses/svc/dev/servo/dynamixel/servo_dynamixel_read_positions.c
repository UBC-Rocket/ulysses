#include <string.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/buffer.h"
#include "ulysses/lib/dynamixel/protocol2/packet.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#include "dev/servo/dynamixel/servo_dynamixel.h"

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
    enum uls_status status = dynamixel_protocol2_serialize_fast_sync_read_request(
        &request, data->parameters, config->parameters_capacity, &parameters_length);
    ULYSSES_ASSERT(status == ULS_STATUS_OK, "Dynamixel parameters must fit the read request");

    uint32_t packet_size;
    status = dynamixel_protocol2_encode_request(
        DYNAMIXEL_PROTOCOL2_BROADCAST_ID, DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ, data->parameters,
        parameters_length, data->packet, config->packet_capacity, &packet_size);
    ULYSSES_ASSERT(status == ULS_STATUS_OK, "Dynamixel packet buffer must fit the request");

    return packet_size;
}

static enum uls_status decode_positions(const dev_servo_t *servo, uint16_t reply_length)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    const uint32_t packet_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + reply_length;
    uint8_t id;
    uint16_t remaining;
    enum uls_status status = dynamixel_protocol2_decode_response(
        data->packet, packet_size, &id, data->parameters, config->parameters_capacity, &remaining);
    ULYSSES_ASSERT(status != ULS_STATUS_NO_SPACE, "Dynamixel parameters must fit the reply");
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

    return remaining == 0 ? ULS_STATUS_OK : ULS_STATUS_INVALID_INPUT;
}

static bool valid_reply_length(const struct dev_servo_dynamixel_config *config, uint16_t length)
{
    const uint32_t packet_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + length;
    return length >= 4 && packet_size <= config->packet_capacity;
}

static void on_full_reply(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        const uint16_t reply_length = dynamixel_protocol2_read_packet_length(data->packet);
        status = decode_positions(servo, reply_length);
    }

    data->busy = false;
    data->read_cb(status, status == ULS_STATUS_OK ? data->positions : NULL, data->user_data);
}

static void on_prefix_reply(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        const uint16_t reply_length = dynamixel_protocol2_read_packet_length(data->packet);
        if (valid_reply_length(config, reply_length)) {
            io_uart_set_receive_cb(config->uart, on_full_reply, servo);
            status = io_uart_receive_async(
                config->uart, data->packet + DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE, reply_length);
        } else {
            status = ULS_STATUS_INVALID_INPUT;
        }
    }

    if (status != ULS_STATUS_OK) {
        data->busy = false;
        data->read_cb(status, NULL, data->user_data);
    }
}

static void on_transmit_finish(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        io_uart_set_receive_cb(config->uart, on_prefix_reply, servo);

        // TODO: Switch the direction GPIO to RX before arming reception.
        // TODO: Use timed async receive when UART supports timeout and cancellation.
        status = io_uart_receive_async(config->uart, data->packet,
                                       DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE);
    }

    if (status != ULS_STATUS_OK) {
        data->busy = false;
        data->read_cb(status, NULL, data->user_data);
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
    if (status == ULS_STATUS_OK) {
        // TODO: Switch the direction GPIO to RX before receiving.
        status = io_uart_receive(config->uart, data->packet, DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE,
                                 DYNAMIXEL_UART_TIMEOUT_MS);
    }
    if (status == ULS_STATUS_OK) {
        const uint16_t reply_length = dynamixel_protocol2_read_packet_length(data->packet);
        if (valid_reply_length(config, reply_length)) {
            status =
                io_uart_receive(config->uart, data->packet + DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE,
                                reply_length, DYNAMIXEL_UART_TIMEOUT_MS);
        } else {
            status = ULS_STATUS_INVALID_INPUT;
        }
    }

    if (status == ULS_STATUS_OK) {
        const uint16_t reply_length = dynamixel_protocol2_read_packet_length(data->packet);
        status = decode_positions(servo, reply_length);
    }
    if (status == ULS_STATUS_OK) {
        memcpy(positions, data->positions, config->count * sizeof(*positions));
    }

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
