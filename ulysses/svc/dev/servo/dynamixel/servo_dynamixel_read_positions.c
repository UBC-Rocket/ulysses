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

    enum uls_status status = ULS_STATUS_INVALID_INPUT;
    uint8_t id;
    uint16_t remaining;
    if (reply_length == dynamixel_protocol2_fast_sync_read_response_length(config->count, 4)) {
        const uint32_t packet_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + reply_length;
        status =
            dynamixel_protocol2_decode_response(data->packet, packet_size, &id, data->parameters,
                                                config->parameters_capacity, &remaining);
    }

    if (status == ULS_STATUS_OK && id != DYNAMIXEL_PROTOCOL2_BROADCAST_ID) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    if (status == ULS_STATUS_OK) {
        const uint8_t *entry = data->parameters;
        for (uint32_t i = 0; i < config->count; i++) {
            struct dynamixel_protocol2_fast_sync_read_response response;
            uint16_t consumed;
            status = dynamixel_protocol2_deserialize_fast_sync_read_response(entry, remaining, 4,
                                                                             &response, &consumed);

            if (status != ULS_STATUS_OK || response.id != config->ids[i]) {
                status = ULS_STATUS_INVALID_INPUT;
                break;
            }
            if (response.error != 0) {
                status = ULS_STATUS_ERROR;
                break;
            }

            const int32_t raw_ticks = buffer_read_int32_le(response.data);
            data->positions[i] =
                dev_servo_dynamixel_ticks_to_position(raw_ticks, config->zero_ticks[i]);

            entry += consumed;
            remaining -= consumed;
        }
    }

    return status;
}

static void complete_read(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        status = decode_positions(servo);
    }
    if (status == ULS_STATUS_OK) {
        memcpy(data->read_positions, data->positions, config->count * sizeof(*data->positions));
    }

    data->busy = false;
    data->read_cb(status, data->user_data);
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
    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_SET);
    enum uls_status status =
        io_uart_transmit(config->uart, data->packet, length, DYNAMIXEL_UART_TIMEOUT_MS);

    if (status == ULS_STATUS_OK) {
        status = dev_servo_dynamixel_receive_status(servo);
    }

    if (status == ULS_STATUS_OK) {
        status = decode_positions(servo);
    }

    if (status == ULS_STATUS_OK) {
        memcpy(positions, data->positions, config->count * sizeof(*positions));
    }

    data->busy = false;
    return status;
}

enum uls_status dev_servo_dynamixel_read_positions_async(dev_servo_t *servo, float *positions,
                                                         dev_servo_read_cb_t cb, void *user_data)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    data->read_positions = positions;
    data->read_cb = cb;
    data->user_data = user_data;

    io_uart_set_transmit_cb(config->uart, on_transmit_finish, servo);

    const uint16_t length = prepare_transmit(servo);
    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_SET);
    enum uls_status status = io_uart_transmit_async(config->uart, data->packet, length);
    if (status != ULS_STATUS_OK) {
        data->busy = false;
    }

    return status;
}
