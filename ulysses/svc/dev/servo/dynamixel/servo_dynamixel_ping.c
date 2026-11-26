#include <stddef.h>

#include "ulysses/lib/dynamixel/protocol2/packet.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#include "dev/servo/dynamixel/servo_dynamixel.h"
#include "dev/servo/dynamixel/servo_dynamixel_receive_status.h"

static enum uls_status ping_servo(const dev_servo_t *servo, uint8_t id)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    uint32_t packet_size;
    dynamixel_protocol2_encode_request(id, DYNAMIXEL_PROTOCOL2_INST_PING, NULL, 0, data->packet,
                                       config->packet_capacity, &packet_size);

    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_SET);
    enum uls_status status =
        io_uart_transmit(config->uart, data->packet, packet_size, DYNAMIXEL_UART_TIMEOUT_MS);

    if (status == ULS_STATUS_OK) {
        status = dev_servo_dynamixel_receive_status(servo);
    }

    uint32_t reply_size = 0;
    if (status == ULS_STATUS_OK) {
        const uint16_t reply_length = dynamixel_protocol2_read_packet_length(data->packet);
        reply_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + reply_length;
    }

    if (status == ULS_STATUS_OK && reply_size > dynamixel_protocol2_required_packet_capacity(4)) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    uint8_t reply_id;
    uint16_t parameters_length;
    if (status == ULS_STATUS_OK) {
        status = dynamixel_protocol2_decode_response(data->packet, reply_size, &reply_id,
                                                     data->parameters, config->parameters_capacity,
                                                     &parameters_length);
    }

    if (status == ULS_STATUS_OK && reply_id != id) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    struct dynamixel_protocol2_ping_response response;
    if (status == ULS_STATUS_OK) {
        status = dynamixel_protocol2_deserialize_ping_response(data->parameters, parameters_length,
                                                               &response);
    }

    if (status == ULS_STATUS_OK && response.error != 0) {
        status = ULS_STATUS_ERROR;
    }

    return status;
}

enum uls_status dev_servo_dynamixel_ping_all(dev_servo_t *servo)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    for (uint32_t i = 0; i < config->count; i++) {
        const enum uls_status status = ping_servo(servo, config->ids[i]);
        if (status != ULS_STATUS_OK) {
            data->busy = false;
            return status;
        }
    }

    data->busy = false;
    return ULS_STATUS_OK;
}
