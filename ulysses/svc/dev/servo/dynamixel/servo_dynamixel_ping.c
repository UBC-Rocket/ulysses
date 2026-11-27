#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/dynamixel/protocol2/packet.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#include "dev/servo/dynamixel/servo_dynamixel.h"
#include "dev/servo/dynamixel/servo_dynamixel_prepare_instruction.h"
#include "dev/servo/dynamixel/servo_dynamixel_receive_status.h"

// XM430-W210 hardware error status uses the same fault bits as enum dev_servo_hw_error.
// https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/#hardware-error-status70
#define HARDWARE_ERROR_STATUS_ADDRESS (0x46)
#define STATUS_INSTRUCTION_ERROR_MASK (0x7F)
#define STATUS_HARDWARE_ALERT_MASK (0x80)
#define PING_REPLY_LENGTH (7)

static enum uls_status ping_servo(const dev_servo_t *servo, uint8_t id, bool *hardware_alert)
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

    if (status == ULS_STATUS_OK &&
        dynamixel_protocol2_read_packet_length(data->packet) != PING_REPLY_LENGTH) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    uint8_t reply_id;
    uint16_t parameters_length;
    if (status == ULS_STATUS_OK) {
        const uint32_t reply_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + PING_REPLY_LENGTH;
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

    if (status == ULS_STATUS_OK && (response.error & STATUS_INSTRUCTION_ERROR_MASK) != 0) {
        status = ULS_STATUS_ERROR;
    }

    if (status == ULS_STATUS_OK) {
        *hardware_alert = (response.error & STATUS_HARDWARE_ALERT_MASK) != 0;
    }

    return status;
}

static enum uls_status read_hardware_errors(const dev_servo_t *servo, uint8_t id,
                                            uint8_t *error_mask)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    const struct dynamixel_protocol2_fast_sync_read_request request = {
        .address = HARDWARE_ERROR_STATUS_ADDRESS,
        .length = 1,
        .id_count = 1,
        .ids = &id,
    };

    const uint16_t packet_size = dev_servo_dynamixel_prepare_fast_sync_read(servo, &request);

    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_SET);
    enum uls_status status =
        io_uart_transmit(config->uart, data->packet, packet_size, DYNAMIXEL_UART_TIMEOUT_MS);

    if (status == ULS_STATUS_OK) {
        status = dev_servo_dynamixel_receive_status(servo);
    }

    const uint32_t expected_reply_length =
        dynamixel_protocol2_fast_sync_read_response_length(request.id_count, request.length);

    if (status == ULS_STATUS_OK &&
        dynamixel_protocol2_read_packet_length(data->packet) != expected_reply_length) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    uint8_t reply_id;
    uint16_t parameters_length;
    if (status == ULS_STATUS_OK) {
        const uint32_t reply_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + expected_reply_length;
        status = dynamixel_protocol2_decode_response(data->packet, reply_size, &reply_id,
                                                     data->parameters, config->parameters_capacity,
                                                     &parameters_length);
    }

    if (status == ULS_STATUS_OK && reply_id != DYNAMIXEL_PROTOCOL2_BROADCAST_ID) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    struct dynamixel_protocol2_fast_sync_read_response response;
    uint16_t consumed;
    if (status == ULS_STATUS_OK) {
        status = dynamixel_protocol2_deserialize_fast_sync_read_response(
            data->parameters, parameters_length, request.length, &response, &consumed);
    }

    if (status == ULS_STATUS_OK && response.id != id) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    if (status == ULS_STATUS_OK && (response.error & STATUS_INSTRUCTION_ERROR_MASK) != 0) {
        status = ULS_STATUS_ERROR;
    }

    if (status == ULS_STATUS_OK) {
        *error_mask = response.data[0];
    }

    return status;
}

enum uls_status dev_servo_dynamixel_ping(dev_servo_t *servo, uint32_t index, uint8_t *error_mask)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    ULYSSES_ASSERT(index < config->count, "servo index is outside the configured ID array");

    *error_mask = 0;
    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    const uint8_t id = config->ids[index];
    bool hardware_alert;
    enum uls_status status = ping_servo(servo, id, &hardware_alert);
    if (status == ULS_STATUS_OK && hardware_alert) {
        *error_mask = DEV_SERVO_HW_ERROR_READ_FAILED;
        status = read_hardware_errors(servo, id, error_mask);
    }

    if (status != ULS_STATUS_OK || *error_mask != 0) {
        status = ULS_STATUS_ERROR;
    }

    data->busy = false;
    return status;
}
