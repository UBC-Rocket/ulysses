#include "ulysses/lib/dynamixel/protocol2/packet.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#include "dev/servo/dynamixel/servo_dynamixel.h"
#include "dev/servo/dynamixel/servo_dynamixel_receive_status.h"

#define TORQUE_ENABLE_ADDRESS (0x40)

enum uls_status dev_servo_dynamixel_set_torque(dev_servo_t *servo, enum dev_servo_torque_mode mode)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    for (uint32_t i = 0; i < config->count; i++) {
        uint8_t *value = (uint8_t *)data->write_devices[i].data;
        *value = mode;
    }

    const struct dynamixel_protocol2_sync_write_request write_request = {
        .address = TORQUE_ENABLE_ADDRESS,
        .length = 1,
        .device_count = config->count,
        .devices = data->write_devices,
    };

    uint16_t parameters_length;
    dynamixel_protocol2_serialize_sync_write_request(
        &write_request, data->parameters, config->parameters_capacity, &parameters_length);

    uint32_t packet_size;
    dynamixel_protocol2_encode_request(
        DYNAMIXEL_PROTOCOL2_BROADCAST_ID, DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE, data->parameters,
        parameters_length, data->packet, config->packet_capacity, &packet_size);

    // TODO: Switch the direction GPIO to TX before transmitting.
    enum uls_status status =
        io_uart_transmit(config->uart, data->packet, packet_size, DYNAMIXEL_UART_TIMEOUT_MS);

    if (status == ULS_STATUS_OK) {
        const struct dynamixel_protocol2_fast_sync_read_request read_request = {
            .address = TORQUE_ENABLE_ADDRESS,
            .length = 1,
            .id_count = config->count,
            .ids = config->ids,
        };

        dynamixel_protocol2_serialize_fast_sync_read_request(
            &read_request, data->parameters, config->parameters_capacity, &parameters_length);

        dynamixel_protocol2_encode_request(DYNAMIXEL_PROTOCOL2_BROADCAST_ID,
                                           DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ,
                                           data->parameters, parameters_length, data->packet,
                                           config->packet_capacity, &packet_size);

        // TODO: Switch the direction GPIO to TX before transmitting.
        status =
            io_uart_transmit(config->uart, data->packet, packet_size, DYNAMIXEL_UART_TIMEOUT_MS);
    }

    if (status == ULS_STATUS_OK) {
        status = dev_servo_dynamixel_receive_status(servo);
    }

    const uint32_t expected_reply_length =
        dynamixel_protocol2_fast_sync_read_response_length(config->count, 1);

    if (status == ULS_STATUS_OK &&
        dynamixel_protocol2_read_packet_length(data->packet) != expected_reply_length) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    uint8_t reply_id;
    uint16_t remaining;
    if (status == ULS_STATUS_OK) {
        const uint32_t reply_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + expected_reply_length;
        status = dynamixel_protocol2_decode_response(data->packet, reply_size, &reply_id,
                                                     data->parameters, config->parameters_capacity,
                                                     &remaining);
    }

    if (status == ULS_STATUS_OK && reply_id != DYNAMIXEL_PROTOCOL2_BROADCAST_ID) {
        status = ULS_STATUS_INVALID_INPUT;
    }

    if (status == ULS_STATUS_OK) {
        const uint8_t *entry = data->parameters;
        for (uint32_t i = 0; i < config->count; i++) {
            struct dynamixel_protocol2_fast_sync_read_response response;
            uint16_t consumed;
            status = dynamixel_protocol2_deserialize_fast_sync_read_response(entry, remaining, 1,
                                                                             &response, &consumed);

            if (status != ULS_STATUS_OK || response.id != config->ids[i]) {
                status = ULS_STATUS_INVALID_INPUT;
                break;
            }
            if (response.data[0] != mode) {
                status = ULS_STATUS_NOT_READY;
                break;
            }
            if (response.error != 0) {
                status = ULS_STATUS_ERROR;
                break;
            }

            entry += consumed;
            remaining -= consumed;
        }
    }

    data->busy = false;
    return status;
}
