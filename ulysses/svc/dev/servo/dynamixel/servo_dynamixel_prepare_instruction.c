#include "ulysses/lib/dynamixel/protocol2/packet.h"

#include "dev/servo/dynamixel/servo_dynamixel_prepare_instruction.h"
#include "dev/servo/dynamixel/servo_dynamixel.h"

uint16_t dev_servo_dynamixel_prepare_fast_sync_read(
    const dev_servo_t *servo, const struct dynamixel_protocol2_fast_sync_read_request *request)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    uint16_t parameters_length;
    dynamixel_protocol2_serialize_fast_sync_read_request(
        request, data->parameters, config->parameters_capacity, &parameters_length);

    uint32_t packet_size;
    dynamixel_protocol2_encode_request(
        DYNAMIXEL_PROTOCOL2_BROADCAST_ID, DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ, data->parameters,
        parameters_length, data->packet, config->packet_capacity, &packet_size);

    return packet_size;
}

uint16_t
dev_servo_dynamixel_prepare_sync_write(const dev_servo_t *servo,
                                       const struct dynamixel_protocol2_sync_write_request *request)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    uint16_t parameters_length;
    dynamixel_protocol2_serialize_sync_write_request(
        request, data->parameters, config->parameters_capacity, &parameters_length);

    uint32_t packet_size;
    dynamixel_protocol2_encode_request(
        DYNAMIXEL_PROTOCOL2_BROADCAST_ID, DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE, data->parameters,
        parameters_length, data->packet, config->packet_capacity, &packet_size);

    return packet_size;
}
