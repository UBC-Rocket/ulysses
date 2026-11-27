#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/dynamixel/protocol2/packet.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#include "dev/servo/dynamixel/servo_dynamixel.h"

static const struct dev_servo_ops dynamixel_ops = {
    .ping_all = dev_servo_dynamixel_ping_all,
    .set_torque = dev_servo_dynamixel_set_torque,
    .read_positions = dev_servo_dynamixel_read_positions,
    .read_positions_async = dev_servo_dynamixel_read_positions_async,
    .write_positions = dev_servo_dynamixel_write_positions,
    .write_positions_async = dev_servo_dynamixel_write_positions_async,
};

void dev_servo_dynamixel_init(dev_servo_t *servo, const struct dev_servo_dynamixel_config *config,
                              struct dev_servo_dynamixel_data *data)
{
    ULYSSES_ASSERT(servo != NULL && config != NULL && data != NULL,
                   "servo bus, configuration and storage must not be NULL");
    ULYSSES_ASSERT(config->ids != NULL && data->write_devices != NULL,
                   "Dynamixel servo IDs and write devices must not be NULL");
    ULYSSES_ASSERT(config->zero_ticks != NULL, "Dynamixel encoder zero values must not be NULL");
    ULYSSES_ASSERT(data->positions != NULL, "Dynamixel position storage must not be NULL");
    ULYSSES_ASSERT(config->count > 0 && config->count <= UINT8_MAX,
                   "Dynamixel servo count must be nonzero and fit in a byte");
    ULYSSES_ASSERT(config->sync_write_max_length >= 4,
                   "Dynamixel goal positions require four writable bytes per servo");

    const uint32_t write_parameters_size =
        dynamixel_protocol2_sync_write_parameters_size(config->count, 4);
    ULYSSES_ASSERT(write_parameters_size <= DYNAMIXEL_PROTOCOL2_MAX_PARAMETERS_COUNT,
                   "Dynamixel servo count must fit a position Sync Write");
    const uint32_t write_packet_size =
        dynamixel_protocol2_required_packet_capacity(write_parameters_size);

    const uint32_t read_parameters_size =
        dynamixel_protocol2_fast_sync_read_response_parameters_size(config->count, 4);
    const uint32_t read_packet_size =
        DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE +
        dynamixel_protocol2_fast_sync_read_response_length(config->count, 4);

    ULYSSES_ASSERT(data->packet != NULL && config->packet_capacity >= write_packet_size &&
                       config->packet_capacity >= read_packet_size,
                   "Dynamixel packet buffer must fit position requests and replies");
    ULYSSES_ASSERT(data->parameters != NULL &&
                       config->parameters_capacity >= write_parameters_size &&
                       config->parameters_capacity >= read_parameters_size,
                   "Dynamixel parameter buffer must fit position requests and replies");

    for (uint32_t i = 0; i < config->count; i++) {
        const uint8_t id = config->ids[i];
        ULYSSES_ASSERT(id <= DYNAMIXEL_PROTOCOL2_MAX_DEVICE_ID, "invalid Dynamixel servo ID");
        for (uint32_t j = 0; j < i; j++) {
            ULYSSES_ASSERT(id != config->ids[j], "Dynamixel servo IDs must be unique");
        }

        struct dynamixel_protocol2_sync_write_device *device = &data->write_devices[i];
        ULYSSES_ASSERT(device->data != NULL, "Dynamixel device write buffer must not be NULL");
        device->id = id;
    }

    data->busy = false;
    servo->config = config;
    servo->data = data;
    servo->ops = &dynamixel_ops;
}
