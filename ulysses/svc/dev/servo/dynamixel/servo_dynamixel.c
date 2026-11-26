#include <stddef.h>

#include "ulysses/lib/assert.h"

#include "dev/servo/dynamixel/servo_dynamixel.h"

static const struct dev_servo_ops dynamixel_ops = {
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
    ULYSSES_ASSERT(config->count <= UINT8_MAX, "Dynamixel servo count must fit in a byte");
    ULYSSES_ASSERT(config->sync_write_max_length >= 4,
                   "Dynamixel goal positions require four writable bytes per servo");

    for (uint32_t i = 0; i < config->count; i++) {
        data->write_devices[i].id = config->ids[i];
    }

    data->busy = false;
    servo->config = config;
    servo->data = data;
    servo->ops = &dynamixel_ops;
}
