#include "dev/servo/dynamixel/servo_dynamixel.h"

#include <math.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/buffer.h"
#include "ulysses/lib/dynamixel/protocol2/packet.h"
#include "ulysses/lib/dynamixel/protocol2/parameters.h"

#define GOAL_POSITION_ADDRESS (0x74)
#define MAX_GOAL_TICKS (0xFFFFF)

static uint16_t prepare_transmit(const dev_servo_t *servo, const float *positions)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    for (uint32_t i = 0; i < config->count; i++) {
        const float position = positions[i];

        ULYSSES_ASSERT(fabsf(position) < (MAX_GOAL_TICKS + 0.5f) / DYNAMIXEL_TICKS_PER_ROTATION,
                       "position is outside the Dynamixel goal position range");

        const long position_ticks = lrintf(position * DYNAMIXEL_TICKS_PER_ROTATION);
        buffer_write_int32_le((uint8_t *)data->write_devices[i].data, position_ticks);
    }

    const struct dynamixel_protocol2_sync_write_request request = {
        .address = GOAL_POSITION_ADDRESS,
        .length = 4,
        .device_count = config->count,
        .devices = data->write_devices,
    };

    uint16_t parameters_length;
    dynamixel_protocol2_serialize_sync_write_request(
        &request, data->parameters, config->parameters_capacity, &parameters_length);

    uint32_t packet_size;
    dynamixel_protocol2_encode_request(
        DYNAMIXEL_PROTOCOL2_BROADCAST_ID, DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE, data->parameters,
        parameters_length, data->packet, config->packet_capacity, &packet_size);

    return packet_size;
}

static void on_transmit_finish(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    struct dev_servo_dynamixel_data *data = servo->data;

    data->busy = false;
    data->write_cb(status, data->user_data);
}

enum uls_status dev_servo_dynamixel_write_positions(dev_servo_t *servo, const float *positions)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    const uint16_t length = prepare_transmit(servo, positions);
    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_SET);
    enum uls_status status =
        io_uart_transmit(config->uart, data->packet, length, DYNAMIXEL_UART_TIMEOUT_MS);

    data->busy = false;
    return status;
}

enum uls_status dev_servo_dynamixel_write_positions_async(dev_servo_t *servo,
                                                          const float *positions,
                                                          dev_servo_write_cb_t cb, void *user_data)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (data->busy) {
        return ULS_STATUS_BUSY;
    }
    data->busy = true;

    data->write_cb = cb;
    data->user_data = user_data;

    io_uart_set_transmit_cb(config->uart, on_transmit_finish, servo);

    const uint16_t length = prepare_transmit(servo, positions);
    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_SET);
    enum uls_status status = io_uart_transmit_async(config->uart, data->packet, length);
    if (status != ULS_STATUS_OK) {
        data->busy = false;
    }

    return status;
}
