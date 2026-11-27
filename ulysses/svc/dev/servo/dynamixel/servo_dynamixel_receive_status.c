#include "ulysses/lib/dynamixel/protocol2/packet.h"

#include "dev/servo/dynamixel/servo_dynamixel_receive_status.h"
#include "dev/servo/dynamixel/servo_dynamixel.h"

enum uls_status dev_servo_dynamixel_receive_status(const dev_servo_t *servo)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_RESET);
    enum uls_status status =
        io_uart_receive(config->uart, data->packet, DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE,
                        DYNAMIXEL_UART_TIMEOUT_MS);

    uint16_t length = 0;
    if (status == ULS_STATUS_OK) {
        length = dynamixel_protocol2_read_packet_length(data->packet);
        const uint32_t packet_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + length;

        if (packet_size > config->packet_capacity) {
            status = ULS_STATUS_INVALID_INPUT;
        }
    }

    if (status == ULS_STATUS_OK) {
        status =
            io_uart_receive(config->uart, data->packet + DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE,
                            length, DYNAMIXEL_UART_TIMEOUT_MS);
    }
    return status;
}

static void on_prefix_received(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;
    io_uart_receive_cb_t cb = data->receive_status_cb;

    uint16_t length = 0;
    if (status == ULS_STATUS_OK) {
        length = dynamixel_protocol2_read_packet_length(data->packet);
        const uint32_t packet_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + length;

        if (packet_size > config->packet_capacity) {
            status = ULS_STATUS_INVALID_INPUT;
        }
    }

    if (status == ULS_STATUS_OK && length != 0) {
        io_uart_set_receive_cb(config->uart, cb, servo);
        status = io_uart_receive_async(
            config->uart, data->packet + DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE, length);
    }

    if (status != ULS_STATUS_OK || length == 0) {
        cb(status, servo);
    }
}

enum uls_status dev_servo_dynamixel_receive_status_async(dev_servo_t *servo,
                                                         io_uart_receive_cb_t cb)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    data->receive_status_cb = cb;
    io_uart_set_receive_cb(config->uart, on_prefix_received, servo);

    io_gpio_write(config->transmit_enable_gpio, IO_GPIO_STATE_RESET);
    // TODO: Use timed async receive when UART supports timeout and cancellation.
    return io_uart_receive_async(config->uart, data->packet,
                                 DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE);
}
