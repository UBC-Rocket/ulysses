#include "dev/servo/dynamixel/servo_dynamixel.h"

#include <math.h>
#include <string.h>

#include "dev/servo/servo.h"
#include "ulysses/lib/assert.h"
#include "ulysses/lib/buffer.h"
#include "ulysses/lib/dynamixel/protocol2_instructions.h"
#include "ulysses/lib/dynamixel/protocol2_packet.h"

#define GOAL_POSITION_ADDRESS (0x74)
#define PRESENT_POSITION_ADDRESS (0x84)

#define TICKS_PER_ROTATION (0x1000)
#define MAX_GOAL_TICKS (0xFFFFF)

#define UART_TIMEOUT_MS (20)

static inline bool claim_bus(struct dev_servo_dynamixel_data *data)
{
    if (data->busy) {
        return false;
    }

    data->busy = true;
    return true;
}

static uint16_t encode_request(struct dev_servo_dynamixel_data *data, uint8_t instruction,
                               uint16_t parameters_length)
{
    uint32_t packet_size;
    enum uls_status status = dynamixel_protocol2_encode_request(
        DYNAMIXEL_PROTOCOL2_BROADCAST_ID, instruction, data->parameters, parameters_length,
        data->packet, sizeof(data->packet), &packet_size);
    ULYSSES_ASSERT(status == ULS_STATUS_OK, "Dynamixel packet buffer must fit the request");

    return packet_size;
}

static uint16_t prepare_read(const dev_servo_t *servo)
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
        &request, data->parameters, sizeof(data->parameters), &parameters_length);
    ULYSSES_ASSERT(status == ULS_STATUS_OK, "Dynamixel parameters must fit the read request");

    return encode_request(data, DYNAMIXEL_PROTOCOL2_INST_FAST_SYNC_READ, parameters_length);
}

static uint16_t prepare_write(const dev_servo_t *servo, const float *positions)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    // The serializer copies these ticks into parameters before the encoder reuses packet.
    for (uint32_t i = 0; i < config->count; i++) {
        const float position_ticks = positions[i] * TICKS_PER_ROTATION;
        ULYSSES_ASSERT(fabsf(position_ticks) < MAX_GOAL_TICKS + 0.5f,
                       "position is outside the Dynamixel goal position range");
        buffer_write_int32_le(&data->packet[4 * i], lrintf(position_ticks));
    }

    const struct dynamixel_protocol2_sync_write_request request = {
        .address = GOAL_POSITION_ADDRESS,
        .length = 4,
        .device_count = config->count,
        .devices = data->write_devices,
    };
    uint16_t parameters_length;

    enum uls_status status = dynamixel_protocol2_serialize_sync_write_request(
        &request, data->parameters, sizeof(data->parameters), &parameters_length);
    ULYSSES_ASSERT(status == ULS_STATUS_OK, "Dynamixel parameters must fit the write request");

    return encode_request(data, DYNAMIXEL_PROTOCOL2_INST_SYNC_WRITE, parameters_length);
}

static enum uls_status read_reply_length(struct dev_servo_dynamixel_data *data)
{
    const uint16_t length = dynamixel_protocol2_read_packet_length(data->packet);

    // A status packet needs at least an instruction, an error byte, and a CRC.
    if (length < 4 || length > sizeof(data->packet) - DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE) {
        return ULS_STATUS_INVALID_INPUT;
    }

    data->reply_length = length;
    return ULS_STATUS_OK;
}

static enum uls_status decode_positions(const dev_servo_t *servo)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    const uint32_t packet_size = DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE + data->reply_length;
    uint8_t id;
    uint16_t remaining;
    enum uls_status status = dynamixel_protocol2_decode_response(
        data->packet, packet_size, &id, data->parameters, sizeof(data->parameters), &remaining);
    if (status != ULS_STATUS_OK || id != DYNAMIXEL_PROTOCOL2_BROADCAST_ID) {
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
        data->positions[i] = position_ticks / TICKS_PER_ROTATION;

        entry += consumed;
        remaining -= consumed;
    }

    return remaining == 0 ? ULS_STATUS_OK : ULS_STATUS_INVALID_INPUT;
}

static enum uls_status receive_reply(const dev_servo_t *servo)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    // TODO: Switch the direction GPIO to RX before receiving.
    enum uls_status status = io_uart_receive(
        config->uart, data->packet, DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE, UART_TIMEOUT_MS);
    if (status == ULS_STATUS_OK) {
        status = read_reply_length(data);
    }
    if (status != ULS_STATUS_OK) {
        return status;
    }

    return io_uart_receive(config->uart, data->packet + DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE,
                           data->reply_length, UART_TIMEOUT_MS);
}

static void on_write_done(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    struct dev_servo_dynamixel_data *data = servo->data;

    data->write_cb(status, data->user_data);
    data->busy = false;
}

static void on_positions_reply(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        status = decode_positions(servo);
    }

    data->read_cb(status, status == ULS_STATUS_OK ? data->positions : NULL, data->user_data);
    data->busy = false;
}

static void on_prefix_done(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        status = read_reply_length(data);
    }
    if (status == ULS_STATUS_OK) {
        io_uart_set_receive_cb(config->uart, data->reply_cb, servo);
        status = io_uart_receive_async(config->uart,
                                       data->packet + DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE,
                                       data->reply_length);
    }

    if (status != ULS_STATUS_OK) {
        data->reply_cb(status, servo);
    }
}

static void on_read_transmit_done(enum uls_status status, void *user_data)
{
    dev_servo_t *servo = user_data;
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (status == ULS_STATUS_OK) {
        io_uart_set_receive_cb(config->uart, on_prefix_done, servo);

        // TODO: Switch the direction GPIO to RX before arming reception.
        // TODO: Use timed async receive when UART supports timeout and cancellation.
        status = io_uart_receive_async(config->uart, data->packet,
                                       DYNAMIXEL_PROTOCOL2_PACKET_PREFIX_SIZE);
    }

    if (status != ULS_STATUS_OK) {
        data->reply_cb(status, servo);
    }
}

static enum uls_status dynamixel_read_positions(dev_servo_t *servo, float *positions)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (!claim_bus(data)) {
        return ULS_STATUS_BUSY;
    }

    const io_uart_dev_t *uart = config->uart;
    const uint16_t length = prepare_read(servo);

    // TODO: Switch the direction GPIO to TX before transmitting.
    enum uls_status status = io_uart_transmit(uart, data->packet, length, UART_TIMEOUT_MS);
    if (status == ULS_STATUS_OK) {
        status = receive_reply(servo);
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

static enum uls_status dynamixel_write_positions(dev_servo_t *servo, const float *positions)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    if (!claim_bus(data)) {
        return ULS_STATUS_BUSY;
    }

    const uint16_t length = prepare_write(servo, positions);

    // TODO: Switch the direction GPIO to TX before transmitting.
    enum uls_status status = io_uart_transmit(config->uart, data->packet, length, UART_TIMEOUT_MS);

    data->busy = false;
    return status;
}

static enum uls_status start_transmit_async(dev_servo_t *servo, uint16_t length,
                                            io_uart_transmit_cb_t cb)
{
    const struct dev_servo_dynamixel_config *config = servo->config;
    struct dev_servo_dynamixel_data *data = servo->data;

    io_uart_set_transmit_cb(config->uart, cb, servo);

    // TODO: Switch the direction GPIO to TX before starting the transmit.
    enum uls_status status = io_uart_transmit_async(config->uart, data->packet, length);
    if (status != ULS_STATUS_OK) {
        data->busy = false;
    }

    return status;
}

static enum uls_status dynamixel_read_positions_async(dev_servo_t *servo, dev_servo_read_cb_t cb,
                                                      void *user_data)
{
    struct dev_servo_dynamixel_data *data = servo->data;

    if (!claim_bus(data)) {
        return ULS_STATUS_BUSY;
    }

    data->read_cb = cb;
    data->user_data = user_data;
    data->reply_cb = on_positions_reply;

    return start_transmit_async(servo, prepare_read(servo), on_read_transmit_done);
}

static enum uls_status dynamixel_write_positions_async(dev_servo_t *servo, const float *positions,
                                                       dev_servo_write_cb_t cb, void *user_data)
{
    struct dev_servo_dynamixel_data *data = servo->data;

    if (!claim_bus(data)) {
        return ULS_STATUS_BUSY;
    }

    data->write_cb = cb;
    data->user_data = user_data;

    return start_transmit_async(servo, prepare_write(servo, positions), on_write_done);
}

static const struct dev_servo_ops dynamixel_ops = {
    .read_positions = dynamixel_read_positions,
    .read_positions_async = dynamixel_read_positions_async,
    .write_positions = dynamixel_write_positions,
    .write_positions_async = dynamixel_write_positions_async,
};

void dev_servo_dynamixel_init(dev_servo_t *servo, const struct dev_servo_dynamixel_config *config,
                              struct dev_servo_dynamixel_data *data)
{
    ULYSSES_ASSERT(servo != NULL && config != NULL && data != NULL,
                   "servo bus, configuration and storage must not be NULL");
    ULYSSES_ASSERT(config->uart != NULL && config->ids != NULL && data->positions != NULL &&
                       data->write_devices != NULL,
                   "Dynamixel bus requires a UART, servo IDs and per-servo storage");
    ULYSSES_ASSERT(config->count > 0 && config->count <= UINT8_MAX,
                   "invalid Dynamixel servo count");

    const uint32_t write_size = 4 + config->count * (1 + 4);
    ULYSSES_ASSERT(write_size <= DYNAMIXEL_PROTOCOL2_MAX_PARAMETERS_COUNT &&
                       dynamixel_protocol2_required_packet_capacity(write_size) <=
                           sizeof(data->packet),
                   "Dynamixel requests must fit the packet buffer");

    for (uint32_t i = 0; i < config->count; i++) {
        data->write_devices[i] = (struct dynamixel_protocol2_sync_write_device){
            .id = config->ids[i],
            .data = &data->packet[i * 4],
        };
    }

    data->busy = false;
    servo->config = config;
    servo->data = data;
    servo->ops = &dynamixel_ops;
}
