#include "ulysses/io/bdshot.h"
#include "ulysses/lib/assert.h"
#include "ulysses/lib/bdshot/frame.h"
#include "ulysses/lib/math.h"
#include "ulysses/svc/dev/esc.h"
#include "dev/esc/bdshot/esc.h"
#include "dev/esc/esc.h"
#include "ulysses/status.h"

static void populate_telemetry_for_channel(uint32_t wire_frame,
                                           const struct dev_esc_bdshot_channel_config *config,
                                           struct dev_esc_telemetry *telemetry)
{
    uint16_t frame;
    enum bdshot_decode_status decode_status = bdshot_frame_decode_from_wire(&frame, wire_frame);

    if (decode_status != BDSHOT_DECODE_STATUS_OK) {
        telemetry->is_error = true;
        return;
    }

    // We don't support EDT frames yet
    if (bdshot_frame_is_edt(frame)) {
        return;
    }

    telemetry->is_valid = true;
    telemetry->is_error = false;
    telemetry->rpm = bdshot_frame_calculate_rpm(frame, config->pole_count);
}

static void populate_telemetry(const dev_esc_t *esc)
{
    const struct dev_esc_bdshot_config *config = esc->config;
    struct dev_esc_bdshot_data *data = esc->data;

    for (size_t i = 0; i < config->channel_count; i++) {
        const struct io_bdshot_response *response = &data->responses[i];

        if (response->status == ULS_STATUS_OK) {
            populate_telemetry_for_channel(response->wire_frame, &config->channels[i],
                                           &data->telemetry[i]);
        } else if (response->status != ULS_STATUS_NOT_READY) {
            data->telemetry[i].is_error = true;
        }
    }
}

static enum uls_status set_arm(const dev_esc_t *esc, uint8_t channel, bool is_armed)
{
    enum uls_status status = ULS_STATUS_ERROR;

    const struct dev_esc_bdshot_config *config = esc->config;
    const struct dev_esc_bdshot_data *data = esc->data;

    ULYSSES_ASSERT(channel < config->channel_count, "ESC channel index is out of range");

    struct dev_esc_bdshot_channel_data *channel_data = &data->channels[channel];

    status = bdshot_frame_pack_command(&channel_data->frame, BDSHOT_COMMAND_MOTOR_STOP, false)
                 ? ULS_STATUS_OK
                 : ULS_STATUS_ERROR;

    if (status == ULS_STATUS_OK) {
        // Defer setting these values since we must synchronize it to
        // after the current transceive is stopped
        channel_data->is_frame_update_pending = true;
        channel_data->is_arm_update_pending = true;
        channel_data->is_armed = is_armed;
    }

    return status;
}

static enum uls_status set_throttle(const dev_esc_t *esc, uint8_t channel, uint16_t throttle)
{
    enum uls_status status = ULS_STATUS_ERROR;

    const struct dev_esc_bdshot_config *config = esc->config;
    const struct dev_esc_bdshot_data *data = esc->data;

    ULYSSES_ASSERT(channel < config->channel_count, "ESC channel index is out of range");

    struct dev_esc_bdshot_channel_data *channel_data = &data->channels[channel];

    status = bdshot_frame_pack_throttle(&channel_data->frame, throttle, false) ? ULS_STATUS_OK
                                                                               : ULS_STATUS_ERROR;

    if (status == ULS_STATUS_OK) {
        // Defer setting these values since we must synchronize it to
        // after the current transceive is stopped
        channel_data->is_frame_update_pending = true;
    }

    return status;
}

static enum uls_status process_pending_commands(const dev_esc_t *esc)
{
    enum uls_status status = ULS_STATUS_OK;

    const struct dev_esc_bdshot_config *config = esc->config;
    const struct dev_esc_bdshot_data *data = esc->data;

    for (uint8_t i = 0; i < config->channel_count; i++) {
        struct dev_esc_bdshot_channel_data *channel_data = &data->channels[i];

        if (channel_data->is_frame_update_pending) {
            io_bdshot_set_frame(config->bdshot, i, channel_data->frame);

            channel_data->is_frame_update_pending = false;
        }

        if (channel_data->is_arm_update_pending) {
            status = io_bdshot_set_active(config->bdshot, i, channel_data->is_armed);

            if (status == ULS_STATUS_OK) {
                channel_data->is_arm_update_pending = false;
            }
        }

        if (status != ULS_STATUS_OK) {
            break;
        }
    }

    return status;
}

static enum uls_status get_telemetry(const dev_esc_t *esc, struct dev_esc_telemetry *telemetry,
                                     size_t telemetry_capacity)
{
    enum uls_status status = ULS_STATUS_ERROR;

    const struct dev_esc_bdshot_config *config = esc->config;
    const struct dev_esc_bdshot_data *data = esc->data;

    const size_t telemetry_count = min(telemetry_capacity, config->channel_count);
    bool is_some_telemetry_valid = false;

    for (size_t i = 0; i < telemetry_count; i++) {
        telemetry[i] = data->telemetry[i];

        if (telemetry[i].is_valid) {
            is_some_telemetry_valid = true;
        }
    }

    if (is_some_telemetry_valid) {
        status = ULS_STATUS_OK;
    } else {
        status = ULS_STATUS_NOT_READY;
    }

    return status;
}

static enum uls_status update(const dev_esc_t *esc)
{
    enum uls_status status = ULS_STATUS_ERROR;

    const struct dev_esc_bdshot_config *config = esc->config;

    status = io_bdshot_transceive_stop(config->bdshot);

    if (status == ULS_STATUS_OK) {
        populate_telemetry(esc);
    }

    if (status == ULS_STATUS_OK) {
        status = process_pending_commands(esc);
    }

    if (status == ULS_STATUS_OK) {
        status = io_bdshot_transceive_start(config->bdshot);
    }

    return status;
}

const struct dev_esc_ops DEV_ESC_OPS = {
    .set_arm = set_arm,
    .set_throttle = set_throttle,
    .get_telemetry = get_telemetry,
    .update = update,
};
