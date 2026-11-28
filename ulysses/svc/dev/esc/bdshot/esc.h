#ifndef ULYSSES_SVC_DEV_ESC_BDSHOT
#define ULYSSES_SVC_DEV_ESC_BDSHOT

#include "ulysses/io/bdshot.h"
#include "ulysses/svc/dev/esc.h"

struct dev_esc_bdshot_channel_config {
    uint8_t pole_count; //< Number of magnetic poles in the motor
};

struct dev_esc_bdshot_config {
    const io_bdshot_t *bdshot;
    const struct dev_esc_bdshot_channel_config *channels;
    uint8_t channel_count;
};

struct dev_esc_bdshot_channel_data {
    bool is_frame_update_pending;
    bool is_arm_update_pending;

    uint16_t frame;
    bool is_armed;
};

struct dev_esc_bdshot_data {
    struct dev_esc_bdshot_channel_data *channels;
    struct io_bdshot_response *responses;
    struct dev_esc_telemetry *telemetry;
};

void dev_esc_bdshot_init(dev_esc_t *esc, const struct dev_esc_bdshot_config *config,
                         struct dev_esc_bdshot_data *data);

#endif // ULYSSES_SVC_DEV_ESC_BDSHOT
