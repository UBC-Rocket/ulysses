#ifndef ULYSSES_SVC_DEV_ESC_PRIVATE
#define ULYSSES_SVC_DEV_ESC_PRIVATE

#include "ulysses/svc/dev/esc.h"

typedef enum uls_status (*dev_esc_set_arm_t)(const dev_esc_t *esc, uint8_t channel, bool is_armed);

typedef enum uls_status (*dev_esc_set_throttle_t)(const dev_esc_t *esc, uint8_t channel,
                                                  uint16_t throttle);

typedef enum uls_status (*dev_esc_get_telemetry_t)(const dev_esc_t *esc,
                                                   struct dev_esc_telemetry *telemetry,
                                                   size_t telemetry_capacity);

typedef enum uls_status (*dev_esc_update_t)(const dev_esc_t *esc);

struct dev_esc_ops {
    dev_esc_set_arm_t set_arm;
    dev_esc_set_throttle_t set_throttle;
    dev_esc_get_telemetry_t get_telemetry;
    dev_esc_update_t update;
};

struct dev_esc {
    const void *config;
    void *data;
    const struct dev_esc_ops *ops;
};

#endif // ULYSSES_SVC_DEV_ESC_PRIVATE
