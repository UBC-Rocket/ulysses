#ifndef ULYSSES_SVC_DEV_STATUS_LED_STATUS_LED_H
#define ULYSSES_SVC_DEV_STATUS_LED_STATUS_LED_H

#include "ulysses/svc/dev/status_led.h"

// Implementation-specific functions, see the matching dev_status_led_*() functions
typedef void (*dev_status_led_on_fn_t)(const dev_status_led_t *led);
typedef void (*dev_status_led_off_fn_t)(const dev_status_led_t *led);

struct dev_status_led_ops {
    dev_status_led_on_fn_t on;   ///< Implementation-specific on function
    dev_status_led_off_fn_t off; ///< Implementation-specific off function
};

struct dev_status_led {
    const void *config;                   ///< Implementation-specific configuration
    const struct dev_status_led_ops *ops; ///< Implementation-specific functions
};

#endif // ULYSSES_SVC_DEV_STATUS_LED_STATUS_LED_H
