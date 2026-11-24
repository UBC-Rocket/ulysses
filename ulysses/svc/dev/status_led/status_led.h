#ifndef ULYSSES_SVC_DEV_STATUS_LED_STATUS_LED_H
#define ULYSSES_SVC_DEV_STATUS_LED_STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "ulysses/svc/dev/status_led.h"

// Implementation-specific functions that turn the LED on and off
typedef void (*dev_status_led_on_fn_t)(const dev_status_led_t *led);
typedef void (*dev_status_led_off_fn_t)(const dev_status_led_t *led);

struct dev_status_led_ops {
    dev_status_led_on_fn_t on;   ///< Implementation-specific on function
    dev_status_led_off_fn_t off; ///< Implementation-specific off function
};

struct dev_status_led {
    const void *config;                   ///< Implementation-specific configuration
    const struct dev_status_led_ops *ops; ///< Implementation-specific functions
    enum dev_status_led_pattern pattern;  ///< Current pattern
    bool on;                              ///< LED is on
    uint32_t change_ms;                   ///< Time the LED last turned on or off
};

#endif // ULYSSES_SVC_DEV_STATUS_LED_STATUS_LED_H
