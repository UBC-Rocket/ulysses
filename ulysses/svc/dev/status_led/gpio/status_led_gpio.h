#ifndef ULYSSES_SVC_DEV_STATUS_LED_GPIO_STATUS_LED_GPIO_H
#define ULYSSES_SVC_DEV_STATUS_LED_GPIO_STATUS_LED_GPIO_H

#include "ulysses/io/gpio.h"
#include "ulysses/svc/dev/status_led.h"

/** @brief Wiring of a status LED driven by a GPIO pin */
struct dev_status_led_gpio_config {
    const io_gpio_dev_t *gpio; ///< GPIO pin the LED is connected to
};

/**
 * @brief Initializes a status LED driven by a GPIO pin
 *
 * The LED is on when the pin is high. It starts with DEV_STATUS_LED_PATTERN_OFF.
 *
 * @param[out] led LED to initialize
 * @param config Wiring of the LED, must stay valid while the LED is used
 */
void dev_status_led_gpio_init(dev_status_led_t *led,
                              const struct dev_status_led_gpio_config *config);

#endif // ULYSSES_SVC_DEV_STATUS_LED_GPIO_STATUS_LED_GPIO_H
