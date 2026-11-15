#ifndef ULYSSES_IO_GPIO_STM32_GPIO_STM32_H
#define ULYSSES_IO_GPIO_STM32_GPIO_STM32_H

#include <stdint.h>

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "ulysses/io/gpio.h"

#include "gpio/gpio.h"

// STM32 wiring of a GPIO pin
struct io_gpio_stm32_config {
    GPIO_TypeDef *port; ///< GPIO port
    uint16_t pin;       ///< GPIO pin
};

/**
 * @brief Initializes a GPIO pin on an STM32
 *
 * The pin must already be configured as an output.
 *
 * @param dev Pin to initialize
 * @param config Wiring of the pin, must stay valid while the pin is used
 */
void io_gpio_stm32_init(io_gpio_dev_t *dev, const struct io_gpio_stm32_config *config);

#endif // ULYSSES_IO_GPIO_STM32_GPIO_STM32_H
