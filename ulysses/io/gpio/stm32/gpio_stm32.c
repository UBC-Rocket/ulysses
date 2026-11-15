#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/gpio.h"

#include "gpio/stm32/gpio_stm32.h"

static void stm32_write(const io_gpio_dev_t *dev, enum io_gpio_state state)
{
    const struct io_gpio_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->port, config->pin,
                      state == IO_GPIO_STATE_SET ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static const struct io_gpio_ops stm32_gpio_ops = {
    .write = stm32_write,
};

void io_gpio_stm32_init(io_gpio_dev_t *dev, const struct io_gpio_stm32_config *config)
{
    ULYSSES_ASSERT(dev != NULL, "GPIO device must be non NULL");
    ULYSSES_ASSERT(config != NULL, "GPIO config must be non NULL");
    ULYSSES_ASSERT(config->port != NULL, "GPIO port must be non NULL");
    ULYSSES_ASSERT(config->pin != 0, "GPIO pin must be non zero");

    dev->config = config;
    dev->ops = &stm32_gpio_ops;
}
