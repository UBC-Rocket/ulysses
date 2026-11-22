#include <stddef.h>

#include "ulysses/io/gpio.h"
#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/status_led.h"

#include "dev/status_led/gpio/status_led_gpio.h"
#include "dev/status_led/status_led.h"

static void gpio_on(const dev_status_led_t *led)
{
    const struct dev_status_led_gpio_config *config = led->config;

    io_gpio_write(config->gpio, IO_GPIO_STATE_SET);
}

static void gpio_off(const dev_status_led_t *led)
{
    const struct dev_status_led_gpio_config *config = led->config;

    io_gpio_write(config->gpio, IO_GPIO_STATE_RESET);
}

static const struct dev_status_led_ops gpio_ops = {
    .on = gpio_on,
    .off = gpio_off,
};

void dev_status_led_gpio_init(dev_status_led_t *led,
                              const struct dev_status_led_gpio_config *config)
{
    ULYSSES_ASSERT(led != NULL, "Status LED must be non NULL");
    ULYSSES_ASSERT(config != NULL, "Status LED config must be non NULL");
    ULYSSES_ASSERT(config->gpio != NULL, "Status LED GPIO pin must be non NULL");

    led->config = config;
    led->ops = &gpio_ops;
}
