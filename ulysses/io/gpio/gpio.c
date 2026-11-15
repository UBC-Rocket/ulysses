#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/gpio.h"

#include "gpio/gpio.h"

void io_gpio_write(const io_gpio_dev_t *dev, enum io_gpio_state state)
{
    ULYSSES_ASSERT(dev != NULL, "GPIO device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "GPIO device must be initialized");
    ULYSSES_ASSERT(dev->ops->write != NULL, "GPIO write function must be non NULL");
    ULYSSES_ASSERT(state == IO_GPIO_STATE_RESET || state == IO_GPIO_STATE_SET,
                   "GPIO state must be IO_GPIO_STATE_RESET or IO_GPIO_STATE_SET");

    dev->ops->write(dev, state);
}
