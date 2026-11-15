#ifndef ULYSSES_IO_GPIO_GPIO_H
#define ULYSSES_IO_GPIO_GPIO_H

#include "ulysses/io/gpio.h"

// Platform function, see io_gpio_write()
typedef void (*io_gpio_write_fn_t)(const io_gpio_dev_t *dev, enum io_gpio_state state);

struct io_gpio_ops {
    io_gpio_write_fn_t write; ///< Platform write function
};

struct io_gpio_dev {
    const void *config;            ///< Platform config
    const struct io_gpio_ops *ops; ///< Platform functions
};

#endif // ULYSSES_IO_GPIO_GPIO_H
