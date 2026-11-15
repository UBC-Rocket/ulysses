#ifndef ULYSSES_IO_GPIO
#define ULYSSES_IO_GPIO

/** @brief GPIO output pin */
typedef struct io_gpio_dev io_gpio_dev_t;

/** @brief Output level of a GPIO pin */
enum io_gpio_state {
    IO_GPIO_STATE_RESET, ///< Low
    IO_GPIO_STATE_SET,   ///< High
};

/**
 * @brief Drives a GPIO pin high or low
 *
 * @param dev Pin to drive
 * @param state Level to drive the pin to
 */
void io_gpio_write(const io_gpio_dev_t *dev, enum io_gpio_state state);

#endif // ULYSSES_IO_GPIO
