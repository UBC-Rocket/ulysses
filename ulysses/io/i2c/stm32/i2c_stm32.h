#ifndef ULYSSES_IO_I2C_STM32_I2C_STM32
#define ULYSSES_IO_I2C_STM32_I2C_STM32

#include "stm32h7xx_hal.h"

#include "ulysses/io/i2c.h"

/** @brief Configuration for the STM32 HAL I2C backend */
struct io_i2c_stm32_config {
    I2C_HandleTypeDef *hw_handle; ///< Non-NULL, initialized HAL handle; must outlive bus use
};

/**
 * @brief Links an I2C bus to an already initialized STM32 HAL handle
 *
 * Stores config by reference; config and its HAL handle must remain valid while the bus is used.
 * Registers the bus for the handle's transfer complete and error IRQ events, so async transfers
 * need the peripheral's event and error interrupts enabled. Call once per bus.
 *
 * @param[out] bus Non-NULL bus to configure
 * @param config Non-NULL configuration whose HAL handle is in HAL_I2C_STATE_READY
 */
void io_i2c_stm32_init(io_i2c_bus_t *bus, const struct io_i2c_stm32_config *config);

#endif // ULYSSES_IO_I2C_STM32_H
