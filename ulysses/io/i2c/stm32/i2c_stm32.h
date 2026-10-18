#ifndef ULYSSES_IO_I2C_STM32_I2C_STM32
#define ULYSSES_IO_I2C_STM32_I2C_STM32

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "ulysses/io/i2c.h"

// STM32 wiring of an I2C bus
struct io_i2c_stm32_config {
    I2C_HandleTypeDef *hw_handle; // HAL I2C handle, initialised by CubeMX
};

// Links bus to its HAL handle. config must stay valid while the bus is used.
void io_i2c_stm32_init(io_i2c_bus_t *bus, const struct io_i2c_stm32_config *config);

#endif // ULYSSES_IO_I2C_STM32_I2C_STM32
