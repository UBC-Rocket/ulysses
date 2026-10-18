#ifndef ULYSSES_IO_SPI_STM32
#define ULYSSES_IO_SPI_STM32

#include <stdint.h>

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "ulysses/io/spi.h"

#include "../spi.h"

// STM32 wiring of an SPI device
struct io_spi_stm32_config {
    SPI_HandleTypeDef *hw_handle; // HAL SPI handle
    GPIO_TypeDef *cs_port;        // chip-select GPIO port
    uint16_t cs_pin;              // chip-select GPIO pin
};

/**
 * @brief Initializes an SPI device on an STM32
 *
 * @param device Device to initialize
 * @param config Wiring of the device, must stay valid while the device is used
 */
void io_spi_stm32_init(io_spi_device_t *device, const struct io_spi_stm32_config *config);

#endif // ULYSSES_IO_SPI_STM32
