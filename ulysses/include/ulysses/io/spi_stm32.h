// STM32 HAL backend for the SPI driver.
//
// Only board init code should include this header. Everything above the IO
// layer uses ulysses/io/spi.h and never sees HAL types.

#ifndef ULYSSES_IO_SPI_STM32
#define ULYSSES_IO_SPI_STM32

#include <stdint.h>

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "ulysses/io/spi.h"

// Wiring of one SPI device on an STM32.
//
// The caller owns this and must keep it alive as long as the device is used,
// e.g. by making it static const.
struct io_spi_stm32_config {
    SPI_HandleTypeDef *hw_handle; // HAL SPI handle the device is wired to
    GPIO_TypeDef *cs_port;        // GPIO port of the chip-select pin
    uint16_t cs_pin;              // chip-select pin within cs_port
};

// Fill in a device so it talks through the STM32 HAL.
//
// Call once per device at startup, before any io_spi_transfer() on it.
//
// device: memory for the device, provided by the caller. Must not be NULL.
// config: wiring of the device. Must not be NULL.
//
// The chip-select pin is driven high (device not selected) before returning.
void io_spi_stm32_init(io_spi_device_t *device, const struct io_spi_stm32_config *config);

#endif // ULYSSES_IO_SPI_STM32
