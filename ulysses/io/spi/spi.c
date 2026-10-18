#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/spi.h"
#include "ulysses/io/spi_stm32.h"

void io_spi_stm32_init(io_spi_device_t *device, const struct io_spi_stm32_config *config)
{
    ULYSSES_ASSERT(device != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(config != NULL, "SPI config must be non NULL");
    ULYSSES_ASSERT(config->hw_handle != NULL, "SPI hardware handle must be non NULL");
    ULYSSES_ASSERT(config->cs_port != NULL, "SPI chip-select port must be non NULL");
    ULYSSES_ASSERT(config->cs_pin != 0, "SPI chip-select pin must be non zero");

    device->config = config;
    // TODO: set device->transfer once the STM32 transfer function exists

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);
}
