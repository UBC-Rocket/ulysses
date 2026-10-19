#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/spi.h"

#include "spi_stm32.h"

static enum io_status to_io_status(HAL_StatusTypeDef hal_status)
{
    switch (hal_status) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_TIMEOUT:
        return IO_STATUS_TIMEOUT;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

static enum io_status stm32_transfer(io_spi_device_t *device, const uint8_t *tx_data,
                                     uint8_t *rx_data, size_t len, uint32_t timeout_ms)
{
    // HAL takes the length as uint16_t
    ULYSSES_ASSERT(len <= UINT16_MAX, "SPI transfer length must fit in 16 bits");

    const struct io_spi_stm32_config *config = device->config;
    HAL_StatusTypeDef hal_status;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_RESET);

    if (rx_data != NULL) {
        hal_status =
            HAL_SPI_TransmitReceive(config->hw_handle, tx_data, rx_data, (uint16_t)len, timeout_ms);
    } else {
        hal_status = HAL_SPI_Transmit(config->hw_handle, tx_data, (uint16_t)len, timeout_ms);
    }

    // Release chip select even if the transfer failed
    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);

    return to_io_status(hal_status);
}

void io_spi_stm32_init(io_spi_device_t *device, const struct io_spi_stm32_config *config)
{
    ULYSSES_ASSERT(device != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(config != NULL, "SPI config must be non NULL");
    ULYSSES_ASSERT(config->hw_handle != NULL, "SPI hardware handle must be non NULL");
    ULYSSES_ASSERT(config->cs_port != NULL, "SPI chip-select port must be non NULL");
    ULYSSES_ASSERT(config->cs_pin != 0, "SPI chip-select pin must be non zero");

    device->config = config;
    device->transfer = stm32_transfer;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);
}
