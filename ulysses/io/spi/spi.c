#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/spi.h"

#include "spi/spi.h"

enum io_status io_spi_transfer(io_spi_device_t *device, const uint8_t *tx_data, uint8_t *rx_data,
                               uint16_t len, uint32_t timeout_ms)
{
    ULYSSES_ASSERT(device != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(device->transfer != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(rx_data != NULL, "SPI rx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transfer length must be non zero");

    return device->transfer(device, tx_data, rx_data, len, timeout_ms);
}

enum io_status io_spi_transmit(io_spi_device_t *device, const uint8_t *tx_data, uint16_t len,
                               uint32_t timeout_ms)
{
    ULYSSES_ASSERT(device != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(device->transmit != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transmit length must be non zero");

    return device->transmit(device, tx_data, len, timeout_ms);
}

enum io_status io_spi_receive(io_spi_device_t *device, uint8_t *rx_data, uint16_t len,
                              uint32_t timeout_ms)
{
    ULYSSES_ASSERT(device != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(device->receive != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(rx_data != NULL, "SPI rx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI receive length must be non zero");

    return device->receive(device, rx_data, len, timeout_ms);
}
