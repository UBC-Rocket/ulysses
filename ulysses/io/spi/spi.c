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
    ULYSSES_ASSERT(len != 0, "SPI transfer length must be non zero");

    return device->transfer(device, tx_data, rx_data, len, timeout_ms);
}
