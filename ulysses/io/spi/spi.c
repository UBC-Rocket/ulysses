#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/spi.h"

#include "spi.h"

enum io_spi_status io_spi_transfer(io_spi_device_t *device, const uint8_t *tx, uint8_t *rx,
                                   size_t len, uint32_t timeout_ms)
{
    ULYSSES_ASSERT(device != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(device->transfer != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(tx != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transfer length must be non zero");

    return device->transfer(device, tx, rx, len, timeout_ms);
}
