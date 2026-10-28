#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/spi.h"

#include "spi/spi.h"

enum io_status io_spi_transfer(io_spi_dev_t *dev, const uint8_t *tx_data, uint8_t *rx_data,
                               uint16_t len, uint32_t timeout_ms)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->transfer != NULL, "SPI transfer function must be non NULL");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(rx_data != NULL, "SPI rx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transfer length must be non zero");

    return dev->ops->transfer(dev, tx_data, rx_data, len, timeout_ms);
}

enum io_status io_spi_transmit(io_spi_dev_t *dev, const uint8_t *tx_data, uint16_t len,
                               uint32_t timeout_ms)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->transmit != NULL, "SPI transmit function must be non NULL");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transmit length must be non zero");

    return dev->ops->transmit(dev, tx_data, len, timeout_ms);
}

enum io_status io_spi_receive(io_spi_dev_t *dev, uint8_t *rx_data, uint16_t len,
                              uint32_t timeout_ms)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->receive != NULL, "SPI receive function must be non NULL");
    ULYSSES_ASSERT(rx_data != NULL, "SPI rx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI receive length must be non zero");

    return dev->ops->receive(dev, rx_data, len, timeout_ms);
}
