#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/spi.h"

#include "spi/spi.h"

static bool claim(io_spi_dev_t *dev)
{
    return !atomic_exchange(&dev->busy, true);
}

static void release(io_spi_dev_t *dev)
{
    dev->cb = NULL;
    dev->cb_user_data = NULL;
    // Cleared last, so a new owner's callback is never overwritten
    atomic_store(&dev->busy, false);
}

enum io_status io_spi_transfer(io_spi_dev_t *dev, const uint8_t *tx_data, uint8_t *rx_data,
                               uint16_t len, uint32_t timeout_ms)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->transfer != NULL, "SPI transfer function must be non NULL");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(rx_data != NULL, "SPI rx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transfer length must be non zero");

    if (!claim(dev)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = dev->ops->transfer(dev, tx_data, rx_data, len, timeout_ms);

    release(dev);

    return status;
}

enum io_status io_spi_transmit(io_spi_dev_t *dev, const uint8_t *tx_data, uint16_t len,
                               uint32_t timeout_ms)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->transmit != NULL, "SPI transmit function must be non NULL");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transmit length must be non zero");

    if (!claim(dev)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = dev->ops->transmit(dev, tx_data, len, timeout_ms);

    release(dev);

    return status;
}

enum io_status io_spi_receive(io_spi_dev_t *dev, uint8_t *rx_data, uint16_t len,
                              uint32_t timeout_ms)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->receive != NULL, "SPI receive function must be non NULL");
    ULYSSES_ASSERT(rx_data != NULL, "SPI rx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI receive length must be non zero");

    if (!claim(dev)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = dev->ops->receive(dev, rx_data, len, timeout_ms);

    release(dev);

    return status;
}

enum io_status io_spi_transfer_it(io_spi_dev_t *dev, const uint8_t *tx_data, uint8_t *rx_data,
                                  uint16_t len, io_spi_cb_t cb, void *user_data)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->transfer_it != NULL, "SPI transfer_it function must be non NULL");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(rx_data != NULL, "SPI rx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transfer length must be non zero");
    ULYSSES_ASSERT(cb != NULL, "SPI callback must be non NULL");

    if (!claim(dev)) {
        return IO_STATUS_BUSY;
    }

    dev->cb = cb;
    dev->cb_user_data = user_data;

    enum io_status status = dev->ops->transfer_it(dev, tx_data, rx_data, len);

    if (status != IO_STATUS_OK) {
        release(dev);
    }

    return status;
}

enum io_status io_spi_transmit_it(io_spi_dev_t *dev, const uint8_t *tx_data, uint16_t len,
                                  io_spi_cb_t cb, void *user_data)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "SPI device must be initialized");
    ULYSSES_ASSERT(dev->ops->transmit_it != NULL, "SPI transmit_it function must be non NULL");
    ULYSSES_ASSERT(tx_data != NULL, "SPI tx buffer must be non NULL");
    ULYSSES_ASSERT(len != 0, "SPI transmit length must be non zero");
    ULYSSES_ASSERT(cb != NULL, "SPI callback must be non NULL");

    if (!claim(dev)) {
        return IO_STATUS_BUSY;
    }

    dev->cb = cb;
    dev->cb_user_data = user_data;

    enum io_status status = dev->ops->transmit_it(dev, tx_data, len);

    if (status != IO_STATUS_OK) {
        release(dev);
    }

    return status;
}

void io_spi_complete(io_spi_dev_t *dev, enum io_status status)
{
    io_spi_cb_t cb = dev->cb;
    void *user_data = dev->cb_user_data;

    if (cb == NULL) {
        return;
    }

    release(dev);

    cb(status, user_data);
}
