#ifndef ULYSSES_IO_SPI_PRIVATE
#define ULYSSES_IO_SPI_PRIVATE

#include <stddef.h>
#include <stdint.h>

#include "ulysses/io/spi.h"

// Platform transfer function, see io_spi_transfer()
typedef enum io_status (*io_spi_transfer_fn_t)(io_spi_device_t *device, const uint8_t *tx_data,
                                               uint8_t *rx_data, size_t len, uint32_t timeout_ms);

struct io_spi_device {
    const void *config;            // platform config
    io_spi_transfer_fn_t transfer; // platform transfer function
};

#endif // ULYSSES_IO_SPI_PRIVATE
