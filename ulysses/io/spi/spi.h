#ifndef ULYSSES_IO_SPI_PRIVATE
#define ULYSSES_IO_SPI_PRIVATE

#include <stdint.h>

#include "ulysses/io/spi.h"

// Platform functions, see the matching io_spi_*() functions
typedef enum io_status (*io_spi_transfer_fn_t)(io_spi_dev_t *dev, const uint8_t *tx_data,
                                               uint8_t *rx_data, uint16_t len, uint32_t timeout_ms);
typedef enum io_status (*io_spi_transmit_fn_t)(io_spi_dev_t *dev, const uint8_t *tx_data,
                                               uint16_t len, uint32_t timeout_ms);
typedef enum io_status (*io_spi_receive_fn_t)(io_spi_dev_t *dev, uint8_t *rx_data, uint16_t len,
                                              uint32_t timeout_ms);

struct io_spi_ops {
    io_spi_transfer_fn_t transfer; // platform transfer function
    io_spi_transmit_fn_t transmit; // platform transmit function
    io_spi_receive_fn_t receive;   // platform receive function
};

struct io_spi_dev {
    const void *config;           // platform config
    const struct io_spi_ops *ops; // platform functions
};

#endif // ULYSSES_IO_SPI_PRIVATE
