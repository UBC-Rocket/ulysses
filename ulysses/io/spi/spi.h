#ifndef ULYSSES_IO_SPI_PRIVATE
#define ULYSSES_IO_SPI_PRIVATE

#include <stdatomic.h>
#include <stdint.h>

#include "ulysses/io/spi.h"

// Platform functions, see the matching io_spi_*() functions
typedef enum io_status (*io_spi_transfer_fn_t)(io_spi_dev_t *dev, const uint8_t *tx_data,
                                               uint8_t *rx_data, uint16_t len, uint32_t timeout_ms);
typedef enum io_status (*io_spi_transmit_fn_t)(io_spi_dev_t *dev, const uint8_t *tx_data,
                                               uint16_t len, uint32_t timeout_ms);
typedef enum io_status (*io_spi_receive_fn_t)(io_spi_dev_t *dev, uint8_t *rx_data, uint16_t len,
                                              uint32_t timeout_ms);
typedef enum io_status (*io_spi_transfer_it_fn_t)(io_spi_dev_t *dev, const uint8_t *tx_data,
                                                  uint8_t *rx_data, uint16_t len);

struct io_spi_ops {
    io_spi_transfer_fn_t transfer;       // platform transfer function
    io_spi_transmit_fn_t transmit;       // platform transmit function
    io_spi_receive_fn_t receive;         // platform receive function
    io_spi_transfer_it_fn_t transfer_it; // platform non-blocking transfer function
};

struct io_spi_dev {
    const void *config;           // platform config
    const struct io_spi_ops *ops; // platform functions
    atomic_bool busy;             // a transfer is in progress
    io_spi_cb_t cb;               // completion callback of the current non-blocking transfer
    void *cb_user_data;           // passed to cb
};

/**
 * @brief Finishes the non-blocking transfer of a device and calls its callback
 *
 * Called by platform code from interrupt context. Does nothing if no non-blocking transfer is in
 * progress.
 *
 * @param dev Device whose transfer finished
 * @param status Result of the transfer
 */
void io_spi_complete(io_spi_dev_t *dev, enum io_status status);

#endif // ULYSSES_IO_SPI_PRIVATE
