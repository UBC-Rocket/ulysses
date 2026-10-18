#ifndef ULYSSES_IO_SPI
#define ULYSSES_IO_SPI

#include <stddef.h>
#include <stdint.h>

#include "ulysses/io/status.h"

// SPI device, set up by a platform init function
typedef struct io_spi_device io_spi_device_t;

/**
 * @brief Sends and receives bytes with an SPI device, blocking until done
 *
 * @param device Device to talk to
 * @param tx Bytes to send
 * @param rx Buffer for the received bytes, or NULL to discard them
 * @param len Number of bytes to send and receive
 * @param timeout_ms Timeout in milliseconds
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status io_spi_transfer(io_spi_device_t *device, const uint8_t *tx, uint8_t *rx, size_t len,
                               uint32_t timeout_ms);

#endif // ULYSSES_IO_SPI
