#ifndef ULYSSES_IO_SPI
#define ULYSSES_IO_SPI

#include <stdint.h>

#include "ulysses/io/status.h"

typedef struct io_spi_device io_spi_device_t;

/**
 * @brief Sends and receives bytes with an SPI device at the same time, blocking until done
 *
 * @param device Device to talk to
 * @param tx_data Bytes to send
 * @param[out] rx_data Buffer for the received bytes
 * @param len Number of bytes to send and receive
 * @param timeout_ms Timeout in milliseconds
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status io_spi_transfer(io_spi_device_t *device, const uint8_t *tx_data, uint8_t *rx_data,
                               uint16_t len, uint32_t timeout_ms);

/**
 * @brief Sends bytes to an SPI device, blocking until done
 *
 * @param device Device to talk to
 * @param tx_data Bytes to send
 * @param len Number of bytes to send
 * @param timeout_ms Timeout in milliseconds
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status io_spi_transmit(io_spi_device_t *device, const uint8_t *tx_data, uint16_t len,
                               uint32_t timeout_ms);

/**
 * @brief Receives bytes from an SPI device, blocking until done
 *
 * @param device Device to talk to
 * @param[out] rx_data Buffer for the received bytes
 * @param len Number of bytes to receive
 * @param timeout_ms Timeout in milliseconds
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status io_spi_receive(io_spi_device_t *device, uint8_t *rx_data, uint16_t len,
                              uint32_t timeout_ms);

#endif // ULYSSES_IO_SPI
