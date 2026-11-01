#ifndef ULYSSES_IO_SPI
#define ULYSSES_IO_SPI

#include <stdint.h>

#include "ulysses/io/status.h"

/** @brief SPI device identified by its peripheral and chip select pin */
typedef struct io_spi_dev io_spi_dev_t;

/** @brief Completion callback for a non-blocking transfer, runs in interrupt context */
typedef void (*io_spi_cb_t)(enum io_status status, void *user_data);

/**
 * @brief Sends and receives bytes with an SPI device at the same time, blocking until done
 *
 * @param dev Device to talk to
 * @param tx_data Bytes to send
 * @param[out] rx_data Buffer for the received bytes
 * @param len Number of bytes to send and receive
 * @param timeout_ms Timeout in milliseconds
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status io_spi_transfer(io_spi_dev_t *dev, const uint8_t *tx_data, uint8_t *rx_data,
                               uint16_t len, uint32_t timeout_ms);

/**
 * @brief Sends bytes to an SPI device, blocking until done
 *
 * @param dev Device to talk to
 * @param tx_data Bytes to send
 * @param len Number of bytes to send
 * @param timeout_ms Timeout in milliseconds
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status io_spi_transmit(io_spi_dev_t *dev, const uint8_t *tx_data, uint16_t len,
                               uint32_t timeout_ms);

/**
 * @brief Receives bytes from an SPI device, blocking until done
 *
 * @param dev Device to talk to
 * @param[out] rx_data Buffer for the received bytes
 * @param len Number of bytes to receive
 * @param timeout_ms Timeout in milliseconds
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status io_spi_receive(io_spi_dev_t *dev, uint8_t *rx_data, uint16_t len,
                              uint32_t timeout_ms);

/**
 * @brief Starts sending and receiving bytes with an SPI device at the same time, without blocking
 *
 * cb runs in interrupt context when the transfer finishes. It is not called if the transfer fails
 * to start.
 *
 * @param dev Device to talk to
 * @param tx_data Bytes to send, must stay valid until cb is called
 * @param[out] rx_data Buffer for the received bytes, must stay valid until cb is called
 * @param len Number of bytes to send and receive
 * @param cb Called when the transfer finishes
 * @param user_data Passed to cb
 *
 * @return IO_STATUS_OK if the transfer started, IO_STATUS_BUSY if the device is in use, otherwise
 *         an IO_STATUS_* error code
 */
enum io_status io_spi_transfer_it(io_spi_dev_t *dev, const uint8_t *tx_data, uint8_t *rx_data,
                                  uint16_t len, io_spi_cb_t cb, void *user_data);

/**
 * @brief Starts sending bytes to an SPI device, without blocking
 *
 * cb runs in interrupt context when the transfer finishes. It is not called if the transfer fails
 * to start.
 *
 * @param dev Device to talk to
 * @param tx_data Bytes to send, must stay valid until cb is called
 * @param len Number of bytes to send
 * @param cb Called when the transfer finishes
 * @param user_data Passed to cb
 *
 * @return IO_STATUS_OK if the transfer started, IO_STATUS_BUSY if the device is in use, otherwise
 *         an IO_STATUS_* error code
 */
enum io_status io_spi_transmit_it(io_spi_dev_t *dev, const uint8_t *tx_data, uint16_t len,
                                  io_spi_cb_t cb, void *user_data);

/**
 * @brief Starts receiving bytes from an SPI device, without blocking
 *
 * cb runs in interrupt context when the transfer finishes. It is not called if the transfer fails
 * to start.
 *
 * @param dev Device to talk to
 * @param[out] rx_data Buffer for the received bytes, must stay valid until cb is called
 * @param len Number of bytes to receive
 * @param cb Called when the transfer finishes
 * @param user_data Passed to cb
 *
 * @return IO_STATUS_OK if the transfer started, IO_STATUS_BUSY if the device is in use, otherwise
 *         an IO_STATUS_* error code
 */
enum io_status io_spi_receive_it(io_spi_dev_t *dev, uint8_t *rx_data, uint16_t len, io_spi_cb_t cb,
                                 void *user_data);

#endif // ULYSSES_IO_SPI
