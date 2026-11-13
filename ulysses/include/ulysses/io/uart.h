#ifndef ULYSSES_IO_UART
#define ULYSSES_IO_UART

#include <stdint.h>

#include "ulysses/io/status.h"

/**
 * @brief UART object
 *
 * Calls that start transmit or receive operations on the same UART must be serialized.
 * Full-duplex permits simultaneous transmit and receive. Half-duplex permits only one
 * direction at a time; starting another transmit or receive before completion returns
 * IO_STATUS_BUSY.
 */
typedef struct io_uart_dev io_uart_dev_t;

/**
 * @brief UART asynchronous transmit callback type
 *
 * Called once in interrupt context after transmission stops. IO_STATUS_OK means the last byte
 * has finished transmitting; IO_STATUS_ERROR means transmission failed and partial data may have
 * been transmitted. The buffer may be reused when the callback runs.
 *
 * @param status Transmit result
 * @param uart UART object that completed the transmit
 */
typedef void (*io_uart_transmit_cb_t)(enum io_status status, const io_uart_dev_t *uart);

/**
 * @brief UART asynchronous receive callback type
 *
 * For fixed-length reception, called once in interrupt context after reception stops.
 * IO_STATUS_OK means all requested bytes were received. IO_STATUS_ERROR means reception failed
 * and partial data may have been received. The buffer may be reused when the callback runs.
 *
 * @param status Receive result
 * @param uart UART object that completed the receive
 */
typedef void (*io_uart_receive_cb_t)(enum io_status status, const io_uart_dev_t *uart);

/**
 * @brief Set or unset transmit callback
 *
 * Configure the callback before starting a transmit and keep it unchanged until completion.
 *
 * @param uart Initialized UART object
 * @param cb Callback function, or NULL to disable transmit callbacks
 */
void io_uart_set_transmit_cb(const io_uart_dev_t *uart, io_uart_transmit_cb_t cb);

/**
 * @brief Set or unset receive callback
 *
 * Configure the callback before starting a receive and keep it unchanged until completion.
 *
 * @param uart Initialized UART object
 * @param cb Callback function, or NULL to disable receive callbacks
 */
void io_uart_set_receive_cb(const io_uart_dev_t *uart, io_uart_receive_cb_t cb);

/**
 * @brief Transmits data synchronously
 *
 * A failed or timed-out transmit may have sent only part of the data.
 *
 * @param uart Initialized UART object
 * @param data Data to transmit
 * @param length Number of bytes to transmit
 * @param timeout_ms Timeout in milliseconds
 *
 * @retval IO_STATUS_OK The transmit completed, or length was zero
 * @retval IO_STATUS_BUSY A transmit, or a receive in half-duplex mode, is in progress
 * @retval IO_STATUS_TIMEOUT The transmit timed out
 * @retval IO_STATUS_ERROR The transmit failed; some bytes may have been transmitted
 */
enum io_status io_uart_transmit(const io_uart_dev_t *uart, const uint8_t *data, uint16_t length,
                                uint32_t timeout_ms);

/**
 * @brief Receives data synchronously
 *
 * A failed or timed-out receive may have written part of the receive buffer.
 *
 * @param uart Initialized UART object in packet receive mode
 * @param[out] data Buffer to store received data
 * @param length Number of bytes to receive
 * @param timeout_ms Timeout in milliseconds
 *
 * @retval IO_STATUS_OK The receive completed, or length was zero
 * @retval IO_STATUS_BUSY A receive, or a transmit in half-duplex mode, is in progress
 * @retval IO_STATUS_TIMEOUT The receive timed out
 * @retval IO_STATUS_ERROR The receive failed; some bytes may have been received
 */
enum io_status io_uart_receive(const io_uart_dev_t *uart, uint8_t *data, uint16_t length,
                               uint32_t timeout_ms);

/**
 * @brief Starts an asynchronous transmit operation
 *
 * Keep data valid and unchanged until the transmit callback runs. A zero length returns
 * IO_STATUS_OK without starting a transmit or calling the callback. A failed start does not
 * call the callback. No timeout is applied to an asynchronous transmit.
 *
 * @param uart Initialized UART object
 * @param data Data to transmit
 * @param length Number of bytes to transmit
 *
 * @retval IO_STATUS_OK The transmit was started, or length was zero
 * @retval IO_STATUS_BUSY A transmit, or a receive in half-duplex mode, is in progress
 * @retval IO_STATUS_ERROR The transmit could not be started
 */
enum io_status io_uart_transmit_async(const io_uart_dev_t *uart, const uint8_t *data,
                                      uint16_t length);

/**
 * @brief Starts an asynchronous fixed-length receive operation
 *
 * Keep data valid and do not access it until the receive callback runs. A zero length returns
 * IO_STATUS_OK without starting a receive or calling the callback. A failed start does not
 * call the callback. No timeout is applied to an asynchronous receive.
 *
 * @param uart Initialized UART object in packet receive mode
 * @param[out] data Buffer to store received data
 * @param length Number of bytes to receive
 *
 * @retval IO_STATUS_OK The receive was started, or length was zero
 * @retval IO_STATUS_BUSY A receive, or a transmit in half-duplex mode, is in progress
 * @retval IO_STATUS_ERROR The receive could not be started
 */
enum io_status io_uart_receive_async(const io_uart_dev_t *uart, uint8_t *data, uint16_t length);

/**
 * @brief Starts continuous reception
 *
 * @param uart Initialized full-duplex UART object in continuous receive mode
 * @param[out] data Buffer to store received data
 * @param length Buffer capacity in bytes
 *
 * @retval IO_STATUS_OK The continuous receive was started
 * @retval IO_STATUS_BUSY A receive is already in progress
 * @retval IO_STATUS_ERROR The continuous receive could not be started
 */
enum io_status io_uart_receive_continuous(const io_uart_dev_t *uart, uint8_t *data,
                                          uint16_t length);

#endif // ULYSSES_IO_UART
