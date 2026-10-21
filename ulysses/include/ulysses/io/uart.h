#ifndef ULYSSES_IO_UART
#define ULYSSES_IO_UART

#include <stdint.h>

#include "ulysses/io/status.h"

/**
 * @brief Opaque UART object
 */
typedef struct io_uart io_uart_t;

/**
 * @brief UART asynchronous transmit callback 
 */
typedef void (*io_uart_transmit_cb_t)(void);

/**
 * @brief UART asynchronous receive callback
 */
typedef void (*io_uart_receive_cb_t)(void);

/**
 * @brief Transmits data synchronously
 *
 * @param uart Initialized UART object
 * @param data Data to transmit
 * @param length Number of bytes to transmit
 * @param timeout_ms Timeout in milliseconds
 *
 * @retval IO_STATUS_OK The transfer completed, or length was zero
 * @retval IO_STATUS_BUSY A transmit operation is already in progress
 * @retval IO_STATUS_TIMEOUT The operation timed out
 * @retval IO_STATUS_ERROR The backend reported a transfer error
 */
enum io_status io_uart_transmit(const io_uart_t *uart, const uint8_t *data, uint16_t length,
                                uint32_t timeout_ms);

/**
 * @brief Receives data synchronously
 *
 * A failed or timed-out transfer may have written part of the receive buffer.
 *
 * @param uart Initialized UART object in packet receive mode
 * @param[out] data Buffer to store received data
 * @param length Number of bytes to receive
 * @param timeout_ms Timeout in milliseconds
 *
 * @retval IO_STATUS_OK The transfer completed, or length was zero
 * @retval IO_STATUS_BUSY A receive operation is already in progress
 * @retval IO_STATUS_TIMEOUT The operation timed out
 * @retval IO_STATUS_ERROR The backend reported a transfer error
 */
enum io_status io_uart_receive(const io_uart_t *uart, uint8_t *data, uint16_t length,
                               uint32_t timeout_ms);

/**
 * @brief Starts an asynchronous transmit operation
 *
 * @param uart Initialized UART object
 * @param data Data to transmit
 * @param length Number of bytes to transmit
 *
 * @retval IO_STATUS_OK The transfer was started, or length was zero
 * @retval IO_STATUS_BUSY A transmit operation is already in progress
 * @retval IO_STATUS_ERROR The operation could not be started
 */
enum io_status io_uart_transmit_async(const io_uart_t *uart, const uint8_t *data, uint16_t length);

/**
 * @brief Starts an asynchronous fixed-length receive operation
 *
 * @param uart Initialized UART object in packet receive mode
 * @param[out] data Buffer to store received data
 * @param length Number of bytes to receive
 *
 * @retval IO_STATUS_OK The receive operation was started, or length was zero
 * @retval IO_STATUS_BUSY A receive operation is already in progress
 * @retval IO_STATUS_ERROR The operation could not be started
 */
enum io_status io_uart_receive_async(const io_uart_t *uart, uint8_t *data, uint16_t length);

/**
 * @brief Starts continuous reception
 *
 * @param uart Initialized UART object in continuous receive mode
 * @param[out] ring_buf Ring buffer to store received data
 * @param length Buffer capacity in bytes
 *
 * @retval IO_STATUS_OK The receive operation was started
 * @retval IO_STATUS_BUSY A receive operation is already in progress
 * @retval IO_STATUS_ERROR The operation could not be started
 */
enum io_status io_uart_receive_continuous(const io_uart_t *uart, uint8_t *ring_buf,
                                          uint16_t length);

#endif // ULYSSES_IO_UART
