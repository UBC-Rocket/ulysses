#ifndef ULYSSES_IO_UART_STM32
#define ULYSSES_IO_UART_STM32

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "uart/uart.h"

/**
 * @brief STM32 internal UART configuration
 */
struct uart_config {
    UART_HandleTypeDef *hw_handle;          ///< Initialized HAL UART handle
    enum io_uart_async_mode async_mode;     ///< Asynchronous transfer implementation
    enum io_uart_receive_mode receive_mode; ///< Requested receive mode
    enum io_uart_duplex_mode duplex_mode;   ///< Full-duplex or single-wire half-duplex operation
    io_uart_transmit_cb_t transmit_cb;      ///< Callback for transmit-complete events
    io_uart_receive_cb_t receive_cb;        ///< Callback for receive-complete events
};

/**
 * @brief STM32-specific layer level UART initialization function
 *
 * The HAL UART must already be initialized and idle. GPIO configuration and wiring must match
 * the requested duplex mode; half-duplex uses the TX pin for both directions.
 *
 * @param uart UART object with caller-provided configuration storage
 * @param uart_config Initialization settings
 */
void io_uart_stm32_init(io_uart_dev_t *uart, const struct uart_config *uart_config);

#endif // ULYSSES_IO_UART_STM32
