#ifndef ULYSSES_IO_UART_STM32
#define ULYSSES_IO_UART_STM32

#include <stdbool.h>

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "uart/uart.h"

enum io_uart_stm32_transfer_direction {
    UART_STM32_TRANSMIT = (1U << 0),
    UART_STM32_RECEIVE = (1U << 1),
};

/**
 * @brief STM32 UART configuration
 */
struct uart_config {
    UART_HandleTypeDef *hw_handle;          ///< Initialized HAL UART handle
    enum io_uart_async_mode async_mode;     ///< Asynchronous transfer implementation
    enum io_uart_receive_mode receive_mode; ///< Requested receive mode
    enum io_uart_duplex_mode duplex_mode;   ///< Full-duplex or single-wire half-duplex operation
    volatile io_uart_transmit_cb_t transmit_cb; ///< Callback for transmit-complete events
    volatile io_uart_receive_cb_t receive_cb;   ///< Callback for receive-complete events
    volatile uint8_t pending;                   ///< Async directions awaiting completion
};

/**
 * @brief Initializes a UART on STM32
 *
 * The HAL UART must already be initialized and idle. GPIO configuration and wiring must match
 * the requested duplex mode; half-duplex uses the transmit pin for both directions.
 * Fixed-length DMA transfers require an initialized DMA handle in normal mode for that direction.
 *
 * @param uart UART object with caller-provided configuration storage
 * @param uart_config Initialization settings
 */
void io_uart_stm32_init(io_uart_dev_t *uart, const struct uart_config *uart_config);

bool io_uart_stm32_dma_abort_pending(const struct uart_config *config);

enum io_status io_uart_stm32_prepare_transfer(const struct uart_config *config,
                                              enum io_uart_stm32_transfer_direction direction);

enum io_status io_uart_stm32_to_io_status(HAL_StatusTypeDef status);

void io_uart_stm32_finish_transmit(const io_uart_dev_t *uart, enum io_status status);

void io_uart_stm32_finish_receive(const io_uart_dev_t *uart, enum io_status status);

#endif // ULYSSES_IO_UART_STM32
