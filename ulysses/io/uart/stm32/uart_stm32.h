#ifndef ULYSSES_IO_UART_STM32
#define ULYSSES_IO_UART_STM32

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "uart/uart.h"

/**
 * @brief STM32 internal UART configuration
 */
typedef struct {
    UART_HandleTypeDef *huart;              ///< Initialized HAL UART handle
    enum io_uart_async_mode async_mode;     ///< Asynchronous transfer implementation
    enum io_uart_receive_mode receive_mode; ///< Requested receive mode
    uint8_t cm_byte;                        ///< Character-match byte
    io_uart_transmit_cb_t *transmit_cb;      ///< Callback for transmit-complete events
    io_uart_receive_cb_t *receive_cb;        ///< Callback for receive-complete events
} uart_config_t;

const io_uart_methods_t *stm32_uart_get_dma_methods(void);
const io_uart_methods_t *stm32_uart_get_irq_methods(void);

void stm32_uart_init(io_uart_t *uart, const uart_config_t *uart_config);

#endif // ULYSSES_IO_UART_STM32
