#ifndef ULYSSES_IO_UART_PRIVATE
#define ULYSSES_IO_UART_PRIVATE

#include "ulysses/io/uart.h"

enum io_uart_async_mode {
    IO_UART_DMA = 0, ///< Use DMA for asynchronous transfers
    IO_UART_IRQ      ///< Use interrupts for asynchronous transfers
};

enum io_uart_receive_mode {
    IO_UART_PACKET = 0, ///< Fixed-length packet reception
    IO_UART_CONTINUOUS  ///< Continuous reception
};

typedef struct {
    enum io_status (*transmit)(const io_uart_t *uart, const uint8_t *data, uint16_t length,
                               uint32_t timeout_ms);
    enum io_status (*receive)(const io_uart_t *uart, uint8_t *data, uint16_t length,
                              uint32_t timeout_ms);
    enum io_status (*transmit_async)(const io_uart_t *uart, const uint8_t *data, uint16_t length);
    enum io_status (*receive_async)(const io_uart_t *uart, uint8_t *data, uint16_t length);
    enum io_status (*receive_continuous)(const io_uart_t *uart, uint8_t *ring_buf, uint16_t length);
} io_uart_methods_t;

struct io_uart {
    void *config;                     ///< Caller-provided backend configuration storage
    const io_uart_methods_t *methods; ///< Shared backend operation table
};

#endif // ULYSSES_IO_UART_PRIVATE
