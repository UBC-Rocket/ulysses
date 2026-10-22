#ifndef ULYSSES_IO_UART_PRIVATE
#define ULYSSES_IO_UART_PRIVATE

#include "ulysses/io/uart.h"

enum io_uart_async_mode {
    IO_UART_DMA = 0, ///< Use DMA for asynchronous transfers
    IO_UART_IRQ,     ///< Use interrupts for asynchronous transfers
    IO_UART_ASYNC_MODE_COUNT
};

enum io_uart_receive_mode {
    IO_UART_PACKET = 0, ///< Fixed-length packet reception
    IO_UART_CONTINUOUS, ///< Continuous reception
    IO_UART_RECEIVE_MODE_COUNT
};

struct io_uart_ops {
    void (*set_transmit_cb)(const io_uart_t *uart, io_uart_transmit_cb_t cb);
    void (*set_receive_cb)(const io_uart_t *uart, io_uart_receive_cb_t cb);
    enum io_status (*transmit)(const io_uart_t *uart, const uint8_t *data, uint16_t length,
                               uint32_t timeout_ms);
    enum io_status (*receive)(const io_uart_t *uart, uint8_t *data, uint16_t length,
                              uint32_t timeout_ms);
    enum io_status (*transmit_async)(const io_uart_t *uart, const uint8_t *data, uint16_t length);
    enum io_status (*receive_async)(const io_uart_t *uart, uint8_t *data, uint16_t length);
    enum io_status (*receive_continuous)(const io_uart_t *uart, uint8_t *data, uint16_t length);
};

struct io_uart {
    const void *config;                ///< Caller-provided backend configuration storage
    const struct io_uart_ops *methods; ///< Shared backend operation table
};

#endif // ULYSSES_IO_UART_PRIVATE
