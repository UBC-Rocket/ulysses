#include <stdbool.h>

#include "ulysses/io/uart.h"
#include "ulysses/lib/assert.h"

#include "usart.h"

struct io_uart {
    UART_HandleTypeDef *huart;
    uint8_t match_byte;
    bool streaming;
    uint8_t *data_buf;
    size_t ring_size;
    size_t scratch_size; // ring + scratch should be the total length of data_buf
    io_status_t (*send)(const io_uart_t *uart, const uint8_t *buf, uint16_t len,
                        uint32_t timeout_ms);
    io_status_t (*recv)(const io_uart_t *uart, const uint8_t *buf, uint16_t len,
                        uint32_t timeout_ms);
    io_status_t (*send_async)(const io_uart_t *uart, const uint8_t *buf, uint16_t len,
                              io_uart_send_cb cb);
    io_status_t (*recv_async)(const io_uart_t *uart, const uint8_t *buf, uint16_t len,
                              io_uart_recv_cb cb);
};

io_uart_t *io_uart_init(void);

io_status_t io_uart_send(const io_uart_t *uart, const uint8_t *buf, uint16_t len,
                         uint32_t timeout_ms)
{
    ULYSSES_ASSERT(uart, "uart is NULL");
    ULYSSES_ASSERT(uart->huart, "uart handle is NULL");

    if (len == 0) {
        return IO_OK;
    }

    return HAL_UART_Transmit(uart->huart, buf, len, timeout_ms) == HAL_OK ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_recv(const io_uart_t *uart, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    ULYSSES_ASSERT(uart, "uart is NULL");
    ULYSSES_ASSERT(uart->huart, "uart handle is NULL");

    if (len == 0) {
        return IO_OK;
    }

    return HAL_UART_Receive(uart->huart, buf, len, timeout_ms) == HAL_OK ? IO_OK : IO_ERR_TIMEOUT;
}

io_status_t io_uart_send_async_dma(const io_uart_t *uart, const uint8_t *buf, uint16_t len,
                                   io_uart_send_cb cb)
{
    // TODO
}
io_status_t io_uart_recv_async_dma(const io_uart_t *uart, io_uart_recv_cb cb)
{
    // TODO
}
