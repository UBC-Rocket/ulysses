#include <stddef.h>

#include "ulysses/lib/assert.h"

#include "uart/uart.h"

static inline void assert_not_null(const io_uart_dev_t *uart)
{
    ULYSSES_ASSERT(uart != NULL, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config != NULL, "uart config must not be NULL");
    ULYSSES_ASSERT(uart->ops != NULL, "uart ops must not be NULL");
}

void io_uart_set_transmit_cb(const io_uart_dev_t *uart, io_uart_transmit_cb_t cb)
{
    assert_not_null(uart);
    uart->ops->set_transmit_cb(uart, cb);
}

void io_uart_set_receive_cb(const io_uart_dev_t *uart, io_uart_receive_cb_t cb)
{
    assert_not_null(uart);
    uart->ops->set_receive_cb(uart, cb);
}

enum io_status io_uart_transmit(const io_uart_dev_t *uart, const uint8_t *data, uint16_t length,
                                uint32_t timeout_ms)
{
    assert_not_null(uart);
    ULYSSES_ASSERT(length == 0 || data != NULL, "uart data must not be NULL for nonzero length");
    return uart->ops->transmit(uart, data, length, timeout_ms);
}

enum io_status io_uart_receive(const io_uart_dev_t *uart, uint8_t *data, uint16_t length,
                               uint32_t timeout_ms)
{
    assert_not_null(uart);
    ULYSSES_ASSERT(length == 0 || data != NULL, "uart data must not be NULL for nonzero length");
    return uart->ops->receive(uart, data, length, timeout_ms);
}

enum io_status io_uart_transmit_async(const io_uart_dev_t *uart, const uint8_t *data,
                                      uint16_t length)
{
    assert_not_null(uart);
    ULYSSES_ASSERT(length == 0 || data != NULL, "uart data must not be NULL for nonzero length");
    return uart->ops->transmit_async(uart, data, length);
}

enum io_status io_uart_receive_async(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    assert_not_null(uart);
    ULYSSES_ASSERT(length == 0 || data != NULL, "uart data must not be NULL for nonzero length");
    return uart->ops->receive_async(uart, data, length);
}

enum io_status io_uart_receive_continuous(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    assert_not_null(uart);
    ULYSSES_ASSERT(length == 0 || data != NULL, "uart data must not be NULL for nonzero length");
    return uart->ops->receive_continuous(uart, data, length);
}
