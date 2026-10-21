#include "ulysses/lib/assert.h"

#include "uart/uart.h"

enum io_status io_uart_transmit(const io_uart_t *uart, const uint8_t *data, uint16_t length,
                                uint32_t timeout_ms)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config, "uart config must not be NULL");
    ULYSSES_ASSERT(uart->methods, "uart methods must not be NULL");

    return uart->methods->transmit(uart, data, length, timeout_ms);
}

enum io_status io_uart_receive(const io_uart_t *uart, uint8_t *data, uint16_t length,
                               uint32_t timeout_ms)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config, "uart config must not be NULL");
    ULYSSES_ASSERT(uart->methods, "uart methods must not be NULL");

    return uart->methods->receive(uart, data, length, timeout_ms);
}

enum io_status io_uart_transmit_async(const io_uart_t *uart, const uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config, "uart config must not be NULL");
    ULYSSES_ASSERT(uart->methods, "uart methods must not be NULL");

    return uart->methods->transmit_async(uart, data, length);
}

enum io_status io_uart_receive_async(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config, "uart config must not be NULL");
    ULYSSES_ASSERT(uart->methods, "uart methods must not be NULL");

    return uart->methods->receive_async(uart, data, length);
}

enum io_status io_uart_receive_continuous(const io_uart_t *uart, uint8_t *ring_buf, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config, "uart config must not be NULL");
    ULYSSES_ASSERT(uart->methods, "uart methods must not be NULL");

    return uart->methods->receive_continuous(uart, ring_buf, length);
}
