#ifndef ULYSSES_IO_UART
#define ULYSSES_IO_UART

#include <stdint.h>

#include "ulysses/io/status.h"

typedef struct io_uart io_uart_t;

enum io_status io_uart_transmit(const io_uart_t *uart, const uint8_t *data, uint16_t length,
                                uint32_t timeout_ms);

enum io_status io_uart_receive(const io_uart_t *uart, uint8_t *data, uint16_t length,
                               uint32_t timeout_ms);

enum io_status io_uart_transmit_async(const io_uart_t *uart, const uint8_t *data, uint16_t length);

enum io_status io_uart_receive_async(const io_uart_t *uart, uint8_t *data, uint16_t length);

enum io_status io_uart_receive_continous(const io_uart_t *uart, uint8_t *data, uint16_t length);

#endif // ULYSSES_IO_UART
