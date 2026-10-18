#ifndef ULYSSES_IO_UART
#define ULYSSES_IO_UART

#include <stddef.h>
#include <stdint.h>
#include <usart.h>

#include "ulysses/io/status.h"

enum io_uart_async_mode {
    IO_UART_DMA = 0,
    IO_UART_IRQ
};

enum io_uart_receive_mode {
    IO_UART_PACKET = 0,
    IO_UART_CONTINUOUS
};

typedef struct io_uart io_uart_t;

typedef void (*io_uart_transmit_cb_t)(void);
typedef void (*io_uart_receive_cb_t)(void);

void io_uart_init(io_uart_t *uart, UART_HandleTypeDef *huart, enum io_uart_async_mode async_mode,
                  enum io_uart_receive_mode receive_mode, uint8_t cm_byte,
                  io_uart_transmit_cb_t transmit_cb, io_uart_receive_cb_t receive_cb);

enum io_status io_uart_transmit(const io_uart_t *uart, const uint8_t *buf, uint16_t len,
                             uint32_t timeout_ms);

enum io_status io_uart_receive(const io_uart_t *uart, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

enum io_status io_uart_transmit_async(const io_uart_t *uart, const uint8_t *buf, uint16_t len);

enum io_status io_uart_receive_async(const io_uart_t *uart, uint8_t *buf, uint16_t len);

enum io_status io_uart_receive_continous(const io_uart_t *uart, uint8_t *buf, uint16_t len);

#endif // ULYSSES_IO_UART
