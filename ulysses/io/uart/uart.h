#include "stm32h7xx_hal.h" // IWYU pragma: keep
#include "stm32h7xx_hal_uart.h"

#include "ulysses/io/uart.h"

typedef void (*io_uart_transmit_cb_t)(void);
typedef void (*io_uart_receive_cb_t)(void);

enum io_uart_async_mode {
    IO_UART_DMA = 0,
    IO_UART_IRQ
};

enum io_uart_receive_mode {
    IO_UART_PACKET = 0,
    IO_UART_CONTINUOUS
};

typedef struct {
    UART_HandleTypeDef *huart;
    enum io_uart_async_mode async_mode;
    enum io_uart_receive_mode receive_mode;
    uint8_t cm_byte;
    io_uart_transmit_cb_t transmit_cb;
    io_uart_receive_cb_t receive_cb;
} uart_config_t;

void io_uart_init(io_uart_t *uart, UART_HandleTypeDef *huart, enum io_uart_async_mode async_mode,
                  enum io_uart_receive_mode receive_mode, uint8_t cm_byte,
                  io_uart_transmit_cb_t transmit_cb, io_uart_receive_cb_t receive_cb);
