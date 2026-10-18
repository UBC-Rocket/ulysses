#include "ulysses/io/uart.h"

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t cm_byte;
    io_uart_transmit_cb_t transmit_cb;
    io_uart_receive_cb_t receive_cb;
} uart_config_t;
