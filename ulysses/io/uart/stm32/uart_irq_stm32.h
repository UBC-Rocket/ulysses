#ifndef ULYSSES_IO_UART_IRQ_STM32
#define ULYSSES_IO_UART_IRQ_STM32

#include "ulysses/io/uart.h"

enum io_status io_uart_stm32_transmit_irq(const io_uart_dev_t *uart, const uint8_t *data,
                                          uint16_t length);
enum io_status io_uart_stm32_receive_irq(const io_uart_dev_t *uart, uint8_t *data, uint16_t length);

enum io_status io_uart_stm32_receive_continuous_irq(const io_uart_dev_t *uart, uint8_t *data,
                                                    uint16_t length);

void io_uart_stm32_transmit_complete_irq(void *uart);
void io_uart_stm32_receive_complete_irq(void *uart);

#endif // ULYSSES_IO_UART_IRQ_STM32
