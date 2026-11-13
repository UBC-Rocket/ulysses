#include "ulysses/lib/assert.h"

#include "uart/stm32/uart_irq_stm32.h"
#include "uart/stm32/uart_stm32.h"

enum io_status io_uart_stm32_transmit_irq(const io_uart_dev_t *uart, const uint8_t *data,
                                          uint16_t length)
{
    struct uart_config *config = (struct uart_config *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    enum io_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_TRANSMIT);
    if (status == IO_STATUS_OK) {
        HAL_StatusTypeDef hal_status = HAL_UART_Transmit_IT(config->hw_handle, data, length);
        if (hal_status == HAL_OK) {
            config->pending |= UART_STM32_TRANSMIT;
        }
        status = io_uart_stm32_to_io_status(hal_status);
    }

    __set_PRIMASK(interrupt_mask);
    return status;
}

static enum io_status start_receive_irq(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    struct uart_config *config = (struct uart_config *)uart->config;

    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    enum io_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_RECEIVE);
    if (status == IO_STATUS_OK) {
        HAL_StatusTypeDef hal_status = HAL_UART_Receive_IT(config->hw_handle, data, length);
        if (hal_status == HAL_OK) {
            config->pending |= UART_STM32_RECEIVE;
        }
        status = io_uart_stm32_to_io_status(hal_status);
    }

    __set_PRIMASK(interrupt_mask);
    return status;
}

enum io_status io_uart_stm32_receive_irq(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }
    return start_receive_irq(uart, data, length);
}

enum io_status io_uart_stm32_receive_continuous_irq(const io_uart_dev_t *uart, uint8_t *data,
                                                    uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->duplex_mode == IO_UART_FULL_DUPLEX,
                   "continuous receive requires full duplex");
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    // TODO: rearm IRQ receive before notifying the user
    return start_receive_irq(uart, data, length);
}

void io_uart_stm32_transmit_complete_irq(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (config->pending & UART_STM32_TRANSMIT) {
        io_uart_stm32_finish_transmit(uart, IO_STATUS_OK);
    }
}

void io_uart_stm32_receive_complete_irq(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    // on error, defer notification until abort completes
    if (config->pending & UART_STM32_RECEIVE &&
        config->hw_handle->ErrorCode == HAL_UART_ERROR_NONE) {
        io_uart_stm32_finish_receive(uart, IO_STATUS_OK);
    }
}
