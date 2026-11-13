#include "ulysses/lib/assert.h"

#include "uart/stm32/uart_dma_stm32.h"
#include "uart/stm32/uart_stm32.h"

enum io_status io_uart_stm32_transmit_dma(const io_uart_dev_t *uart, const uint8_t *data,
                                          uint16_t length)
{
    struct uart_config *config = (struct uart_config *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    ULYSSES_ASSERT(config->hw_handle->hdmatx != NULL, "uart transmit DMA handle must not be NULL");

    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    enum io_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_TRANSMIT);
    if (status == IO_STATUS_OK) {
        HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(config->hw_handle, data, length);
        if (hal_status == HAL_OK) {
            config->pending |= UART_STM32_TRANSMIT;
            __HAL_DMA_DISABLE_IT(config->hw_handle->hdmatx, DMA_IT_HT);
        }
        status = io_uart_stm32_to_io_status(hal_status);
    }

    __set_PRIMASK(interrupt_mask);
    return status;
}

static enum io_status start_receive_dma(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->hw_handle->hdmarx != NULL, "uart receive DMA handle must not be NULL");

    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    enum io_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_RECEIVE);
    if (status == IO_STATUS_OK) {
        HAL_StatusTypeDef hal_status = HAL_UART_Receive_DMA(config->hw_handle, data, length);
        if (hal_status == HAL_OK) {
            config->pending |= UART_STM32_RECEIVE;
            __HAL_DMA_DISABLE_IT(config->hw_handle->hdmarx, DMA_IT_HT);
            if (config->receive_mode == IO_UART_CONTINUOUS) {
                __HAL_UART_ENABLE_IT(config->hw_handle, UART_IT_CM);
            }
        }
        status = io_uart_stm32_to_io_status(hal_status);
    }

    __set_PRIMASK(interrupt_mask);
    return status;
}

enum io_status io_uart_stm32_receive_dma(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }
    return start_receive_dma(uart, data, length);
}

enum io_status io_uart_stm32_receive_continuous_dma(const io_uart_dev_t *uart, uint8_t *data,
                                                    uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->duplex_mode == IO_UART_FULL_DUPLEX,
                   "continuous receive requires full duplex");
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    return start_receive_dma(uart, data, length);
}

void io_uart_stm32_transmit_complete_dma(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (!(config->pending & UART_STM32_TRANSMIT)) {
        return;
    }

    // on error, defer notification until abort completes
    if (HAL_DMA_GetError(config->hw_handle->hdmatx) == HAL_DMA_ERROR_NONE &&
        !io_uart_stm32_dma_abort_pending(config)) {
        io_uart_stm32_finish_transmit(uart, IO_STATUS_OK);
    }
}

void io_uart_stm32_receive_complete_dma(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (!(config->pending & UART_STM32_RECEIVE)) {
        return;
    }

    // on error, defer notification until abort completes
    if (HAL_DMA_GetError(config->hw_handle->hdmarx) == HAL_DMA_ERROR_NONE &&
        !io_uart_stm32_dma_abort_pending(config)) {
        io_uart_stm32_finish_receive(uart, IO_STATUS_OK);
    }
}
