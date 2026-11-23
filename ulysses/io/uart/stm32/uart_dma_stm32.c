#include "ulysses/lib/assert.h"

#include "uart/stm32/uart_dma_stm32.h"
#include "uart/stm32/uart_stm32.h"

enum uls_status io_uart_stm32_transmit_dma(const io_uart_dev_t *uart, const uint8_t *data,
                                           uint16_t length)
{
    struct uart_config *config = (struct uart_config *)uart->config;

    if (length == 0) {
        return ULS_STATUS_OK;
    }

    ULYSSES_ASSERT(config->hw_handle->hdmatx != NULL, "uart transmit DMA handle must not be NULL");

    enum uls_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_TRANSMIT);
    if (status == ULS_STATUS_OK) {
        config->transmit_pending = true;
        HAL_StatusTypeDef hal_status = HAL_UART_Transmit_DMA(config->hw_handle, data, length);
        if (hal_status == HAL_OK) {
            __HAL_DMA_DISABLE_IT(config->hw_handle->hdmatx, DMA_IT_HT);
        } else {
            config->transmit_pending = false;
        }
        status = io_uart_stm32_to_uls_status(hal_status);
    }

    return status;
}

static enum uls_status start_receive_dma(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->hw_handle->hdmarx != NULL, "uart receive DMA handle must not be NULL");

    enum uls_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_RECEIVE);
    if (status == ULS_STATUS_OK) {
        config->receive_pending = true;
        HAL_StatusTypeDef hal_status = HAL_UART_Receive_DMA(config->hw_handle, data, length);
        if (hal_status == HAL_OK) {
            __HAL_DMA_DISABLE_IT(config->hw_handle->hdmarx, DMA_IT_HT);
            if (config->receive_mode == IO_UART_CONTINUOUS) {
                __HAL_UART_ENABLE_IT(config->hw_handle, UART_IT_CM);
            }
        } else {
            config->receive_pending = false;
        }
        status = io_uart_stm32_to_uls_status(hal_status);
    }

    return status;
}

enum uls_status io_uart_stm32_receive_dma(const io_uart_dev_t *uart, uint8_t *data, uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return ULS_STATUS_OK;
    }
    return start_receive_dma(uart, data, length);
}

enum uls_status io_uart_stm32_receive_continuous_dma(const io_uart_dev_t *uart, uint8_t *data,
                                                     uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    return start_receive_dma(uart, data, length);
}

void io_uart_stm32_transmit_complete_dma(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (!config->transmit_pending) {
        return;
    }

    // on error, defer notification until abort completes
    if (HAL_DMA_GetError(config->hw_handle->hdmatx) == HAL_DMA_ERROR_NONE &&
        !io_uart_stm32_dma_abort_pending(config)) {
        io_uart_stm32_finish_transmit(uart, ULS_STATUS_OK);
    }
}

void io_uart_stm32_receive_complete_dma(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (!config->receive_pending) {
        return;
    }

    // on error, defer notification until abort completes
    if (HAL_DMA_GetError(config->hw_handle->hdmarx) == HAL_DMA_ERROR_NONE &&
        !io_uart_stm32_dma_abort_pending(config)) {
        io_uart_stm32_finish_receive(uart, ULS_STATUS_OK);
    }
}
