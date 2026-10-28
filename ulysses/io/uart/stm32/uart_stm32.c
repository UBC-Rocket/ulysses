// TODO: Add actual error handling

#include <stdbool.h>
#include <string.h>

#include "ulysses/io/irq.h"
#include "ulysses/io/status.h"
#include "ulysses/lib/assert.h"

#include "uart/stm32/uart_stm32.h"

static void stm32_uart_set_transmit_cb(const io_uart_dev_t *uart, io_uart_transmit_cb_t cb)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    config->transmit_cb = cb;
}

static void stm32_uart_set_receive_cb(const io_uart_dev_t *uart, io_uart_receive_cb_t cb)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    config->receive_cb = cb;
}

static enum io_status to_io_status(HAL_StatusTypeDef hal_status)
{
    switch (hal_status) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_TIMEOUT:
        return IO_STATUS_TIMEOUT;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

static enum io_status stm32_uart_transmit_blocking(const io_uart_dev_t *uart, const uint8_t *data,
                                                   uint16_t length, uint32_t timeout_ms)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->hw_handle != NULL, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit(config->hw_handle, data, length, timeout_ms));
}

static enum io_status stm32_uart_receive_blocking(const io_uart_dev_t *uart, uint8_t *data,
                                                  uint16_t length, uint32_t timeout_ms)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");
    ULYSSES_ASSERT(config->hw_handle != NULL, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Receive(config->hw_handle, data, length, timeout_ms));
}

static enum io_status stm32_uart_transmit_dma(const io_uart_dev_t *uart, const uint8_t *data,
                                              uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit_DMA(config->hw_handle, data, length));
}

static enum io_status stm32_uart_receive_dma(const io_uart_dev_t *uart, uint8_t *data,
                                             uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    __HAL_DMA_ENABLE_IT(config->hw_handle->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_DISABLE_IT(config->hw_handle, UART_IT_CM);

    return to_io_status(HAL_UART_Receive_DMA(config->hw_handle, data, length));
}

static enum io_status stm32_uart_receive_continuous_dma(const io_uart_dev_t *uart, uint8_t *data,
                                                        uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    if (length == 0) {
        return IO_STATUS_ERROR;
    }

    // TODO: HAL_UART_Receive_DMA() reenables HT/TC
    __HAL_DMA_DISABLE_IT(config->hw_handle->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(config->hw_handle, UART_IT_CM);

    return to_io_status(HAL_UART_Receive_DMA(config->hw_handle, data, length));
}

static enum io_status stm32_uart_transmit_irq(const io_uart_dev_t *uart, const uint8_t *data,
                                              uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit_IT(config->hw_handle, data, length));
}

static enum io_status stm32_uart_receive_irq(const io_uart_dev_t *uart, uint8_t *data,
                                             uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Receive_IT(config->hw_handle, data, length));
}

static enum io_status stm32_uart_receive_continuous_irq(const io_uart_dev_t *uart, uint8_t *data,
                                                        uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    // TODO: Rearm IRQ RX before notifying the user
    return to_io_status(HAL_UART_Receive_IT(config->hw_handle, data, length));
}

static const struct io_uart_ops stm32_uart_dma_ops = {
    .set_transmit_cb = stm32_uart_set_transmit_cb,
    .set_receive_cb = stm32_uart_set_receive_cb,
    .transmit = stm32_uart_transmit_blocking,
    .receive = stm32_uart_receive_blocking,
    .transmit_async = stm32_uart_transmit_dma,
    .receive_async = stm32_uart_receive_dma,
    .receive_continuous = stm32_uart_receive_continuous_dma,
};

static const struct io_uart_ops stm32_uart_irq_ops = {
    .set_transmit_cb = stm32_uart_set_transmit_cb,
    .set_receive_cb = stm32_uart_set_receive_cb,
    .transmit = stm32_uart_transmit_blocking,
    .receive = stm32_uart_receive_blocking,
    .transmit_async = stm32_uart_transmit_irq,
    .receive_async = stm32_uart_receive_irq,
    .receive_continuous = stm32_uart_receive_continuous_irq,
};

static void stm32_uart_event_error_handler(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    const uint32_t errors = config->hw_handle->ErrorCode;
    if (errors == HAL_UART_ERROR_NONE) {
        return;
    }

    const uint32_t dma_requests = config->hw_handle->Instance->CR3;
    const uint32_t dma_rx = dma_requests & USART_CR3_DMAR;
    const uint32_t dma_tx = dma_requests & USART_CR3_DMAT;
    const uint32_t rx_errors =
        HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    ULYSSES_ASSERT(!(errors & HAL_UART_ERROR_INVALID_CALLBACK),
                   "invalid uart callback registration");
#endif

    if (config->receive_cb != NULL) {
        if ((errors & rx_errors) || (dma_rx && (errors & HAL_UART_ERROR_DMA))) {
            config->receive_cb(IO_STATUS_ERROR, uart);
        } else if (errors & HAL_UART_ERROR_RTO) {
            config->receive_cb(IO_STATUS_TIMEOUT, uart);
        }
    }

    if (config->transmit_cb != NULL && dma_tx && (errors & HAL_UART_ERROR_DMA)) {
        config->transmit_cb(IO_STATUS_ERROR, uart);
    }
}

static void stm32_uart_event_transmit_complete_handler(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (config->transmit_cb != NULL) {
        config->transmit_cb(IO_STATUS_OK, uart);
    }
}

static void stm32_uart_event_receive_complete_handler(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    // TODO: Handle CM events
    if (config->receive_cb != NULL) {
        config->receive_cb(IO_STATUS_OK, uart);
    }
}

void io_uart_stm32_init(io_uart_dev_t *uart, const struct uart_config *uart_config)
{
    ULYSSES_ASSERT(uart != NULL, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config != NULL, "uart config must not be NULL");
    ULYSSES_ASSERT(uart_config != NULL, "uart initialization config must not be NULL");
    ULYSSES_ASSERT(uart_config->hw_handle != NULL, "uart handle must not be NULL");

    struct uart_config *config = (struct uart_config *)uart->config;
    memcpy(config, uart_config, sizeof(*config));

    // TODO: Configure CM

    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(config->hw_handle), IO_IRQ_EVENT_ERROR,
                    stm32_uart_event_error_handler, uart);
    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(config->hw_handle), IO_IRQ_EVENT_TX_COMPLETE,
                    stm32_uart_event_transmit_complete_handler, uart);
    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(config->hw_handle), IO_IRQ_EVENT_RX_COMPLETE,
                    stm32_uart_event_receive_complete_handler, uart);

    switch (config->async_mode) {
    case IO_UART_DMA:
        uart->ops = &stm32_uart_dma_ops;
        break;
    case IO_UART_IRQ:
        uart->ops = &stm32_uart_irq_ops;
        break;
    default:
        ULYSSES_ASSERT(false, "invalid uart asynchronous mode");
    }
}
