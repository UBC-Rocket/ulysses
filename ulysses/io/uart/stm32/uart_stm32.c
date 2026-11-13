#include <stdbool.h>
#include <string.h>

#include "ulysses/io/irq.h"
#include "ulysses/io/status.h"
#include "ulysses/lib/assert.h"

#include "uart/stm32/uart_stm32.h"
#include "uart/stm32/uart_dma_stm32.h"
#include "uart/stm32/uart_irq_stm32.h"

bool io_uart_stm32_dma_abort_pending(const struct uart_config *config)
{
    const UART_HandleTypeDef *handle = config->hw_handle;
    return config->async_mode == IO_UART_DMA &&
           ((handle->hdmatx != NULL && HAL_DMA_GetState(handle->hdmatx) == HAL_DMA_STATE_ABORT) ||
            (handle->hdmarx != NULL && HAL_DMA_GetState(handle->hdmarx) == HAL_DMA_STATE_ABORT));
}

static void stm32_set_transmit_cb(const io_uart_dev_t *uart, io_uart_transmit_cb_t cb)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    config->transmit_cb = cb;
}

static void stm32_set_receive_cb(const io_uart_dev_t *uart, io_uart_receive_cb_t cb)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    config->receive_cb = cb;
}

enum io_status io_uart_stm32_to_io_status(HAL_StatusTypeDef hal_status)
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

enum io_status io_uart_stm32_prepare_transfer(const struct uart_config *config,
                                              enum io_uart_stm32_transfer_direction direction)
{
    const uint8_t busy_directions = config->duplex_mode == IO_UART_HALF_DUPLEX
                                        ? (UART_STM32_TRANSMIT | UART_STM32_RECEIVE)
                                        : direction;
    if ((config->pending & busy_directions) || io_uart_stm32_dma_abort_pending(config)) {
        return IO_STATUS_BUSY;
    }

    if (config->duplex_mode == IO_UART_FULL_DUPLEX) {
        return IO_STATUS_OK;
    }

    HAL_StatusTypeDef status = direction == UART_STM32_TRANSMIT
                                   ? HAL_HalfDuplex_EnableTransmitter(config->hw_handle)
                                   : HAL_HalfDuplex_EnableReceiver(config->hw_handle);
    return io_uart_stm32_to_io_status(status);
}

static enum io_status stm32_transmit_blocking(const io_uart_dev_t *uart, const uint8_t *data,
                                              uint16_t length, uint32_t timeout_ms)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    enum io_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_TRANSMIT);
    if (status != IO_STATUS_OK) {
        return status;
    }
    return io_uart_stm32_to_io_status(
        HAL_UART_Transmit(config->hw_handle, data, length, timeout_ms));
}

static enum io_status stm32_receive_blocking(const io_uart_dev_t *uart, uint8_t *data,
                                             uint16_t length, uint32_t timeout_ms)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    enum io_status status = io_uart_stm32_prepare_transfer(config, UART_STM32_RECEIVE);
    if (status != IO_STATUS_OK) {
        return status;
    }
    return io_uart_stm32_to_io_status(
        HAL_UART_Receive(config->hw_handle, data, length, timeout_ms));
}

static const struct io_uart_ops stm32_dma_ops = {
    .set_transmit_cb = stm32_set_transmit_cb,
    .set_receive_cb = stm32_set_receive_cb,
    .transmit = stm32_transmit_blocking,
    .receive = stm32_receive_blocking,
    .transmit_async = io_uart_stm32_transmit_dma,
    .receive_async = io_uart_stm32_receive_dma,
    .receive_continuous = io_uart_stm32_receive_continuous_dma,
};

static const struct io_uart_ops stm32_irq_ops = {
    .set_transmit_cb = stm32_set_transmit_cb,
    .set_receive_cb = stm32_set_receive_cb,
    .transmit = stm32_transmit_blocking,
    .receive = stm32_receive_blocking,
    .transmit_async = io_uart_stm32_transmit_irq,
    .receive_async = io_uart_stm32_receive_irq,
    .receive_continuous = io_uart_stm32_receive_continuous_irq,
};

void io_uart_stm32_finish_transmit(const io_uart_dev_t *uart, enum io_status status)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    config->pending &= ~UART_STM32_TRANSMIT;

    if (config->transmit_cb != NULL) {
        config->transmit_cb(status, uart);
    }
}

void io_uart_stm32_finish_receive(const io_uart_dev_t *uart, enum io_status status)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    config->pending &= ~UART_STM32_RECEIVE;

    // TODO: handle CM events

    if (config->receive_cb != NULL) {
        config->receive_cb(status, uart);
    }
}

// Abort both directions on HAL errors; notify pending transfers after abort completion.
static void stm32_error_handler(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    // automatic DMA abort delivers another error event when it finishes
    if (!io_uart_stm32_dma_abort_pending(config)) {
        (void)HAL_UART_Abort_IT(config->hw_handle);
    }
}

static void stm32_abort_complete_handler(void *_uart)
{
    const io_uart_dev_t *uart = (const io_uart_dev_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (io_uart_stm32_dma_abort_pending(config)) {
        return;
    }

    // snapshot pending transfers before the transmit callback can start another receive
    const uint8_t pending = config->pending;
    if (pending & UART_STM32_TRANSMIT) {
        io_uart_stm32_finish_transmit(uart, IO_STATUS_ERROR);
    }
    if (pending & UART_STM32_RECEIVE) {
        io_uart_stm32_finish_receive(uart, IO_STATUS_ERROR);
    }
}

void io_uart_stm32_init(io_uart_dev_t *uart, const struct uart_config *uart_config)
{
    ULYSSES_ASSERT(uart != NULL, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config != NULL, "uart config must not be NULL");
    ULYSSES_ASSERT(uart_config != NULL, "uart initialization config must not be NULL");
    ULYSSES_ASSERT(uart_config->hw_handle != NULL, "uart handle must not be NULL");
    ULYSSES_ASSERT(uart_config->duplex_mode == IO_UART_FULL_DUPLEX ||
                       uart_config->duplex_mode == IO_UART_HALF_DUPLEX,
                   "invalid uart duplex mode");
    ULYSSES_ASSERT(HAL_UART_GetState(uart_config->hw_handle) == HAL_UART_STATE_READY,
                   "uart handle must be initialized and idle");

    struct uart_config *config = (struct uart_config *)uart->config;
    memcpy(config, uart_config, sizeof(*config));
    config->pending = 0;

    __HAL_UART_DISABLE(config->hw_handle);
    if (config->duplex_mode == IO_UART_HALF_DUPLEX) {
        SET_BIT(config->hw_handle->Instance->CR3, USART_CR3_HDSEL);
    } else {
        CLEAR_BIT(config->hw_handle->Instance->CR3, USART_CR3_HDSEL);
    }
    __HAL_UART_ENABLE(config->hw_handle);

    // TODO: configure CM

    const uint64_t id = IO_IRQ_HAL_HANDLE_TO_ID(config->hw_handle);
    io_irq_register(id, IO_IRQ_EVENT_ERROR, stm32_error_handler, uart);
    io_irq_register(id, IO_IRQ_EVENT_ABORT_COMPLETE, stm32_abort_complete_handler, uart);
    switch (config->async_mode) {
    case IO_UART_DMA:
        uart->ops = &stm32_dma_ops;
        io_irq_register(id, IO_IRQ_EVENT_TX_COMPLETE, io_uart_stm32_transmit_complete_dma, uart);
        io_irq_register(id, IO_IRQ_EVENT_RX_COMPLETE, io_uart_stm32_receive_complete_dma, uart);
        break;
    case IO_UART_IRQ:
        uart->ops = &stm32_irq_ops;
        io_irq_register(id, IO_IRQ_EVENT_TX_COMPLETE, io_uart_stm32_transmit_complete_irq, uart);
        io_irq_register(id, IO_IRQ_EVENT_RX_COMPLETE, io_uart_stm32_receive_complete_irq, uart);
        break;
    default:
        ULYSSES_ASSERT(false, "invalid uart asynchronous mode");
    }
}
