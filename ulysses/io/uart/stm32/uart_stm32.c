// TODO: Add async error reporting and RX recovery

#include <stdbool.h>
#include <string.h>

#include "ulysses/io/irq.h"
#include "ulysses/lib/assert.h"

#include "uart/stm32/uart_stm32.h"

void stm32_uart_set_transmit_cb(const io_uart_t *uart, io_uart_transmit_cb_t cb)
{
    struct uart_config *config = (struct uart_config *)uart->config;
    config->transmit_cb = cb;
}

void stm32_uart_set_receive_cb(const io_uart_t *uart, io_uart_receive_cb_t cb)
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

static enum io_status stm32_uart_transmit_blocking(const io_uart_t *uart, const uint8_t *data,
                                                   uint16_t length, uint32_t timeout_ms)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->huart != NULL, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit(config->huart, data, length, timeout_ms));
}

static enum io_status stm32_uart_receive_blocking(const io_uart_t *uart, uint8_t *data,
                                                  uint16_t length, uint32_t timeout_ms)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");
    ULYSSES_ASSERT(config->huart != NULL, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Receive(config->huart, data, length, timeout_ms));
}

static enum io_status stm32_uart_transmit_dma(const io_uart_t *uart, const uint8_t *data,
                                              uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit_DMA(config->huart, data, length));
}

static enum io_status stm32_uart_receive_dma(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    __HAL_DMA_ENABLE_IT(config->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_DISABLE_IT(config->huart, UART_IT_CM);

    return to_io_status(HAL_UART_Receive_DMA(config->huart, data, length));
}

static enum io_status stm32_uart_receive_continuous_dma(const io_uart_t *uart, uint8_t *data,
                                                        uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    if (length == 0) {
        return IO_STATUS_ERROR;
    }

    // TODO: HAL_UART_Receive_DMA() reenables HT/TC
    __HAL_DMA_DISABLE_IT(config->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(config->huart, UART_IT_CM);

    return to_io_status(HAL_UART_Receive_DMA(config->huart, data, length));
}

static enum io_status stm32_uart_transmit_irq(const io_uart_t *uart, const uint8_t *data,
                                              uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit_IT(config->huart, data, length));
}

static enum io_status stm32_uart_receive_irq(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Receive_IT(config->huart, data, length));
}

static enum io_status stm32_uart_receive_continuous_irq(const io_uart_t *uart, uint8_t *data,
                                                        uint16_t length)
{
    const struct uart_config *config = (const struct uart_config *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    // TODO: Rearm IRQ RX before notifying the user
    return to_io_status(HAL_UART_Receive_IT(config->huart, data, length));
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

static void stm32_uart_event_transmit_complete_handler(void *_uart)
{
    const io_uart_t *uart = (const io_uart_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    if (config->transmit_cb != NULL) {
        config->transmit_cb();
    }
}

static void stm32_uart_event_receive_complete_handler(void *_uart)
{
    const io_uart_t *uart = (const io_uart_t *)_uart;
    const struct uart_config *config = (const struct uart_config *)uart->config;

    // TODO: Handle CM events
    if (config->receive_cb != NULL) {
        config->receive_cb();
    }
}

void stm32_uart_init(io_uart_t *uart, const struct uart_config *uart_config)
{
    ULYSSES_ASSERT(uart != NULL, "uart object must not be NULL");
    ULYSSES_ASSERT(uart->config != NULL, "uart config must not be NULL");
    ULYSSES_ASSERT(uart_config != NULL, "uart initialization config must not be NULL");
    ULYSSES_ASSERT(uart_config->huart != NULL, "uart handle must not be NULL");

    struct uart_config *config = (struct uart_config *)uart->config;
    memcpy(config, uart_config, sizeof(*config));

    // TODO: Configure CM

    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(config->huart), IO_IRQ_EVENT_TX_COMPLETE,
                    stm32_uart_event_transmit_complete_handler, uart);
    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(config->huart), IO_IRQ_EVENT_RX_COMPLETE,
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
