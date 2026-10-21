// TODO: Add async error reporting and RX recovery

#include <string.h>

#include "ulysses/io/irq.h"
#include "ulysses/lib/assert.h"

#include "uart/stm32/uart_stm32.h"

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
    ULYSSES_ASSERT(uart, "uart object must not be NULL");

    uart_config_t *config = (uart_config_t *)uart->config;
    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit(config->huart, data, length, timeout_ms));
}

static enum io_status stm32_uart_receive_blocking(const io_uart_t *uart, uint8_t *data,
                                                  uint16_t length, uint32_t timeout_ms)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");

    uart_config_t *config = (uart_config_t *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");
    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Receive(config->huart, data, length, timeout_ms));
}

static enum io_status stm32_uart_transmit_dma(const io_uart_t *uart, const uint8_t *data,
                                              uint16_t length)
{
    uart_config_t *config = (uart_config_t *)uart->config;
    ULYSSES_ASSERT(config->huart->hdmatx, "uart tx dma handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit_DMA(config->huart, data, length));
}

static enum io_status stm32_uart_receive_dma(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    uart_config_t *config = (uart_config_t *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");
    ULYSSES_ASSERT(config->huart->hdmarx, "uart rx dma handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    __HAL_DMA_ENABLE_IT(config->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_DISABLE_IT(config->huart, UART_IT_CM);

    return to_io_status(HAL_UART_Receive_DMA(config->huart, data, length));
}

static enum io_status stm32_uart_receive_continuous_dma(const io_uart_t *uart, uint8_t *ring_buf,
                                                        uint16_t length)
{
    uart_config_t *config = (uart_config_t *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");
    ULYSSES_ASSERT(config->huart->hdmarx, "uart rx dma handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_ERROR;
    }

    // TODO: HAL_UART_Receive_DMA() reenables HT/TC
    __HAL_DMA_DISABLE_IT(config->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(config->huart, UART_IT_CM);

    return to_io_status(HAL_UART_Receive_DMA(config->huart, ring_buf, length));
}

static enum io_status stm32_uart_transmit_irq(const io_uart_t *uart, const uint8_t *data,
                                              uint16_t length)
{
    uart_config_t *config = (uart_config_t *)uart->config;

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Transmit_IT(config->huart, data, length));
}

static enum io_status stm32_uart_receive_irq(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    uart_config_t *config = (uart_config_t *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_PACKET, "uart must be in packet receive mode");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return to_io_status(HAL_UART_Receive_IT(config->huart, data, length));
}

static enum io_status stm32_uart_receive_continuous_irq(const io_uart_t *uart, uint8_t *data,
                                                        uint16_t length)
{
    uart_config_t *config = (uart_config_t *)uart->config;
    ULYSSES_ASSERT(config->receive_mode == IO_UART_CONTINUOUS,
                   "uart must be in continuous receive mode");

    // TODO: Rearm IRQ RX before notifying the user
    return to_io_status(HAL_UART_Receive_IT(config->huart, data, length));
}

const io_uart_methods_t *stm32_uart_get_dma_methods(void)
{
    static const io_uart_methods_t methods = {
        .transmit = stm32_uart_transmit_blocking,
        .receive = stm32_uart_receive_blocking,
        .transmit_async = stm32_uart_transmit_dma,
        .receive_async = stm32_uart_receive_dma,
        .receive_continuous = stm32_uart_receive_continuous_dma,
    };

    return &methods;
}

const io_uart_methods_t *stm32_uart_get_irq_methods(void)
{
    static const io_uart_methods_t methods = {
        .transmit = stm32_uart_transmit_blocking,
        .receive = stm32_uart_receive_blocking,
        .transmit_async = stm32_uart_transmit_irq,
        .receive_async = stm32_uart_receive_irq,
        .receive_continuous = stm32_uart_receive_continuous_irq,
    };

    return &methods;
}

static void stm32_uart_event_transmit_complete_handler(void *_uart)
{
    io_uart_t *uart = (io_uart_t *)_uart;
    uart_config_t *config = (uart_config_t *)uart->config;

    config->transmit_cb();
}

static void stm32_uart_event_receive_complete_handler(void *_uart)
{
    io_uart_t *uart = (io_uart_t *)_uart;
    uart_config_t *config = (uart_config_t *)uart->config;

    // TODO: Handle CM events and DMA buffer wraparound
    config->receive_cb();
}

void stm32_uart_init(io_uart_t *uart, const uart_config_t *uart_config)
{
    uart_config_t *config = (uart_config_t *)uart->config;
    memcpy(config, uart_config, sizeof(*config));

    // TODO: Configure character matching and circular RX DMA

    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(config->huart), IO_IRQ_EVENT_TX_COMPLETE,
                    stm32_uart_event_transmit_complete_handler, uart);
    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(config->huart), IO_IRQ_EVENT_RX_COMPLETE,
                    stm32_uart_event_receive_complete_handler, uart);

    switch (config->async_mode) {
    case IO_UART_DMA:
        uart->methods = stm32_uart_get_dma_methods();
        break;
    case IO_UART_IRQ:
        uart->methods = stm32_uart_get_irq_methods();
        break;
    }
}
