#include "ulysses/io/irq.h"
#include "ulysses/lib/assert.h"

#include "uart/uart.h"

struct io_uart {
    void *config;
    enum io_status (*transmit_async)(const io_uart_t *uart, const uint8_t *data, uint16_t length);
    enum io_status (*receive_async)(const io_uart_t *uart, uint8_t *data, uint16_t length);
    enum io_status (*receive_continuous)(const io_uart_t *uart, uint8_t *ring_buf, uint16_t length);
};

enum io_status io_uart_transmit(const io_uart_t *uart, const uint8_t *data, uint16_t length,
                             uint32_t timeout_ms)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return HAL_UART_Transmit(config->huart, data, length, timeout_ms) == HAL_OK ? IO_STATUS_OK
                                                                            : IO_STATUS_TIMEOUT;
}

enum io_status io_uart_receive(const io_uart_t *uart, uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    return HAL_UART_Receive(config->huart, data, length, timeout_ms) == HAL_OK ? IO_STATUS_OK : IO_STATUS_TIMEOUT;
}

enum io_status io_uart_transmit_async(const io_uart_t *uart, const uint8_t *data, uint16_t length)
{
    return uart->transmit_async(uart, data, length);
}

enum io_status io_uart_receive_async(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart->receive_async,
                   "must not call receive_async on continuous mode uart object");

    return uart->receive_async(uart, data, length);
}

enum io_status io_uart_receive_continous(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart->receive_async,
                   "must not call receive_continuous on packet mode uart object");

    return uart->receive_continuous(uart, data, length);
}

enum io_status io_uart_transmit_dma(const io_uart_t *uart, const uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    if (length == 0) {
        return IO_STATUS_OK;
    }

    switch (HAL_UART_Transmit_DMA(config->huart, data, length)) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

enum io_status io_uart_receive_dma(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");
    ULYSSES_ASSERT(config->huart->hdmarx, "uart rx dma handle must not be NULL");

    __HAL_DMA_ENABLE_IT(config->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_DISABLE_IT(config->huart, UART_IT_CM);

    switch (HAL_UART_Receive_DMA(config->huart, data, length)) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

enum io_status io_uart_receive_continuous_dma(const io_uart_t *uart, uint8_t *ring_buf, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");
    ULYSSES_ASSERT(config->huart->hdmarx, "uart rx dma handle must not be NULL");

    __HAL_DMA_DISABLE_IT(config->huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(config->huart, UART_IT_CM);

    switch (HAL_UART_Receive_DMA(config->huart, ring_buf, length)) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

enum io_status io_uart_transmit_irq(const io_uart_t *uart, const uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    switch (HAL_UART_Transmit_IT(config->huart, data, length)) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

enum io_status io_uart_receive_irq(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    switch (HAL_UART_Receive_IT(config->huart, data, length)) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

enum io_status io_uart_receive_continuous_irq(const io_uart_t *uart, uint8_t *data, uint16_t length)
{
    ULYSSES_ASSERT(uart, "uart object must not be NULL");
    uart_config_t *config = (uart_config_t *)uart->config;

    ULYSSES_ASSERT(config->huart, "uart handle must not be NULL");

    switch (HAL_UART_Receive_IT(config->huart, data, length)) {
    case HAL_OK:
        return IO_STATUS_OK;
    case HAL_BUSY:
        return IO_STATUS_BUSY;
    default:
        return IO_STATUS_ERROR;
    }
}

void event_transmit_complete_handler(void *_uart)
{
    io_uart_t *uart = (io_uart_t *)_uart;
    uart_config_t *config = (uart_config_t *)uart->config;

    config->transmit_cb();
}

void event_receive_complete_handler(void *_uart)
{
    io_uart_t *uart = (io_uart_t *)_uart;
    uart_config_t *config = (uart_config_t *)uart->config;

    config->transmit_cb();
}

void io_uart_init(io_uart_t *uart, UART_HandleTypeDef *huart, enum io_uart_async_mode async_mode,
                  enum io_uart_receive_mode receive_mode, uint8_t cm_byte,
                  io_uart_transmit_cb_t transmit_cb, io_uart_receive_cb_t receive_cb)
{
    uart_config_t *config = (uart_config_t *)uart->config;
    config->huart = huart;
    config->cm_byte = cm_byte;
    config->transmit_cb = transmit_cb;
    config->receive_cb = receive_cb;

    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(huart), IO_IRQ_EVENT_TX_COMPLETE,
                    event_transmit_complete_handler, uart);
    io_irq_register(IO_IRQ_HAL_HANDLE_TO_ID(huart), IO_IRQ_EVENT_RX_COMPLETE,
                    event_receive_complete_handler, uart);

    switch (async_mode) {
    case IO_UART_DMA:
        uart->transmit_async = io_uart_transmit_dma;
        uart->receive_async = io_uart_receive_dma;
        uart->receive_continuous = io_uart_receive_continuous_dma;
        break;
    case IO_UART_IRQ:
        uart->transmit_async = io_uart_transmit_irq;
        uart->receive_async = io_uart_receive_irq;
        uart->receive_continuous = io_uart_receive_continuous_irq;
        break;
    }

    switch (receive_mode) {
    case IO_UART_PACKET:
        uart->receive_continuous = NULL;
        break;
    case IO_UART_CONTINUOUS:
        uart->receive_async = NULL;
        break;
    }
}
