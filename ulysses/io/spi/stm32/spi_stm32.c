#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/irq.h"
#include "ulysses/io/spi.h"

#include "spi/stm32/spi_stm32.h"

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

static enum io_status stm32_transfer(io_spi_dev_t *dev, const uint8_t *tx_data, uint8_t *rx_data,
                                     uint16_t len, uint32_t timeout_ms)
{
    const struct io_spi_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status =
        HAL_SPI_TransmitReceive(config->hw_handle, tx_data, rx_data, len, timeout_ms);
    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);

    return to_io_status(hal_status);
}

static enum io_status stm32_transmit(io_spi_dev_t *dev, const uint8_t *tx_data, uint16_t len,
                                     uint32_t timeout_ms)
{
    const struct io_spi_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(config->hw_handle, tx_data, len, timeout_ms);
    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);

    return to_io_status(hal_status);
}

static enum io_status stm32_receive(io_spi_dev_t *dev, uint8_t *rx_data, uint16_t len,
                                    uint32_t timeout_ms)
{
    const struct io_spi_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status = HAL_SPI_Receive(config->hw_handle, rx_data, len, timeout_ms);
    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);

    return to_io_status(hal_status);
}

static enum io_status stm32_transfer_it(io_spi_dev_t *dev, const uint8_t *tx_data, uint8_t *rx_data,
                                        uint16_t len)
{
    const struct io_spi_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status =
        HAL_SPI_TransmitReceive_IT(config->hw_handle, tx_data, rx_data, len);
    if (hal_status != HAL_OK) {
        HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);
    }

    return to_io_status(hal_status);
}

static enum io_status stm32_transmit_it(io_spi_dev_t *dev, const uint8_t *tx_data, uint16_t len)
{
    const struct io_spi_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit_IT(config->hw_handle, tx_data, len);
    if (hal_status != HAL_OK) {
        HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);
    }

    return to_io_status(hal_status);
}

static const struct io_spi_ops stm32_spi_ops = {
    .transfer = stm32_transfer,
    .transmit = stm32_transmit,
    .receive = stm32_receive,
    .transfer_it = stm32_transfer_it,
    .transmit_it = stm32_transmit_it,
};

static void stm32_complete_handler(void *user_data)
{
    io_spi_dev_t *dev = user_data;
    const struct io_spi_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);
    io_spi_complete(dev, IO_STATUS_OK);
}

static void stm32_error_handler(void *user_data)
{
    io_spi_dev_t *dev = user_data;
    const struct io_spi_stm32_config *config = dev->config;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);
    io_spi_complete(dev, IO_STATUS_ERROR);
}

void io_spi_stm32_init(io_spi_dev_t *dev, const struct io_spi_stm32_config *config)
{
    ULYSSES_ASSERT(dev != NULL, "SPI device must be non NULL");
    ULYSSES_ASSERT(config != NULL, "SPI config must be non NULL");
    ULYSSES_ASSERT(config->hw_handle != NULL, "SPI hardware handle must be non NULL");
    ULYSSES_ASSERT(config->hw_handle->State == HAL_SPI_STATE_READY,
                   "SPI hardware handle must be initialized");
    ULYSSES_ASSERT(config->cs_port != NULL, "SPI chip-select port must be non NULL");
    ULYSSES_ASSERT(config->cs_pin != 0, "SPI chip-select pin must be non zero");

    dev->config = config;
    dev->ops = &stm32_spi_ops;
    atomic_init(&dev->busy, false);
    dev->cb = NULL;
    dev->cb_user_data = NULL;

    HAL_GPIO_WritePin(config->cs_port, config->cs_pin, GPIO_PIN_SET);

    uint64_t id = IO_IRQ_HAL_HANDLE_TO_ID(config->hw_handle);
    io_irq_register(id, IO_IRQ_EVENT_TX_RX_COMPLETE, stm32_complete_handler, dev);
    io_irq_register(id, IO_IRQ_EVENT_TX_COMPLETE, stm32_complete_handler, dev);
    io_irq_register(id, IO_IRQ_EVENT_ERROR, stm32_error_handler, dev);
}
