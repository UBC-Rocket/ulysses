#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "i2c/i2c_bus.h"
#include "i2c/stm32/i2c_stm32.h"

#include "ulysses/io/irq.h"
#include "ulysses/lib/assert.h"

/**
 * @brief Maps HAL transfer status and timeout error flags to an I/O status
 *
 * Success and busy statuses take precedence over error flags.
 *
 * @param status HAL transfer result
 * @param hal_error HAL handle error flags after the transfer
 *
 * @retval IO_STATUS_OK HAL reported success
 * @retval IO_STATUS_BUSY HAL reported busy
 * @retval IO_STATUS_TIMEOUT HAL reported a timeout or set HAL_I2C_ERROR_TIMEOUT
 * @retval IO_STATUS_ERROR Any other HAL failure
 */
static enum io_status to_io_status(HAL_StatusTypeDef status, uint32_t hal_error)
{
    if (status == HAL_OK) {
        return IO_STATUS_OK;
    }

    if (status == HAL_BUSY) {
        return IO_STATUS_BUSY;
    }

    if (status == HAL_TIMEOUT || (hal_error & HAL_I2C_ERROR_TIMEOUT) != 0U) {
        return IO_STATUS_TIMEOUT;
    }

    return IO_STATUS_ERROR;
}

static enum io_status stm32_mem_read(const void *config, uint8_t addr, uint8_t reg, uint8_t *data,
                                     uint16_t len, uint32_t timeout_ms)
{
    const struct io_i2c_stm32_config *cfg = (const struct io_i2c_stm32_config *)config;

    I2C_HandleTypeDef *hi2c = cfg->hw_handle;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1U), reg,
                                                I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);

    return to_io_status(status, hi2c->ErrorCode);
}

static enum io_status stm32_mem_write(const void *config, uint8_t addr, uint8_t reg,
                                      const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    const struct io_i2c_stm32_config *cfg = (const struct io_i2c_stm32_config *)config;

    I2C_HandleTypeDef *hi2c = cfg->hw_handle;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        hi2c, (uint16_t)(addr << 1U), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, timeout_ms);

    return to_io_status(status, hi2c->ErrorCode);
}

static enum io_status stm32_mem_read_it(const void *config, uint8_t addr, uint8_t reg,
                                        uint8_t *data, uint16_t len)
{
    const struct io_i2c_stm32_config *cfg = (const struct io_i2c_stm32_config *)config;

    I2C_HandleTypeDef *hi2c = cfg->hw_handle;

    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read_IT(hi2c, (uint16_t)(addr << 1U), reg, I2C_MEMADD_SIZE_8BIT, data, len);

    return to_io_status(status, hi2c->ErrorCode);
}

static enum io_status stm32_mem_write_it(const void *config, uint8_t addr, uint8_t reg,
                                         const uint8_t *data, uint16_t len)
{
    const struct io_i2c_stm32_config *cfg = (const struct io_i2c_stm32_config *)config;

    I2C_HandleTypeDef *hi2c = cfg->hw_handle;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(hi2c, (uint16_t)(addr << 1U), reg,
                                                    I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len);

    return to_io_status(status, hi2c->ErrorCode);
}

/** @brief Shared operations */
static const struct io_i2c_bus_ops stm32_i2c_ops = {.mem_read = stm32_mem_read,
                                                    .mem_write = stm32_mem_write,
                                                    .mem_read_it = stm32_mem_read_it,
                                                    .mem_write_it = stm32_mem_write_it};

static void stm32_transfer_complete_handler(void *user_data)
{
    io_i2c_bus_complete((io_i2c_bus_t *)user_data, IO_STATUS_OK);
}

static void stm32_transfer_error_handler(void *user_data)
{
    io_i2c_bus_t *bus = (io_i2c_bus_t *)user_data;
    const struct io_i2c_stm32_config *cfg = (const struct io_i2c_stm32_config *)bus->config;

    io_i2c_bus_complete(bus, to_io_status(HAL_ERROR, cfg->hw_handle->ErrorCode));
}

void io_i2c_stm32_init(io_i2c_bus_t *bus, const struct io_i2c_stm32_config *config)
{
    ULYSSES_ASSERT(bus != NULL, "I2C bus pointer is NULL");
    ULYSSES_ASSERT(config != NULL, "I2C config pointer is NULL");
    ULYSSES_ASSERT(config->hw_handle != NULL, "I2C HAL handle pointer is NULL");
    ULYSSES_ASSERT(config->hw_handle->State == HAL_I2C_STATE_READY, "I2C peripheral is not ready");

    bus->config = config;
    bus->ops = &stm32_i2c_ops;
    bus->cb = NULL;
    bus->cb_user_data = NULL;
    bus->busy = false;

    uint64_t id = IO_IRQ_HAL_HANDLE_TO_ID(config->hw_handle);

    io_irq_register(id, IO_IRQ_EVENT_TX_COMPLETE, stm32_transfer_complete_handler, bus);
    io_irq_register(id, IO_IRQ_EVENT_RX_COMPLETE, stm32_transfer_complete_handler, bus);
    io_irq_register(id, IO_IRQ_EVENT_ERROR, stm32_transfer_error_handler, bus);
}
