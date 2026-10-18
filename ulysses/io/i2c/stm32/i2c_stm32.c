#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "i2c/i2c.h"
#include "i2c/stm32/i2c_stm32.h"
#include "ulysses/io/status.h"
#include "ulysses/lib/assert.h"

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

// Clearing PE resets the peripheral and releases SCL/SDA, so a glitch can't leave the bus stuck
static void bus_reset(I2C_HandleTypeDef *hi2c)
{
    __HAL_I2C_DISABLE(hi2c);
    while ((hi2c->Instance->CR1 & I2C_CR1_PE) != 0U) {
    }
    __HAL_I2C_ENABLE(hi2c);
}

static enum io_status stm32_transfer(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *buf,
                                     uint16_t len, uint32_t timeout_ms, bool is_read)
{
    const struct io_i2c_stm32_config *config = bus->config;
    uint16_t hal_addr = (uint16_t)(addr << 1);
    HAL_StatusTypeDef hal_status;

    if (is_read) {
        hal_status = HAL_I2C_Mem_Read(config->hw_handle, hal_addr, reg, I2C_MEMADD_SIZE_8BIT, buf,
                                      len, timeout_ms);
    } else {
        hal_status = HAL_I2C_Mem_Write(config->hw_handle, hal_addr, reg, I2C_MEMADD_SIZE_8BIT, buf,
                                       len, timeout_ms);
    }

    // A NACK just means the device didn't answer; the HAL already sent a STOP, so no reset needed
    bool is_nack = (HAL_I2C_GetError(config->hw_handle) & HAL_I2C_ERROR_AF) != 0U;
    if (((hal_status == HAL_ERROR) || (hal_status == HAL_TIMEOUT)) && !is_nack) {
        bus_reset(config->hw_handle);
    }

    return to_io_status(hal_status);
}

void io_i2c_stm32_init(io_i2c_bus_t *bus, const struct io_i2c_stm32_config *config)
{
    ULYSSES_ASSERT(bus != NULL, "I2C bus is NULL");
    ULYSSES_ASSERT(config != NULL, "I2C config is NULL");
    ULYSSES_ASSERT(config->hw_handle != NULL, "I2C hardware handle is NULL");

    bus->config = config;
    bus->transfer = stm32_transfer;
}
