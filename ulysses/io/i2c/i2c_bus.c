#include <stddef.h>
#include <stdint.h>

#include "i2c/i2c_bus.h"

#include "ulysses/lib/assert.h"

enum io_status io_i2c_bus_mem_read(
    const io_i2c_bus_t *bus,
    uint8_t addr,
    uint8_t reg,
    uint8_t *data,
    uint16_t len,
    uint32_t timeout_ms
) {
    ULYSSES_ASSERT(bus != NULL, "I2C bus pointer is NULL");
    ULYSSES_ASSERT(bus->ops != NULL, "I2C bus operations are NULL");
    ULYSSES_ASSERT(bus->ops->mem_read != NULL, "I2C bus read operation is NULL");
    ULYSSES_ASSERT(data != NULL, "Data pointer is NULL");

    return bus->ops->mem_read(bus->config, addr, reg, data, len, timeout_ms);
}

enum io_status io_i2c_bus_mem_write(
    const io_i2c_bus_t *bus,
    uint8_t addr,
    uint8_t reg,
    const uint8_t *data,
    uint16_t len,
    uint32_t timeout_ms
) {
    ULYSSES_ASSERT(bus != NULL, "I2C bus pointer is NULL");
    ULYSSES_ASSERT(bus->ops != NULL, "I2C bus operations are NULL");
    ULYSSES_ASSERT(bus->ops->mem_write != NULL, "I2C bus write operation is NULL");
    ULYSSES_ASSERT(data != NULL, "Data pointer is NULL");

    return bus->ops->mem_write(bus->config, addr, reg, data, len, timeout_ms);
}

