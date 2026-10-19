#include <stddef.h>
#include <stdint.h>

#include "i2c/i2c_dev.h"
#include "i2c/i2c_bus.h"

#include "ulysses/lib/assert.h"

/**
 * @brief Asserts device transfer preconditions before dispatching to the bus
 *
 * @param dev Device with a non-NULL bus and an address in 0x01-0x7F
 * @param data Non-NULL transfer buffer
 * @param len Transfer size in bytes; must be greater than zero
 */
static void validate_transfer(
    const io_i2c_dev_t *dev,
    const void *data, 
    uint16_t len
) {
    ULYSSES_ASSERT(dev != NULL, "I2C device pointer is NULL");
    ULYSSES_ASSERT(dev->bus != NULL, "I2C bus pointer is NULL");
    ULYSSES_ASSERT(data != NULL, "Data pointer is NULL");
    ULYSSES_ASSERT(len > 0U, "Length must be greater than 0");
    ULYSSES_ASSERT((dev->addr != 0U && dev->addr < 0x80U), "I2C address must be a 7-bit value");
}

enum io_status io_i2c_mem_read(
    const io_i2c_dev_t *dev, 
    uint8_t reg, 
    uint8_t *data,
    uint16_t len, 
    uint32_t timeout_ms
) {
    validate_transfer(dev, data, len);
    return io_i2c_bus_mem_read(dev->bus, dev->addr, reg, data, len, timeout_ms);
}

enum io_status io_i2c_mem_write(
    const io_i2c_dev_t *dev,
    uint8_t reg,
    const uint8_t *data,
    uint16_t len,
    uint32_t timeout_ms
) {
    validate_transfer(dev, data, len);
    return io_i2c_bus_mem_write(dev->bus, dev->addr, reg, data, len, timeout_ms);
}
