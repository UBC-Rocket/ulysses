#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "i2c/i2c.h"
#include "ulysses/io/i2c.h"
#include "ulysses/lib/assert.h"

io_i2c_bus_t IO_I2C1;
io_i2c_bus_t IO_I2C2;
io_i2c_bus_t IO_I2C3;
io_i2c_bus_t IO_I2C4;

static enum io_status transfer(const io_i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len,
                               uint32_t timeout_ms, bool is_read)
{
    ULYSSES_ASSERT(dev != NULL, "I2C device is NULL");
    ULYSSES_ASSERT(dev->bus->transfer != NULL, "I2C bus was not initialised");
    ULYSSES_ASSERT((dev->addr != 0U) && (dev->addr <= 0x7FU), "I2C address is not 7-bit");
    ULYSSES_ASSERT(buf != NULL, "I2C buffer is NULL");
    ULYSSES_ASSERT(len > 0U, "I2C transfer length is 0");

    return dev->bus->transfer(dev->bus, dev->addr, reg, buf, len, timeout_ms, is_read);
}

enum io_status io_i2c_mem_read(const io_i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len,
                               uint32_t timeout_ms)
{
    return transfer(dev, reg, buf, len, timeout_ms, true);
}

enum io_status io_i2c_mem_write(const io_i2c_dev_t *dev, uint8_t reg, const uint8_t *buf,
                                uint16_t len, uint32_t timeout_ms)
{
    // The backend never writes through the buffer of a write
    return transfer(dev, reg, (uint8_t *)buf, len, timeout_ms, false);
}
