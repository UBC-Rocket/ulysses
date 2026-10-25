#include <stdint.h>

#include "i2c/i2c_bus.h"

enum io_status io_i2c_bus_mem_read(const io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                   uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    return bus->ops->mem_read(bus->config, addr, reg, data, len, timeout_ms);
}

enum io_status io_i2c_bus_mem_write(const io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                    const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    return bus->ops->mem_write(bus->config, addr, reg, data, len, timeout_ms);
}
