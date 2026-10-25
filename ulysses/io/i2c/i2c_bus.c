#include <stdbool.h>
#include <stdint.h>

#include "cmsis_compiler.h" // IWYU pragma: keep

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

/**
 * @brief Claims the bus for an async transfer and records its callback
 *
 * The callback is recorded before the backend starts, since completion can interrupt the caller.
 *
 * @param bus Bus to claim
 * @param cb Completion callback
 * @param user_data Data passed to cb
 *
 * @return True if the bus was idle and is now claimed
 */
static bool claim(io_i2c_bus_t *bus, io_i2c_cb_t cb, void *user_data)
{
    bool claimed = false;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    if (!bus->busy) {
        bus->busy = true;
        bus->cb = cb;
        bus->cb_user_data = user_data;
        claimed = true;
    }

    __set_PRIMASK(primask);

    return claimed;
}

enum io_status io_i2c_bus_mem_read_async(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                         uint8_t *data, uint16_t len, io_i2c_cb_t cb,
                                         void *user_data)
{
    if (!claim(bus, cb, user_data)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = bus->ops->mem_read_async(bus->config, addr, reg, data, len);

    if (status != IO_STATUS_OK) {
        bus->busy = false;
    }

    return status;
}

enum io_status io_i2c_bus_mem_write_async(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                          const uint8_t *data, uint16_t len, io_i2c_cb_t cb,
                                          void *user_data)
{
    if (!claim(bus, cb, user_data)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = bus->ops->mem_write_async(bus->config, addr, reg, data, len);

    if (status != IO_STATUS_OK) {
        bus->busy = false;
    }

    return status;
}

void io_i2c_bus_complete(io_i2c_bus_t *bus, enum io_status status)
{
    io_i2c_cb_t cb = bus->cb;
    void *user_data = bus->cb_user_data;

    bus->busy = false;

    cb(status, user_data);
}
