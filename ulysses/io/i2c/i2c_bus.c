#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "cmsis_compiler.h" // IWYU pragma: keep
#include "ulysses/io/status.h"

#include "i2c/i2c_bus.h"

// TODO: Change claim() and release() to atomic operations in the future to avoid disabling interrupts.

// Records cb before the transfer starts, since completion can interrupt the caller
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

/** @brief Releases a transfer without invoking its callback */
static void release(io_i2c_bus_t *bus)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    bus->cb = NULL;
    bus->cb_user_data = NULL;
    bus->busy = false;

    __set_PRIMASK(primask);
}

enum io_status io_i2c_bus_mem_read(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *data,
                                   uint16_t len, uint32_t timeout_ms)
{
    if (!claim(bus, NULL, NULL)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = bus->ops->mem_read(bus->config, addr, reg, data, len, timeout_ms);

    release(bus);

    return status;
}

enum io_status io_i2c_bus_mem_write(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                    const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!claim(bus, NULL, NULL)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = bus->ops->mem_write(bus->config, addr, reg, data, len, timeout_ms);

    release(bus);

    return status;
}

enum io_status io_i2c_bus_mem_read_it(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *data,
                                      uint16_t len, io_i2c_cb_t cb, void *user_data)
{
    if (!claim(bus, cb, user_data)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = bus->ops->mem_read_it(bus->config, addr, reg, data, len);

    if (status != IO_STATUS_OK) {
        release(bus);
    }

    return status;
}

enum io_status io_i2c_bus_mem_write_it(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                       const uint8_t *data, uint16_t len, io_i2c_cb_t cb,
                                       void *user_data)
{
    if (!claim(bus, cb, user_data)) {
        return IO_STATUS_BUSY;
    }

    enum io_status status = bus->ops->mem_write_it(bus->config, addr, reg, data, len);

    if (status != IO_STATUS_OK) {
        release(bus);
    }

    return status;
}

void io_i2c_bus_complete(io_i2c_bus_t *bus, enum io_status status)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    // A blocking transfer owns the bus without registering a callback.
    if (!bus->busy || bus->cb == NULL) {
        __set_PRIMASK(primask);
        return;
    }

    io_i2c_cb_t cb = bus->cb;
    void *user_data = bus->cb_user_data;

    bus->cb = NULL;
    bus->cb_user_data = NULL;
    bus->busy = false;

    __set_PRIMASK(primask);

    cb(status, user_data);
}
