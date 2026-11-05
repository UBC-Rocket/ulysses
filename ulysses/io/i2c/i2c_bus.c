#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "ulysses/io/status.h"

#include "i2c/i2c_bus.h"

// Records cb before the transfer starts, since completion can interrupt the caller
static bool claim(io_i2c_bus_t *bus, io_i2c_cb_t cb, void *user_data)
{
    bool expected = false;

    if (!atomic_compare_exchange_strong_explicit(&bus->busy, &expected, true, memory_order_acquire,
                                                 memory_order_relaxed)) {
        return false;
    }

    bus->cb = cb;
    bus->cb_user_data = user_data;

    return true;
}

/** @brief Releases a transfer without invoking its callback */
static void release(io_i2c_bus_t *bus)
{
    bus->cb = NULL;
    bus->cb_user_data = NULL;

    atomic_store_explicit(&bus->busy, false, memory_order_release);
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
    // A blocking transfer owns the bus without registering a callback.
    if (!atomic_load_explicit(&bus->busy, memory_order_acquire) || bus->cb == NULL) {
        return;
    }

    io_i2c_cb_t cb = bus->cb;
    void *user_data = bus->cb_user_data;

    release(bus);

    cb(status, user_data);
}
