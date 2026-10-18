#ifndef ULYSSES_IO_I2C_I2C
#define ULYSSES_IO_I2C_I2C

#include <stdbool.h>
#include <stdint.h>

#include "ulysses/io/i2c.h"
#include "ulysses/io/status.h"

// Platform transfer function. Reads (is_read) or writes len bytes at register reg.
typedef enum io_status (*io_i2c_transfer_fn_t)(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                               uint8_t *buf, uint16_t len, uint32_t timeout_ms,
                                               bool is_read);

// Filled in by a platform init function, e.g. io_i2c_stm32_init()
struct io_i2c_bus {
    const void *config;            // platform config
    io_i2c_transfer_fn_t transfer; // platform transfer function
};

struct io_i2c_dev {
    io_i2c_bus_t *bus;
    uint8_t addr; // 7-bit address
};

#endif // ULYSSES_IO_I2C_I2C
