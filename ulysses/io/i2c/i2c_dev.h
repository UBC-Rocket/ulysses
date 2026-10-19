#ifndef ULYSSES_IO_I2C_I2C_DEV_H
#define ULYSSES_IO_I2C_I2C_DEV_H

#include <stdint.h>

#include "ulysses/io/i2c.h"

/** @brief Device on an initialized I2C bus */
struct io_i2c_dev {
    io_i2c_bus_t *bus;      ///< Non-NULL bus that must remain valid while the device is used
    uint8_t addr;           ///< Unshifted 7-bit address in 0x01-0x7F; zero is rejected
};

#endif // ULYSSES_IO_I2C_I2C_DEV_H