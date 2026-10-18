#ifndef ULYSSES_IO_I2C
#define ULYSSES_IO_I2C

#include <stdint.h>

#include "ulysses/io/status.h"

typedef struct io_i2c_bus io_i2c_bus_t;
typedef struct io_i2c_dev io_i2c_dev_t;

// The MCU's I2C peripherals. Every device sits on one of these.
extern io_i2c_bus_t IO_I2C1;
extern io_i2c_bus_t IO_I2C2;
extern io_i2c_bus_t IO_I2C3;
extern io_i2c_bus_t IO_I2C4;

// Reads len bytes starting at register reg of dev.
enum io_status io_i2c_mem_read(const io_i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len,
                               uint32_t timeout_ms);

// Writes len bytes starting at register reg of dev.
enum io_status io_i2c_mem_write(const io_i2c_dev_t *dev, uint8_t reg, const uint8_t *buf,
                                uint16_t len, uint32_t timeout_ms);

#endif // ULYSSES_IO_I2C
