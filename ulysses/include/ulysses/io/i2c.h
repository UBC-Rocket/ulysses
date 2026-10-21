#ifndef ULYSSES_IO_I2C
#define ULYSSES_IO_I2C

#include <stdint.h>

#include "ulysses/io/status.h"

/** @brief I2C bus backed by platform-specific operations */
typedef struct io_i2c_bus io_i2c_bus_t;

/** @brief I2C device identified by its bus and 7-bit address */
typedef struct io_i2c_dev io_i2c_dev_t;

/**
 * @brief Reads bytes starting at an 8-bit device register
 *
 * Blocks until the backend transfer completes or fails.
 *
 * @param dev Non-NULL device on an initialized bus, with an unshifted address in 0x01-0x7F
 * @param reg Starting register address
 * @param[out] data Non-NULL output buffer with space for at least len bytes
 * @param len Number of bytes to read; must be greater than zero
 * @param timeout_ms Backend timeout in milliseconds
 *
 * @return Status returned by the bus backend
 */
enum io_status io_i2c_mem_read(const io_i2c_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len,
                               uint32_t timeout_ms);

/**
 * @brief Writes bytes starting at an 8-bit device register
 *
 * Blocks until the backend transfer completes or fails.
 *
 * @param dev Non-NULL device on an initialized bus, with an unshifted address in 0x01-0x7F
 * @param reg Starting register address
 * @param data Non-NULL input buffer containing at least len bytes
 * @param len Number of bytes to write; must be greater than zero
 * @param timeout_ms Backend timeout in milliseconds
 *
 * @return Status returned by the bus backend
 */
enum io_status io_i2c_mem_write(const io_i2c_dev_t *dev, uint8_t reg, const uint8_t *data,
                                uint16_t len, uint32_t timeout_ms);

#endif // ULYSSES_IO_I2C
