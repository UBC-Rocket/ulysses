#ifndef ULYSSES_IO_I2C_I2C_BUS_H
#define ULYSSES_IO_I2C_I2C_BUS_H

#include <stdint.h>

#include "ulysses/io/i2c.h"

/** @brief Backend operations for synchronous I2C register transfers */
struct io_i2c_bus_ops {
    /**
     * @brief Reads device registers through the backend
     *
     * @param config Backend-specific configuration
     * @param addr Unshifted 7-bit device address
     * @param reg Starting 8-bit register address
     * @param[out] data Non-NULL output buffer with space for at least len bytes
     * @param len Number of bytes to read
     * @param timeout_ms Backend timeout in milliseconds
     *
     * @return Transfer status
     */
    enum io_status (*mem_read)(const void *config, uint8_t addr, uint8_t reg, uint8_t *data,
                               uint16_t len, uint32_t timeout_ms);

    /**
     * @brief Writes device registers through the backend
     *
     * @param config Backend-specific configuration
     * @param addr Unshifted 7-bit device address
     * @param reg Starting 8-bit register address
     * @param data Non-NULL input buffer containing at least len bytes
     * @param len Number of bytes to write
     * @param timeout_ms Backend timeout in milliseconds
     *
     * @return Transfer status
     */
    enum io_status (*mem_write)(const void *config, uint8_t addr, uint8_t reg, const uint8_t *data,
                                uint16_t len, uint32_t timeout_ms);
};

/** @brief I2C bus retaining references to its backend configuration and operations */
struct io_i2c_bus {
    const void *config;               ///< Backend configuration that must remain valid while in use
    const struct io_i2c_bus_ops *ops; ///< Non-NULL operations that must remain valid while in use
};

/**
 * @brief Dispatches a synchronous register read to the bus backend
 *
 * Address and length validation is left to the caller or backend.
 *
 * @param bus Non-NULL bus with a non-NULL mem_read operation
 * @param addr Unshifted 7-bit device address
 * @param reg Starting 8-bit register address
 * @param[out] data Non-NULL output buffer with space for at least len bytes
 * @param len Number of bytes to read
 * @param timeout_ms Backend timeout in milliseconds
 *
 * @return Status returned by the backend without modification
 */
enum io_status io_i2c_bus_mem_read(const io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                   uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief Dispatches a synchronous register write to the bus backend
 *
 * Address and length validation is left to the caller or backend.
 *
 * @param bus Non-NULL bus with a non-NULL mem_write operation
 * @param addr Unshifted 7-bit device address
 * @param reg Starting 8-bit register address
 * @param data Non-NULL input buffer containing at least len bytes
 * @param len Number of bytes to write
 * @param timeout_ms Backend timeout in milliseconds
 *
 * @return Status returned by the backend without modification
 */
enum io_status io_i2c_bus_mem_write(const io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                    const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#endif // ULYSSES_IO_I2C_I2C_BUS_H
