#ifndef ULYSSES_IO_I2C_I2C_BUS_H
#define ULYSSES_IO_I2C_I2C_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "ulysses/io/i2c.h"

/** @brief Backend operations for I2C register transfers */
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

    /**
     * @brief Starts a register read; the backend calls io_i2c_bus_complete() when it ends
     *
     * A failed start must not produce a completion notification.
     */
    enum io_status (*mem_read_it)(const void *config, uint8_t addr, uint8_t reg, uint8_t *data,
                                  uint16_t len);

    /**
     * @brief Starts a register write; the backend calls io_i2c_bus_complete() when it ends
     *
     * A failed start must not produce a completion notification.
     */
    enum io_status (*mem_write_it)(const void *config, uint8_t addr, uint8_t reg,
                                   const uint8_t *data, uint16_t len);
};

/** @brief I2C bus retaining references to its backend configuration and operations */
struct io_i2c_bus {
    const void *config;               ///< Backend configuration that must remain valid while in use
    const struct io_i2c_bus_ops *ops; ///< Non-NULL operations that must remain valid while in use
    io_i2c_cb_t cb;                   ///< Callback of the in-flight async transfer
    void *cb_user_data;               ///< Passed to cb
    volatile bool busy;               ///< Set while a blocking or async transfer owns the bus
};

/**
 * @brief Dispatches a synchronous register read to the bus backend
 *
 * Preconditions are checked by the device layer.
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
enum io_status io_i2c_bus_mem_read(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *data,
                                   uint16_t len, uint32_t timeout_ms);

/**
 * @brief Dispatches a synchronous register write to the bus backend
 *
 * Preconditions are checked by the device layer.
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
enum io_status io_i2c_bus_mem_write(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                    const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/** @brief Starts an async register read; see io_i2c_mem_read_it() */
enum io_status io_i2c_bus_mem_read_it(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *data,
                                      uint16_t len, io_i2c_cb_t cb, void *user_data);

/** @brief Starts an async register write; see io_i2c_mem_write_it() */
enum io_status io_i2c_bus_mem_write_it(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                       const uint8_t *data, uint16_t len, io_i2c_cb_t cb,
                                       void *user_data);

/**
 * @brief Clears the active async transfer, then invokes its callback with status
 *
 * Notifications received while idle or during a blocking transfer are ignored.
 * The backend must notify only for the current transfer; notifications do not carry a transfer ID.
 * Callback state is cleared before invocation so the callback can start another transfer.
 */
void io_i2c_bus_complete(io_i2c_bus_t *bus, enum io_status status);

#endif // ULYSSES_IO_I2C_I2C_BUS_H
