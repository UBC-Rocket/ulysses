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
     * @brief Starts a non-blocking device register read through the backend
     *
     * The backend must call io_i2c_bus_complete() once the started transfer finishes or fails.
     *
     * @param config Backend-specific configuration
     * @param addr Unshifted 7-bit device address
     * @param reg Starting 8-bit register address
     * @param[out] data Non-NULL output buffer that must remain valid until completion
     * @param len Number of bytes to read
     *
     * @return Status of starting the transfer
     */
    enum io_status (*mem_read_async)(const void *config, uint8_t addr, uint8_t reg, uint8_t *data,
                                     uint16_t len);

    /**
     * @brief Starts a non-blocking device register write through the backend
     *
     * The backend must call io_i2c_bus_complete() once the started transfer finishes or fails.
     *
     * @param config Backend-specific configuration
     * @param addr Unshifted 7-bit device address
     * @param reg Starting 8-bit register address
     * @param data Non-NULL input buffer that must remain valid until completion
     * @param len Number of bytes to write
     *
     * @return Status of starting the transfer
     */
    enum io_status (*mem_write_async)(const void *config, uint8_t addr, uint8_t reg,
                                      const uint8_t *data, uint16_t len);
};

/** @brief I2C bus retaining references to its backend configuration and operations */
struct io_i2c_bus {
    const void *config;               ///< Backend configuration that must remain valid while in use
    const struct io_i2c_bus_ops *ops; ///< Non-NULL operations that must remain valid while in use
    io_i2c_cb_t cb;                   ///< Completion callback of the in-flight async transfer
    void *cb_user_data;               ///< User data passed to cb
    volatile bool busy;               ///< True while an async transfer is in flight
};

/**
 * @brief Dispatches a synchronous register read to the bus backend
 *
 * Preconditions are not re-checked here; the device layer validates the transfer and the backend
 * init guarantees the operations are set.
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
 * Preconditions are not re-checked here; the device layer validates the transfer and the backend
 * init guarantees the operations are set.
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

/**
 * @brief Starts a non-blocking register read on the bus backend
 *
 * Preconditions are not re-checked here; the device layer validates the transfer and the backend
 * init guarantees the operations are set. cb is not called if the transfer fails to start.
 *
 * @param bus Non-NULL bus with a non-NULL mem_read_async operation
 * @param addr Unshifted 7-bit device address
 * @param reg Starting 8-bit register address
 * @param[out] data Non-NULL output buffer that must remain valid until cb runs
 * @param len Number of bytes to read
 * @param cb Non-NULL callback run from interrupt context when the transfer finishes or fails
 * @param user_data Data passed to cb
 *
 * @retval IO_STATUS_BUSY An async transfer is already in flight on the bus
 * @return Otherwise, the status of starting the transfer returned by the backend
 */
enum io_status io_i2c_bus_mem_read_async(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                         uint8_t *data, uint16_t len, io_i2c_cb_t cb,
                                         void *user_data);

/**
 * @brief Starts a non-blocking register write on the bus backend
 *
 * Preconditions are not re-checked here; the device layer validates the transfer and the backend
 * init guarantees the operations are set. cb is not called if the transfer fails to start.
 *
 * @param bus Non-NULL bus with a non-NULL mem_write_async operation
 * @param addr Unshifted 7-bit device address
 * @param reg Starting 8-bit register address
 * @param data Non-NULL input buffer that must remain valid until cb runs
 * @param len Number of bytes to write
 * @param cb Non-NULL callback run from interrupt context when the transfer finishes or fails
 * @param user_data Data passed to cb
 *
 * @retval IO_STATUS_BUSY An async transfer is already in flight on the bus
 * @return Otherwise, the status of starting the transfer returned by the backend
 */
enum io_status io_i2c_bus_mem_write_async(io_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                          const uint8_t *data, uint16_t len, io_i2c_cb_t cb,
                                          void *user_data);

/**
 * @brief Finishes the in-flight async transfer and runs its callback
 *
 * Called by backends from interrupt context. The bus is marked idle before the callback runs, so
 * the callback may start the next transfer.
 *
 * @param bus Non-NULL bus with an async transfer in flight
 * @param status Final transfer status passed to the callback
 */
void io_i2c_bus_complete(io_i2c_bus_t *bus, enum io_status status);

#endif // ULYSSES_IO_I2C_I2C_BUS_H
