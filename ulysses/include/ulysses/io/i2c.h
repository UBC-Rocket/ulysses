#ifndef ULYSSES_IO_I2C
#define ULYSSES_IO_I2C

#include <stdint.h>

#include "ulysses/io/status.h"

/** @brief I2C bus backed by platform-specific operations */
typedef struct io_i2c_bus io_i2c_bus_t;

/** @brief I2C device identified by its bus and 7-bit address */
typedef struct io_i2c_dev io_i2c_dev_t;

/**
 * @brief Completion callback for a non-blocking I2C transfer
 *
 * Runs in interrupt context.
 *
 * @param status Final transfer status
 * @param user_data Data given when the transfer was started
 */
typedef void (*io_i2c_cb_t)(enum io_status status, void *user_data);

/**
 * @brief Reads bytes starting at an 8-bit device register
 *
 * Blocks until the backend transfer completes or fails. Returns IO_STATUS_BUSY if an async
 * transfer is in flight on the same bus.
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
 * Blocks until the backend transfer completes or fails. Returns IO_STATUS_BUSY if an async
 * transfer is in flight on the same bus.
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

/**
 * @brief Starts a non-blocking read of bytes starting at an 8-bit device register
 *
 * Returns once the transfer has started. cb runs in interrupt context with the final status, and
 * is not called if the transfer fails to start. Only one async transfer can be in flight per bus.
 *
 * @param dev Non-NULL device on an initialized bus, with an unshifted address in 0x01-0x7F
 * @param reg Starting register address
 * @param[out] data Non-NULL output buffer with space for at least len bytes; must remain valid
 *                  until cb runs
 * @param len Number of bytes to read; must be greater than zero
 * @param cb Non-NULL completion callback
 * @param user_data Data passed to cb
 *
 * @retval IO_STATUS_OK The transfer started and cb will be called
 * @retval IO_STATUS_BUSY A transfer is already in flight on the bus
 * @retval IO_STATUS_ERROR The transfer could not be started
 */
enum io_status io_i2c_mem_read_async(const io_i2c_dev_t *dev, uint8_t reg, uint8_t *data,
                                     uint16_t len, io_i2c_cb_t cb, void *user_data);

/**
 * @brief Starts a non-blocking write of bytes starting at an 8-bit device register
 *
 * Returns once the transfer has started. cb runs in interrupt context with the final status, and
 * is not called if the transfer fails to start. Only one async transfer can be in flight per bus.
 *
 * @param dev Non-NULL device on an initialized bus, with an unshifted address in 0x01-0x7F
 * @param reg Starting register address
 * @param data Non-NULL input buffer containing at least len bytes; must remain valid until cb runs
 * @param len Number of bytes to write; must be greater than zero
 * @param cb Non-NULL completion callback
 * @param user_data Data passed to cb
 *
 * @retval IO_STATUS_OK The transfer started and cb will be called
 * @retval IO_STATUS_BUSY A transfer is already in flight on the bus
 * @retval IO_STATUS_ERROR The transfer could not be started
 */
enum io_status io_i2c_mem_write_async(const io_i2c_dev_t *dev, uint8_t reg, const uint8_t *data,
                                      uint16_t len, io_i2c_cb_t cb, void *user_data);

#endif // ULYSSES_IO_I2C
