#ifndef ULYSSES_IO_I2C
#define ULYSSES_IO_I2C

#include <stdint.h>

#include "ulysses/io/status.h"

/** @brief I2C bus backed by platform-specific operations */
typedef struct io_i2c_bus io_i2c_bus_t;

/** @brief I2C device identified by its bus and 7-bit address */
typedef struct io_i2c_dev io_i2c_dev_t;

/** @brief Async transfer completion callback; runs in interrupt context */
typedef void (*io_i2c_cb_t)(enum io_status status, void *user_data);

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

/**
 * @brief Starts a non-blocking I2C register read.
 *
 * Returns once the transfer is started. Completion is
 * reported through cb, which runs in interrupt context.
 * cb is not called if the operation fails to start.
 *
 * @param dev Non-NULL device on an initialized bus
 * @param reg Starting 8-bit register address
 * @param[out] data Output buffer with space for len bytes;
 *                  must remain valid until completion
 * @param len Number of bytes to read; must be > 0
 * @param cb Non-NULL completion callback
 * @param user_data Context passed to cb
 *
 * @return IO_STATUS_OK if started, IO_STATUS_BUSY if
 *         the bus is occupied, or another error status
 */
enum io_status io_i2c_mem_read_it(const io_i2c_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len,
                                  io_i2c_cb_t cb, void *user_data);

/**
 * @brief Starts a non-blocking I2C register write.
 *
 * Returns once the transfer is started. Completion is
 * reported through cb, which runs in interrupt context.
 * cb is not called if the operation fails to start.
 *
 * @param dev Non-NULL device on an initialized bus
 * @param reg Starting 8-bit register address
 * @param data Input buffer containing len bytes;
 *             must remain valid until completion
 * @param len Number of bytes to write; must be > 0
 * @param cb Non-NULL completion callback
 * @param user_data Context passed to cb
 *
 * @return IO_STATUS_OK if started, IO_STATUS_BUSY if
 *         the bus is occupied, or another error status
 */
enum io_status io_i2c_mem_write_it(const io_i2c_dev_t *dev, uint8_t reg, const uint8_t *data,
                                   uint16_t len, io_i2c_cb_t cb, void *user_data);

#endif // ULYSSES_IO_I2C
