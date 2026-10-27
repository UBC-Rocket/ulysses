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
 * @brief Like io_i2c_mem_read(), but returns once started and reports the result through cb
 *
 * cb is not called if this returns an error. Returns IO_STATUS_BUSY if the bus has a transfer in
 * flight.
 *
 * @param[out] data Buffer that must stay valid until cb runs
 * @param cb Non-NULL completion callback
 * @param user_data Passed to cb
 */
enum io_status io_i2c_mem_read_it(const io_i2c_dev_t *dev, uint8_t reg, uint8_t *data,
                                     uint16_t len, io_i2c_cb_t cb, void *user_data);

/**
 * @brief Like io_i2c_mem_write(), but returns once started and reports the result through cb
 *
 * cb is not called if this returns an error. Returns IO_STATUS_BUSY if the bus has a transfer in
 * flight.
 *
 * @param data Buffer that must stay valid until cb runs
 * @param cb Non-NULL completion callback
 * @param user_data Passed to cb
 */
enum io_status io_i2c_mem_write_it(const io_i2c_dev_t *dev, uint8_t reg, const uint8_t *data,
                                      uint16_t len, io_i2c_cb_t cb, void *user_data);

#endif // ULYSSES_IO_I2C
