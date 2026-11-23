#ifndef ULYSSES_SVC_DEV_BARO_BMP585_BARO_BMP585_H
#define ULYSSES_SVC_DEV_BARO_BMP585_BARO_BMP585_H

#include <stdatomic.h>
#include <stdint.h>

#include "ulysses/io/i2c.h"
#include "ulysses/status.h"
#include "ulysses/svc/dev/baro.h"

/** @brief Wiring of a BMP585 barometer */
struct dev_baro_bmp585_config {
    const io_i2c_dev_t *i2c; ///< I2C device the sensor is connected to
};

/** @brief State of a BMP585 barometer, filled in by dev_baro_bmp585_init() */
struct dev_baro_bmp585_data {
    uint8_t rx_data[6]; ///< Receive buffer of the current non-blocking read
    atomic_bool busy;   ///< A non-blocking read is in progress
    dev_baro_cb_t cb;   ///< Callback of the current non-blocking read
    void *cb_user_data; ///< User data passed to cb
};

/**
 * @brief Resets the BMP585, checks its ID and NVM, and starts sampling at 50 Hz
 *
 * Blocks for about 10 ms.
 *
 * @param[out] baro Barometer to initialize
 * @param config Wiring of the sensor, must stay valid while the barometer is used
 * @param[out] data State of the sensor, must stay valid while the barometer is used
 *
 * @return Transfer status, or ULS_STATUS_ERROR if a check fails
 */
enum uls_status dev_baro_bmp585_init(dev_baro_t *baro, const struct dev_baro_bmp585_config *config,
                                     struct dev_baro_bmp585_data *data);

#endif // ULYSSES_SVC_DEV_BARO_BMP585_BARO_BMP585_H
