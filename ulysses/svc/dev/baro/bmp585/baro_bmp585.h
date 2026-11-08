#ifndef ULYSSES_SVC_DEV_BARO_BMP585_BARO_BMP585_H
#define ULYSSES_SVC_DEV_BARO_BMP585_BARO_BMP585_H

#include "ulysses/io/i2c.h"
#include "ulysses/io/status.h"
#include "ulysses/svc/dev/baro.h"

/** @brief Wiring of a BMP585 barometer */
struct dev_baro_bmp585_config {
    const io_i2c_dev_t *i2c; ///< I2C device the sensor is connected to
};

/**
 * @brief Resets the BMP585, checks its ID and NVM, and starts sampling at 50 Hz
 *
 * Blocks for about 10 ms.
 *
 * @param[out] baro Barometer to initialize
 * @param config Wiring of the sensor, must stay valid while the barometer is used
 *
 * @return Transfer status, or IO_STATUS_ERROR if a check fails
 */
enum io_status dev_baro_bmp585_init(dev_baro_t *baro, const struct dev_baro_bmp585_config *config);

#endif // ULYSSES_SVC_DEV_BARO_BMP585_BARO_BMP585_H
