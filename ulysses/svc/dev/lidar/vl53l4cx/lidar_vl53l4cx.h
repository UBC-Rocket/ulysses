#ifndef ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H
#define ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H

#include "ulysses/io/i2c.h"
#include "ulysses/io/status.h"
#include "ulysses/svc/dev/lidar.h"

/** @brief Wiring of a VL53L4CX lidar */
struct dev_lidar_vl53l4cx_config {
    const io_i2c_dev_t *i2c; ///< I2C device the sensor is connected to
};

/**
 * @brief Links a lidar to the VL53L4CX backend
 *
 * Doesn't communicate with the sensor yet.
 *
 * @param[out] lidar Lidar to initialize
 * @param config Wiring of the sensor, must stay valid while the lidar is used
 *
 * @return IO_STATUS_OK
 */
enum io_status dev_lidar_vl53l4cx_init(dev_lidar_t *lidar,
                                       const struct dev_lidar_vl53l4cx_config *config);

#endif // ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H
