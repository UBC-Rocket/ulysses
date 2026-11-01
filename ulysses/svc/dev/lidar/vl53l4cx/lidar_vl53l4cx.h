#ifndef ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H
#define ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H

#include "ulysses/io/i2c.h"
#include "ulysses/svc/dev/lidar.h"

/** @brief Configuration for vendor-specific code (VL53L4CX) */
struct dev_lidar_vl53l4cx_config {
    const io_i2c_dev_t *i2c; ///< Non-NULL device from the board wiring
};

/**
 * @brief Links a lidar to the VL53L4CX backend
 *
 * Stores config by reference; config must remain valid while the lidar is used. No
 * communication with the sensor.
 *
 * @param[out] lidar Non-NULL lidar to configure
 * @param config Non-NULL configuration
 */
void dev_lidar_vl53l4cx_init(dev_lidar_t *lidar, const struct dev_lidar_vl53l4cx_config *config);

#endif // ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H
