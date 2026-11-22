#ifndef ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H
#define ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H

// ST's VL53L4CX driver
#include "vl53l4cx.h"

#include "ulysses/io/gpio.h"
#include "ulysses/io/status.h"
#include "ulysses/svc/dev/lidar.h"

/** @brief Wiring of a VL53L4CX lidar */
struct dev_lidar_vl53l4cx_config {
    VL53L4CX_IO_t *bus;          ///< Bus functions for ST's driver
    const io_gpio_dev_t *enable; ///< Enable (XSHUT) pin, high powers the sensor on
};

/** @brief State of a VL53L4CX lidar, filled in by dev_lidar_vl53l4cx_init() */
struct dev_lidar_vl53l4cx_data {
    VL53L4CX_Object_t sensor; ///< ST's driver state, about 9 KB
};

/**
 * @brief Powers on the VL53L4CX, checks its ID, and starts continuous long-range measurements
 *
 * Uses a 33 ms timing budget. Blocks for a few milliseconds while ST's driver boots and configures
 * the sensor; ST's waits busy-loop on the bus tick.
 *
 * Once started, dev_lidar_read() never blocks. It reports an invalid sample when no new
 * measurement is ready yet. ST's driver ignores bus errors while checking for new data, so a sensor
 * that stops responding also shows up as invalid samples.
 *
 * @param[out] lidar Lidar to initialize
 * @param config Wiring of the sensor, must stay valid while the lidar is used
 * @param[out] data State of the sensor, must stay valid while the lidar is used
 *
 * @return IO_STATUS_TIMEOUT if ST's driver timed out, IO_STATUS_ERROR if any other step or the ID
 *         check fails, otherwise IO_STATUS_OK
 */
enum io_status dev_lidar_vl53l4cx_init(dev_lidar_t *lidar,
                                       const struct dev_lidar_vl53l4cx_config *config,
                                       struct dev_lidar_vl53l4cx_data *data);

#endif // ULYSSES_SVC_DEV_LIDAR_VL53L4CX_LIDAR_VL53L4CX_H
