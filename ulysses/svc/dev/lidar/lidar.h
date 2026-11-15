#ifndef ULYSSES_SVC_DEV_LIDAR_LIDAR_H
#define ULYSSES_SVC_DEV_LIDAR_LIDAR_H

#include "ulysses/io/status.h"
#include "ulysses/svc/dev/lidar.h"

struct dev_lidar_ops {
    enum io_status (*read)(const dev_lidar_t *lidar, struct dev_lidar_sample *sample);
};

struct dev_lidar {
    const void *config;              ///< Sensor-specific configuration
    const struct dev_lidar_ops *ops; ///< Sensor-specific functions, NULL until initialized
};

#endif // ULYSSES_SVC_DEV_LIDAR_LIDAR_H
