#ifndef ULYSSES_SVC_DEV_LIDAR_LIDAR_H
#define ULYSSES_SVC_DEV_LIDAR_LIDAR_H

#include "ulysses/io/status.h"
#include "ulysses/svc/dev/lidar.h"

/** @brief Vendor-specific operations behind the generic LiDAR API */
struct dev_lidar_ops {
    /** @brief Like dev_lidar_read(), with arguments already checked */
    enum io_status (*read)(const dev_lidar_t *lidar, struct dev_lidar_sample *sample);
};

/** @brief LiDAR retaining references to its backend configuration and operations */
struct dev_lidar {
    const void *config;              ///< Backend configuration that must remain valid while in use
    const struct dev_lidar_ops *ops; ///< Non-NULL operations that must remain valid while in use
};

#endif // ULYSSES_SVC_DEV_LIDAR_LIDAR_H
