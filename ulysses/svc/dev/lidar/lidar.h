#ifndef ULYSSES_SVC_DEV_LIDAR_LIDAR_H
#define ULYSSES_SVC_DEV_LIDAR_LIDAR_H

#include <stdbool.h>

#include "ulysses/io/status.h"
#include "ulysses/svc/dev/lidar.h"

/** @brief Vendor-specific operations behind the generic LiDAR API */
struct dev_lidar_ops {
    /** @brief Like dev_lidar_is_data_ready(), with arguments already checked */
    enum io_status (*is_data_ready)(dev_lidar_t *lidar, bool *is_ready);

    /** @brief Like dev_lidar_read(), with arguments already checked */
    enum io_status (*read)(dev_lidar_t *lidar, struct dev_lidar_sample *sample);
};

/** @brief LiDAR retaining references to its backend state, configuration, and operations */
struct dev_lidar {
    void *data;                      ///< Backend runtime state, or NULL if the backend has none
    const void *config;              ///< Backend configuration that must remain valid while in use
    const struct dev_lidar_ops *ops; ///< Non-NULL operations that must remain valid while in use
};

#endif // ULYSSES_SVC_DEV_LIDAR_LIDAR_H
