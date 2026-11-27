#ifndef ULYSSES_SVC_DEV_LIDAR
#define ULYSSES_SVC_DEV_LIDAR

#include <stdbool.h>

#include "ulysses/io/status.h"

/** @brief One distance measurement */
struct dev_lidar_sample {
    float distance_m; ///< Distance to the nearest target in metres, only meaningful if is_valid
    bool is_valid;    ///< Whether a new measurement found a target
};

/** @brief Lidar, initialized by a sensor-specific init function */
typedef struct dev_lidar dev_lidar_t;

/**
 * @brief Reads the latest measurement
 *
 * If no new measurement is ready yet, or it has no usable target, the sample's is_valid flag is
 * false and the status is still IO_STATUS_OK.
 *
 * @param lidar Non-NULL initialized lidar
 * @param[out] sample Non-NULL sample to fill
 *
 * @return Transfer status
 */
enum io_status dev_lidar_read(const dev_lidar_t *lidar, struct dev_lidar_sample *sample);

#endif // ULYSSES_SVC_DEV_LIDAR
