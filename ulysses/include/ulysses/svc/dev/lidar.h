#ifndef ULYSSES_SVC_DEV_LIDAR
#define ULYSSES_SVC_DEV_LIDAR

#include <stdbool.h>

#include "ulysses/io/status.h"

/** @brief Lidar backed by vendor-specific operations */
typedef struct dev_lidar dev_lidar_t;

/** @brief One distance measurement */
struct dev_lidar_sample {
    float distance_m; ///< Distance to the target in metres; only meaningful if is_valid is set
    bool is_valid;    ///< Whether the sensor reported a usable distance
};

/**
 * @brief Checks whether a new measurement is ready to be read
 *
 * @param lidar Non-NULL lidar initialized by a vendor backend
 * @param[out] is_ready Non-NULL flag set to true if a new measurement is ready
 *
 * @return Status returned by the backend
 */
enum io_status dev_lidar_is_data_ready(dev_lidar_t *lidar, bool *is_ready);

/**
 * @brief Reads the latest measurement
 *
 * A failed transfer is reported through the return value. A completed transfer without a usable
 * target is reported through the sample's is_valid flag instead.
 *
 * @param lidar Non-NULL lidar initialized by a vendor backend
 * @param[out] sample Non-NULL sample to fill
 *
 * @return Status returned by the backend
 */
enum io_status dev_lidar_read(dev_lidar_t *lidar, struct dev_lidar_sample *sample);

#endif // ULYSSES_SVC_DEV_LIDAR
