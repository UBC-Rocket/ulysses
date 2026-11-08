#ifndef ULYSSES_SVC_DEV_MAG
#define ULYSSES_SVC_DEV_MAG

#include <stdbool.h>

#include "ulysses/io/status.h"

/** @brief Magnetic field sample in the sensor's own axes */
struct dev_mag_sample {
    float x; ///< Field along the X axis in gauss
    float y; ///< Field along the Y axis in gauss
    float z; ///< Field along the Z axis in gauss
};

/** @brief Magnetometer, initialized by a sensor-specific init function */
typedef struct dev_mag dev_mag_t;

/**
 * @brief Reads the latest magnetic field sample
 *
 * @param mag Magnetometer to read from
 * @param[out] sample Latest magnetic field sample
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status dev_mag_read(dev_mag_t *mag, struct dev_mag_sample *sample);

/**
 * @brief Checks whether a new magnetic field sample is ready to read
 *
 * @param mag Magnetometer to check
 *
 * @return true if a new sample is ready, false if not or if the check failed
 */
bool dev_mag_is_data_ready(dev_mag_t *mag);

#endif // ULYSSES_SVC_DEV_MAG
