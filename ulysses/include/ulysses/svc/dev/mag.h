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

struct dev_mag;

/** @brief Sensor-specific function, see dev_mag_read() */
typedef enum io_status (*dev_mag_read_fn_t)(struct dev_mag *mag, struct dev_mag_sample *sample);

/** @brief Sensor-specific function, see dev_mag_is_data_ready() */
typedef bool (*dev_mag_is_data_ready_fn_t)(struct dev_mag *mag);

/** @brief Magnetometer, filled in by a sensor-specific init function */
struct dev_mag {
    void *data;                               ///< Sensor-specific state
    const void *config;                       ///< Sensor-specific configuration
    dev_mag_read_fn_t read;                   ///< Sensor-specific read function
    dev_mag_is_data_ready_fn_t is_data_ready; ///< Sensor-specific data ready function
};

/**
 * @brief Reads the latest magnetic field sample
 *
 * @param mag Magnetometer to read from
 * @param[out] sample Latest magnetic field sample
 *
 * @return IO_STATUS_OK on success, otherwise an IO_STATUS_* error code
 */
enum io_status dev_mag_read(struct dev_mag *mag, struct dev_mag_sample *sample);

/**
 * @brief Checks whether a new magnetic field sample is ready to read
 *
 * @param mag Magnetometer to check
 *
 * @return true if a new sample is ready, false if not or if the check failed
 */
bool dev_mag_is_data_ready(struct dev_mag *mag);

#endif // ULYSSES_SVC_DEV_MAG
