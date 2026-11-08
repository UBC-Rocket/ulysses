#ifndef ULYSSES_SVC_DEV_BARO
#define ULYSSES_SVC_DEV_BARO

#include "ulysses/io/status.h"

/** @brief One pressure and temperature measurement */
struct dev_baro_sample {
    float pressure_pa; ///< Pressure in pascals
    float temp_c;      ///< Temperature in degrees Celsius
};

/** @brief Barometer, initialized by a sensor-specific init function */
typedef struct dev_baro dev_baro_t;

/**
 * @brief Reads the latest measurement
 *
 * @param baro Non-NULL initialized barometer
 * @param[out] sample Non-NULL sample to fill
 *
 * @return Transfer status
 */
enum io_status dev_baro_read(const dev_baro_t *baro, struct dev_baro_sample *sample);

#endif // ULYSSES_SVC_DEV_BARO
