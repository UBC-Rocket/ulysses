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
 * @brief Completion callback for a non-blocking read, runs in interrupt context
 *
 * sample is NULL if status is not IO_STATUS_OK, and is only valid until the callback returns.
 */
typedef void (*dev_baro_cb_t)(enum io_status status, const struct dev_baro_sample *sample,
                              void *user_data);

/**
 * @brief Reads the latest measurement, blocking until the transfer finishes
 *
 * @param baro Non-NULL initialized barometer
 * @param[out] sample Non-NULL sample to fill
 *
 * @return Transfer status
 */
enum io_status dev_baro_read_blocking(dev_baro_t *baro, struct dev_baro_sample *sample);

/**
 * @brief Starts reading the latest measurement, without blocking
 *
 * cb runs in interrupt context when the read finishes. It is not called if the read fails to
 * start.
 *
 * @param baro Non-NULL initialized barometer
 * @param cb Non-NULL callback, called with the sample when the read finishes
 * @param user_data Passed to cb
 *
 * @return IO_STATUS_OK if the read started, IO_STATUS_BUSY if a read is in progress or the bus is
 *         busy, otherwise an IO_STATUS_* error code
 */
enum io_status dev_baro_read_async(dev_baro_t *baro, dev_baro_cb_t cb, void *user_data);

#endif // ULYSSES_SVC_DEV_BARO
