#ifndef ULYSSES_SVC_DEV_IMU_H
#define ULYSSES_SVC_DEV_IMU_H

#include "ulysses/status.h"

/** @brief One accelerometer and gyroscope measurement in sensor X, Y, Z order */
struct dev_imu_sample {
    float accel_m_s2[3]; ///< Acceleration in m/s^2
    float gyro_rad_s[3]; ///< Angular velocity in rad/s
};

/** @brief IMU, initialized by a sensor-specific init function */
typedef struct dev_imu dev_imu_t;

/**
 * @brief Completion callback for a non-blocking read, runs in interrupt context
 *
 * @param status Read status
 * @param sample Measurement, valid only until the callback returns; NULL on failure
 * @param user_data Context passed to dev_imu_read_async()
 */
typedef void (*dev_imu_cb_t)(enum uls_status status, const struct dev_imu_sample *sample,
                             void *user_data);

/**
 * @brief Reads the latest measurement, blocking until the transfer finishes
 *
 * @param imu Non-NULL initialized IMU
 * @param[out] sample Non-NULL sample to fill, unchanged on failure
 *
 * @return ULS_STATUS_OK on success, ULS_STATUS_BUSY if the IMU or bus is in use, otherwise
 *         an ULS_STATUS_* error code
 */
enum uls_status dev_imu_read(dev_imu_t *imu, struct dev_imu_sample *sample);

/**
 * @brief Starts reading the latest measurement, without blocking
 *
 * cb runs in interrupt context when the read finishes. It is not called if the read fails to
 * start. The IMU and user_data must stay valid until cb returns.
 *
 * @param imu Non-NULL initialized IMU
 * @param cb Non-NULL completion callback
 * @param user_data Context passed to cb, may be NULL
 *
 * @return ULS_STATUS_OK if the read started, ULS_STATUS_BUSY if the IMU or bus is in use,
 *         otherwise an ULS_STATUS_* error code
 */
enum uls_status dev_imu_read_async(dev_imu_t *imu, dev_imu_cb_t cb, void *user_data);

#endif // ULYSSES_SVC_DEV_IMU_H
