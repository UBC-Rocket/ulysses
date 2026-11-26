#ifndef ULYSSES_SVC_DEV_IMU_H
#define ULYSSES_SVC_DEV_IMU_H

#include "ulysses/status.h"

// One measurement from the IMU device.
struct dev_imu_sample {
    float accel_m_s2[3]; ///< Acceleration in m/s^2
    float gyro_rad_s[3]; ///< Angular velocity in rad/s
};

// Opaque IMU Handle
typedef struct dev_imu dev_imu_t;

// Completion callback for a non-blocking read, runs in interrupt context
typedef void (*dev_imu_cb_t)(enum uls_status status, const struct dev_imu_sample *sample,
                             void *user_data);

enum uls_status dev_imu_read(dev_imu_t *imu, struct dev_imu_sample *sample);

enum uls_status dev_imu_read_async(dev_imu_t *imu, dev_imu_cb_t cb, void *user_data);

#endif // ULYSSES_SVC_DEV_IMU_H
