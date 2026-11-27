#ifndef ULYSSES_SVC_DEV_IMU_IMU_H
#define ULYSSES_SVC_DEV_IMU_IMU_H

#include "ulysses/svc/dev/imu.h"

/** @brief Sensor-specific reads; caller arguments are checked by the public IMU API */
struct dev_imu_ops {
    /** @brief Implements dev_imu_read() */
    enum uls_status (*read)(dev_imu_t *imu, struct dev_imu_sample *sample);
    /** @brief Implements dev_imu_read_async() */
    enum uls_status (*read_async)(dev_imu_t *imu, struct dev_imu_sample *sample, dev_imu_cb_t cb,
                                  void *user_data);
};

/** @brief IMU instance populated by the sensor-specific init function */
struct dev_imu {
    const void *config;            ///< Sensor configuration
    void *data;                    ///< Sensor runtime state
    const struct dev_imu_ops *ops; ///< Sensor operations, installed after successful initialization
};

#endif // ULYSSES_SVC_DEV_IMU_IMU_H
