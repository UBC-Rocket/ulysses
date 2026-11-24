#ifndef ULYSSES_SVC_DEV_IMU_IMU_H
#define ULYSSES_SVC_DEV_IMU_IMU_H

#include "ulysses/include/ulysses/svc/dev/imu.h"

struct dev_imu_ops {
    enum io_status (*read)(dev_imu_t *imu, struct dev_imu_sample *sample);
    enum io_status (*read_async)(dev_imu_t *imu, dev_imu_cb_t cb, void *user_data);
}; 

struct dev_imu {
    const void *config; 
    void *data;
    const struct dev_imu_ops *ops;
};


#endif // ULYSSES_SVC_DEV_IMU_IMU_H