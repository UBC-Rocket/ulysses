#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "imu.h"

enum uls_status dev_imu_read(dev_imu_t *imu, struct dev_imu_sample *sample)
{
    ULYSSES_ASSERT(imu != NULL, "IMU cannot be NULL");
    ULYSSES_ASSERT(imu->config != NULL && imu->ops != NULL, "IMU not initialized");
    ULYSSES_ASSERT(imu->ops->read != NULL, "IMU read function not provided");
    ULYSSES_ASSERT(sample != NULL, "Sample cannot be NULL");

    return imu->ops->read(imu, sample);
}

enum uls_status dev_imu_read_async(dev_imu_t *imu, dev_imu_cb_t cb, void *user_data)
{
    ULYSSES_ASSERT(imu != NULL, "IMU cannot be NULL");
    ULYSSES_ASSERT(imu->config != NULL && imu->ops != NULL, "IMU not initialized");
    ULYSSES_ASSERT(imu->ops->read_async != NULL, "IMU async read function not provided");
    ULYSSES_ASSERT(cb != NULL, "Callback function not initialized");

    return imu->ops->read_async(imu, cb, user_data);
}
