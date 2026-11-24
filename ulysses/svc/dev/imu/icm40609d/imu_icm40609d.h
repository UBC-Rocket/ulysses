#ifndef ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H
#define ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H

#include <stdint.h>
#include <stdatomic.h>

#include "ulysses/io/spi.h"
#include "ulysses/io/status.h"
#include "ulysses/svc/dev/imu.h"

struct dev_imu_icm40609d_config {
    io_spi_dev_t *spi;
};

struct dev_imu_icm40609d_data {
    uint8_t rx_data[13]; ///< Receive buffer of the current non-blocking read
    atomic_bool busy;    ///< A non-blocking read is in progress
    dev_imu_cb_t cb;     ///< Callback of the current non-blocking read
    void *cb_user_data;  ///< User data passed to cb
};

enum io_status dev_imu_icm40609d_init(dev_imu_t *imu, const struct dev_imu_icm40609d_config *config,
                                      struct dev_imu_icm40609d_data *data);

#endif // ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H
