#ifndef ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H
#define ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H

#include <stdint.h>
#include <stdatomic.h>

#include "ulysses/io/spi.h"
#include "ulysses/io/status.h"
#include "ulysses/svc/dev/imu.h"

#define ICM_SAMPLE_LEN (12U)
#define ICM_TRANSFER_LEN    (ICM_SAMPLE_LEN + 1U)

enum icm40609d_accel_fsr {
    ICM_ACCEL_FSR_32G = 0x00,   ///< Default
    ICM_ACCEL_FSR_16G = 0x01,
    ICM_ACCEL_FSR_8G  = 0x02,
    ICM_ACCEL_FSR_4G  = 0x03,
};

enum icm40609d_gyro_fsr {
    ICM_GYRO_FSR_2000DPS   = 0x00,  ///< Default
    ICM_GYRO_FSR_1000DPS   = 0x01,
    ICM_GYRO_FSR_500DPS    = 0x02,
    ICM_GYRO_FSR_250DPS    = 0x03,
    ICM_GYRO_FSR_125DPS    = 0x04,
    ICM_GYRO_FSR_62_5DPS   = 0x05,
    ICM_GYRO_FSR_31_25DPS  = 0x06,
    ICM_GYRO_FSR_15_625DPS = 0x07,
};

enum icm40609d_odr {
    ICM_ODR_32KHZ   = 0x01,     ///< LN mode only
    ICM_ODR_16KHZ   = 0x02,     ///< LN mode only
    ICM_ODR_8KHZ    = 0x03,     ///< LN mode only
    ICM_ODR_4KHZ    = 0x04,     ///< LN mode only
    ICM_ODR_2KHZ    = 0x05,     ///< LN mode only
    ICM_ODR_1KHZ    = 0x06,     ///< LN mode only; Default
    ICM_ODR_200HZ   = 0x07,     ///< LP and LN modes
    ICM_ODR_100HZ   = 0x08,     ///< LP and LN modes
    ICM_ODR_50HZ    = 0x09,     ///< LP and LN modes
    ICM_ODR_25HZ    = 0x0A,     ///< LP and LN modes
    ICM_ODR_12_5HZ  = 0x0B,     ///< LP and LN modes
    ICM_ODR_6_25HZ   = 0x0C,    ///< LP mode only
    ICM_ODR_3_125HZ  = 0x0D,    ///< LP mode only
    ICM_ODR_1_5625HZ = 0x0E,    ///< LP mode only
    ICM_ODR_500HZ   = 0x0F,     ///< LP and LN modes
};

struct dev_imu_icm40609d_config {
    io_spi_dev_t *spi;

    enum icm40609d_accel_fsr accel_fsr;
    enum icm40609d_gyro_fsr gyro_fsr;
    enum icm40609d_odr odr;
};

struct dev_imu_icm40609d_data {
    uint8_t rx_data[ICM_TRANSFER_LEN]; ///< Receive buffer of the current non-blocking read
    atomic_bool busy;    ///< A non-blocking read is in progress
    dev_imu_cb_t cb;     ///< Callback of the current non-blocking read
    void *cb_user_data;  ///< User data passed to cb

    float accel_scale; ///< m/s^2 per LSB for accelerometer readings
    float gyro_scale;  ///< rad/s per LSB for gyroscope readings
};

enum io_status dev_imu_icm40609d_init(dev_imu_t *imu, const struct dev_imu_icm40609d_config *config,
                                      struct dev_imu_icm40609d_data *data);

#endif // ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H
