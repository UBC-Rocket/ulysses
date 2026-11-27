#ifndef ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H
#define ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H

#include <stdint.h>
#include <stdatomic.h>

#include "ulysses/io/spi.h"
#include "ulysses/status.h"
#include "ulysses/svc/dev/imu.h"

#define ICM_SAMPLE_LEN (12U)                   ///< Six 16-bit accelerometer and gyroscope values
#define ICM_TRANSFER_LEN (ICM_SAMPLE_LEN + 1U) ///< SPI command byte plus sample bytes

/** @brief Accelerometer full-scale ranges (+/- g), encoded for ACCEL_FS_SEL */
enum icm40609d_accel_fsr {
    ICM_ACCEL_FSR_32G = 0x00, ///< Reset default
    ICM_ACCEL_FSR_16G = 0x01,
    ICM_ACCEL_FSR_8G = 0x02,
    ICM_ACCEL_FSR_4G = 0x03,
};

/** @brief Gyroscope full-scale ranges (+/- degrees/s), encoded for GYRO_FS_SEL */
enum icm40609d_gyro_fsr {
    ICM_GYRO_FSR_2000DPS = 0x00, ///< Reset default
    ICM_GYRO_FSR_1000DPS = 0x01,
    ICM_GYRO_FSR_500DPS = 0x02,
    ICM_GYRO_FSR_250DPS = 0x03,
    ICM_GYRO_FSR_125DPS = 0x04,
    ICM_GYRO_FSR_62_5DPS = 0x05,
    ICM_GYRO_FSR_31_25DPS = 0x06,
    ICM_GYRO_FSR_15_625DPS = 0x07,
};

/**
 * @brief Output data rate encodings for ACCEL_ODR and GYRO_ODR
 *
 * LP (low power) and LN (low noise) notes describe accelerometer modes. Rates below 12.5 Hz
 * are accelerometer-only and cannot be used by this driver, which enables both sensors in LN mode.
 * See datasheet DS-000330 v1.2, sections 13.30 and 13.31.
 */
enum icm40609d_odr {
    ICM_ODR_32KHZ = 0x01,    ///< LN mode only
    ICM_ODR_16KHZ = 0x02,    ///< LN mode only
    ICM_ODR_8KHZ = 0x03,     ///< LN mode only
    ICM_ODR_4KHZ = 0x04,     ///< LN mode only
    ICM_ODR_2KHZ = 0x05,     ///< LN mode only
    ICM_ODR_1KHZ = 0x06,     ///< LN mode only; reset default
    ICM_ODR_200HZ = 0x07,    ///< LP and LN modes
    ICM_ODR_100HZ = 0x08,    ///< LP and LN modes
    ICM_ODR_50HZ = 0x09,     ///< LP and LN modes
    ICM_ODR_25HZ = 0x0A,     ///< LP and LN modes
    ICM_ODR_12_5HZ = 0x0B,   ///< LP and LN modes
    ICM_ODR_6_25HZ = 0x0C,   ///< LP mode only
    ICM_ODR_3_125HZ = 0x0D,  ///< LP mode only
    ICM_ODR_1_5625HZ = 0x0E, ///< LP mode only
    ICM_ODR_500HZ = 0x0F,    ///< LP and LN modes
};

/** @brief SPI connection and sampling settings for an ICM40609D */
struct dev_imu_icm40609d_config {
    io_spi_dev_t *spi; ///< Initialized SPI device connected to the sensor

    enum icm40609d_accel_fsr accel_fsr; ///< Accelerometer full-scale range
    enum icm40609d_gyro_fsr gyro_fsr;   ///< Gyroscope full-scale range
    enum icm40609d_odr odr;             ///< Rate for both sensors; must be 12.5 Hz or higher
};

/** @brief Runtime state initialized by dev_imu_icm40609d_init() */
struct dev_imu_icm40609d_data {
    uint8_t rx_data[ICM_TRANSFER_LEN]; ///< Receive buffer of the current non-blocking read
    atomic_bool busy;                  ///< A blocking or non-blocking read is in progress
    dev_imu_cb_t cb;                   ///< Callback of the current non-blocking read
    void *cb_user_data;                ///< User data passed to cb

    float accel_scale; ///< m/s^2 per LSB for accelerometer readings
    float gyro_scale;  ///< rad/s per LSB for gyroscope readings

    struct dev_imu_sample *sample;
};

/**
 * @brief Resets the ICM40609D, checks its ID, and configures both sensors in low-noise mode
 *
 * Blocks during SPI transfers and startup delays. config and data must stay valid while the
 * IMU is used; config must remain unchanged.
 *
 * @param[out] imu Non-NULL IMU to initialize
 * @param config Non-NULL configuration with an initialized SPI device and ODR of at least 12.5 Hz
 * @param[out] data Non-NULL storage for runtime state
 *
 * @return ULS_STATUS_OK on success, ULS_STATUS_ERROR if identity or register verification fails,
 *         otherwise an ULS_STATUS_* transfer error code
 */
enum uls_status dev_imu_icm40609d_init(dev_imu_t *imu,
                                       const struct dev_imu_icm40609d_config *config,
                                       struct dev_imu_icm40609d_data *data);

#endif // ULYSSES_SVC_DEV_IMU_ICM40609D_IMU_ICM40609D_H
