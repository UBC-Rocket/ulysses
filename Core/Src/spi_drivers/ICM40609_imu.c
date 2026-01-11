/**
 * @file ICM40609_imu.c
 * @brief ICM-40609-D protocol layer implementation.
 *
 * Packet builders and parsers for the ICM-40609-D IMU.
 *
 * SPI convention:
 * - Write: first byte = (addr & 0x7F), then data byte(s)
 * - Read:  first byte = (addr | 0x80), then N dummy 0x00 to clock out N data bytes
 *
 * UBC Rocket, Jan 2026
 */

#include "ICM40609_imu.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static inline uint8_t spi_wr(uint8_t reg) { return (uint8_t)(reg & 0x7F); }
static inline uint8_t spi_rd(uint8_t reg) { return (uint8_t)(reg | 0x80); }

/**
 * @brief Get accel scale factor (g/LSB) from FSR setting.
 */
static float accel_fsr_to_scale(icm40609_accel_fsr_t fsr)
{
    switch (fsr) {
        case ICM40609_ACCEL_FSR_32G: return 32.0f / 32768.0f;
        case ICM40609_ACCEL_FSR_16G: return 16.0f / 32768.0f;
        case ICM40609_ACCEL_FSR_8G:  return 8.0f / 32768.0f;
        case ICM40609_ACCEL_FSR_4G:  return 4.0f / 32768.0f;
        default: return 16.0f / 32768.0f;
    }
}

/**
 * @brief Get gyro scale factor (dps/LSB) from FSR setting.
 */
static float gyro_fsr_to_scale(icm40609_gyro_fsr_t fsr)
{
    switch (fsr) {
        case ICM40609_GYRO_FSR_2000DPS:  return 2000.0f / 32768.0f;
        case ICM40609_GYRO_FSR_1000DPS:  return 1000.0f / 32768.0f;
        case ICM40609_GYRO_FSR_500DPS:   return 500.0f / 32768.0f;
        case ICM40609_GYRO_FSR_250DPS:   return 250.0f / 32768.0f;
        case ICM40609_GYRO_FSR_125DPS:   return 125.0f / 32768.0f;
        case ICM40609_GYRO_FSR_62_5DPS:  return 62.5f / 32768.0f;
        case ICM40609_GYRO_FSR_31_25DPS: return 31.25f / 32768.0f;
        default: return 1000.0f / 32768.0f;
    }
}

/**
 * @brief Get accel ODR in Hz from ODR code.
 */
static uint16_t accel_odr_to_hz(icm40609_accel_odr_t odr)
{
    switch (odr) {
        case ICM40609_ACCEL_ODR_32KHZ:  return 32000;
        case ICM40609_ACCEL_ODR_16KHZ:  return 16000;
        case ICM40609_ACCEL_ODR_8KHZ:   return 8000;
        case ICM40609_ACCEL_ODR_4KHZ:   return 4000;
        case ICM40609_ACCEL_ODR_2KHZ:   return 2000;
        case ICM40609_ACCEL_ODR_1KHZ:   return 1000;
        case ICM40609_ACCEL_ODR_500HZ:  return 500;
        case ICM40609_ACCEL_ODR_200HZ:  return 200;
        case ICM40609_ACCEL_ODR_100HZ:  return 100;
        case ICM40609_ACCEL_ODR_50HZ:   return 50;
        case ICM40609_ACCEL_ODR_25HZ:   return 25;
        case ICM40609_ACCEL_ODR_12_5HZ: return 12;
        default: return 1000;
    }
}

/**
 * @brief Get gyro ODR in Hz from ODR code.
 */
static uint16_t gyro_odr_to_hz(icm40609_gyro_odr_t odr)
{
    switch (odr) {
        case ICM40609_GYRO_ODR_32KHZ:  return 32000;
        case ICM40609_GYRO_ODR_16KHZ:  return 16000;
        case ICM40609_GYRO_ODR_8KHZ:   return 8000;
        case ICM40609_GYRO_ODR_4KHZ:   return 4000;
        case ICM40609_GYRO_ODR_2KHZ:   return 2000;
        case ICM40609_GYRO_ODR_1KHZ:   return 1000;
        case ICM40609_GYRO_ODR_500HZ:  return 500;
        case ICM40609_GYRO_ODR_200HZ:  return 200;
        case ICM40609_GYRO_ODR_100HZ:  return 100;
        case ICM40609_GYRO_ODR_50HZ:   return 50;
        case ICM40609_GYRO_ODR_25HZ:   return 25;
        case ICM40609_GYRO_ODR_12_5HZ: return 12;
        default: return 1000;
    }
}

/* -------------------------------------------------------------------------- */
/* Basic Packet Builders                                                      */
/* -------------------------------------------------------------------------- */

size_t icm40609_build_read(uint8_t reg, uint8_t *tx, size_t n_data)
{
    tx[0] = spi_rd(reg);
    memset(&tx[1], 0x00, n_data);
    return 1 + n_data;
}

size_t icm40609_build_write(uint8_t reg, uint8_t val, uint8_t *tx)
{
    tx[0] = spi_wr(reg);
    tx[1] = val;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* High-Level Packet Builders                                                 */
/* -------------------------------------------------------------------------- */

size_t icm40609_build_softreset(uint8_t *tx)
{
    return icm40609_build_write(ICM40609_REG_DEVICE_CONFIG,
                                ICM40609_SOFT_RESET_VALUE, tx);
}

size_t icm40609_build_read_whoami(uint8_t *tx)
{
    return icm40609_build_read(ICM40609_REG_WHO_AM_I, tx, 1);
}

size_t icm40609_build_read_data(uint8_t *tx)
{
    /* Burst read starting at TEMP_DATA1:
     * TEMP(2) + ACCEL_XYZ(6) + GYRO_XYZ(6) = 14 bytes
     */
    return icm40609_build_read(ICM40609_REG_TEMP_DATA1, tx, 14);
}

size_t icm40609_build_accel_config(icm40609_accel_fsr_t fsr,
                                   icm40609_accel_odr_t odr,
                                   uint8_t *tx,
                                   icm40609_t *dev)
{
    /* ACCEL_CONFIG0: [7:5] = FS_SEL, [3:0] = ODR */
    uint8_t val = (uint8_t)(((fsr & 0x07) << 5) | (odr & 0x0F));

    if (dev) {
        dev->accel_config0 = val;
        dev->accel_scale = accel_fsr_to_scale(fsr);
        dev->accel_odr_hz = accel_odr_to_hz(odr);
    }

    return icm40609_build_write(ICM40609_REG_ACCEL_CONFIG0, val, tx);
}

size_t icm40609_build_gyro_config(icm40609_gyro_fsr_t fsr,
                                  icm40609_gyro_odr_t odr,
                                  uint8_t *tx,
                                  icm40609_t *dev)
{
    /* GYRO_CONFIG0: [7:5] = FS_SEL, [3:0] = ODR */
    uint8_t val = (uint8_t)(((fsr & 0x07) << 5) | (odr & 0x0F));

    if (dev) {
        dev->gyro_config0 = val;
        dev->gyro_scale = gyro_fsr_to_scale(fsr);
        dev->gyro_odr_hz = gyro_odr_to_hz(odr);
    }

    return icm40609_build_write(ICM40609_REG_GYRO_CONFIG0, val, tx);
}

size_t icm40609_build_power_on(uint8_t *tx, icm40609_t *dev)
{
    /* PWR_MGMT0: [3:2] = GYRO_MODE, [1:0] = ACCEL_MODE
     * Low-noise mode for both = 0x0F (binary: 0000 1111)
     * - GYRO_MODE = 11 (low-noise)
     * - ACCEL_MODE = 11 (low-noise)
     */
    uint8_t val = 0x0F;

    if (dev) {
        dev->pwr_mgmt0 = val;
    }

    return icm40609_build_write(ICM40609_REG_PWR_MGMT0, val, tx);
}

size_t icm40609_build_int_config(icm40609_int_pin_t pin,
                                 icm40609_int_drive_t drive,
                                 icm40609_int_polarity_t polarity,
                                 uint8_t *tx)
{
    /* INT_CONFIG (0x14):
     * [2] INT1_DRIVE_CIRCUIT: 0=open-drain, 1=push-pull
     * [1] INT1_POLARITY: 0=active-low, 1=active-high
     * [5] INT2_DRIVE_CIRCUIT
     * [4] INT2_POLARITY
     */
    uint8_t val = 0;

    if (pin == ICM40609_INT1) {
        if (drive == ICM40609_INT_PUSH_PULL)
            val |= (1u << 2);
        if (polarity == ICM40609_INT_ACTIVE_HIGH)
            val |= (1u << 1);
    } else {
        if (drive == ICM40609_INT_PUSH_PULL)
            val |= (1u << 5);
        if (polarity == ICM40609_INT_ACTIVE_HIGH)
            val |= (1u << 4);
    }

    return icm40609_build_write(ICM40609_REG_INT_CONFIG, val, tx);
}

size_t icm40609_build_int_source0_drdy(uint8_t *tx)
{
    /* INT_SOURCE0 (0x65): [3] = DRDY_INT1_EN */
    return icm40609_build_write(ICM40609_REG_INT_SOURCE0, 0x08, tx);
}

size_t icm40609_build_disable_i2c(uint8_t *tx)
{
    /* INTF_CONFIG0 (0x4C): UI_SIFS_CFG[1:0] = 11 disables I2C
     * Bits [1:0] = 0b11 = disable I2C, SPI only
     */
    return icm40609_build_write(ICM40609_REG_INTF_CONFIG0, 0x30, tx);
}

size_t icm40609_build_intf_config1_normal(uint8_t *tx)
{
    /* INTF_CONFIG1 (0x4D): EN_TEST_MODE[1:0] = 01 for normal operation */
    return icm40609_build_write(ICM40609_REG_INTF_CONFIG1, 0x01, tx);
}

/* -------------------------------------------------------------------------- */
/* Parsers                                                                    */
/* -------------------------------------------------------------------------- */

uint8_t icm40609_parse_whoami(const uint8_t *rx_data)
{
    /* rx_data[0] = response to command byte, rx_data[1] = WHO_AM_I value */
    return rx_data[1];
}

float icm40609_temp_to_celsius(int16_t raw)
{
    /* Temperature conversion: Temp_C = (raw / 132.48) + 25 */
    return ((float)raw / 132.48f) + 25.0f;
}

bool icm40609_parse_data(const uint8_t *rx_data,
                         icm40609_sample_t *sample,
                         const icm40609_t *dev)
{
    if (!rx_data || !sample || !dev) return false;

    /* rx_data layout (after skipping command byte at rx[0]):
     * [1]  = TEMP_DATA1 (MSB)
     * [2]  = TEMP_DATA0 (LSB)
     * [3]  = ACCEL_X1 (MSB)
     * [4]  = ACCEL_X0 (LSB)
     * [5]  = ACCEL_Y1 (MSB)
     * [6]  = ACCEL_Y0 (LSB)
     * [7]  = ACCEL_Z1 (MSB)
     * [8]  = ACCEL_Z0 (LSB)
     * [9]  = GYRO_X1 (MSB)
     * [10] = GYRO_X0 (LSB)
     * [11] = GYRO_Y1 (MSB)
     * [12] = GYRO_Y0 (LSB)
     * [13] = GYRO_Z1 (MSB)
     * [14] = GYRO_Z0 (LSB)
     */

    /* Temperature (MSB first) */
    sample->temp_raw = (int16_t)((rx_data[1] << 8) | rx_data[2]);
    sample->temp_c = icm40609_temp_to_celsius(sample->temp_raw);

    /* Accelerometer (MSB first) */
    sample->ax_raw = (int16_t)((rx_data[3] << 8) | rx_data[4]);
    sample->ay_raw = (int16_t)((rx_data[5] << 8) | rx_data[6]);
    sample->az_raw = (int16_t)((rx_data[7] << 8) | rx_data[8]);

    /* Gyroscope (MSB first) */
    sample->gx_raw = (int16_t)((rx_data[9] << 8) | rx_data[10]);
    sample->gy_raw = (int16_t)((rx_data[11] << 8) | rx_data[12]);
    sample->gz_raw = (int16_t)((rx_data[13] << 8) | rx_data[14]);

    /* Convert accel to m/s² (scale is g/LSB, multiply by 9.80665 for m/s²) */
    const float kG = 9.80665f;
    sample->ax = (float)sample->ax_raw * dev->accel_scale * kG;
    sample->ay = (float)sample->ay_raw * dev->accel_scale * kG;
    sample->az = (float)sample->az_raw * dev->accel_scale * kG;

    /* Convert gyro to rad/s (scale is dps/LSB, multiply by pi/180 for rad/s) */
    const float kDeg2Rad = 0.017453292519943295769f;
    sample->gx = (float)sample->gx_raw * dev->gyro_scale * kDeg2Rad;
    sample->gy = (float)sample->gy_raw * dev->gyro_scale * kDeg2Rad;
    sample->gz = (float)sample->gz_raw * dev->gyro_scale * kDeg2Rad;

    return true;
}
