/**
 * @file ICM40609_imu.h
 * @brief Driver for the ICM-40609-D 6-axis IMU (accelerometer + gyroscope).
 *
 * This driver provides packet builders and parsers for the ICM-40609-D.
 * It is completely bus-agnostic - the device layer handles SPI transactions.
 *
 * Key features:
 * - 6-axis motion sensing (3-axis accel + 3-axis gyro)
 * - Up to 32 kHz ODR, SPI up to 24 MHz
 * - Temperature sensor
 * - FIFO support (up to 2K bytes)
 *
 * UBC Rocket, Jan 2026
 */

#ifndef ICM40609_IMU_H
#define ICM40609_IMU_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sync.h"

/* -------------------------------------------------------------------------- */
/* Register Map (Bank 0 - primary)                                            */
/* -------------------------------------------------------------------------- */

#define ICM40609_REG_DEVICE_CONFIG      0x11
#define ICM40609_REG_INT_CONFIG         0x14
#define ICM40609_REG_FIFO_CONFIG        0x16

#define ICM40609_REG_TEMP_DATA1         0x1D  /**< Temperature MSB */
#define ICM40609_REG_TEMP_DATA0         0x1E  /**< Temperature LSB */
#define ICM40609_REG_ACCEL_DATA_X1      0x1F  /**< Accel X MSB */
#define ICM40609_REG_ACCEL_DATA_X0      0x20  /**< Accel X LSB */
#define ICM40609_REG_ACCEL_DATA_Y1      0x21  /**< Accel Y MSB */
#define ICM40609_REG_ACCEL_DATA_Y0      0x22  /**< Accel Y LSB */
#define ICM40609_REG_ACCEL_DATA_Z1      0x23  /**< Accel Z MSB */
#define ICM40609_REG_ACCEL_DATA_Z0      0x24  /**< Accel Z LSB */
#define ICM40609_REG_GYRO_DATA_X1       0x25  /**< Gyro X MSB */
#define ICM40609_REG_GYRO_DATA_X0       0x26  /**< Gyro X LSB */
#define ICM40609_REG_GYRO_DATA_Y1       0x27  /**< Gyro Y MSB */
#define ICM40609_REG_GYRO_DATA_Y0       0x28  /**< Gyro Y LSB */
#define ICM40609_REG_GYRO_DATA_Z1       0x29  /**< Gyro Z MSB */
#define ICM40609_REG_GYRO_DATA_Z0       0x2A  /**< Gyro Z LSB */

#define ICM40609_REG_INT_STATUS         0x2D
#define ICM40609_REG_FIFO_COUNTH        0x2E
#define ICM40609_REG_FIFO_COUNTL        0x2F
#define ICM40609_REG_FIFO_DATA          0x30

#define ICM40609_REG_INT_STATUS2        0x37

#define ICM40609_REG_SIGNAL_PATH_RESET  0x4B
#define ICM40609_REG_INTF_CONFIG0       0x4C
#define ICM40609_REG_INTF_CONFIG1       0x4D
#define ICM40609_REG_PWR_MGMT0          0x4E
#define ICM40609_REG_GYRO_CONFIG0       0x4F
#define ICM40609_REG_ACCEL_CONFIG0      0x50
#define ICM40609_REG_GYRO_CONFIG1       0x51
#define ICM40609_REG_ACCEL_CONFIG1      0x53

#define ICM40609_REG_INT_SOURCE0        0x65
#define ICM40609_REG_INT_SOURCE1        0x66

#define ICM40609_REG_WHO_AM_I           0x75
#define ICM40609_REG_BANK_SEL           0x76

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

#define ICM40609_WHO_AM_I_VALUE         0x3B  /**< Expected WHO_AM_I response */
#define ICM40609_SOFT_RESET_VALUE       0x01  /**< Write to DEVICE_CONFIG for reset */

#define ICM40609_SAMPLE_Q_SIZE          32

/* -------------------------------------------------------------------------- */
/* Configuration Enums                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Accelerometer full-scale range.
 */
typedef enum {
    ICM40609_ACCEL_FSR_32G = 0x01,  /**< ±32g */
    ICM40609_ACCEL_FSR_16G = 0x02,  /**< ±16g */
    ICM40609_ACCEL_FSR_8G  = 0x03,  /**< ±8g */
    ICM40609_ACCEL_FSR_4G  = 0x04   /**< ±4g */
} icm40609_accel_fsr_t;

/**
 * @brief Accelerometer output data rate.
 */
typedef enum {
    ICM40609_ACCEL_ODR_32KHZ   = 0x01,
    ICM40609_ACCEL_ODR_16KHZ   = 0x02,
    ICM40609_ACCEL_ODR_8KHZ    = 0x03,
    ICM40609_ACCEL_ODR_4KHZ    = 0x04,
    ICM40609_ACCEL_ODR_2KHZ    = 0x05,
    ICM40609_ACCEL_ODR_1KHZ    = 0x06,
    ICM40609_ACCEL_ODR_500HZ   = 0x0F,
    ICM40609_ACCEL_ODR_200HZ   = 0x07,
    ICM40609_ACCEL_ODR_100HZ   = 0x08,
    ICM40609_ACCEL_ODR_50HZ    = 0x09,
    ICM40609_ACCEL_ODR_25HZ    = 0x0A,
    ICM40609_ACCEL_ODR_12_5HZ  = 0x0B
} icm40609_accel_odr_t;

/**
 * @brief Gyroscope full-scale range.
 */
typedef enum {
    ICM40609_GYRO_FSR_2000DPS = 0x00,  /**< ±2000 deg/s */
    ICM40609_GYRO_FSR_1000DPS = 0x01,  /**< ±1000 deg/s */
    ICM40609_GYRO_FSR_500DPS  = 0x02,  /**< ±500 deg/s */
    ICM40609_GYRO_FSR_250DPS  = 0x03,  /**< ±250 deg/s */
    ICM40609_GYRO_FSR_125DPS  = 0x04,  /**< ±125 deg/s */
    ICM40609_GYRO_FSR_62_5DPS = 0x05,  /**< ±62.5 deg/s */
    ICM40609_GYRO_FSR_31_25DPS= 0x06   /**< ±31.25 deg/s */
} icm40609_gyro_fsr_t;

/**
 * @brief Gyroscope output data rate.
 */
typedef enum {
    ICM40609_GYRO_ODR_32KHZ   = 0x01,
    ICM40609_GYRO_ODR_16KHZ   = 0x02,
    ICM40609_GYRO_ODR_8KHZ    = 0x03,
    ICM40609_GYRO_ODR_4KHZ    = 0x04,
    ICM40609_GYRO_ODR_2KHZ    = 0x05,
    ICM40609_GYRO_ODR_1KHZ    = 0x06,
    ICM40609_GYRO_ODR_500HZ   = 0x0F,
    ICM40609_GYRO_ODR_200HZ   = 0x07,
    ICM40609_GYRO_ODR_100HZ   = 0x08,
    ICM40609_GYRO_ODR_50HZ    = 0x09,
    ICM40609_GYRO_ODR_25HZ    = 0x0A,
    ICM40609_GYRO_ODR_12_5HZ  = 0x0B
} icm40609_gyro_odr_t;

/**
 * @brief Interrupt pin selection.
 */
typedef enum {
    ICM40609_INT1 = 0,
    ICM40609_INT2 = 1
} icm40609_int_pin_t;

/**
 * @brief Interrupt pin drive mode.
 */
typedef enum {
    ICM40609_INT_OPEN_DRAIN = 0,
    ICM40609_INT_PUSH_PULL  = 1
} icm40609_int_drive_t;

/**
 * @brief Interrupt pin polarity.
 */
typedef enum {
    ICM40609_INT_ACTIVE_LOW  = 0,
    ICM40609_INT_ACTIVE_HIGH = 1
} icm40609_int_polarity_t;

/* -------------------------------------------------------------------------- */
/* Device State Struct                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief ICM-40609 device state.
 *
 * Tracks current configuration and scale factors for data conversion.
 */
typedef struct {
    uint8_t accel_config0;   /**< Last written ACCEL_CONFIG0 */
    uint8_t gyro_config0;    /**< Last written GYRO_CONFIG0 */
    uint8_t pwr_mgmt0;       /**< Last written PWR_MGMT0 */
    uint8_t int_config;      /**< Last written INT_CONFIG */

    float accel_scale;       /**< g/LSB scale factor */
    float gyro_scale;        /**< dps/LSB scale factor */
    uint16_t accel_odr_hz;   /**< Effective accel ODR in Hz */
    uint16_t gyro_odr_hz;    /**< Effective gyro ODR in Hz */
} icm40609_t;

/* -------------------------------------------------------------------------- */
/* Sample Struct                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief ICM-40609 sensor sample.
 *
 * Contains raw and converted accelerometer and gyroscope data.
 */
typedef struct {
    uint64_t t_us;           /**< Host timestamp in microseconds */

    /* Raw values */
    int16_t ax_raw, ay_raw, az_raw;  /**< Raw accel counts */
    int16_t gx_raw, gy_raw, gz_raw;  /**< Raw gyro counts */
    int16_t temp_raw;                /**< Raw temperature */

    /* Converted values */
    float ax, ay, az;        /**< Acceleration in m/s² */
    float gx, gy, gz;        /**< Angular rate in rad/s */
    float temp_c;            /**< Temperature in °C */
} icm40609_sample_t;

/* -------------------------------------------------------------------------- */
/* Packet Builders                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build a register read command.
 *
 * @param reg Register address (0x00-0x7F).
 * @param tx Buffer to write command to.
 * @param n_data Number of data bytes to read.
 * @return Total TX length (1 + n_data).
 */
size_t icm40609_build_read(uint8_t reg, uint8_t *tx, size_t n_data);

/**
 * @brief Build a single-byte register write command.
 *
 * @param reg Register address (0x00-0x7F).
 * @param val Value to write.
 * @param tx Buffer to write command to.
 * @return TX length (always 2).
 */
size_t icm40609_build_write(uint8_t reg, uint8_t val, uint8_t *tx);

/**
 * @brief Build soft reset command (write 0x01 to DEVICE_CONFIG).
 *
 * @param tx Buffer to write command to.
 * @return TX length (always 2).
 */
size_t icm40609_build_softreset(uint8_t *tx);

/**
 * @brief Build WHO_AM_I read command.
 *
 * @param tx Buffer to write command to.
 * @return TX length (always 2).
 */
size_t icm40609_build_read_whoami(uint8_t *tx);

/**
 * @brief Build burst read command for temp + accel + gyro data.
 *
 * Reads 14 bytes: TEMP(2) + ACCEL_XYZ(6) + GYRO_XYZ(6).
 *
 * @param tx Buffer to write command to.
 * @return TX length (always 15).
 */
size_t icm40609_build_read_data(uint8_t *tx);

/**
 * @brief Build ACCEL_CONFIG0 write command.
 *
 * @param fsr Full-scale range.
 * @param odr Output data rate.
 * @param tx Buffer to write command to.
 * @param dev Device state (updated with new config).
 * @return TX length (always 2).
 */
size_t icm40609_build_accel_config(icm40609_accel_fsr_t fsr,
                                   icm40609_accel_odr_t odr,
                                   uint8_t *tx,
                                   icm40609_t *dev);

/**
 * @brief Build GYRO_CONFIG0 write command.
 *
 * @param fsr Full-scale range.
 * @param odr Output data rate.
 * @param tx Buffer to write command to.
 * @param dev Device state (updated with new config).
 * @return TX length (always 2).
 */
size_t icm40609_build_gyro_config(icm40609_gyro_fsr_t fsr,
                                  icm40609_gyro_odr_t odr,
                                  uint8_t *tx,
                                  icm40609_t *dev);

/**
 * @brief Build PWR_MGMT0 write command to enable accel + gyro.
 *
 * @param tx Buffer to write command to.
 * @param dev Device state (updated).
 * @return TX length (always 2).
 */
size_t icm40609_build_power_on(uint8_t *tx, icm40609_t *dev);

/**
 * @brief Build INT_CONFIG write command.
 *
 * @param pin Which interrupt pin (INT1 or INT2).
 * @param drive Push-pull or open-drain.
 * @param polarity Active high or low.
 * @param tx Buffer to write command to.
 * @return TX length (always 2).
 */
size_t icm40609_build_int_config(icm40609_int_pin_t pin,
                                 icm40609_int_drive_t drive,
                                 icm40609_int_polarity_t polarity,
                                 uint8_t *tx);

/**
 * @brief Build INT_SOURCE0 write to route DATA_RDY to INT1.
 *
 * @param tx Buffer to write command to.
 * @return TX length (always 2).
 */
size_t icm40609_build_int_source0_drdy(uint8_t *tx);

/**
 * @brief Build INTF_CONFIG0 write to disable I2C interface.
 *
 * @param tx Buffer to write command to.
 * @return TX length (always 2).
 */
size_t icm40609_build_disable_i2c(uint8_t *tx);

/**
 * @brief Build INTF_CONFIG1 write to set normal test mode.
 *
 * @param tx Buffer to write command to.
 * @return TX length (always 2).
 */
size_t icm40609_build_intf_config1_normal(uint8_t *tx);

/* -------------------------------------------------------------------------- */
/* Parsers                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parse burst read data (temp + accel + gyro).
 *
 * @param rx_data RX buffer (skip command byte, starts at rx[1]).
 * @param sample Output sample struct.
 * @param dev Device state (for scale factors).
 * @return true on success.
 */
bool icm40609_parse_data(const uint8_t *rx_data,
                         icm40609_sample_t *sample,
                         const icm40609_t *dev);

/**
 * @brief Parse WHO_AM_I response.
 *
 * @param rx_data RX buffer (skip command byte, starts at rx[1]).
 * @return WHO_AM_I value.
 */
uint8_t icm40609_parse_whoami(const uint8_t *rx_data);

/**
 * @brief Convert raw temperature to Celsius.
 *
 * Formula: Temp_C = (raw / 132.48) + 25
 *
 * @param raw Raw temperature value.
 * @return Temperature in degrees Celsius.
 */
float icm40609_temp_to_celsius(int16_t raw);

/* -------------------------------------------------------------------------- */
/* SPSC Ring Buffer for Samples                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief SPSC ring buffer for ICM-40609 samples.
 *
 * Thread Safety:
 * - Single Producer (ISR/DMA callback) writes via icm40609_sample_queue()
 * - Single Consumer (state estimation task) reads via icm40609_sample_dequeue()
 * - Memory barriers ensure proper ordering.
 */
typedef struct {
    icm40609_sample_t samples[ICM40609_SAMPLE_Q_SIZE];
    volatile uint8_t head;  /**< Written by producer only */
    volatile uint8_t tail;  /**< Written by consumer only */
} icm40609_sample_queue_t;

/**
 * @brief Check if sample queue is empty.
 */
static inline bool icm40609_sample_queue_empty(icm40609_sample_queue_t *q) {
    return q->head == q->tail;
}

/**
 * @brief Check if sample queue is full.
 */
static inline bool icm40609_sample_queue_full(icm40609_sample_queue_t *q) {
    return ((q->head + 1) % ICM40609_SAMPLE_Q_SIZE) == q->tail;
}

/**
 * @brief Enqueue a sample (producer side - ISR context).
 *
 * @param q Pointer to the sample queue.
 * @param sample Pointer to sample to enqueue.
 * @return true if successful, false if queue full.
 */
static inline bool icm40609_sample_queue(icm40609_sample_queue_t *q,
                                         const icm40609_sample_t *sample) {
    if (icm40609_sample_queue_full(q)) return false;
    q->samples[q->head] = *sample;
    SYNC_DMB();  /* Ensure sample data written before head update */
    q->head = (q->head + 1) % ICM40609_SAMPLE_Q_SIZE;
    return true;
}

/**
 * @brief Dequeue a sample (consumer side - task context).
 *
 * @param q Pointer to the sample queue.
 * @param sample Pointer to receive dequeued sample.
 * @return true if successful, false if queue empty.
 */
static inline bool icm40609_sample_dequeue(icm40609_sample_queue_t *q,
                                           icm40609_sample_t *sample) {
    if (icm40609_sample_queue_empty(q)) return false;
    SYNC_DMB();  /* Ensure we see sample data written before head update */
    *sample = q->samples[q->tail];
    SYNC_DMB();  /* Ensure sample read completes before tail update */
    q->tail = (q->tail + 1) % ICM40609_SAMPLE_Q_SIZE;
    return true;
}

#endif /* ICM40609_IMU_H */
