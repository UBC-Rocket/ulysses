/**
 * @file icm40609_device.c
 * @brief ICM-40609-D device integration layer.
 *
 * Provides blocking initialization, interrupt handlers, and DMA callbacks
 * for the ICM-40609-D IMU on SPI4.
 *
 * UBC Rocket, Jan 2026
 */

#include "SPI_device_interactions.h"
#include "ICM40609_imu.h"
#include "main.h"

/* Forward declaration of config struct - defined in sensor_config.h */
struct icm40609_config;
typedef struct icm40609_config icm40609_config_t;

/* -------------------------------------------------------------------------- */
/* SPI Helpers                                                                */
/* -------------------------------------------------------------------------- */

static void write_frame(SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin,
                        const uint8_t *tx,
                        size_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_SPI_Transmit(hspi, (uint8_t *)tx, len, HAL_MAX_DELAY);
    delay_us(1);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(1);
}

static void read_frame(SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port,
                       uint16_t cs_pin,
                       const uint8_t *tx,
                       uint8_t *rx,
                       size_t len)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx, rx, len, HAL_MAX_DELAY);
    delay_us(1);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(1);
}

/* -------------------------------------------------------------------------- */
/* ICM-40609 Initialization                                                   */
/* -------------------------------------------------------------------------- */

uint8_t icm40609_init_with_config(SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port,
                                  uint16_t cs_pin,
                                  icm40609_t *dev,
                                  icm40609_accel_fsr_t accel_fsr,
                                  icm40609_accel_odr_t accel_odr,
                                  icm40609_gyro_fsr_t gyro_fsr,
                                  icm40609_gyro_odr_t gyro_odr,
                                  icm40609_int_pin_t int_pin,
                                  icm40609_int_drive_t int_drive,
                                  icm40609_int_polarity_t int_polarity)
{
    uint8_t tx[24], rx[24];
    size_t n;

    /* --- 1. Soft reset --- */
    n = icm40609_build_softreset(tx);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(1000);  /* Wait 1ms after reset */

    /* --- 2. Verify WHO_AM_I --- */
    n = icm40609_build_read_whoami(tx);
    read_frame(hspi, cs_port, cs_pin, tx, rx, n);
    delay_us(100);

    uint8_t whoami = icm40609_parse_whoami(rx);
    if (whoami != ICM40609_WHO_AM_I_VALUE)
        return 1;  /* WHO_AM_I mismatch */

    /* --- 3. Disable I2C interface (SPI only) --- */
    n = icm40609_build_disable_i2c(tx);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 4. Set normal test mode --- */
    n = icm40609_build_intf_config1_normal(tx);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 5. Enable accel + gyro in low-noise mode --- */
    n = icm40609_build_power_on(tx, dev);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(45000);  /* Wait 45ms for gyro startup */

    /* --- 6. Configure accelerometer --- */
    n = icm40609_build_accel_config(accel_fsr, accel_odr, tx, dev);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 7. Configure gyroscope --- */
    n = icm40609_build_gyro_config(gyro_fsr, gyro_odr, tx, dev);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 8. Configure interrupt pin --- */
    n = icm40609_build_int_config(int_pin, int_drive, int_polarity, tx);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 9. Route DATA_RDY to INT1 --- */
    n = icm40609_build_int_source0_drdy(tx);
    write_frame(hspi, cs_port, cs_pin, tx, n);
    delay_us(100);

    /* --- 10. Verify first reading --- */
    n = icm40609_build_read_data(tx);
    read_frame(hspi, cs_port, cs_pin, tx, rx, n);

    icm40609_sample_t sample;
    if (!icm40609_parse_data(rx, &sample, dev))
        return 2;  /* Data parse failed */

    return 0;
}

uint8_t icm40609_init(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port,
                      uint16_t cs_pin,
                      icm40609_t *dev)
{
    /* Default configuration: 1 kHz, ±16g, ±1000 dps */
    return icm40609_init_with_config(hspi, cs_port, cs_pin, dev,
                                     ICM40609_ACCEL_FSR_16G,
                                     ICM40609_ACCEL_ODR_1KHZ,
                                     ICM40609_GYRO_FSR_1000DPS,
                                     ICM40609_GYRO_ODR_1KHZ,
                                     ICM40609_INT1,
                                     ICM40609_INT_PUSH_PULL,
                                     ICM40609_INT_ACTIVE_HIGH);
}

/* -------------------------------------------------------------------------- */
/* DMA Callback                                                               */
/* -------------------------------------------------------------------------- */

static void icm40609_done(spi_job_t *job,
                          const uint8_t *rx_buf,
                          void *arg)
{
    icm40609_t *dev = (icm40609_t *)arg;
    if (!dev) return;

    icm40609_sample_t sample;
    if (icm40609_parse_data(rx_buf, &sample, dev)) {
        sample.t_us = job->t_sample;
        icm40609_sample_queue(&icm40609_sample_ring, &sample);
    }
}

/* -------------------------------------------------------------------------- */
/* Interrupt Handler                                                          */
/* -------------------------------------------------------------------------- */

void icm40609_interrupt(icm40609_t *dev,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin,
                        spi_job_queue_t *jobq)
{
    const uint64_t now = timestamp_us();

    spi_job_t job;

    job.cs_port  = cs_port;
    job.cs_pin   = cs_pin;
    job.len      = icm40609_build_read_data(job.tx);
    job.t_sample = now;
    job.type     = SPI_XFER_TXRX;
    job.done     = icm40609_done;
    job.done_arg = dev;
    job.sensor   = SENSOR_ID_IMU2;
    job.task_notification_flag = ICM40609_IMU2_SAMPLE_FLAG;

    spi_submit_job(job, jobq);
}
