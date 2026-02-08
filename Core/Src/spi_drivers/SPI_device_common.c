#include "SPI_device_interactions.h"

bmi088_accel_sample_queue_t bmi088_acc_sample_ring;
bmi088_gyro_sample_queue_t bmi088_gyro_sample_ring;
ms5611_sample_queue_t ms5611_sample_ring;
icm20948_sample_queue_t icm_sample_ring;

bmi088_accel_t accel;
bmi088_gyro_t gyro;
icm20948_t icm20948;

spi_job_queue_t jobq_spi_2;
volatile bool bmi088_accel_ready = false;
volatile bool bmi088_gyro_ready = false;
volatile bool icm20948_ready = false;
