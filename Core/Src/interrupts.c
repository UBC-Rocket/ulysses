#include "stm32h5xx_hal.h"
#include "SPI_queue.h"
#include "BMI088_accel.h"
#include "ICM40609_imu.h"
#include "main.h"
#include "SPI_device_interactions.h"

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMI_ACC_INT_1_Pin) {
        if (!bmi088_accel_ready) {
            return;
        }
        bmi088_accel_interrupt();
    } else if (GPIO_Pin == BMI_GYRO_INT_1_Pin) {
        if (!bmi088_gyro_ready) {
            return;
        }
        bmi088_gyro_interrupt();
    }
#ifdef IMU2_INT_Pin
    else if (GPIO_Pin == IMU2_INT_Pin) {
        if (!icm40609_ready) {
            return;
        }
        icm40609_interrupt(&imu2, IMU2_CS_GPIO_Port, IMU2_CS_Pin, &jobq_spi_4);
    }
#endif
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}
