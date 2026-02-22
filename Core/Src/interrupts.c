#include "stm32h5xx_hal.h"
#include "SPI_queue.h"
#include "BMI088_accel.h"
#include "main.h"
#include "SPI_device_interactions.h"
#include "spi_drivers/SEN0306_mmwave.h"

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
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

/**
 * @brief UART RX Event Callback — routes idle/DMA events to the correct driver.
 *
 * HAL_UARTEx_ReceiveToIdle_DMA on STM32H5/GPDMA fires once per IDLE event and
 * then stops. sen0306_irq_handler() restarts the DMA internally at the end of
 * each call, so do NOT call HAL_UARTEx_ReceiveToIdle_DMA again here.
 */