#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            // Top of cycle — notify control task
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)ControlsHandle, &woken);
            portYIELD_FROM_ISR(woken);
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            // Fixed-phase output — write command to actuator
            // Your SPI bus arbitration / command latch goes here
            // dummy for now latch_actuator_command();

            apply_servo_positions();
        }
    }
}
