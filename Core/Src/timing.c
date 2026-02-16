#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include "PDI6121_servo.h"

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            // Top of cycle — notify control task
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)ControlsHandle, &woken);
            portYIELD_FROM_ISR(woken);
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            // Fixed-phase output — write command to actuator
            // Your SPI bus arbitration / command latch goes here
            // dummy for now latch_actuator_command();

            PDI6121_servo_pwm_device_set_ticks(servo_G, current_pulse_ticks);

        }
    }
}
