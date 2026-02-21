#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include "motor_drivers/servo_driver.h"
#include "motor_drivers/esc_driver.h"

static uint8_t servo_div = 0;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            /* 800Hz — notify controls task */
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR((TaskHandle_t)ControlsHandle, &woken);
            portYIELD_FROM_ISR(woken);
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            /* ESC: called at 800Hz, internal divider makes 400Hz effective */
            ESC_apply_pair();

            /* Servo: 800Hz / 4 = 200Hz */
            if (++servo_div >= 4) {
                servo_div = 0;
                apply_servo_pair_degrees();
            }
        }
    }
}

/**
 * @brief Actuator apply task: woken by TIM3 CH2, reads control output, writes PWM.
 *
 * Runs at osPriorityRealtime so it preempts immediately when notified by the
 * CH2 ISR, ensuring deterministic actuator update timing within each control cycle.
 */
void actuator_apply_task_start(void *argument)
{
    (void)argument;
    motor_driver_init();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        control_output_t out;
        state_exchange_get_control_output(&out);

        uint32_t now_ms = HAL_GetTick();

        /* TODO: convert control_output_t → control_out_t (motor_driver.h)
         * Map (theta_x_cmd, theta_y_cmd, T_cmd) to
         * (servo_yl, servo_yr, motor_top, motor_bot) percentages [0..100].
         */
        control_out_t cmd = {0};
        /* cmd.servo_yl  = ...; */
        /* cmd.servo_yr  = ...; */
        /* cmd.motor_top = ...; */
        /* cmd.motor_bot = ...; */

        motor_driver_set(&cmd, now_ms);
        motor_driver_update(now_ms);
    }
}
