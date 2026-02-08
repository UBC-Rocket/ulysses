#include "main.h"
#include "tim.h"
#include "PDI6121_servo.h"

void PDI6121_servo_device_init(PDI6121_servo_pwm_t *pwm) {

    if (!pwm || !pwm->htim) return;

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)pwm->htim;
    (void)HAL_TIM_PWM_Start(htim, pwm->channel);

}


void PDI6121_servo_pwm_device_enable(PDI6121_servo_pwm_t *pwm, bool enable) {
    if (pwm == NULL) return;

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)pwm->htim;

    if (enable) {
        (void)HAL_TIM_PWM_Start(htim, pwm->channel);
    }
    else {
        (void)HAL_TIM_PWM_Stop(htim, pwm->channel);
    }
}

void PDI6121_servo_pwm_device_set_ticks(PDI6121_servo_pwm_t *pwm, uint32_t pulse_ticks) {
    if (pwm == NULL) return;

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)pwm->htim;

    /* Clamp to ARR (safety) */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    if (pulse_ticks > arr) pulse_ticks = arr;

    __HAL_TIM_SET_COMPARE(htim, pwm->channel, pulse_ticks);
}
