#include "main.h"
#include "tim.h"
#include "PDI6121_servo.h"

uint8_t PDI6121_servo_device_init(TIM_HandleTypeDef *htim,
                          uint32_t channel,
                          uint32_t timer_hz,
                          uint32_t period_ticks, 
                          PDI6121_servo_t *servo) 
{
    if (!htim || !servo) return 1;

    /* 1. Store hardware handle */
    servo->pwm.htim = (void*)htim;
    servo->pwm.channel = channel;
    servo->pwm.timer_hz = timer_hz;
    servo->pwm.period_ticks = period_ticks;

    /* 2. Default calibration */
    servo->us_min = US_MIN;
    servo->us_mid = US_MID;
    servo->us_max = US_MAX;

    servo->us_last = servo->us_mid;
    
    /* 3. Start PWM */

    if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) return 2;

    /* 4. Safe position to start (middle) */

    PDI6121_servo_set_us(servo, servo->us_mid);

    HAL_Delay(50);
    return 0;
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
