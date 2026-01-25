#include "main.h"
#include "tim.h"
#include "PDI6121_servo.h"

uint8_t PDI6121_servo_init(TIM_HandleTypeDef *htim,
                          uint32_t channel,
                          uint32_t timer_hz,
                          uint32_t period_ticks, 
                          PDI6121_servo_t *servo) 
{
    if (!htim || !servo) return 1;

    /* 1. Store hardware handle */
    servo->htim = htim;
    servo->channel = channel;
    servo->timer_hz = timer_hz;
    servo->period_ticks = period_ticks;

    /* 2. Default calibration */
    servo->us_min = US_MIN;
    servo->us_mid = US_MID;
    servo->us_max = US_MAX;

    servo->us_last = servo->us_mid;
    
    /* 3. Start PWM */

    if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK) return 2;

    /* 4. Safe position to start (middle) */

    PDI6121_servo_write_us(servo, servo->us_mid);

    HAL_Delay(50);
    return 0;
}

void PDI6121_write_us(PDI6121_servo_t *servo, uint16_t pulse_us) {
    if (!servo) return;
    PDI6121_set_us(servo, pulse_us);
}

void PDI6121_write_norm(PDI6121_servo_t *servo, float x) {
    if (!servo) return; 
    PDI6121_set_norm(servo, x);
}