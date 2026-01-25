#include "main.h"
#include "tim.h"
#include "PDI6121_servo.h"

uint8_t PDI6121_servo_init(TIM_HandleTypeDef *htim,
                          uint32_t channel,
                          uint32_t timer_hz,
                          uint32_t period_ticks, 
                          PDI6121_servo_t *servo) 
{

    /* 1. Store hardware handle */
    servo->htim = htim;
    servo->channel = channel;
    servo->timer_hz = timer_hz;
    servo->period_ticks = period_ticks;


}