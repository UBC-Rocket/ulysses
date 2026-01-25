#include "PDI6121_servo.h"

#include <stddef.h> 
#include <stdint.h>

/*
Device layer hooks 
*/
void PDI6121_servo_pwm_device_init(PDI6121_servo_pwm_t *pwm);
void PDI6121_servo_pwm_device_enable(PDI6121_servo_pwm_t *pwm, bool enable);
void PDI6121_servo_pwm_device_set_ticks(PDI6121_servo_pwm_t *pwm, uint32_t pulse_ticks);

/*
Local helper clampfunctions
*/ 
static uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float clamp_f(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/*
Convert microseconds to timer ticks
*/
static uint32_t us_to_ticks(PDI6121_servo_pwm_t *pwm, uint16_t us) {
    if (pwm == NULL || pwm->timer_hz == 0) {
        return 0;
    }
    uint64_t ticks = (uint64_t)us * (uint64_t)pwm->timer_hz;
    ticks /= 1000000ULL;
    if (ticks > 0xFFFFFFFFULL) {
        ticks = 0xFFFFFFFFULL;
    }
    return (uint32_t)ticks;
}

/* 
Clamp pulse ticks so it cannot exceed the PWM period.
 */
static uint32_t clamp_ticks_to_period(const PDI6121_servo_pwm_t *pwm, uint32_t pulse_ticks) {
    if (pwm == NULL || pwm->period_ticks == 0) {
        return pulse_ticks;
    }

    /* prevent pulse >= period. */
    if (pulse_ticks >= pwm->period_ticks) {
        return pwm->period_ticks - 1;
    }
    return pulse_ticks;
}

/* Default calibration for a typical hobby servo signal. */
static void set_default_cal(PDI6121_servo_t *servo) {
    servo->us_min = 1000; 
    servo->us_mid = 1500;
    servo->us_max = 2000;
}