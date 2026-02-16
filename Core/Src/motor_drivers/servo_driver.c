#include "motor_drivers/servo_driver.h"

#include "main.h"

#include <stddef.h>
#include <stdint.h>

static uint16_t clamp_u16(uint16_t x, uint16_t lo, uint16_t hi);
static float clamp_f(float x, float lo, float hi);
static uint32_t us_to_ticks(PDI6121_servo_pwm_t *pwm, uint16_t us);
static uint32_t clamp_ticks_to_period(const PDI6121_servo_pwm_t *pwm, uint32_t pulse_ticks);
static void set_default_cal(PDI6121_servo_t *servo);

/* Global Variables for the Interrupt to access */
static servo_pair_t servos;
static volatile bool g_servo_pair_ready = false;

/* Controls Methods */
void set_servo_degree(PDI6121_servo_t *servo, float degree) {
    if (servo == NULL) {
        return;
    }

    /* Mapping:
     * 0 deg -> us_min
     * 90 deg -> us_mid
     * 180 deg -> us_max
     *
     * If your gimbal mechanics aren't 0..180, you should either:
     * - change this function signature to accept deg_min/deg_max, OR
     * - don’t use degrees and use norm/deflection instead.
     */

    float mid = servo->mid_pt;
    float range_half = (servo->deg_range)/2;
    float min_deg = mid - range_half;
    float max_deg = mid + range_half;

    float d = clamp_f(degree, min_deg, max_deg);

    float us_f;
    if (d <= 90.0f) {
        float t = d / 90.0f;
        us_f = (float)servo->us_min + t * (float)(servo->us_mid - servo->us_min);
    } else {
        float t = (d - 90.0f) / 90.0f;
        us_f = (float)servo->us_mid + t * (float)(servo->us_max - servo->us_mid);
    }

    uint16_t us = (uint16_t)(us_f + 0.5f);
    servo->compare_val = us_to_ticks(&servo->pwm, us);
}

void set_servo_pair_degrees(float degree1, float degree2) {
    set_servo_degree(&servos.servo1, degree1);
    set_servo_degree(&servos.servo2, degree2);
}

/* Hardware Functions */
void apply_servo_pair_degrees() {
    if (!g_servo_pair_ready) {
        return;
    }
    PDI6121_servo_pwm_device_set_ticks(&servos.servo1.pwm, servos.servo1.compare_val);
    PDI6121_servo_pwm_device_set_ticks(&servos.servo2.pwm, servos.servo2.compare_val);
}

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


void servo_pair_init(PDI6121_servo_pwm_t *pwm1, PDI6121_servo_pwm_t *pwm2) {
    PDI6121_servo_init(&servos.servo1, pwm1);
    PDI6121_servo_init(&servos.servo2, pwm2);
    g_servo_pair_ready = true;
}




/* Local helper clamp functions */
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

/* Convert microseconds to timer ticks */
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

/* Clamp pulse ticks so it cannot exceed the PWM period. */
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
    servo->us_min = US_MIN;
    servo->us_mid = US_MID;
    servo->us_max = US_MAX;
}

/* Public API implementations */
void PDI6121_servo_init(PDI6121_servo_t *servo, PDI6121_servo_pwm_t *pwm) {
    if (servo == NULL || pwm == NULL) {
        return;
    }

    


    servo->pwm = *pwm;

    /* Set default calibration */
    set_default_cal(servo);

    servo->enabled = false;
    servo->us_last = servo->us_mid;

    /* Hardware init */
    PDI6121_servo_device_init(&servo->pwm);

    /* Put hardware into a known safe state:
     * - Set compare to mid
     * - Keep disabled until explicitly enabled
     */
    {
        uint32_t ticks = us_to_ticks(&servo->pwm, servo->us_mid);
        ticks = clamp_ticks_to_period(&servo->pwm, ticks);
        PDI6121_servo_pwm_device_set_ticks(&servo->pwm, ticks);
    }
    PDI6121_servo_pwm_device_enable(&servo->pwm, false);
}

void PDI6121_servo_init_with_deg_range(PDI6121_servo_t *servo, PDI6121_servo_pwm_t *pwm, float deg_range, float mid_pt) {
    PDI6121_servo_set_deg_range(servo, deg_range, mid_pt);
    PDI6121_servo_init(servo, pwm);
}

void PDI6121_servo_enable(PDI6121_servo_t *servo, bool enable) {
    if (servo == NULL) {
        return;
    }

    servo->enabled = enable;

    if (enable) {

        /* apply last command clamp again defensively.*/
        uint16_t us = clamp_u16(servo->us_last, servo->us_min, servo->us_max);
        servo->us_last = us;

        uint32_t ticks = us_to_ticks(&servo->pwm, us);
        ticks = clamp_ticks_to_period(&servo->pwm, ticks);
        PDI6121_servo_pwm_device_set_ticks(&servo->pwm, ticks);

        PDI6121_servo_pwm_device_enable(&servo->pwm, true);
    } else {
        /* On disable: command neutral.*/
        uint32_t ticks = us_to_ticks(&servo->pwm, servo->us_mid);
        ticks = clamp_ticks_to_period(&servo->pwm, ticks);
        PDI6121_servo_pwm_device_set_ticks(&servo->pwm, ticks);

        PDI6121_servo_pwm_device_enable(&servo->pwm, false);
    }
}

void PDI6121_servo_set_us(PDI6121_servo_t *servo, uint16_t us) {
    if (servo == NULL) {
        return;
    }

    /* Clamp to calibration bounds. */
    uint16_t clamped = clamp_u16(us, servo->us_min, servo->us_max);
    servo->us_last = clamped;

    /* If disabled, store the command but don’t drive hardware. */
    if (!servo->enabled) {
        return;
    }

    uint32_t ticks = us_to_ticks(&servo->pwm, clamped);
    ticks = clamp_ticks_to_period(&servo->pwm, ticks);
    PDI6121_servo_pwm_device_set_ticks(&servo->pwm, ticks);
}

void PDI6121_servo_set_deg(PDI6121_servo_t *servo, float degrees) {
    if (servo == NULL) {
        return;
    }

    /* Mapping:
     * 0 deg -> us_min
     * 90 deg -> us_mid
     * 180 deg -> us_max
     *
     * If your gimbal mechanics aren't 0..180, you should either:
     * - change this function signature to accept deg_min/deg_max, OR
     * - don’t use degrees and use norm/deflection instead.
     */

    float mid = servo->mid_pt;
    float range_half = (servo->deg_range)/2;
    float min_deg = mid - range_half;
    float max_deg = mid + range_half;

    float d = clamp_f(degrees, min_deg, max_deg);

    float us_f;
    if (d <= 90.0f) {
        float t = d / 90.0f;
        us_f = (float)servo->us_min + t * (float)(servo->us_mid - servo->us_min);
    } else {
        float t = (d - 90.0f) / 90.0f;
        us_f = (float)servo->us_mid + t * (float)(servo->us_max - servo->us_mid);
    }

    uint16_t us = (uint16_t)(us_f + 0.5f);
    PDI6121_servo_set_us(servo, us);

}




void PDI6121_servo_set_deg_range(PDI6121_servo_t *servo, float deg_range, float mid_pt) {
    if (servo == NULL) {
        return;
    }
    servo->deg_range = deg_range;
    servo->mid_pt = mid_pt;
}




