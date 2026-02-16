#include "tim.h"
#include "main.h"
#include "PDI6121_servo.h"

#include <stddef.h> 
#include <stdint.h>

/*Global Variables for the Interrupt to access*/
static servo_pair_t servos;

void set_servo_degrees(float s1, float s2) {
    float mid1 = servos.servo1.mid_pt;
    float mid2 = servos.servo2.mid_pt;
    float range1 = servos.servo1.deg_range;
    float range2 = servos.servo2.deg_range;

    if (range1 <= 0.0f) range1 = 180.0f;
    if (range2 <= 0.0f) range2 = 180.0f;

    float min1 = mid1 - (range1 * 0.5f);
    float max1 = mid1 + (range1 * 0.5f);
    float min2 = mid2 - (range2 * 0.5f);
    float max2 = mid2 + (range2 * 0.5f);

    float d1 = s1;
    float d2 = s2;
    if (d1 < min1) d1 = min1;
    if (d1 > max1) d1 = max1;
    if (d2 < min2) d2 = min2;
    if (d2 > max2) d2 = max2;

    float t1 = (max1 > min1) ? ((d1 - min1) / (max1 - min1)) : 0.5f;
    float t2 = (max2 > min2) ? ((d2 - min2) / (max2 - min2)) : 0.5f;

    uint16_t us1 = (uint16_t)((float)servos.servo1.us_min +
                              t1 * (float)(servos.servo1.us_max - servos.servo1.us_min) +
                              0.5f);
    uint16_t us2 = (uint16_t)((float)servos.servo2.us_min +
                              t2 * (float)(servos.servo2.us_max - servos.servo2.us_min) +
                              0.5f);

    uint64_t ticks1 = ((uint64_t)us1 * (uint64_t)servos.servo1.pwm.timer_hz) / 1000000ULL;
    uint64_t ticks2 = ((uint64_t)us2 * (uint64_t)servos.servo2.pwm.timer_hz) / 1000000ULL;

    if (servos.servo1.pwm.period_ticks != 0U && ticks1 >= servos.servo1.pwm.period_ticks) {
        ticks1 = servos.servo1.pwm.period_ticks - 1U;
    }
    if (servos.servo2.pwm.period_ticks != 0U && ticks2 >= servos.servo2.pwm.period_ticks) {
        ticks2 = servos.servo2.pwm.period_ticks - 1U;
    }

    servos.servo1.compare_val = (uint32_t)ticks1;
    servos.servo2.compare_val = (uint32_t)ticks2;
}

void apply_servo_positions() {
    PDI6121_servo_pwm_device_set_ticks(&servos.servo1.pwm, servos.servo1.compare_val);
    PDI6121_servo_pwm_device_set_ticks(&servos.servo2.pwm, servos.servo2.compare_val);
}

/* Hardware Functions */

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
    PDI6121_servo_pwm_device_init(&servo->pwm);

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

/* Normalize input to [-1.0, 1.0] */
void PDI6121_servo_set_norm(PDI6121_servo_t *servo, float norm) {
    if (servo == NULL) {
        return;
    }

    /* Clamp to [-1.0, 1.0] */
    float clamped = clamp_f(norm, -1.0f, 1.0f);

    /* Map to microseconds */
    uint16_t us;
    if (clamped >= 0.0f) {
        us = (uint16_t)(servo->us_mid + clamped * (float)(servo->us_max - servo->us_mid));
    } else {
        us = (uint16_t)(servo->us_mid + clamped * (float)(servo->us_mid - servo->us_min));
    }

    PDI6121_servo_set_us(servo, us);
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

    if (d <= 90.0f) {
        float t = d / 90.0f;
        float us_f = (float)servo->us_min + t * (float)(servo->us_mid - servo->us_min);
        uint16_t us = (uint16_t)(us_f + 0.5f);
        PDI6121_servo_set_us(servo, us);
    } else {
        float t = (d - 90.0f) / 90.0f;
        float us_f = (float)servo->us_mid + t * (float)(servo->us_max - servo->us_mid);
        uint16_t us = (uint16_t)(us_f + 0.5f);
        PDI6121_servo_set_us(servo, us);
    }

}

void PDI6121_servo_set_cal(PDI6121_servo_t *servo) {
    if (servo == NULL) {
        return;
    }
    /* Reset to default calibration */
    set_default_cal(servo);

    /* Clamp last command to new range. */
    servo->us_last = clamp_u16(servo->us_last, servo->us_min, servo->us_max);
    if (servo->enabled) {
        PDI6121_servo_set_us(servo, servo->us_last);
    }
}



void PDI6121_servo_set_deg_range(PDI6121_servo_t *servo, float deg_range, float mid_pt) {
    if (servo == NULL) {
        return;
    }
    servo->deg_range = deg_range;
    servo->mid_pt = mid_pt;
}



void PDI6121_servo_set_position_deg(PDI6121_servo_t *servoX, PDI6121_servo_t *servoY, float x, float y) {
    // if (servoX == NULL || servoY == NULL) {
    //     return;
    // }

    float originX = servoX->mid_pt;
    // float originY = servoY->mid_pt;

    PDI6121_servo_set_deg(servoX, originX + x);
    // PDI6121_servo_set_deg(servoY, originY + y);

}
