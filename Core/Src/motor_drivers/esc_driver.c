#include "motor_drivers/esc_driver.h"

#include "main.h"

#include <stddef.h>

#define ESC_LUT_POINTS 11U

static float clamp_f32(float x, float lo, float hi);
static uint32_t us_to_ticks(const esc_pwm_t *pwm, uint16_t us);
static uint32_t clamp_ticks_to_period(const esc_pwm_t *pwm, uint32_t pulse_ticks);
static uint16_t thrust_to_us_lut(float thrust);

static esc_pair_t g_esc_pair;
static volatile bool g_esc_pair_ready = false;

static const uint16_t g_esc_us_lut[ESC_LUT_POINTS] = {
    /* F45A 3-6S baseline thrust curve, tune on bench if needed. */
    1000U, 1080U, 1160U, 1240U, 1320U, 1400U, 1520U, 1640U, 1760U, 1880U, 2000U
};

void ESC_init(esc_t *esc, const esc_pwm_t *pwm) {
    if (esc == NULL || pwm == NULL || pwm->htim == NULL) {
        return;
    }

    esc->pwm = *pwm;
    esc->desired_thrust = 0.0f;
    esc->desired_pulse_us = ESC_PWM_MIN_US;
    esc->desired_pulse_ticks = us_to_ticks(&esc->pwm, esc->desired_pulse_us);
    esc->desired_pulse_ticks = clamp_ticks_to_period(&esc->pwm, esc->desired_pulse_ticks);
    esc->update_divider_counter = 0U;
    esc->initialized = true;

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)esc->pwm.htim;
    __HAL_TIM_SET_COMPARE(htim, esc->pwm.channel, esc->desired_pulse_ticks);
    (void)HAL_TIM_PWM_Start(htim, esc->pwm.channel);
}

void ESC_set_thrust(esc_t *esc, float thrust) {
    if (esc == NULL || !esc->initialized) {
        return;
    }

    float clamped = clamp_f32(thrust, 0.0f, 1.0f);
    uint16_t pulse_us = thrust_to_us_lut(clamped);
    uint32_t pulse_ticks = us_to_ticks(&esc->pwm, pulse_us);
    pulse_ticks = clamp_ticks_to_period(&esc->pwm, pulse_ticks);

    /* Control-stage only: update desired software state. */
    esc->desired_thrust = clamped;
    esc->desired_pulse_us = pulse_us;
    esc->desired_pulse_ticks = pulse_ticks;
}

void ESC_apply(esc_t *esc) {
    if (esc == NULL || !esc->initialized || esc->pwm.htim == NULL) {
        return;
    }

    esc->update_divider_counter++;
    if (esc->update_divider_counter < ESC_ISR_TO_PWM_DIVIDER) {
        return;
    }
    esc->update_divider_counter = 0U;

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)esc->pwm.htim;
    __HAL_TIM_SET_COMPARE(htim, esc->pwm.channel, esc->desired_pulse_ticks);
}

void ESC_pair_init(const esc_pwm_t *pwm1, const esc_pwm_t *pwm2) {
    ESC_init(&g_esc_pair.esc1, pwm1);
    ESC_init(&g_esc_pair.esc2, pwm2);
    g_esc_pair_ready = true;
}

void ESC_set_pair_thrust(float thrust1, float thrust2) {
    if (!g_esc_pair_ready) {
        return;
    }
    ESC_set_thrust(&g_esc_pair.esc1, thrust1);
    ESC_set_thrust(&g_esc_pair.esc2, thrust2);
}

void ESC_apply_pair(void) {
    if (!g_esc_pair_ready) {
        return;
    }
    ESC_apply(&g_esc_pair.esc1);
    ESC_apply(&g_esc_pair.esc2);
}

static float clamp_f32(float x, float lo, float hi) {
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static uint32_t us_to_ticks(const esc_pwm_t *pwm, uint16_t us) {
    if (pwm == NULL || pwm->timer_hz == 0U) {
        return 0U;
    }

    uint64_t ticks = (uint64_t)us * (uint64_t)pwm->timer_hz;
    ticks /= 1000000ULL;
    if (ticks > 0xFFFFFFFFULL) {
        ticks = 0xFFFFFFFFULL;
    }
    return (uint32_t)ticks;
}

static uint32_t clamp_ticks_to_period(const esc_pwm_t *pwm, uint32_t pulse_ticks) {
    if (pwm == NULL || pwm->period_ticks == 0U) {
        return pulse_ticks;
    }
    if (pulse_ticks >= pwm->period_ticks) {
        return pwm->period_ticks - 1U;
    }
    return pulse_ticks;
}

static uint16_t thrust_to_us_lut(float thrust) {
    float clamped = clamp_f32(thrust, 0.0f, 1.0f);

    float scaled = clamped * (float)(ESC_LUT_POINTS - 1U);
    uint32_t i = (uint32_t)scaled;
    if (i >= (ESC_LUT_POINTS - 1U)) {
        return g_esc_us_lut[ESC_LUT_POINTS - 1U];
    }

    float frac = scaled - (float)i;
    uint16_t us0 = g_esc_us_lut[i];
    uint16_t us1 = g_esc_us_lut[i + 1U];
    float us = (float)us0 + frac * (float)(us1 - us0);
    return (uint16_t)(us + 0.5f);
}
