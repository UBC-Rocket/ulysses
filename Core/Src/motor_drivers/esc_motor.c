#include "esc_motor.h"

/* ---- helpers ---- */

static uint16_t clamp_us(uint16_t us, uint16_t lo, uint16_t hi) {
    if (us < lo) return lo;
    if (us > hi) return hi;
    return us;
}

static uint16_t pct_to_us(uint16_t us_min, uint16_t us_max, float pct) {
    float usf = (float)us_min + (pct * 0.01f) * (float)(us_max - us_min);
    return (uint16_t)(usf + 0.5f);
}

static void write_us(esc_motor_t *m, uint16_t us) {
    uint16_t clamped = clamp_us(us, m->us_min, m->us_max);

    uint32_t psc      = m->htim->Init.Prescaler + 1U;
    uint32_t arr      = __HAL_TIM_GET_AUTORELOAD(m->htim);
    uint32_t pclk1    = HAL_RCC_GetPCLK1Freq();
    uint32_t apb1_div = HAL_RCC_GetHCLKFreq() / pclk1;
    uint32_t timer_clk = (apb1_div > 1U) ? (pclk1 * 2U) : pclk1;
    uint32_t timer_hz = timer_clk / psc;
    uint64_t ticks    = (uint64_t)clamped * (uint64_t)timer_hz / 1000000ULL;
    if (ticks > arr) ticks = arr;

    __HAL_TIM_SET_COMPARE(m->htim, m->channel, (uint32_t)ticks);
    m->us_output = clamped;
}

/* ---- public API ---- */

void esc_motor_init(esc_motor_t *m, TIM_HandleTypeDef *htim, uint32_t channel) {
    if (m == NULL || htim == NULL) return;

    m->htim    = htim;
    m->channel = channel;

    m->us_min    = ESC_PULSE_MIN_US;
    m->us_max    = ESC_PULSE_MAX_US;
    m->us_target = m->us_min;
    m->us_output = m->us_min;
    m->armed     = false;
    m->last_cmd_ms = 0U;
}

void esc_motor_arm(esc_motor_t *m, uint32_t now_ms) {
    if (m == NULL) return;

    if (HAL_TIM_PWM_Start(m->htim, m->channel) != HAL_OK) return;

    m->armed       = true;
    m->last_cmd_ms = now_ms;
    m->us_target   = m->us_min;
    write_us(m, m->us_min);
}

void esc_motor_disarm(esc_motor_t *m) {
    if (m == NULL) return;

    m->armed     = false;
    m->us_target = m->us_min;
    write_us(m, m->us_min);
    HAL_TIM_PWM_Stop(m->htim, m->channel);
}

void esc_motor_set_throttle_pct(esc_motor_t *m, float pct, uint32_t now_ms) {
    if (m == NULL || !m->armed) return;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f)  pct = 100.0f;

    m->us_target   = pct_to_us(m->us_min, m->us_max, pct);
    m->last_cmd_ms = now_ms;
    write_us(m, m->us_target);
}

void esc_motor_set_us(esc_motor_t *m, uint16_t us, uint32_t now_ms) {
    if (m == NULL || !m->armed) return;

    m->us_target   = clamp_us(us, m->us_min, m->us_max);
    m->last_cmd_ms = now_ms;
    write_us(m, m->us_target);
}

void esc_motor_update(esc_motor_t *m, uint32_t now_ms) {
    if (m == NULL) return;

    if (!m->armed) {
        if (m->us_output != m->us_min) {
            write_us(m, m->us_min);
        }
        return;
    }

    /* Command timeout: auto-disarm if no command received while above idle. */
    if (m->us_output > m->us_min) {
        uint32_t dt = now_ms - m->last_cmd_ms;
        if (dt > ESC_CMD_TIMEOUT_MS) {
            m->armed     = false;
            m->us_target = m->us_min;
            write_us(m, m->us_min);
            HAL_TIM_PWM_Stop(m->htim, m->channel);
            return;
        }
    }

    if (m->us_output != m->us_target) {
        write_us(m, m->us_target);
    }
}
