#ifndef ESC_MOTOR_H
#define ESC_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Standard PWM for BLHeli_32 ESC: 50 Hz, 1000-2000µs pulse width. */
#define ESC_PWM_FREQ_HZ       50U
#define ESC_PULSE_MIN_US      1000U   /* 0% throttle */
#define ESC_PULSE_MAX_US      2000U   /* 100% throttle */

#define ESC_CMD_TIMEOUT_MS     200U

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;         /* TIM_CHANNEL_x */

    uint16_t us_min;          /* pulse width at 0% throttle */
    uint16_t us_max;          /* pulse width at 100% throttle */
    uint16_t us_target;       /* requested pulse width */
    uint16_t us_output;       /* last written pulse width */

    bool     armed;
    uint32_t last_cmd_ms;
} esc_motor_t;

void esc_motor_init(esc_motor_t *m, TIM_HandleTypeDef *htim, uint32_t channel);
void esc_motor_arm(esc_motor_t *m, uint32_t now_ms);
void esc_motor_disarm(esc_motor_t *m);
void esc_motor_set_throttle_pct(esc_motor_t *m, float pct, uint32_t now_ms);
void esc_motor_set_us(esc_motor_t *m, uint16_t us, uint32_t now_ms);
void esc_motor_update(esc_motor_t *m, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* ESC_MOTOR_H */
