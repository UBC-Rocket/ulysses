#include "motor_driver.h"
#include "esc_motor.h"
#include "tim.h"

/*
 * TODO: Change htim3 to a dedicated PWM timer once configured in CubeMX.
 *       TIM3 is used for control loop timing (OC interrupts) and cannot
 *       also generate PWM output. Pick a free timer (e.g. TIM2, TIM4, TIM5)
 *       with pins routed to the ESC/servo outputs.
 */

static esc_motor_t servo_yl;
static esc_motor_t servo_yr;
static esc_motor_t motor_top;
static esc_motor_t motor_bot;

void motor_driver_init(void) {
    esc_motor_init(&servo_yl,  &htim3, TIM_CHANNEL_1);
    esc_motor_init(&servo_yr,  &htim3, TIM_CHANNEL_2);
    esc_motor_init(&motor_top, &htim3, TIM_CHANNEL_3);
    esc_motor_init(&motor_bot, &htim3, TIM_CHANNEL_4);
}

void motor_driver_arm(uint32_t now_ms) {
    esc_motor_arm(&servo_yl,  now_ms);
    esc_motor_arm(&servo_yr,  now_ms);
    esc_motor_arm(&motor_top, now_ms);
    esc_motor_arm(&motor_bot, now_ms);
}

void motor_driver_disarm(void) {
    esc_motor_disarm(&servo_yl);
    esc_motor_disarm(&servo_yr);
    esc_motor_disarm(&motor_top);
    esc_motor_disarm(&motor_bot);
}

void motor_driver_set(const control_out_t *cmd, uint32_t now_ms) {
    if (cmd == NULL) return;
    esc_motor_set_throttle_pct(&servo_yl,  cmd->servo_yl,  now_ms);
    esc_motor_set_throttle_pct(&servo_yr,  cmd->servo_yr,  now_ms);
    esc_motor_set_throttle_pct(&motor_top, cmd->motor_top, now_ms);
    esc_motor_set_throttle_pct(&motor_bot, cmd->motor_bot, now_ms);
}

void motor_driver_update(uint32_t now_ms) {
    esc_motor_update(&servo_yl,  now_ms);
    esc_motor_update(&servo_yr,  now_ms);
    esc_motor_update(&motor_top, now_ms);
    esc_motor_update(&motor_bot, now_ms);
}
