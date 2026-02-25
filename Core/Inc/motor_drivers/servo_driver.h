#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "motor_drivers/pwm_output.h"

typedef struct {
    pwm_output_t pwm;

    /* Calibration parameters */
    uint16_t us_min;
    uint16_t us_mid;
    uint16_t us_max;

    float deg_range;
    float mid_pt;

    volatile uint32_t compare_val;

    uint16_t us_last;
    bool enabled;
} servo_t;

/** Calibration: add these [deg] to commanded angle so straight-down lines up. Tune per axis. */
#define GIMBAL_SERVO1_DEG_OFFSET -73.0f
#define GIMBAL_SERVO2_DEG_OFFSET -75.0f

typedef enum {
    SERVO_US_MIN = 500,
    SERVO_US_MID = 1500,
    SERVO_US_MAX = 2500
} servo_cal_t;

typedef struct {
    servo_t servo1;
    servo_t servo2;
} servo_pair_t;

void servo_init(servo_t *servo, const pwm_output_t *pwm);
void servo_init_with_deg_range(servo_t *servo, const pwm_output_t *pwm, float deg_range, float mid_pt);
void servo_enable(servo_t *servo, bool enable);
void servo_set_deg_range(servo_t *servo, float deg_range, float mid_pt);

/* Task-level: store desired angle (called from controls task). */
void set_servo_degree(servo_t *servo, float degree);
void set_servo_pair_degrees(float s1, float s2);

/* ISR-level: latch stored compare_val to hardware (called from TIM4 CH4). */
void apply_servo_pair_degrees(void);

void servo_pair_init(const pwm_output_t *pwm1, const pwm_output_t *pwm2);
void servo_pair_enable(bool enable);
