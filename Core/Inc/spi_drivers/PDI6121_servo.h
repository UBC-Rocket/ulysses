#pragma once

#include "stdint.h"
#include "stdbool.h"

typedef struct {
    void *htim;
    uint32_t channel;
    uint32_t timer_hz;
    uint32_t period_ticks;
} PDI6121_servo_pwm_t;

typedef struct {
    PDI6121_servo_pwm_t pwm;

    /* Calibration parameters */
    uint16_t us_min;
    uint16_t us_mid;
    uint16_t us_max;

    uint16_t us_last;
    bool enabled;
} PDI6121_servo_t;

int offset = 100;

typedef enum {
    US_MIN = 500 + offset,
    US_MID = 1500 + offset,
    US_MAX = 2500 + offset
} PDI6121_servo_cal_t;

void PDI6121_servo_init(PDI6121_servo_t *servo, PDI6121_servo_pwm_t *pwm);

void PDI6121_servo_enable(PDI6121_servo_t *servo, bool enable);

void PDI6121_servo_set_us(PDI6121_servo_t *servo, uint16_t pulse_us);

void PDI6121_servo_set_norm(PDI6121_servo_t *servo, float norm);

void PDI6121_servo_set_deg(PDI6121_servo_t *servo, float degrees);

void PDI6121_servo_set_cal(PDI6121_servo_t *servo);


/* Device layer hooks */
void PDI6121_servo_device_init(PDI6121_servo_pwm_t *pwm);
void PDI6121_servo_pwm_device_enable(PDI6121_servo_pwm_t *pwm, bool enable);
void PDI6121_servo_pwm_device_set_ticks(PDI6121_servo_pwm_t *pwm, uint32_t pulse_ticks);
