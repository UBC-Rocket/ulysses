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

    float deg_range;
    float mid_pt;

    uint16_t compare_val;

    uint16_t us_last;
    bool enabled;

} PDI6121_servo_t;

typedef enum {
    US_MIN = 500,
    US_MID = 1500,
    US_MAX = 2500
} PDI6121_servo_cal_t;

typedef struct {
    PDI6121_servo_t servo1;
    PDI6121_servo_t servo2;
} servo_pair_t;

void PDI6121_servo_init(PDI6121_servo_t *servo, PDI6121_servo_pwm_t *pwm);

void PDI6121_servo_init_with_deg_range(PDI6121_servo_t *servo, PDI6121_servo_pwm_t *pwm, float deg_range, float mid_pt);

void PDI6121_servo_enable(PDI6121_servo_t *servo, bool enable);

void PDI6121_servo_set_us(PDI6121_servo_t *servo, uint16_t pulse_us);

void PDI6121_servo_set_deg(PDI6121_servo_t *servo, float degrees);


// API Controls

void PDI6121_servo_set_deg_range(PDI6121_servo_t *servo, float deg_range, float mid_point); // Calibrated for each servo


/* Device layer hooks */
void PDI6121_servo_device_init(PDI6121_servo_pwm_t *pwm);

void PDI6121_servo_pwm_device_enable(PDI6121_servo_pwm_t *pwm, bool enable);

void PDI6121_servo_pwm_device_set_ticks(PDI6121_servo_pwm_t *pwm, uint32_t pulse_ticks);



void set_servo_degree(PDI6121_servo_t *servo, float degree);

void set_servo_pair_degrees(float s1, float s2);

void apply_servo_pair_degrees();
