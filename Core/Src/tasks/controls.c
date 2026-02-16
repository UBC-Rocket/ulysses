/**
 * @file    controls.c
 * @brief   Controls task: runs flight controller at 1 kHz, publishes control output.
 *
 * Reads state and flight_state from state_exchange; fills ref (e.g. hover);
 * calls flight_controller_run, state_exchange_publish_control_output, and
 * set_servo_pair_degrees for gimbal (theta_x_cmd, theta_y_cmd [rad] -> degrees).
 */
#include <stdbool.h>
#include <string.h>
#include "cmsis_os2.h"
#include "state_exchange.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h5xx_hal.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "controls/flight_controller.h"
#include "motor_drivers/servo_driver.h"

#define RAD_TO_DEG (180.0f / 3.14159265f)

#define CONTROLS_DT_S 0.001f  /**< Control period [s] (1 kHz). */
#define STALE_STATE_THRESHOLD_TICKS 100  /**< If state_seq unchanged for this many ticks, treat as stale and output safe (zero). */

/** Fill config with default gains and limits (tune in use). */
static void init_default_config(flight_controller_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    /* Attitude: diagonal gains and identity-like inertia (tune in use) */
    cfg->attitude.Kp[0][0] = 1.0f;
    cfg->attitude.Kp[1][1] = 1.0f;
    cfg->attitude.Kp[2][2] = 1.0f;
    cfg->attitude.Kd[0][0] = 0.1f;
    cfg->attitude.Kd[1][1] = 0.1f;
    cfg->attitude.Kd[2][2] = 0.1f;
    cfg->attitude.I[0][0] = 0.01f;
    cfg->attitude.I[1][1] = 0.01f;
    cfg->attitude.I[2][2] = 0.01f;
    /* Allocation: thrust along -z body (z-up) */
    cfg->allocation.t_hat[0] = 0.0f;
    cfg->allocation.t_hat[1] = 0.0f;
    cfg->allocation.t_hat[2] = -1.0f;
    /* Gimbal */
    cfg->gimbal.L = 0.05f;
    cfg->gimbal.theta_min = -0.5f;
    cfg->gimbal.theta_max = 0.5f;
    /* Thrust */
    cfg->thrust.m = 1.0f;
    cfg->thrust.g = 9.8067f;
    cfg->thrust.T_min = 0.0f;
    cfg->thrust.T_max = 50.0f;
    cfg->thrust.kp = 2.0f;
    cfg->thrust.ki = 0.5f;
    cfg->thrust.kd = 1.0f;
    cfg->thrust.integral_limit = 2.0f;
    cfg->thrust.a_z_min = -20.0f;
    cfg->thrust.a_z_max = 20.0f;
}

/** Set ref to identity attitude and zero z setpoint (hover). */
static void init_default_ref(flight_controller_ref_t *ref)
{
    ref->q_ref.w = 1.0f;
    ref->q_ref.x = 0.0f;
    ref->q_ref.y = 0.0f;
    ref->q_ref.z = 0.0f;
    ref->z_ref = 0.0f;
    ref->vz_ref = 0.0f;
}

/**
 * @brief FreeRTOS entry: 1 ms period, get state -> flight_controller_run -> publish control output.
 * @param argument Unused.
 */
void controls_task_start(void *argument)
{
    (void)argument;
    state_t current_state = {0};
    flight_state_t flight_state = IDLE;
    flight_controller_config_t config = {0};
    flight_controller_ref_t ref = {0};
    control_output_t control_output = {0};
    uint8_t config_done = 0;
    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    uint32_t last_state_seq = 0;
    uint32_t stale_tick_count = 0;

    init_default_config(&config);
    init_default_ref(&ref);

    for (;;) {
        TickType_t cycle_start = xTaskGetTickCount();

        uint32_t state_seq = state_exchange_get_state(&current_state);
        state_exchange_get_flight_state(&flight_state);

        if (!config_done) {
            flight_controller_init(&config);
            config_done = 1;
        }

        /* Staleness: if state sequence has not changed for N ticks, treat as stale. */
        if (state_seq == last_state_seq) {
            stale_tick_count++;
        } else {
            last_state_seq = state_seq;
            stale_tick_count = 0;
        }
        bool state_stale = (stale_tick_count >= STALE_STATE_THRESHOLD_TICKS);

        /* Do not run controller on uninitialized or stale state. */
        if (state_seq == 0 || state_stale) {
            memset(&control_output, 0, sizeof(control_output));
        } else {
            flight_controller_run(&current_state, &ref, &config, &control_output, CONTROLS_DT_S);
        }
        state_exchange_publish_control_output(&control_output);

        /* Gimbal: apply theta_x_cmd, theta_y_cmd [rad] to servo pair (converted to degrees). */
        set_servo_pair_degrees(
            control_output.theta_x_cmd * RAD_TO_DEG,
            control_output.theta_y_cmd * RAD_TO_DEG);
    }
}
