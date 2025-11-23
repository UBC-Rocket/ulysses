/**
 ******************************************************************************
 * @file    pid.c
 * @brief   PID Controller Implementation for Drone Flight Control
 * @author  Ulysses Flight Controller
 * @date    2025-11-23
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "controls/pid.h"

/* Private function prototypes -----------------------------------------------*/
static float clamp(float value, float min, float max);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize PID controller with gains and limits
 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integralLimit,
              float outputMin,
              float outputMax)
{
    /* Set tuning gains */
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    /* Initialize state to zero */
    pid->integralSum = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->dt = 0.0f;

    /* Set limits */
    pid->integralLimit = integralLimit;
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;
}

/**
 * @brief  Compute PID output using derivative-on-measurement algorithm
 */
float PID_Compute(PID_Controller_t *pid,
                  float setpoint,
                  float measurement,
                  float dt)
{
    float error;
    float proportional;
    float integral;
    float derivative;
    float output;

    /* Store dt for reference */
    pid->dt = dt;

    /* Calculate error: how far we are from desired setpoint */
    error = setpoint - measurement;

    /* --- PROPORTIONAL TERM --- */
    /* Responds immediately to current error */
    proportional = pid->Kp * error;

    /* --- INTEGRAL TERM --- */
    /* Accumulates error over time to eliminate steady-state error */
    /* Integrate: sum up error over time */
    pid->integralSum += error * dt;

    /* Apply anti-windup: clamp integral to prevent overflow */
    /* This prevents the integral from growing unbounded when output is saturated */
    pid->integralSum = clamp(pid->integralSum, -pid->integralLimit, pid->integralLimit);

    /* Calculate integral contribution */
    integral = pid->Ki * pid->integralSum;

    /* --- DERIVATIVE TERM --- */
    /* Derivative on measurement (not error) to prevent derivative kick */
    /* When setpoint changes suddenly, derivative of error would spike */
    /* Using derivative of measurement avoids this problem */
    /* Note: Negative sign because we want to oppose rapid changes in measurement */
    derivative = -pid->Kd * (measurement - pid->prevMeasurement) / dt;

    /* Store current measurement for next iteration */
    pid->prevMeasurement = measurement;

    /* --- COMPUTE OUTPUT --- */
    /* Combine all three terms */
    output = proportional + integral + derivative;

    /* Apply output limits to ensure actuators stay within safe range */
    output = clamp(output, pid->outputMin, pid->outputMax);

    return output;
}

/**
 * @brief  Reset PID controller internal state
 */
void PID_Reset(PID_Controller_t *pid)
{
    /* Clear integral accumulator */
    pid->integralSum = 0.0f;

    /* Reset previous measurement to avoid derivative spike */
    pid->prevMeasurement = 0.0f;

    /* Reset dt */
    pid->dt = 0.0f;
}

/**
 * @brief  Update PID gains at runtime
 */
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/**
 * @brief  Update output and integral limits at runtime
 */
void PID_SetLimits(PID_Controller_t *pid,
                   float integralLimit,
                   float outputMin,
                   float outputMax)
{
    /* Update limits */
    pid->integralLimit = integralLimit;
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;

    /* Clamp existing integral to new limits */
    pid->integralSum = clamp(pid->integralSum, -integralLimit, integralLimit);
}

/**
 * @brief  Get current integral term value
 */
float PID_GetIntegral(PID_Controller_t *pid)
{
    return pid->integralSum;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Clamp a value between min and max
 * @param  value: Value to clamp
 * @param  min: Minimum allowed value
 * @param  max: Maximum allowed value
 * @retval Clamped value
 */
static float clamp(float value, float min, float max)
{
    if (value > max) {
        return max;
    } else if (value < min) {
        return min;
    }
    return value;
}
