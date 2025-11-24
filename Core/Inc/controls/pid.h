/**
 ******************************************************************************
 * @file    pid.h
 * @brief   PID Controller Library for Drone Flight Control
 * @author  Ulysses Flight Controller
 * @date    2025-11-23
 ******************************************************************************
 * @attention
 *
 * This PID controller is designed for drone rate control (roll, pitch, yaw).
 * It implements:
 *   - Derivative on measurement (prevents derivative kick on setpoint changes)
 *   - Integral anti-windup (prevents integral term overflow)
 *   - Output limiting (constrains control output)
 *
 * Usage Example:
 * @code
 * // Initialize PID controller for roll axis
 * PID_Controller_t rollPID;
 * PID_Init(&rollPID, 1.5f, 0.5f, 0.05f, 100.0f, -500.0f, 500.0f);
 *
 * // In your main flight control loop:
 * float dt = 0.002f;  // 2ms loop time (500Hz)
 * float desiredRollRate = 45.0f;  // deg/s from pilot stick input
 * float currentGyroRate = gyro.x;  // deg/s from IMU
 *
 * float motorCorrection = PID_Compute(&rollPID, desiredRollRate, currentGyroRate, dt);
 * @endcode
 *
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PID Controller structure
 *
 * This structure holds all state and configuration for a single PID controller.
 * For drone control, you'll typically need three instances: roll, pitch, yaw.
 */
typedef struct {
    /* Tuning gains */
    float Kp;                   /**< Proportional gain */
    float Ki;                   /**< Integral gain */
    float Kd;                   /**< Derivative gain */

    /* Internal state (do not modify directly) */
    float integralSum;          /**< Accumulated integral term */
    float prevMeasurement;      /**< Previous measurement for derivative calculation */

    /* Limits */
    float integralLimit;        /**< Maximum absolute value for integral term (anti-windup) */
    float outputMin;            /**< Minimum output value */
    float outputMax;            /**< Maximum output value */

    /* Configuration */
    float dt;                   /**< Last time step used (for reference) */

} PID_Controller_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize PID controller with gains and limits
 * @param  pid: Pointer to PID controller structure
 * @param  kp: Proportional gain
 * @param  ki: Integral gain
 * @param  kd: Derivative gain
 * @param  integralLimit: Maximum absolute value for integral term (anti-windup)
 * @param  outputMin: Minimum output limit
 * @param  outputMax: Maximum output limit
 * @retval None
 *
 * @note   After initialization, all internal state is zeroed. Call this once
 *         during setup for each PID controller instance.
 */
void PID_Init(PID_Controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integralLimit,
              float outputMin,
              float outputMax);

/**
 * @brief  Compute PID output
 * @param  pid: Pointer to PID controller structure
 * @param  setpoint: Desired value (e.g., desired roll rate in deg/s)
 * @param  measurement: Current measured value (e.g., current gyro rate in deg/s)
 * @param  dt: Time step since last call in seconds (e.g., 0.002 for 500Hz)
 * @retval PID output value (constrained to outputMin/outputMax)
 *
 * @note   This function should be called at a consistent rate in your control loop.
 *         The dt parameter accounts for timing variations if needed.
 *
 * Algorithm:
 *   - error = setpoint - measurement
 *   - integral += error * dt (with anti-windup limiting)
 *   - derivative = -(measurement - prevMeasurement) / dt  (derivative on measurement)
 *   - output = Kp*error + Ki*integral + Kd*derivative
 *   - output is clamped to [outputMin, outputMax]
 */
float PID_Compute(PID_Controller_t *pid,
                  float setpoint,
                  float measurement,
                  float dt);

/**
 * @brief  Reset PID controller state
 * @param  pid: Pointer to PID controller structure
 * @retval None
 *
 * @note   Clears integral accumulator and previous measurement.
 *         Use this when:
 *         - Switching flight modes
 *         - After a crash/disarm
 *         - When PID has been inactive
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief  Update PID gains at runtime
 * @param  pid: Pointer to PID controller structure
 * @param  kp: New proportional gain
 * @param  ki: New integral gain
 * @param  kd: New derivative gain
 * @retval None
 *
 * @note   Useful for in-flight tuning or adaptive control.
 *         Does not reset integral or other state.
 */
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd);

/**
 * @brief  Update output and integral limits at runtime
 * @param  pid: Pointer to PID controller structure
 * @param  integralLimit: New maximum absolute value for integral term
 * @param  outputMin: New minimum output limit
 * @param  outputMax: New maximum output limit
 * @retval None
 *
 * @note   Immediately clamps existing integral sum if it exceeds new limits.
 */
void PID_SetLimits(PID_Controller_t *pid,
                   float integralLimit,
                   float outputMin,
                   float outputMax);

/**
 * @brief  Get current integral term value
 * @param  pid: Pointer to PID controller structure
 * @retval Current integral sum
 *
 * @note   Useful for debugging and monitoring controller state.
 */
float PID_GetIntegral(PID_Controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
