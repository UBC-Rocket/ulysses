#ifndef ULYSSES_SVC_DEV_SERVO
#define ULYSSES_SVC_DEV_SERVO

#include <stdint.h>

#include "ulysses/status.h"

/**
 * @brief Servo bus with a fixed list of servos
 *
 * Calls to functions on the same servo bus must be serialized.
 *
 * After a transmit or receive failure, a partial request or unread reply bytes can interfere with
 * the next operation if it starts immediately.
 */
typedef struct dev_servo dev_servo_t;

/**
 * @brief Completion callback for an asynchronous position read
 *
 * - ULS_STATUS_OK Positions read successfully
 * - ULS_STATUS_BUSY The transport was busy when the driver tried to start reception
 * - ULS_STATUS_TIMEOUT Reception timed out
 * - ULS_STATUS_INVALID_INPUT The reply is malformed or has an unexpected ID
 * - ULS_STATUS_ERROR A servo or transport reported an error
 *
 * @param status Read result
 * @param user_data User data passed to the read operation
 */
typedef void (*dev_servo_read_cb_t)(enum uls_status status, void *user_data);

/**
 * @brief Completion callback for an asynchronous position write
 *
 * - ULS_STATUS_OK The command was transmitted
 * - ULS_STATUS_ERROR The transport failed to transmit the command
 *
 * @param status Write result
 * @param user_data User data passed to the write operation
 */
typedef void (*dev_servo_write_cb_t)(enum uls_status status, void *user_data);

enum dev_servo_hw_error {
    DEV_SERVO_HW_ERROR_INPUT_VOLTAGE = (1 << 0),    ///< Input Voltage Error
    DEV_SERVO_HW_ERROR_OVERHEATING = (1 << 2),      ///< Overheating Error
    DEV_SERVO_HW_ERROR_ENCODER = (1 << 3),          ///< Motor Encoder Error
    DEV_SERVO_HW_ERROR_ELECTRICAL_SHOCK = (1 << 4), ///< Electrical Shock Error
    DEV_SERVO_HW_ERROR_OVERLOAD = (1 << 5),         ///< Overload Error
    DEV_SERVO_HW_ERROR_READ_FAILED = (1 << 7),      ///< Status read failed after ping
};

/**
 * @brief Pings one servo synchronously and reports hardware errors
 *
 * Reads the hardware error status if the servo signals a hardware fault in its ping reply.
 *
 * @param servo Initialized servo bus
 * @param index Zero-based index of a configured servo
 * @param[out] error_mask Non-NULL output for the hardware error mask
 *
 * @retval ULS_STATUS_OK The servo reported no hardware errors. The mask is zero.
 * @retval ULS_STATUS_BUSY The servo bus is busy. The mask is zero.
 * @retval ULS_STATUS_ERROR The mask is zero if the ping failed, DEV_SERVO_HW_ERROR_READ_FAILED if the
 *                          status read failed, or the reported hardware errors if the read succeeded.
 */
enum uls_status dev_servo_ping(dev_servo_t *servo, uint32_t index, uint8_t *error_mask);

enum dev_servo_torque_mode {
    DEV_SERVO_TORQUE_DISABLED = 0,
    DEV_SERVO_TORQUE_ENABLED = 1,
};

/**
 * @brief Sets torque mode for all configured servos synchronously
 *
 * Returns the first failure found in the torque readback.
 *
 * @param servo Initialized servo bus
 * @param mode Torque mode to send
 *
 * @retval ULS_STATUS_OK Every servo reported the requested torque mode with no device error
 * @retval ULS_STATUS_NOT_READY A servo reported a different torque mode
 * @retval ULS_STATUS_BUSY The bus or transport is busy
 * @retval ULS_STATUS_TIMEOUT A transport operation timed out
 * @retval ULS_STATUS_ERROR A servo or transport reported an error
 * @retval ULS_STATUS_INVALID_INPUT The readback reply is malformed
 */
enum uls_status dev_servo_set_torque(dev_servo_t *servo, enum dev_servo_torque_mode mode);

/**
 * @brief Reads positions synchronously
 *
 * Positions are relative to each servo's calibrated zero and wrap to [-0.5, 0.5) rotations.
 *
 * @param servo Initialized servo bus
 * @param[out] positions Array of one float per servo, in rotations and configured ID order.
 *                       Filled on success.
 *
 * @retval ULS_STATUS_OK Positions read successfully
 * @retval ULS_STATUS_BUSY The bus or transport is busy
 * @retval ULS_STATUS_INVALID_INPUT The reply is malformed
 * @retval ULS_STATUS_TIMEOUT A transport operation timed out
 * @retval ULS_STATUS_ERROR A servo or transport reported an error
 */
enum uls_status dev_servo_read_positions(dev_servo_t *servo, float *positions);

/**
 * @brief Starts an asynchronous position read
 *
 * Positions are relative to each servo's calibrated zero and wrap to [-0.5, 0.5) rotations.
 *
 * When a started read finishes, cb runs once in interrupt context. The bus is available when cb
 * runs. A failed start does not call cb.
 *
 * @param servo Initialized servo bus
 * @param[out] positions Array of one float per servo, in rotations and configured ID order.
 *                       Keep it valid until cb runs. The array is filled before cb on success
 *                       and left unchanged on failure.
 * @param cb Read completion callback
 * @param user_data Context passed to cb. May be NULL.
 *
 * @retval ULS_STATUS_OK The read was started
 * @retval ULS_STATUS_BUSY The bus or transport is busy
 * @retval ULS_STATUS_ERROR The transmit could not be started
 */
enum uls_status dev_servo_read_positions_async(dev_servo_t *servo, float *positions,
                                               dev_servo_read_cb_t cb, void *user_data);

/**
 * @brief Writes goal positions synchronously
 *
 * Goals are positions relative to each servo's calibrated zero. They can span multiple rotations
 * within the backend's goal range.
 *
 * @param servo Initialized servo bus
 * @param positions Array of one float per servo, in rotations and configured ID order
 *
 * @retval ULS_STATUS_OK The command was transmitted
 * @retval ULS_STATUS_BUSY The bus or transport is busy
 * @retval ULS_STATUS_TIMEOUT The transmit timed out
 * @retval ULS_STATUS_ERROR The transmit failed
 */
enum uls_status dev_servo_write_positions(dev_servo_t *servo, const float *positions);

/**
 * @brief Starts an asynchronous goal position write
 *
 * Goals are positions relative to each servo's calibrated zero. They can span multiple rotations
 * within the backend's goal range.
 *
 * When a started write finishes, cb runs once in interrupt context. The bus is available when cb
 * runs. A failed start does not call cb.
 *
 * @param servo Initialized servo bus
 * @param positions Array of one float per servo, in rotations and configured ID order. May be
 *                  reused after this call returns.
 * @param cb Write completion callback
 * @param user_data Context passed to cb. May be NULL.
 *
 * @retval ULS_STATUS_OK The write was started
 * @retval ULS_STATUS_BUSY The bus or transport is busy
 * @retval ULS_STATUS_ERROR The transmit could not be started
 */
enum uls_status dev_servo_write_positions_async(dev_servo_t *servo, const float *positions,
                                                dev_servo_write_cb_t cb, void *user_data);

#endif // ULYSSES_SVC_DEV_SERVO
