#ifndef ULYSSES_SVC_DEV_SERVO
#define ULYSSES_SVC_DEV_SERVO

#include "ulysses/status.h"

/**
 * @brief Servo bus with a fixed list of servos
 *
 * Calls to functions on the same servo bus must be serialized.
 */
typedef struct dev_servo dev_servo_t;

/**
 * @brief Completion callback for an asynchronous position read
 * @param status ULS_STATUS_OK on success, ULS_STATUS_INVALID_INPUT for a malformed reply,
 *               ULS_STATUS_ERROR for a servo or UART failure
 * @param positions Positions in rotations, in configured ID order. Valid until the next read
 *                  updates the array. NULL on failure.
 * @param user_data User data passed to the read operation
 */
typedef void (*dev_servo_read_cb_t)(enum uls_status status, const float *positions,
                                    void *user_data);

/**
 * @brief Completion callback for an asynchronous position write
 * @param status ULS_STATUS_OK when transmission finishes, or ULS_STATUS_ERROR on UART failure
 * @param user_data User data passed to the write operation
 */
typedef void (*dev_servo_write_cb_t)(enum uls_status status, void *user_data);

/**
 * @brief Reads positions synchronously
 *
 * @param servo Initialized servo bus
 * @param[out] positions Array of one float per servo, in rotations and configured ID order
 *
 * @retval ULS_STATUS_OK Positions read successfully
 * @retval ULS_STATUS_BUSY The bus is busy
 * @retval ULS_STATUS_INVALID_INPUT The reply is malformed
 * @retval ULS_STATUS_TIMEOUT The UART operation timed out
 * @retval ULS_STATUS_ERROR A servo or UART reported an error
 */
enum uls_status dev_servo_read_positions(dev_servo_t *servo, float *positions);

/**
 * @brief Starts an asynchronous position read
 *
 * When a started read finishes, cb runs once in interrupt context. The bus is available when cb
 * runs. Starting another read can overwrite the positions supplied to cb.
 *
 * @param servo Initialized servo bus
 * @param cb Read completion callback
 * @param user_data Context passed to cb. May be NULL.
 *
 * @retval ULS_STATUS_OK The read was started
 * @retval ULS_STATUS_BUSY The bus or UART is busy
 * @retval ULS_STATUS_ERROR The UART transmit could not be started
 */
enum uls_status dev_servo_read_positions_async(dev_servo_t *servo, dev_servo_read_cb_t cb,
                                               void *user_data);

/**
 * @brief Writes goal positions synchronously
 *
 * Success means the command was transmitted.
 *
 * @param servo Initialized servo bus
 * @param positions Array of one float per servo, in rotations and configured ID order
 *
 * @retval ULS_STATUS_OK The command was transmitted
 * @retval ULS_STATUS_BUSY The bus or UART is busy
 * @retval ULS_STATUS_TIMEOUT The UART transmit timed out
 * @retval ULS_STATUS_ERROR The UART transmit failed
 */
enum uls_status dev_servo_write_positions(dev_servo_t *servo, const float *positions);

/**
 * @brief Starts an asynchronous goal position write
 *
 * When a started write finishes, cb runs once in interrupt context. The bus is available when cb
 * runs.
 *
 * @param servo Initialized servo bus
 * @param positions Array of one float per servo, in rotations and configured ID order. May be
 *                  reused after this call returns.
 * @param cb Write completion callback
 * @param user_data Context passed to cb. May be NULL.
 *
 * @retval ULS_STATUS_OK The write was started
 * @retval ULS_STATUS_BUSY The bus or UART is busy
 * @retval ULS_STATUS_ERROR The UART transmit could not be started
 */
enum uls_status dev_servo_write_positions_async(dev_servo_t *servo, const float *positions,
                                                dev_servo_write_cb_t cb, void *user_data);

#endif // ULYSSES_SVC_DEV_SERVO
