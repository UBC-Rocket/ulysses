#ifndef ULYSSES_SVC_DEV_SERVO
#define ULYSSES_SVC_DEV_SERVO

#include "ulysses/status.h"

/**
 * @brief Servo bus with a fixed, ordered list of servos
 *
 * Start all position operations on a bus from the same task.
 */
typedef struct dev_servo dev_servo_t;

/**
 * @brief Completion callback for an asynchronous position read
 * @param status Read result
 * @param positions One position in rotations per configured servo, in bus configuration order.
 *                  The array is valid until the callback returns. NULL on failure.
 * @param user_data User data passed to the read operation
 */
typedef void (*dev_servo_read_cb_t)(enum uls_status status, const float *positions,
                                    void *user_data);

/**
 * @brief Completion callback for an asynchronous position write
 * @param status Transmission result. Success means the command finished transmitting.
 * @param user_data User data passed to the write operation
 */
typedef void (*dev_servo_write_cb_t)(enum uls_status status, void *user_data);

/**
 * @brief Reads all positions, blocking until the transaction finishes
 * @param servo Initialized servo bus
 * @param[out] positions Array with room for one float per configured servo. Filled on success
 *                       with positions in rotations, in bus configuration order.
 * @return Transaction status, or ULS_STATUS_BUSY if the bus has an operation in progress
 */
enum uls_status dev_servo_read_positions(dev_servo_t *servo, float *positions);

/**
 * @brief Starts an asynchronous read of all positions
 *
 * When a started read finishes, cb runs once in interrupt context. The bus remains busy until cb
 * returns.
 *
 * @param servo Initialized servo bus
 * @param cb Read completion callback
 * @param user_data Passed to cb
 * @return ULS_STATUS_OK if started, ULS_STATUS_BUSY if the bus has an operation in progress,
 *         or another ULS_STATUS_* error code
 */
enum uls_status dev_servo_read_positions_async(dev_servo_t *servo, dev_servo_read_cb_t cb,
                                               void *user_data);

/**
 * @brief Sends all goal positions, blocking until transmission finishes
 * @param servo Initialized servo bus
 * @param positions Array containing one float position in rotations per configured servo,
 *                  in bus configuration order.
 * @return Transmission status, or ULS_STATUS_BUSY if the bus has an operation in progress
 */
enum uls_status dev_servo_write_positions(dev_servo_t *servo, const float *positions);

/**
 * @brief Starts an asynchronous write of all goal positions
 *
 * When a started write finishes, cb runs once in interrupt context. The bus remains busy until cb
 * returns.
 *
 * @param servo Initialized servo bus
 * @param positions Array containing one float position in rotations per configured servo,
 *                  in bus configuration order. The array may be reused after this call returns.
 * @param cb Write completion callback
 * @param user_data Passed to cb
 * @return ULS_STATUS_OK if started, ULS_STATUS_BUSY if the bus has an operation in progress,
 *         or another ULS_STATUS_* error code
 */
enum uls_status dev_servo_write_positions_async(dev_servo_t *servo, const float *positions,
                                                dev_servo_write_cb_t cb, void *user_data);

#endif // ULYSSES_SVC_DEV_SERVO
