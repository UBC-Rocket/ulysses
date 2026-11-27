#include "dev/servo/servo.h"

#include <stddef.h>

#include "ulysses/lib/assert.h"

enum uls_status dev_servo_ping(dev_servo_t *servo, uint32_t index, uint8_t *error_mask)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(error_mask != NULL, "error mask must not be NULL");

    return servo->ops->ping(servo, index, error_mask);
}

enum uls_status dev_servo_set_torque(dev_servo_t *servo, enum dev_servo_torque_mode mode)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(mode == DEV_SERVO_TORQUE_DISABLED || mode == DEV_SERVO_TORQUE_ENABLED,
                   "invalid servo torque mode");

    return servo->ops->set_torque(servo, mode);
}

enum uls_status dev_servo_read_positions(dev_servo_t *servo, float *positions)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(positions != NULL, "positions must not be NULL");

    return servo->ops->read_positions(servo, positions);
}

enum uls_status dev_servo_read_positions_async(dev_servo_t *servo, float *positions,
                                               dev_servo_read_cb_t cb, void *user_data)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(positions != NULL, "positions must not be NULL");
    ULYSSES_ASSERT(cb != NULL, "read callback must not be NULL");

    return servo->ops->read_positions_async(servo, positions, cb, user_data);
}

enum uls_status dev_servo_write_positions(dev_servo_t *servo, const float *positions)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(positions != NULL, "positions must not be NULL");

    return servo->ops->write_positions(servo, positions);
}

enum uls_status dev_servo_write_positions_async(dev_servo_t *servo, const float *positions,
                                                dev_servo_write_cb_t cb, void *user_data)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(positions != NULL, "positions must not be NULL");
    ULYSSES_ASSERT(cb != NULL, "write callback must not be NULL");

    return servo->ops->write_positions_async(servo, positions, cb, user_data);
}
