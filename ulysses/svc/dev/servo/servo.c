#include "dev/servo/servo.h"

#include <stddef.h>

#include "ulysses/lib/assert.h"

enum uls_status dev_servo_read_positions(dev_servo_t *servo, float *positions)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(positions != NULL, "positions must not be NULL");

    return servo->ops->read_positions(servo, positions);
}

enum uls_status dev_servo_read_positions_async(dev_servo_t *servo, dev_servo_read_cb_t cb,
                                               void *user_data)
{
    ULYSSES_ASSERT(servo != NULL && servo->ops != NULL, "servo bus must be initialized");
    ULYSSES_ASSERT(cb != NULL, "read callback must not be NULL");

    return servo->ops->read_positions_async(servo, cb, user_data);
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
