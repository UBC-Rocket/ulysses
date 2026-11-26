#ifndef ULYSSES_SVC_DEV_SERVO_PRIVATE
#define ULYSSES_SVC_DEV_SERVO_PRIVATE

#include "ulysses/svc/dev/servo.h"

struct dev_servo_ops {
    enum uls_status (*read_positions)(dev_servo_t *servo, float *positions);
    enum uls_status (*read_positions_async)(dev_servo_t *servo, dev_servo_read_cb_t cb,
                                            void *user_data);
    enum uls_status (*write_positions)(dev_servo_t *servo, const float *positions);
    enum uls_status (*write_positions_async)(dev_servo_t *servo, const float *positions,
                                             dev_servo_write_cb_t cb, void *user_data);
};

struct dev_servo {
    const void *config;              ///< Caller-provided backend configuration
    void *data;                      ///< Caller-provided backend data storage
    const struct dev_servo_ops *ops; ///< Shared backend operation table
};

#endif // ULYSSES_SVC_DEV_SERVO_PRIVATE
