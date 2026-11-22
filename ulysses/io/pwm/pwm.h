#ifndef ULYSSES_IO_PWM_PWM_H
#define ULYSSES_IO_PWM_PWM_H

#include <stdint.h>

#include "ulysses/io/pwm.h"

// Platform functions, see the matching io_pwm_*() functions
typedef void (*io_pwm_start_fn_t)(const io_pwm_dev_t *dev, uint32_t freq_hz, uint8_t duty_pct);
typedef void (*io_pwm_stop_fn_t)(const io_pwm_dev_t *dev);

struct io_pwm_ops {
    io_pwm_start_fn_t start; ///< Platform start function
    io_pwm_stop_fn_t stop;   ///< Platform stop function
};

struct io_pwm_dev {
    const void *config;           ///< Platform config
    const struct io_pwm_ops *ops; ///< Platform functions
};

#endif // ULYSSES_IO_PWM_PWM_H
