#ifndef ULYSSES_IO_PWM_PWM_H
#define ULYSSES_IO_PWM_PWM_H

#include <stdint.h>

#include "ulysses/io/pwm.h"

// Platform function, see io_pwm_set()
typedef void (*io_pwm_set_fn_t)(const io_pwm_dev_t *dev, uint32_t freq_hz, uint8_t duty_pct);

struct io_pwm_ops {
    io_pwm_set_fn_t set; ///< Platform set function
};

struct io_pwm_dev {
    const void *config;           ///< Platform config
    const struct io_pwm_ops *ops; ///< Platform functions
};

#endif // ULYSSES_IO_PWM_PWM_H
