#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/pwm.h"

#include "pwm/pwm.h"

void io_pwm_start(const io_pwm_dev_t *dev, uint32_t freq_hz, uint8_t duty_pct)
{
    ULYSSES_ASSERT(dev != NULL, "PWM device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "PWM device must be initialized");
    ULYSSES_ASSERT(dev->ops->start != NULL, "PWM start function must be non NULL");
    ULYSSES_ASSERT(freq_hz > 0, "PWM frequency must be greater than 0");
    ULYSSES_ASSERT(duty_pct <= 100, "PWM duty cycle must be at most 100");

    dev->ops->start(dev, freq_hz, duty_pct);
}

void io_pwm_stop(const io_pwm_dev_t *dev)
{
    ULYSSES_ASSERT(dev != NULL, "PWM device must be non NULL");
    ULYSSES_ASSERT(dev->ops != NULL, "PWM device must be initialized");
    ULYSSES_ASSERT(dev->ops->stop != NULL, "PWM stop function must be non NULL");

    dev->ops->stop(dev);
}
