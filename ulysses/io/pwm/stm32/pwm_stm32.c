#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/pwm.h"

#include "pwm/stm32/pwm_stm32.h"

// Counts per period of a 16-bit timer
#define MAX_PERIOD_TICKS (65536U)

static void stm32_set(const io_pwm_dev_t *dev, uint32_t freq_hz, uint8_t duty_pct)
{
    const struct io_pwm_stm32_config *config = dev->config;

    ULYSSES_ASSERT(freq_hz <= config->clock_hz, "PWM frequency must be at most the timer clock");

    // Smallest prescaler that fits the period in the counter
    uint32_t ticks = config->clock_hz / freq_hz;
    uint32_t prescaler = (ticks - 1) / MAX_PERIOD_TICKS + 1;
    uint32_t period = ticks / prescaler;

    __HAL_TIM_SET_PRESCALER(config->hw_handle, prescaler - 1);
    __HAL_TIM_SET_AUTORELOAD(config->hw_handle, period - 1);
    __HAL_TIM_SET_COMPARE(config->hw_handle, config->channel, period * duty_pct / 100);

    // Applies the prescaler now instead of at the end of the current period
    HAL_StatusTypeDef status = HAL_TIM_GenerateEvent(config->hw_handle, TIM_EVENTSOURCE_UPDATE);
    ULYSSES_ASSERT(status == HAL_OK, "PWM timer must not be busy");
}

static const struct io_pwm_ops stm32_pwm_ops = {
    .set = stm32_set,
};

void io_pwm_stm32_init(io_pwm_dev_t *dev, const struct io_pwm_stm32_config *config)
{
    ULYSSES_ASSERT(dev != NULL, "PWM device must be non NULL");
    ULYSSES_ASSERT(config != NULL, "PWM config must be non NULL");
    ULYSSES_ASSERT(config->hw_handle != NULL, "PWM timer handle must be non NULL");
    ULYSSES_ASSERT(config->clock_hz != 0, "PWM timer clock must be non zero");

    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(config->hw_handle, config->channel);
    ULYSSES_ASSERT(status == HAL_OK, "PWM timer channel must be configured and not started");

    dev->config = config;
    dev->ops = &stm32_pwm_ops;
}
