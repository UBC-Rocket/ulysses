#ifndef ULYSSES_IO_PWM_STM32_PWM_STM32_H
#define ULYSSES_IO_PWM_STM32_PWM_STM32_H

#include <stdint.h>

#include "stm32h7xx_hal.h" // IWYU pragma: keep

#include "ulysses/io/pwm.h"

#include "pwm/pwm.h"

// STM32 wiring of a PWM output
struct io_pwm_stm32_config {
    TIM_HandleTypeDef *hw_handle; ///< HAL timer handle
    uint32_t channel;             ///< Timer channel (TIM_CHANNEL_x)
    uint32_t clock_hz;            ///< Timer input clock in Hz
};

/**
 * @brief Initializes a PWM output on an STM32
 *
 * The timer channel must already be configured for PWM. The output stays stopped until
 * io_pwm_start() is called.
 *
 * @param dev Output to initialize
 * @param config Wiring of the output, must stay valid while the output is used
 */
void io_pwm_stm32_init(io_pwm_dev_t *dev, const struct io_pwm_stm32_config *config);

#endif // ULYSSES_IO_PWM_STM32_PWM_STM32_H
