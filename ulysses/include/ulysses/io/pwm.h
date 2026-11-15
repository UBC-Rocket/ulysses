#ifndef ULYSSES_IO_PWM
#define ULYSSES_IO_PWM

#include <stdint.h>

/** @brief PWM output */
typedef struct io_pwm_dev io_pwm_dev_t;

/**
 * @brief Sets the frequency and duty cycle of a PWM output
 *
 * A duty cycle of 0 keeps the output low.
 *
 * @param dev Output to set
 * @param freq_hz Frequency in Hz
 * @param duty_pct Duty cycle in percent, from 0 to 100
 */
void io_pwm_set(const io_pwm_dev_t *dev, uint32_t freq_hz, uint8_t duty_pct);

#endif // ULYSSES_IO_PWM
