#ifndef ULYSSES_IO_PWM
#define ULYSSES_IO_PWM

#include <stdint.h>

/** @brief PWM output */
typedef struct io_pwm_dev io_pwm_dev_t;

/**
 * @brief Starts a PWM output
 *
 * @param dev Output to start, must not be running
 * @param freq_hz Frequency in Hz
 * @param duty_pct Duty cycle in percent, from 0 to 100
 */
void io_pwm_start(const io_pwm_dev_t *dev, uint32_t freq_hz, uint8_t duty_pct);

/**
 * @brief Stops a PWM output
 *
 * @param dev Output to stop
 */
void io_pwm_stop(const io_pwm_dev_t *dev);

#endif // ULYSSES_IO_PWM
