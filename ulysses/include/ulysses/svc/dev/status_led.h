#ifndef ULYSSES_SVC_DEV_STATUS_LED
#define ULYSSES_SVC_DEV_STATUS_LED

#include <stdint.h>

/** @brief Status LED, initialized by an implementation-specific init function */
typedef struct dev_status_led dev_status_led_t;

/** @brief Blink pattern of a status LED */
enum dev_status_led_pattern {
    DEV_STATUS_LED_PATTERN_OFF,   ///< Always off
    DEV_STATUS_LED_PATTERN_ON,    ///< Always on
    DEV_STATUS_LED_PATTERN_ARMED, ///< On for 0.5 s, then off for 0.5 s
};

/**
 * @brief Starts showing a blink pattern from its first step
 *
 * @param led Non-NULL initialized LED
 * @param pattern Pattern to show
 * @param now_ms Current time in milliseconds
 */
void dev_status_led_set_pattern(dev_status_led_t *led, enum dev_status_led_pattern pattern,
                                uint32_t now_ms);

/**
 * @brief Moves the blink pattern forward
 *
 * Call periodically. The LED changes at most once per call.
 *
 * @param led Non-NULL initialized LED
 * @param now_ms Current time in milliseconds
 */
void dev_status_led_update(dev_status_led_t *led, uint32_t now_ms);

#endif // ULYSSES_SVC_DEV_STATUS_LED
