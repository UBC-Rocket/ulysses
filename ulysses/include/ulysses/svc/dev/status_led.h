#ifndef ULYSSES_SVC_DEV_STATUS_LED
#define ULYSSES_SVC_DEV_STATUS_LED

/** @brief Status LED, initialized by an implementation-specific init function */
typedef struct dev_status_led dev_status_led_t;

/**
 * @brief Turns a status LED on
 *
 * @param led Non-NULL initialized LED
 */
void dev_status_led_on(const dev_status_led_t *led);

/**
 * @brief Turns a status LED off
 *
 * @param led Non-NULL initialized LED
 */
void dev_status_led_off(const dev_status_led_t *led);

#endif // ULYSSES_SVC_DEV_STATUS_LED
