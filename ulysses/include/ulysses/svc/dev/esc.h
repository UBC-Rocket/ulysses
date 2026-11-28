#ifndef ULYSSES_SVC_DEV_ESC
#define ULYSSES_SVC_DEV_ESC

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "ulysses/status.h"

typedef struct dev_esc dev_esc_t;

struct dev_esc_telemetry {
    bool is_valid; //< Are the telemetry values valid to interpret?

    /**
     * @brief Is there an error in receiving the telemetry
     * 
     * If there is an error, the telemetry is usually stale, and should
     * be treated with caution when interpreting it.
     */
    bool is_error;

    float rpm; //< RPM of the motor connected to the ESC channel
};

/**
 * @brief Arms or disarms the ESC channel
 *
 * After arming, you must ensure no throttle commands are sent for a duration
 * of around 3 seconds (this may be more or less depending on the ESC).
 *
 * Must be serialized with calls to update.
 * 
 * @param esc ESC instance
 * @param channel Channel to arm
 * @param is_armed Whether to arm or disarm
 * @return enum uls_status 
 */
enum uls_status dev_esc_set_arm(const dev_esc_t *esc, uint8_t channel, bool is_armed);

/**
 * @brief Sets the throttle on the ESC channel
 * 
 * Must be serialized with calls to update.
 *
 * @param esc ESC instance
 * @param channel Channel to set the throttle for
 * @param throttle Desired throttle of the motor
 * @return enum uls_status 
 */
enum uls_status dev_esc_set_throttle(const dev_esc_t *esc, uint8_t channel, uint16_t throttle);

/**
 * @brief Retrieves the telemetry of the ESC channel
 *
 * @param esc ESC instance
 * @param[out] telemetry Array of telemetry values indexed by the channel
 * @param telemetry_capacity Amount of telemetry to retrieve
 * @return enum uls_status 
 */
enum uls_status dev_esc_get_telemetry(const dev_esc_t *esc, struct dev_esc_telemetry *telemetry,
                                      size_t telemetry_capacity);

/**
 * @brief Updates the ESC
 * 
 * This function at a regular interval (around every 1 millisecond).
 *
 * Must not be called in an ISR.
 *
 * @param esc ESC channel
 * @return enum uls_status 
 */
enum uls_status dev_esc_update(const dev_esc_t *esc);

#endif // ULYSSES_SVC_DEV_ESC
