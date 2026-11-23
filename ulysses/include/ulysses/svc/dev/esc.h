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

enum uls_status dev_esc_set_arm(const dev_esc_t *esc, uint8_t channel, bool is_armed);
enum uls_status dev_esc_set_throttle(const dev_esc_t *esc, uint8_t channel, uint16_t throttle);
enum uls_status dev_esc_get_telemetry(const dev_esc_t *esc, struct dev_esc_telemetry *telemetry,
                                      size_t telemetry_capacity);
enum uls_status dev_esc_update(const dev_esc_t *esc);

#endif // ULYSSES_SVC_DEV_ESC
