#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/esc.h"

#include "dev/esc/esc.h"

enum uls_status dev_esc_set_arm(const dev_esc_t *esc, uint8_t channel, bool is_armed)
{
    ULYSSES_ASSERT(esc != NULL, "ESC must not be NULL");
    ULYSSES_ASSERT(esc->ops != NULL, "ESC must be initialized");
    ULYSSES_ASSERT(esc->ops->set_arm != NULL, "Function must not be NULL");

    return esc->ops->set_arm(esc, channel, is_armed);
}

enum uls_status dev_esc_set_throttle(const dev_esc_t *esc, uint8_t channel, uint16_t throttle)
{
    ULYSSES_ASSERT(esc != NULL, "ESC must not be NULL");
    ULYSSES_ASSERT(esc->ops != NULL, "ESC must be initialized");
    ULYSSES_ASSERT(esc->ops->set_throttle != NULL, "Function must not be NULL");

    return esc->ops->set_throttle(esc, channel, throttle);
}

enum uls_status dev_esc_get_telemetry(const dev_esc_t *esc, struct dev_esc_telemetry *telemetry,
                                      size_t telemetry_capacity)
{
    ULYSSES_ASSERT(esc != NULL, "ESC must not be NULL");
    ULYSSES_ASSERT(esc->ops != NULL, "ESC must be initialized");
    ULYSSES_ASSERT(esc->ops->get_telemetry != NULL, "Function must not be NULL");
    ULYSSES_ASSERT(telemetry != NULL, "Telemetry must not be NULL");
    ULYSSES_ASSERT(telemetry_capacity > 0, "Telemetry capacity must be greater than zero");

    return esc->ops->get_telemetry(esc, telemetry, telemetry_capacity);
}

enum uls_status dev_esc_update(const dev_esc_t *esc)
{
    ULYSSES_ASSERT(esc != NULL, "ESC must not be NULL");
    ULYSSES_ASSERT(esc->ops != NULL, "ESC must be initialized");
    ULYSSES_ASSERT(esc->ops->update != NULL, "Function must not be NULL");

    return esc->ops->update(esc);
}
