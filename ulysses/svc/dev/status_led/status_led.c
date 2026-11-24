#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/status_led.h"

#include "dev/status_led/status_led.h"

#define ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))

struct pattern {
    uint32_t on_ms;  ///< Time on per blink, 0 for always off
    uint32_t off_ms; ///< Time off per blink, 0 for always on
};

static const struct pattern PATTERNS[] = {
    [DEV_STATUS_LED_PATTERN_OFF] = {.on_ms = 0, .off_ms = 1000},
    [DEV_STATUS_LED_PATTERN_ON] = {.on_ms = 1000, .off_ms = 0},
    [DEV_STATUS_LED_PATTERN_ARMED] = {.on_ms = 500, .off_ms = 500},
};

static void set_led(dev_status_led_t *led, bool on, uint32_t now_ms)
{
    led->on = on;
    led->change_ms = now_ms;

    if (on) {
        led->ops->on(led);
    } else {
        led->ops->off(led);
    }
}

void dev_status_led_set_pattern(dev_status_led_t *led, enum dev_status_led_pattern pattern,
                                uint32_t now_ms)
{
    ULYSSES_ASSERT(led != NULL, "Status LED must be non NULL");
    ULYSSES_ASSERT(led->ops != NULL, "Status LED must be initialized");
    ULYSSES_ASSERT((size_t)pattern < ARRAY_LEN(PATTERNS), "Status LED pattern must be valid");

    led->pattern = pattern;
    set_led(led, PATTERNS[pattern].on_ms > 0, now_ms);
}

void dev_status_led_update(dev_status_led_t *led, uint32_t now_ms)
{
    ULYSSES_ASSERT(led != NULL, "Status LED must be non NULL");
    ULYSSES_ASSERT(led->ops != NULL, "Status LED must be initialized");

    const struct pattern *pattern = &PATTERNS[led->pattern];
    // Always on and always off patterns never change
    if (pattern->on_ms == 0 || pattern->off_ms == 0) {
        return;
    }

    uint32_t duration_ms = led->on ? pattern->on_ms : pattern->off_ms;
    if (now_ms - led->change_ms < duration_ms) {
        return;
    }

    set_led(led, !led->on, now_ms);
}
