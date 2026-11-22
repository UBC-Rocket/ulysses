#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/status_led.h"

#include "dev/status_led/status_led.h"

void dev_status_led_on(const dev_status_led_t *led)
{
    ULYSSES_ASSERT(led != NULL, "Status LED must be non NULL");
    ULYSSES_ASSERT(led->ops != NULL, "Status LED must be initialized");
    ULYSSES_ASSERT(led->ops->on != NULL, "Status LED on function must be non NULL");

    led->ops->on(led);
}

void dev_status_led_off(const dev_status_led_t *led)
{
    ULYSSES_ASSERT(led != NULL, "Status LED must be non NULL");
    ULYSSES_ASSERT(led->ops != NULL, "Status LED must be initialized");
    ULYSSES_ASSERT(led->ops->off != NULL, "Status LED off function must be non NULL");

    led->ops->off(led);
}
