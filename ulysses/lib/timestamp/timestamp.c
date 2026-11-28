#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/lib/timestamp.h"

static source_cb_t timestamp_source_us;

uint64_t timestamp_now_us(void)
{
    ULYSSES_ASSERT(timestamp_source_us != NULL, "Timestamp source must be set before reading");
    return timestamp_source_us();
}

uint64_t timestamp_now_ms(void)
{
    return timestamp_now_us() / 1000;
}

void timestamp_set_source(source_cb_t source_us)
{
    ULYSSES_ASSERT(source_us != NULL, "Timestamp source must not be NULL");
    timestamp_source_us = source_us;
}
