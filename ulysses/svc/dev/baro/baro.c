#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/baro.h"

#include "dev/baro/baro.h"

enum io_status dev_baro_read_blocking(dev_baro_t *baro, struct dev_baro_sample *sample)
{
    ULYSSES_ASSERT(baro != NULL, "Barometer must not be NULL");
    ULYSSES_ASSERT(baro->ops != NULL, "Barometer must be initialized");
    ULYSSES_ASSERT(sample != NULL, "Sample must not be NULL");

    return baro->ops->read_blocking(baro, sample);
}

enum io_status dev_baro_read_async(dev_baro_t *baro, dev_baro_cb_t cb, void *user_data)
{
    ULYSSES_ASSERT(baro != NULL, "Barometer must not be NULL");
    ULYSSES_ASSERT(baro->ops != NULL, "Barometer must be initialized");
    ULYSSES_ASSERT(cb != NULL, "Callback must not be NULL");

    return baro->ops->read_async(baro, cb, user_data);
}
