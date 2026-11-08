#include <stdbool.h>
#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/mag.h"

#include "dev/mag/mag.h"

enum io_status dev_mag_read(dev_mag_t *mag, struct dev_mag_sample *sample)
{
    ULYSSES_ASSERT(mag != NULL, "Magnetometer must be non NULL");
    ULYSSES_ASSERT(mag->ops != NULL, "Magnetometer must be initialized");
    ULYSSES_ASSERT(mag->ops->read != NULL, "Magnetometer read function must be non NULL");
    ULYSSES_ASSERT(sample != NULL, "Magnetometer sample must be non NULL");

    return mag->ops->read(mag, sample);
}

bool dev_mag_is_data_ready(dev_mag_t *mag)
{
    ULYSSES_ASSERT(mag != NULL, "Magnetometer must be non NULL");
    ULYSSES_ASSERT(mag->ops != NULL, "Magnetometer must be initialized");
    ULYSSES_ASSERT(mag->ops->is_data_ready != NULL,
                   "Magnetometer data ready function must be non NULL");

    return mag->ops->is_data_ready(mag);
}
