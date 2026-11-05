#include <stdbool.h>
#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/mag.h"

enum io_status dev_mag_read(struct dev_mag *mag, struct dev_mag_sample *sample)
{
    ULYSSES_ASSERT(mag != NULL, "Magnetometer must be non NULL");
    ULYSSES_ASSERT(mag->read != NULL, "Magnetometer must be initialized");
    ULYSSES_ASSERT(sample != NULL, "Magnetometer sample must be non NULL");

    return mag->read(mag, sample);
}

bool dev_mag_is_data_ready(struct dev_mag *mag)
{
    ULYSSES_ASSERT(mag != NULL, "Magnetometer must be non NULL");
    ULYSSES_ASSERT(mag->is_data_ready != NULL, "Magnetometer must be initialized");

    return mag->is_data_ready(mag);
}
