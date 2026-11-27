#include <stddef.h>

#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/lidar.h"

#include "dev/lidar/lidar.h"

enum io_status dev_lidar_read(const dev_lidar_t *lidar, struct dev_lidar_sample *sample)
{
    ULYSSES_ASSERT(lidar != NULL, "Lidar must not be NULL");
    ULYSSES_ASSERT(lidar->ops != NULL, "Lidar must be initialized");
    ULYSSES_ASSERT(sample != NULL, "Sample must not be NULL");

    return lidar->ops->read(lidar, sample);
}
