#include <stddef.h>

#include "dev/lidar/lidar.h"

#include "ulysses/lib/assert.h"

enum io_status dev_lidar_read(const dev_lidar_t *lidar, struct dev_lidar_sample *sample)
{
    ULYSSES_ASSERT(lidar != NULL, "Lidar pointer is NULL");
    ULYSSES_ASSERT(lidar->ops != NULL, "Lidar operations are NULL");
    ULYSSES_ASSERT(lidar->ops->read != NULL, "Lidar read operation is NULL");
    ULYSSES_ASSERT(sample != NULL, "Sample pointer is NULL");

    return lidar->ops->read(lidar, sample);
}
