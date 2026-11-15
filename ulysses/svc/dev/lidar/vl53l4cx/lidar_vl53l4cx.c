#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/compiler.h"
#include "ulysses/io/i2c.h"
#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/lidar.h"

#include "dev/lidar/lidar.h"
#include "dev/lidar/vl53l4cx/lidar_vl53l4cx.h"

// VL53LX_RANGESTATUS_RANGE_VALID in ST's driver. Every other status is a failure or a range that
// skipped one of the driver's checks.
#define VL53L4CX_RANGE_STATUS_VALID (0U)

// Maximum targets per measurement (VL53LX_MAX_RANGE_RESULTS in ST's driver)
#define VL53L4CX_MAX_TARGETS (4U)

/** @brief Fields from ST's VL53LX_TargetRangeData_t */
struct vl53l4cx_target {
    int16_t range_mm;     ///< Distance to target in millimetres
    uint8_t range_status; ///< VL53LX_RANGESTATUS_* code for this target
};

/** @brief Fields from ST's VL53LX_MultiRangingData_t */
struct vl53l4cx_result {
    uint8_t target_count; ///< Number of filled entries in targets
    struct vl53l4cx_target targets[VL53L4CX_MAX_TARGETS];
};

/** @brief Stub until ST's VL53L4CX driver and 16-bit I2C register addresses are added */
static enum io_status fetch_result(ULYSSES_UNUSED const io_i2c_dev_t *i2c,
                                   ULYSSES_UNUSED struct vl53l4cx_result *result)
{
    return IO_STATUS_ERROR;
}

static enum io_status vl53l4cx_read(const dev_lidar_t *lidar, struct dev_lidar_sample *sample);

static const struct dev_lidar_ops vl53l4cx_ops = {
    .read = vl53l4cx_read,
};

enum io_status dev_lidar_vl53l4cx_init(dev_lidar_t *lidar,
                                       const struct dev_lidar_vl53l4cx_config *config)
{
    ULYSSES_ASSERT(lidar != NULL, "Lidar must not be NULL");
    ULYSSES_ASSERT(config != NULL, "VL53L4CX config must not be NULL");

    lidar->config = config;
    lidar->ops = &vl53l4cx_ops;
    return IO_STATUS_OK;
}

static enum io_status vl53l4cx_read(const dev_lidar_t *lidar, struct dev_lidar_sample *sample)
{
    const struct dev_lidar_vl53l4cx_config *config = lidar->config;

    struct vl53l4cx_result result = {0};
    enum io_status status = fetch_result(config->i2c, &result);
    if (status != IO_STATUS_OK) {
        return status;
    }

    // Report the nearest valid target
    sample->distance_m = 0.0f;
    sample->is_valid = false;

    for (uint8_t i = 0; i < result.target_count; i++) {
        const struct vl53l4cx_target *target = &result.targets[i];

        // Offset calibration can push a valid range for a very close target below zero
        if (target->range_status != VL53L4CX_RANGE_STATUS_VALID || target->range_mm < 0) {
            continue;
        }

        float distance_m = (float)target->range_mm / 1000.0f;
        if (!sample->is_valid || distance_m < sample->distance_m) {
            sample->distance_m = distance_m;
            sample->is_valid = true;
        }
    }

    return IO_STATUS_OK;
}
