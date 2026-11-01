#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dev/lidar/lidar.h"
#include "dev/lidar/vl53l4cx/lidar_vl53l4cx.h"

#include "ulysses/compiler.h"
#include "ulysses/lib/assert.h"

// VL53LX_RANGESTATUS_RANGE_VALID in ST's driver. Every other status is a failure or a range that
// skipped one of the driver's checks.
#define VL53L4CX_RANGE_STATUS_VALID (0U)

// Maximum targets reported per measurement from ST's driver (VL53LX_MAX_RANGE_RESULTS)
#define VL53L4CX_MAX_TARGETS (4U)

#define MM_PER_M (1000.0f)

/** @brief Fields from ST's VL53LX_TargetRangeData_t */
struct vl53l4cx_target {
    int16_t range_mm;     ///< Distance to target in millimetres
    uint8_t range_status; ///< VL53LX_RANGESTATUS_* code for this target
};

/** @brief Fields from ST's VL53LX_MultiRangingData_t */
struct vl53l4cx_result {
    uint8_t target_count;                                 ///< Number of filled entries in targets
    struct vl53l4cx_target targets[VL53L4CX_MAX_TARGETS]; ///< Targets detected in the measurement
};

/**
 * @brief Reads the latest ranging result from the sensor
 *
 * Stub until we add ST's VL53L4CX driver
 * Also need 16-bit register addresses in io_i2c
 */
static enum io_status fetch_result(ULYSSES_UNUSED const struct dev_lidar_vl53l4cx_config *config,
                                   ULYSSES_UNUSED struct vl53l4cx_result *result)
{
    return IO_STATUS_ERROR;
}

/** @brief Converts a ranging result into a sample using the nearest valid target */
static void parse_result(const struct vl53l4cx_result *result, struct dev_lidar_sample *sample)
{
    ULYSSES_ASSERT(result->target_count <= VL53L4CX_MAX_TARGETS, "Too many LiDAR targets");

    sample->distance_m = 0.0f;
    sample->is_valid = false;

    for (uint8_t i = 0; i < result->target_count; i++) {
        const struct vl53l4cx_target *target = &result->targets[i];

        // Offset calibration can push a valid range for a very close target below zero
        if (target->range_status != VL53L4CX_RANGE_STATUS_VALID || target->range_mm < 0) {
            continue;
        }

        float distance_m = (float)target->range_mm / MM_PER_M;

        if (!sample->is_valid || distance_m < sample->distance_m) {
            sample->distance_m = distance_m;
            sample->is_valid = true;
        }
    }
}

/** @brief Stub */
static enum io_status vl53l4cx_is_data_ready(ULYSSES_UNUSED dev_lidar_t *lidar, bool *is_ready)
{
    *is_ready = false;

    return IO_STATUS_ERROR;
}

static enum io_status vl53l4cx_read(dev_lidar_t *lidar, struct dev_lidar_sample *sample)
{
    const struct dev_lidar_vl53l4cx_config *cfg =
        (const struct dev_lidar_vl53l4cx_config *)lidar->config;

    struct vl53l4cx_result result = {0};
    enum io_status status = fetch_result(cfg, &result);
    if (status != IO_STATUS_OK) {
        return status;
    }

    parse_result(&result, sample);

    return IO_STATUS_OK;
}

static const struct dev_lidar_ops vl53l4cx_ops = {.is_data_ready = vl53l4cx_is_data_ready,
                                                  .read = vl53l4cx_read};

void dev_lidar_vl53l4cx_init(dev_lidar_t *lidar, const struct dev_lidar_vl53l4cx_config *config)
{
    ULYSSES_ASSERT(lidar != NULL, "LiDAR pointer is NULL");
    ULYSSES_ASSERT(config != NULL, "LiDAR config pointer is NULL");
    ULYSSES_ASSERT(config->i2c != NULL, "LiDAR I2C device pointer is NULL");

    // ST's driver state goes here when we add it
    lidar->data = NULL;
    lidar->config = config;
    lidar->ops = &vl53l4cx_ops;
}
