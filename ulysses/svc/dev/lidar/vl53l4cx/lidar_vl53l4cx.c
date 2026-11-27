#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/io/delay.h"
#include "ulysses/io/gpio.h"
#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/lidar.h"

#include "dev/lidar/lidar.h"
#include "dev/lidar/vl53l4cx/lidar_vl53l4cx.h"

// Time from the enable pin going high until the sensor answers on I2C (tBOOT is 1.2 ms max)
#define VL53L4CX_BOOT_MS (2U)

// Long range mode with a 33 ms timing budget, about 30 measurements a second
#define VL53L4CX_RANGING_PROFILE (VL53L4CX_PROFILE_LONG)
#define VL53L4CX_TIMING_BUDGET_MS (33U)

// VL53LX_RANGESTATUS_RANGE_VALID in ST's driver. Every other status is a failure or a range that
// skipped one of the driver's checks.
#define VL53L4CX_RANGE_STATUS_VALID (0U)

static enum io_status vl53l4cx_read(const dev_lidar_t *lidar, struct dev_lidar_sample *sample);

static const struct dev_lidar_ops vl53l4cx_ops = {
    .read = vl53l4cx_read,
};

/** @brief Maps a status from ST's driver to an I/O status */
static enum io_status to_io_status(int32_t ret)
{
    if (ret == VL53L4CX_OK) {
        return IO_STATUS_OK;
    }

    if (ret == VL53L4CX_TIMEOUT) {
        return IO_STATUS_TIMEOUT;
    }

    return IO_STATUS_ERROR;
}

enum io_status dev_lidar_vl53l4cx_init(dev_lidar_t *lidar,
                                       const struct dev_lidar_vl53l4cx_config *config,
                                       struct dev_lidar_vl53l4cx_data *data)
{
    ULYSSES_ASSERT(lidar != NULL, "Lidar must not be NULL");
    ULYSSES_ASSERT(config != NULL, "VL53L4CX config must not be NULL");
    ULYSSES_ASSERT(config->bus != NULL, "VL53L4CX bus must not be NULL");
    ULYSSES_ASSERT(config->enable != NULL, "VL53L4CX enable pin must not be NULL");
    ULYSSES_ASSERT(data != NULL, "VL53L4CX data must not be NULL");

    VL53L4CX_Object_t *sensor = &data->sensor;
    int32_t ret;

    lidar->config = config;
    lidar->data = data;
    lidar->ops = NULL;

    io_gpio_write(config->enable, IO_GPIO_STATE_SET);
    io_spin_delay_ms(VL53L4CX_BOOT_MS);

    // Also initializes the bus
    ret = VL53L4CX_RegisterBusIO(sensor, config->bus);
    if (ret != VL53L4CX_OK) {
        return to_io_status(ret);
    }

    ret = VL53L4CX_Init(sensor);
    if (ret != VL53L4CX_OK) {
        return to_io_status(ret);
    }

    uint32_t id;
    ret = VL53L4CX_ReadID(sensor, &id);
    if (ret != VL53L4CX_OK) {
        return to_io_status(ret);
    }
    if (id != VL53L4CX_ID) {
        return IO_STATUS_ERROR;
    }

    VL53L4CX_ProfileConfig_t profile = {
        .RangingProfile = VL53L4CX_RANGING_PROFILE,
        .TimingBudget = VL53L4CX_TIMING_BUDGET_MS,
    };
    ret = VL53L4CX_ConfigProfile(sensor, &profile);
    if (ret != VL53L4CX_OK) {
        return to_io_status(ret);
    }

    ret = VL53L4CX_Start(sensor, VL53L4CX_MODE_ASYNC_CONTINUOUS);
    if (ret != VL53L4CX_OK) {
        return to_io_status(ret);
    }

    lidar->ops = &vl53l4cx_ops;
    return IO_STATUS_OK;
}

static enum io_status vl53l4cx_read(const dev_lidar_t *lidar, struct dev_lidar_sample *sample)
{
    struct dev_lidar_vl53l4cx_data *data = lidar->data;

    sample->distance_m = 0.0f;
    sample->is_valid = false;

    VL53L4CX_Result_t result;
    int32_t ret = VL53L4CX_GetDistance(&data->sensor, &result);

    // In async mode, a timeout means no new measurement is ready yet
    if (ret == VL53L4CX_TIMEOUT) {
        return IO_STATUS_OK;
    }
    if (ret != VL53L4CX_OK) {
        return to_io_status(ret);
    }

    // Report the nearest valid target
    const VL53L4CX_TargetResult_t *zone = &result.ZoneResult[0];

    for (uint32_t i = 0; i < zone->NumberOfTargets; i++) {
        if (zone->Status[i] != VL53L4CX_RANGE_STATUS_VALID) {
            continue;
        }

        float distance_m = (float)zone->Distance[i] / 1000.0f;
        if (!sample->is_valid || distance_m < sample->distance_m) {
            sample->distance_m = distance_m;
            sample->is_valid = true;
        }
    }

    return IO_STATUS_OK;
}
