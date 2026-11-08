#ifndef ULYSSES_SVC_DEV_MAG_PRIVATE
#define ULYSSES_SVC_DEV_MAG_PRIVATE

#include <stdbool.h>

#include "ulysses/io/status.h"
#include "ulysses/svc/dev/mag.h"

// Sensor-specific functions, see the matching dev_mag_*() functions
typedef enum io_status (*dev_mag_read_fn_t)(dev_mag_t *mag, struct dev_mag_sample *sample);
typedef bool (*dev_mag_is_data_ready_fn_t)(dev_mag_t *mag);

struct dev_mag_ops {
    dev_mag_read_fn_t read;                   ///< Sensor-specific read function
    dev_mag_is_data_ready_fn_t is_data_ready; ///< Sensor-specific data ready function
};

struct dev_mag {
    const void *config;            ///< Sensor-specific configuration
    void *data;                    ///< Sensor-specific state
    const struct dev_mag_ops *ops; ///< Sensor-specific functions
};

#endif // ULYSSES_SVC_DEV_MAG_PRIVATE
