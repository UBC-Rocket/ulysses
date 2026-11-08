#ifndef ULYSSES_SVC_DEV_BARO_BARO_H
#define ULYSSES_SVC_DEV_BARO_BARO_H

#include "ulysses/io/status.h"
#include "ulysses/svc/dev/baro.h"

struct dev_baro_ops {
    enum io_status (*read)(const dev_baro_t *baro, struct dev_baro_sample *sample);
};

struct dev_baro {
    const void *config;             ///< Sensor-specific configuration
    const struct dev_baro_ops *ops; ///< Sensor-specific functions, NULL until initialized
};

#endif // ULYSSES_SVC_DEV_BARO_BARO_H
