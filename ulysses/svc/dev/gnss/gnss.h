#ifndef ULYSSES_SVC_GNSS_GNSS_PRIV
#define ULYSSES_SVC_GNSS_GNSS_PRIV

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/status.h"
#include "ulysses/svc/dev/gnss.h"

struct dev_gnss_ops {
    enum uls_status (*update)(const void *config, void *data);
    bool (*is_sample_ready)(const void *data);
    enum uls_status (*get_sample)(void *data, struct dev_gnss_sample *sample);
    enum uls_status (*send_corrections)(const void *config, void *data, const uint8_t *corrections,
                                        size_t len);
    enum uls_status (*reset)(const void *config, void *data);
};

struct dev_gnss {
    const void *config;
    void *data;
    const struct dev_gnss_ops *ops;
};

#endif // ULYSSES_SVC_GNSS_GNSS_PRIV
