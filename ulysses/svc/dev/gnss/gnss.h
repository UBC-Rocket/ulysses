#ifndef ULYSSES_SVC_GNSS_GNSS_PRIV
#define ULYSSES_SVC_GNSS_GNSS_PRIV

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/io/status.h"
#include "ulysses/svc/dev/gnss.h"

struct dev_gnss_ops {
    enum io_status (*update)(const void *config, void *ctx);
    bool (*is_data_ready)(const void *ctx);
    enum io_status (*get_data)(void *ctx, struct dev_gnss_data *data);
    enum io_status (*send_corrections)(const void *config, void *ctx, const uint8_t *data,
                                       size_t len);
    enum io_status (*reset)(const void *config, void *ctx);
};

struct dev_gnss {
    const void *config;
    void *ctx;
    const struct dev_gnss_ops *ops;
};

#endif // ULYSSES_SVC_GNSS_GNSS_PRIV
