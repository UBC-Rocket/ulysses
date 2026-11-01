#ifndef ULYSSES_SVC_GNSS_GNSS_PRIV
#define ULYSSES_SVC_GNSS_GNSS_PRIV

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/svc/status.h"
#include "ulysses/svc/gnss.h"

struct svc_gnss_ops {
    enum svc_status (*update)(const void *config, void *ctx);
    bool (*is_data_ready)(const void *ctx);
    enum svc_status (*get_data)(void *ctx, struct svc_gnss_data *data);
    enum svc_status (*send_corrections)(const void *config, void *ctx, const uint8_t *data,
                                        size_t len);
    enum svc_status (*reset)(const void *config, void *ctx);
};

struct svc_gnss {
    const void *config;
    void *ctx;
    const struct svc_gnss_ops *ops;
};

#endif // ULYSSES_SVC_GNSS_GNSS_PRIV
