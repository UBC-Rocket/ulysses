#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dev/gnss/gnss_priv.h"
#include "ulysses/lib/assert.h"
#include "ulysses/svc/status.h"

enum svc_status svc_gnss_update(svc_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "SVC_GNSS_UPDATE: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "SVC_GNSS_UPDATE: gnss->ops is null");
    ULYSSES_ASSERT(gnss->ops->update != NULL, "SVC_GNSS_UPDATE: gnss->ops->update is null");

    return gnss->ops->update(gnss->config, gnss->ctx);
}

bool svc_gnss_is_data_ready(const svc_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "SVC_GNSS_IS_DATA_READY: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "SVC_GNSS_IS_DATA_READY: gnss->ops is null");
    ULYSSES_ASSERT(gnss->ops->is_data_ready != NULL,
                   "SVC_GNSS_IS_DATA_READY: gnss->ops->is_data_ready is null");

    return gnss->ops->is_data_ready(gnss->ctx);
}

enum svc_status svc_gnss_get_data(svc_gnss_t *gnss, struct svc_gnss_data *data)
{
    ULYSSES_ASSERT(gnss != NULL, "SVC_GNSS_GET_DATA: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "SVC_GNSS_GET_DATA: gnss->ops is null");
    ULYSSES_ASSERT(gnss->ops->get_data != NULL, "SVC_GNSS_GET_DATA: gnss->ops->get_data is null");
    ULYSSES_ASSERT(data != NULL, "SVC_GNSS_GET_DATA: data is null");

    return gnss->ops->get_data(gnss->ctx, data);
}

enum svc_status svc_gnss_send_corrections(svc_gnss_t *gnss, const uint8_t *data, size_t len)
{
    ULYSSES_ASSERT(gnss != NULL, "SVC_GNSS_SEND_CORRECTIONS: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "SVC_GNSS_SEND_CORRECTIONS: gnss->ops is null");
    ULYSSES_ASSERT(data != NULL, "SVC_GNSS_SEND_CORRECTIONS: data is null");
    ULYSSES_ASSERT(len > 0U, "SVC_GNSS_SEND_CORRECTIONS: len must be greater than 0");

    if (gnss->ops->send_corrections == NULL) {
        return SVC_STATUS_UNSUPPORTED;
    }

    return gnss->ops->send_corrections(gnss->config, gnss->ctx, data, len);
}

enum svc_status svc_gnss_reset(svc_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "SVC_GNSS_RESET: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "SVC_GNSS_RESET: gnss->ops is null");

    if (gnss->ops->reset == NULL) {
        return SVC_STATUS_UNSUPPORTED;
    }

    return gnss->ops->reset(gnss->config, gnss->ctx);
}
