#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dev/gnss/gnss.h"
#include "ulysses/lib/assert.h"
#include "ulysses/io/status.h"

enum io_status dev_gnss_update(dev_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "DEV_GNSS_UPDATE: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "DEV_GNSS_UPDATE: gnss->ops is null");
    ULYSSES_ASSERT(gnss->ops->update != NULL, "DEV_GNSS_UPDATE: gnss->ops->update is null");

    return gnss->ops->update(gnss->config, gnss->ctx);
}

bool dev_gnss_is_data_ready(const dev_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "DEV_GNSS_IS_DATA_READY: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "DEV_GNSS_IS_DATA_READY: gnss->ops is null");
    ULYSSES_ASSERT(gnss->ops->is_data_ready != NULL,
                   "DEV_GNSS_IS_DATA_READY: gnss->ops->is_data_ready is null");

    return gnss->ops->is_data_ready(gnss->ctx);
}

enum io_status dev_gnss_get_data(dev_gnss_t *gnss, struct dev_gnss_data *data)
{
    ULYSSES_ASSERT(gnss != NULL, "DEV_GNSS_GET_DATA: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "DEV_GNSS_GET_DATA: gnss->ops is null");
    ULYSSES_ASSERT(gnss->ops->get_data != NULL, "DEV_GNSS_GET_DATA: gnss->ops->get_data is null");
    ULYSSES_ASSERT(data != NULL, "DEV_GNSS_GET_DATA: data is null");

    return gnss->ops->get_data(gnss->ctx, data);
}

enum io_status dev_gnss_send_corrections(dev_gnss_t *gnss, const uint8_t *data, size_t len)
{
    ULYSSES_ASSERT(gnss != NULL, "DEV_GNSS_SEND_CORRECTIONS: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "DEV_GNSS_SEND_CORRECTIONS: gnss->ops is null");
    ULYSSES_ASSERT(data != NULL, "DEV_GNSS_SEND_CORRECTIONS: data is null");
    ULYSSES_ASSERT(len > 0U, "DEV_GNSS_SEND_CORRECTIONS: len must be greater than 0");

    if (gnss->ops->send_corrections == NULL) {
        return IO_STATUS_ERROR;
    }

    return gnss->ops->send_corrections(gnss->config, gnss->ctx, data, len);
}

enum io_status dev_gnss_reset(dev_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "DEV_GNSS_RESET: gnss is null");
    ULYSSES_ASSERT(gnss->ops != NULL, "DEV_GNSS_RESET: gnss->ops is null");

    if (gnss->ops->reset == NULL) {
        return IO_STATUS_ERROR;
    }

    return gnss->ops->reset(gnss->config, gnss->ctx);
}
