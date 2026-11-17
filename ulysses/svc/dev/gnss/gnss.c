#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dev/gnss/gnss.h"
#include "ulysses/lib/assert.h"
#include "ulysses/io/status.h"

enum io_status dev_gnss_update(dev_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "GNSS must not be null");
    ULYSSES_ASSERT(gnss->ops != NULL, "GNSS must be intialized");
    ULYSSES_ASSERT(gnss->ops->update != NULL, "GNSS update func must not be null");

    return gnss->ops->update(gnss->config, gnss->data);
}

bool dev_gnss_is_sample_ready(const dev_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "GNSS must not be null");
    ULYSSES_ASSERT(gnss->ops != NULL, "GNSS must be intialized");
    ULYSSES_ASSERT(gnss->ops->is_sample_ready != NULL, "GNSS is_sample_ready func must not be null");

    return gnss->ops->is_sample_ready(gnss->data);
}

enum io_status dev_gnss_get_sample(dev_gnss_t *gnss, struct dev_gnss_sample *sample)
{
    ULYSSES_ASSERT(gnss != NULL, "GNSS must not be null");
    ULYSSES_ASSERT(gnss->ops != NULL, "GNSS must be intialized");
    ULYSSES_ASSERT(gnss->ops->get_sample != NULL, "GNSS get_sample func must not be null");
    ULYSSES_ASSERT(sample != NULL, "Sample must not be null");

    return gnss->ops->get_sample(gnss->data, sample);
}

enum io_status dev_gnss_send_corrections(dev_gnss_t *gnss, const uint8_t *corrections, size_t len)
{
    ULYSSES_ASSERT(gnss != NULL, "GNSS must not be null");
    ULYSSES_ASSERT(gnss->ops != NULL, "GNSS must be intialized");
    ULYSSES_ASSERT(corrections != NULL, "Corrections must not be null");
    ULYSSES_ASSERT(len > 0U, "len must be greater than 0");

    if (gnss->ops->send_corrections == NULL) {
        return IO_STATUS_ERROR;
    }

    return gnss->ops->send_corrections(gnss->config, gnss->data, corrections, len);
}

enum io_status dev_gnss_reset(dev_gnss_t *gnss)
{
    ULYSSES_ASSERT(gnss != NULL, "GNSS must not be null");
    ULYSSES_ASSERT(gnss->ops != NULL, "GNSS must be intialized");

    if (gnss->ops->reset == NULL) {
        return IO_STATUS_ERROR;
    }

    return gnss->ops->reset(gnss->config, gnss->data);
}
