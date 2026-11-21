#include <stddef.h>

#include "bdshot/bdshot.h"

#include "ulysses/lib/assert.h"

void io_bdshot_set_frame(const io_bdshot_t *bdshot, uint8_t channel, uint16_t frame)
{
    ULYSSES_ASSERT(bdshot != NULL, "Instance must be non NULL");

    bdshot->ops->set_frame(bdshot, channel, frame);
}

void io_bdshot_set_response(const io_bdshot_t *bdshot, uint8_t channel,
                            struct io_bdshot_response *response)
{
    ULYSSES_ASSERT(bdshot != NULL, "Instance must be non NULL");
    ULYSSES_ASSERT(response != NULL, "Response must be non NULL");

    bdshot->ops->set_response(bdshot, channel, response);
}

enum io_status io_bdshot_set_active(const io_bdshot_t *bdshot, uint8_t channel, bool is_active)
{
    ULYSSES_ASSERT(bdshot != NULL, "Instance must be non NULL");

    return bdshot->ops->set_active(bdshot, channel, is_active);
}

enum io_status io_bdshot_transceive_start(const io_bdshot_t *bdshot)
{
    ULYSSES_ASSERT(bdshot != NULL, "Instance must be non NULL");

    return bdshot->ops->transceive_start(bdshot);
}

enum io_status io_bdshot_transceive_stop(const io_bdshot_t *bdshot)
{
    ULYSSES_ASSERT(bdshot != NULL, "Instance must be non NULL");

    return bdshot->ops->transceive_stop(bdshot);
}
