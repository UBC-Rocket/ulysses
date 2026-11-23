#ifndef ULYSSES_IO_BDSHOT_PRIVATE
#define ULYSSES_IO_BDSHOT_PRIVATE

#include "ulysses/status.h"
#include "ulysses/io/bdshot.h"

typedef void (*io_bdshot_set_frame_t)(const io_bdshot_t *bdshot, uint8_t channel, uint16_t frame);

typedef void (*io_bdshot_set_response_t)(const io_bdshot_t *bdshot, uint8_t channel,
                                         struct io_bdshot_response *response);

typedef enum uls_status (*io_bdshot_set_active_t)(const io_bdshot_t *bdshot, uint8_t channel,
                                                  bool is_active);

typedef enum uls_status (*io_bdshot_transceive_start_t)(const io_bdshot_t *bdshot);

typedef enum uls_status (*io_bdshot_transceive_stop_t)(const io_bdshot_t *bdshot);

struct io_bdshot_ops {
    io_bdshot_set_frame_t set_frame;
    io_bdshot_set_response_t set_response;
    io_bdshot_set_active_t set_active;
    io_bdshot_transceive_start_t transceive_start;
    io_bdshot_transceive_stop_t transceive_stop;
};

struct io_bdshot {
    void *data;
    const void *config;
    const struct io_bdshot_ops *ops;
};

#endif // ULYSSES_IO_BDSHOT_PRIVATE
