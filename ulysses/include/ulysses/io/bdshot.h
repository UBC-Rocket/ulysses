#ifndef ULYSSES_IO_BDSHOT
#define ULYSSES_IO_BDSHOT

#include <stdint.h>
#include <stdbool.h>

#include "ulysses/io/status.h"

typedef struct io_bdshot io_bdshot_t;

/**
 * @brief Response for a transceive transaction
 */
struct io_bdshot_response {
    enum io_status status; ///< Status of the transaction
    uint32_t wire_frame;   ///< Wire response received if status is ok
};

/**
 * @brief Sets the frame transmitted to a channel
 *
 * After setting, the frame is reused for all future transceives.
 *
 * Caller must ensure that no transceives are running before calling this.
 *
 * @param bdshot BDShot IO instance
 * @param channel Channel to transceive over
 * @param frame Frame to transmit
 */
void io_bdshot_set_frame(const io_bdshot_t *bdshot, uint8_t channel, uint16_t frame);

/**
 * @brief Sets the location to store reponses for a channel
 *
 * After setting, the location of the response is reused for all future
 * transceives. As such, it must remain valid for the duration of its usage.
 *
 * Responses are only valid after stopping the current transceive.
 *
 * Caller must ensure that no transceives are running before calling this.
 *
 * @param bdshot BDShot IO instance
 * @param channel Channel to transceive over
 * @param[out] response Pointer to the location to store the response
 */
void io_bdshot_set_response(const io_bdshot_t *bdshot, uint8_t channel,
                            struct io_bdshot_response *response);

/**
 * @brief Configures whether the channel is active for DShot transactions
 *
 * Caller must ensure that no transceives are running before calling this.
 *
 * @param bdshot BDShot IO instance
 * @param channel Channel to configure
 * @param is_active Whether the channel will be active
 *
 * @return Status of the set active operation
 */
enum io_status io_bdshot_set_active(const io_bdshot_t *bdshot, uint8_t channel, bool is_active);

/**
 * @brief Starts a Bidirectional DShot transmit and receive transaction
 *
 * @param bdshot BDShot IO instance
 *
 * @return Status of the start operation
 */
enum io_status io_bdshot_transceive_start(const io_bdshot_t *bdshot);

/**
 * @brief Completes a Bidirectional DShot transmit and receive transaction
 *
 * Before stopping the transaction, you must ensure there was sufficient time
 * at the configured BDShot speed for the frame to be sent and response to be
 * received.
 *
 * @param bdshot BDShot IO instance
 *
 * @return Status of the stop operation
 */
enum io_status io_bdshot_transceive_stop(const io_bdshot_t *bdshot);

#endif // ULYSSES_IO_BDSHOT
