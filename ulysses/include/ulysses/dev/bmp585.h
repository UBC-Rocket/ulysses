#ifndef ULYSSES_DEV_BMP585
#define ULYSSES_DEV_BMP585

#include "ulysses/io/i2c.h"
#include "ulysses/io/status.h"

/** @brief One pressure and temperature measurement */
struct dev_bmp585_sample {
    float pressure_pa;
    float temp_c;
};

/**
 * @brief Resets the BMP585, checks its ID and NVM, and starts sampling at 50 Hz
 *
 * Blocks for about 10 ms.
 *
 * @param i2c Device handle from the board wiring
 *
 * @return Transfer status, or IO_STATUS_ERROR if a check fails
 */
enum io_status dev_bmp585_init(const io_i2c_dev_t *i2c);

/**
 * @brief Reads the latest measurement
 *
 * @param i2c Device handle passed to dev_bmp585_init()
 * @param[out] out Non-NULL sample to fill
 *
 * @return Transfer status
 */
enum io_status dev_bmp585_read(const io_i2c_dev_t *i2c, struct dev_bmp585_sample *out);

#endif // ULYSSES_DEV_BMP585
