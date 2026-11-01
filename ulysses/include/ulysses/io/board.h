#ifndef ULYSSES_IO_BOARD
#define ULYSSES_IO_BOARD

#include "ulysses/io/i2c.h"

// Each core only defines the devices wired to it, so using one from the other core fails to link.

/** @brief Barometer (CM7) */
extern const io_i2c_dev_t IO_BARO;

/** @brief LiDAR (CM7) */
extern const io_i2c_dev_t IO_LIDAR;

/** @brief Fuel gauge (CM4) */
extern const io_i2c_dev_t IO_FUEL_GAUGE;

/** @brief Battery management system (CM4) */
extern const io_i2c_dev_t IO_BMS;

/**
 * @brief Links this core's buses to their CubeMX handles
 *
 * Call once after the CubeMX MX_*_Init() calls and before any device is used.
 */
void io_board_init(void);

#endif // ULYSSES_IO_BOARD
