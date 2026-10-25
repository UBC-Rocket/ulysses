#ifndef ULYSSES_SVC_DEV_MAG_MMC5983
#define ULYSSES_SVC_DEV_MAG_MMC5983

#include <stdint.h>

#include "ulysses/io/spi.h"
#include "ulysses/io/status.h"
#include "ulysses/svc/dev/mag.h"

/** @brief Wiring of an MMC5983MA magnetometer */
struct dev_mag_mmc5983_config {
    io_spi_device_t *spi; ///< SPI device the sensor is connected to
};

/** @brief State of an MMC5983MA magnetometer, filled in by dev_mag_mmc5983_init() */
struct dev_mag_mmc5983_data {
    int32_t offset[3]; ///< Raw X, Y and Z output at zero field, measured at init
};

/**
 * @brief Initializes an MMC5983MA magnetometer
 *
 * @param[out] mag Magnetometer to initialize
 * @param config Wiring of the sensor, must stay valid while the magnetometer is used
 * @param[out] data State of the sensor, must stay valid while the magnetometer is used
 *
 * @return IO_STATUS_OK on success, IO_STATUS_ERROR if the sensor is not an MMC5983MA, otherwise an
 *         IO_STATUS_* error code
 */
enum io_status dev_mag_mmc5983_init(struct dev_mag *mag,
                                    const struct dev_mag_mmc5983_config *config,
                                    struct dev_mag_mmc5983_data *data);

#endif // ULYSSES_SVC_DEV_MAG_MMC5983
