#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/delay.h"
#include "ulysses/io/spi.h"
#include "ulysses/svc/dev/mag.h"
#include "ulysses/svc/dev/mag_mmc5983.h"

#define REG_CTRL1 (0x0A)
#define REG_PRODUCT_ID (0x2F)

#define CTRL1_BW_200HZ (0x01) // 4 ms per measurement
#define CTRL1_SW_RST (0x80)
#define PRODUCT_ID_VALUE (0x30)

#define SPI_READ_BIT (0x80)
#define SPI_TIMEOUT_MS (10)
#define SW_RST_TIME_MS (10)

static enum io_status write_reg(io_spi_device_t *spi, uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg, value};

    return io_spi_transmit(spi, tx_data, sizeof(tx_data), SPI_TIMEOUT_MS);
}

static enum io_status read_reg(io_spi_device_t *spi, uint8_t reg, uint8_t *value)
{
    uint8_t tx_data[2] = {SPI_READ_BIT | reg, 0};
    uint8_t rx_data[2];

    enum io_status status = io_spi_transfer(spi, tx_data, rx_data, sizeof(tx_data), SPI_TIMEOUT_MS);
    if (status != IO_STATUS_OK) {
        return status;
    }

    *value = rx_data[1];
    return IO_STATUS_OK;
}

enum io_status dev_mag_mmc5983_init(struct dev_mag *mag,
                                    const struct dev_mag_mmc5983_config *config)
{
    ULYSSES_ASSERT(mag != NULL, "Magnetometer must be non NULL");
    ULYSSES_ASSERT(config != NULL, "MMC5983 config must be non NULL");
    ULYSSES_ASSERT(config->spi != NULL, "MMC5983 SPI device must be non NULL");

    mag->data = NULL;
    mag->config = config;
    mag->read = NULL;
    mag->is_data_ready = NULL;

    enum io_status status = write_reg(config->spi, REG_CTRL1, CTRL1_SW_RST);
    if (status != IO_STATUS_OK) {
        return status;
    }
    io_delay_ms(SW_RST_TIME_MS);

    uint8_t product_id;
    status = read_reg(config->spi, REG_PRODUCT_ID, &product_id);
    if (status != IO_STATUS_OK) {
        return status;
    }

    if (product_id != PRODUCT_ID_VALUE) {
        return IO_STATUS_ERROR;
    }

    return write_reg(config->spi, REG_CTRL1, CTRL1_BW_200HZ);
}
