#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ulysses/lib/assert.h"
#include "ulysses/io/delay.h"
#include "ulysses/io/spi.h"
#include "ulysses/svc/dev/mag.h"
#include "ulysses/svc/dev/mag_mmc5983.h"

#define REG_XOUT0 (0x00)
#define REG_STATUS (0x08)
#define REG_CTRL0 (0x09)
#define REG_CTRL1 (0x0A)
#define REG_CTRL2 (0x0B)
#define REG_PRODUCT_ID (0x2F)

#define STATUS_MEAS_M_DONE (0x01)
#define CTRL0_TM_M (0x01)
#define CTRL0_SET (0x08)
#define CTRL0_RESET (0x10)
#define CTRL0_AUTO_SR_EN (0x20)
#define CTRL1_BW_200HZ (0x01) // 4 ms per measurement
#define CTRL1_SW_RST (0x80)
#define CTRL2_CM_FREQ_100HZ (0x05)
#define CTRL2_CMM_EN (0x08)
#define CTRL2_PRD_SET_EVERY (0x00) // SET before every measurement
#define CTRL2_EN_PRD_SET (0x80)
#define PRODUCT_ID_VALUE (0x30)

#define COUNTS_PER_GAUSS (16384.0f)

#define SPI_READ_BIT (0x80)
#define SPI_TIMEOUT_MS (10)
#define SW_RST_TIME_MS (10)
#define SET_RESET_TIME_MS (1)
#define MEAS_POLL_INTERVAL_MS (1)
#define MEAS_POLL_COUNT (10)
#define RAW_LEN (7) // Xout0 to XYZout2

static enum io_status write_reg(io_spi_dev_t *spi, uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg, value};

    return io_spi_transmit(spi, tx_data, sizeof(tx_data), SPI_TIMEOUT_MS);
}

static enum io_status read_reg(io_spi_dev_t *spi, uint8_t reg, uint8_t *value)
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

static enum io_status read_raw(io_spi_dev_t *spi, int32_t raw[3])
{
    uint8_t tx_data[1 + RAW_LEN] = {SPI_READ_BIT | REG_XOUT0};
    uint8_t rx_data[1 + RAW_LEN];

    enum io_status status = io_spi_transfer(spi, tx_data, rx_data, sizeof(tx_data), SPI_TIMEOUT_MS);
    if (status != IO_STATUS_OK) {
        return status;
    }

    // 18-bit value per axis: 8 bits in out0, 8 bits in out1, 2 bits in XYZout2
    const uint8_t *out = &rx_data[1];
    raw[0] = ((int32_t)out[0] << 10) | ((int32_t)out[1] << 2) | ((out[6] >> 6) & 0x3);
    raw[1] = ((int32_t)out[2] << 10) | ((int32_t)out[3] << 2) | ((out[6] >> 4) & 0x3);
    raw[2] = ((int32_t)out[4] << 10) | ((int32_t)out[5] << 2) | ((out[6] >> 2) & 0x3);
    return IO_STATUS_OK;
}

static enum io_status measure(io_spi_dev_t *spi, uint8_t pulse, int32_t raw[3])
{
    enum io_status status = write_reg(spi, REG_CTRL0, pulse);
    if (status != IO_STATUS_OK) {
        return status;
    }
    io_delay_ms(SET_RESET_TIME_MS);

    status = write_reg(spi, REG_CTRL0, CTRL0_TM_M);
    if (status != IO_STATUS_OK) {
        return status;
    }

    for (uint32_t i = 0; i < MEAS_POLL_COUNT; i++) {
        io_delay_ms(MEAS_POLL_INTERVAL_MS);

        uint8_t device_status;
        status = read_reg(spi, REG_STATUS, &device_status);
        if (status != IO_STATUS_OK) {
            return status;
        }

        if (device_status & STATUS_MEAS_M_DONE) {
            return read_raw(spi, raw);
        }
    }

    return IO_STATUS_TIMEOUT;
}

static enum io_status mmc5983_read(struct dev_mag *mag, struct dev_mag_sample *sample)
{
    const struct dev_mag_mmc5983_config *config = mag->config;
    const struct dev_mag_mmc5983_data *data = mag->data;

    int32_t raw[3];
    enum io_status status = read_raw(config->spi, raw);
    if (status != IO_STATUS_OK) {
        return status;
    }

    sample->x = (float)(raw[0] - data->offset[0]) / COUNTS_PER_GAUSS;
    sample->y = (float)(raw[1] - data->offset[1]) / COUNTS_PER_GAUSS;
    sample->z = (float)(raw[2] - data->offset[2]) / COUNTS_PER_GAUSS;
    return IO_STATUS_OK;
}

static bool mmc5983_is_data_ready(struct dev_mag *mag)
{
    const struct dev_mag_mmc5983_config *config = mag->config;

    uint8_t device_status;
    if (read_reg(config->spi, REG_STATUS, &device_status) != IO_STATUS_OK) {
        return false;
    }

    return (device_status & STATUS_MEAS_M_DONE) != 0;
}

enum io_status dev_mag_mmc5983_init(struct dev_mag *mag,
                                    const struct dev_mag_mmc5983_config *config,
                                    struct dev_mag_mmc5983_data *data)
{
    ULYSSES_ASSERT(mag != NULL, "Magnetometer must be non NULL");
    ULYSSES_ASSERT(config != NULL, "MMC5983 config must be non NULL");
    ULYSSES_ASSERT(config->spi != NULL, "MMC5983 SPI device must be non NULL");
    ULYSSES_ASSERT(data != NULL, "MMC5983 data must be non NULL");

    mag->data = data;
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

    status = write_reg(config->spi, REG_CTRL1, CTRL1_BW_200HZ);
    if (status != IO_STATUS_OK) {
        return status;
    }

    // RESET first and SET last, so the sensor stays in the SET state afterwards
    int32_t reset_raw[3];
    status = measure(config->spi, CTRL0_RESET, reset_raw);
    if (status != IO_STATUS_OK) {
        return status;
    }

    int32_t set_raw[3];
    status = measure(config->spi, CTRL0_SET, set_raw);
    if (status != IO_STATUS_OK) {
        return status;
    }

    data->offset[0] = (set_raw[0] + reset_raw[0]) / 2;
    data->offset[1] = (set_raw[1] + reset_raw[1]) / 2;
    data->offset[2] = (set_raw[2] + reset_raw[2]) / 2;

    status = write_reg(config->spi, REG_CTRL0, CTRL0_AUTO_SR_EN);
    if (status != IO_STATUS_OK) {
        return status;
    }

    status = write_reg(config->spi, REG_CTRL2,
                       CTRL2_EN_PRD_SET | CTRL2_PRD_SET_EVERY | CTRL2_CMM_EN | CTRL2_CM_FREQ_100HZ);
    if (status != IO_STATUS_OK) {
        return status;
    }

    mag->read = mmc5983_read;
    mag->is_data_ready = mmc5983_is_data_ready;
    return IO_STATUS_OK;
}
