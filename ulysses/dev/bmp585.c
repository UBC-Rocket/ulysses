#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "ulysses/dev/bmp585.h"
#include "ulysses/lib/assert.h"

// Register map, section numbers refer to BST-BMP585-DS003
#define BMP585_REG_CHIP_ID (0x01U)
#define BMP585_REG_TEMP_DATA_XLSB (0x1DU)
#define BMP585_REG_INT_STATUS (0x27U)
#define BMP585_REG_STATUS (0x28U)
#define BMP585_REG_OSR_CONFIG (0x36U)
#define BMP585_REG_ODR_CONFIG (0x37U)
#define BMP585_REG_OSR_EFF (0x38U)
#define BMP585_REG_CMD (0x7EU)

#define BMP585_CHIP_ID (0x51U)
#define BMP585_CMD_SOFT_RESET (0xB6U)
#define BMP585_INT_STATUS_POR (0x10U)
#define BMP585_STATUS_NVM_RDY (1U << 1)
#define BMP585_STATUS_NVM_ERR (1U << 2)
#define BMP585_OSR_EFF_ODR_IS_VALID (1U << 7)

// Pressure enabled, pressure x16, temperature x1. Table 7 allows up to 80 Hz.
#define BMP585_OSR_CONFIG ((1U << 6) | (0x4U << 3) | 0x0U)
// Deep standby disabled, 50 Hz. Power mode goes in bits [1:0].
#define BMP585_ODR_CONFIG ((1U << 7) | (0x0FU << 2))
#define BMP585_PWR_MODE_STANDBY (0x0U)
#define BMP585_PWR_MODE_NORMAL (0x1U)

// Delays rounded up to whole milliseconds (section 2)
#define BMP585_SOFT_RESET_MS (2U)
#define BMP585_STANDBY_MS (3U)
#define BMP585_STARTUP_MS (3U)

#define BMP585_TIMEOUT_MS (10U)

static enum io_status read_reg(const io_i2c_dev_t *i2c, uint8_t reg, uint8_t *val)
{
    return io_i2c_mem_read(i2c, reg, val, 1U, BMP585_TIMEOUT_MS);
}

static enum io_status write_reg(const io_i2c_dev_t *i2c, uint8_t reg, uint8_t val)
{
    return io_i2c_mem_write(i2c, reg, &val, 1U, BMP585_TIMEOUT_MS);
}

/** @brief Sign-extends a little-endian 24-bit value */
static int32_t to_int24(const uint8_t *b)
{
    uint32_t raw = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16);
    return (int32_t)(raw << 8) >> 8;
}

enum io_status dev_bmp585_init(const io_i2c_dev_t *i2c)
{
    enum io_status status;
    uint8_t val;

    // The chip doesn't ACK a soft reset over I2C (section 8.36), so the status is ignored
    (void)write_reg(i2c, BMP585_REG_CMD, BMP585_CMD_SOFT_RESET);
    HAL_Delay(BMP585_SOFT_RESET_MS);

    // Post-power-up checks (section 4.3.9)
    status = read_reg(i2c, BMP585_REG_CHIP_ID, &val);
    if (status != IO_STATUS_OK) {
        return status;
    }
    if (val != BMP585_CHIP_ID) {
        return IO_STATUS_ERROR;
    }

    status = read_reg(i2c, BMP585_REG_STATUS, &val);
    if (status != IO_STATUS_OK) {
        return status;
    }
    if ((val & BMP585_STATUS_NVM_RDY) == 0U || (val & BMP585_STATUS_NVM_ERR) != 0U) {
        return IO_STATUS_ERROR;
    }

    status = read_reg(i2c, BMP585_REG_INT_STATUS, &val);
    if (status != IO_STATUS_OK) {
        return status;
    }
    if (val != BMP585_INT_STATUS_POR) {
        return IO_STATUS_ERROR;
    }

    // Reset leaves the chip in deep standby, and modes only change from standby (section 4.3.7)
    status = write_reg(i2c, BMP585_REG_ODR_CONFIG, BMP585_ODR_CONFIG | BMP585_PWR_MODE_STANDBY);
    if (status != IO_STATUS_OK) {
        return status;
    }
    HAL_Delay(BMP585_STANDBY_MS);

    status = write_reg(i2c, BMP585_REG_OSR_CONFIG, BMP585_OSR_CONFIG);
    if (status != IO_STATUS_OK) {
        return status;
    }

    status = write_reg(i2c, BMP585_REG_ODR_CONFIG, BMP585_ODR_CONFIG | BMP585_PWR_MODE_NORMAL);
    if (status != IO_STATUS_OK) {
        return status;
    }
    HAL_Delay(BMP585_STARTUP_MS);

    // The OSR/ODR check only runs in normal mode (section 4.3.4)
    status = read_reg(i2c, BMP585_REG_OSR_EFF, &val);
    if (status != IO_STATUS_OK) {
        return status;
    }
    if ((val & BMP585_OSR_EFF_ODR_IS_VALID) == 0U) {
        return IO_STATUS_ERROR;
    }

    return IO_STATUS_OK;
}

enum io_status dev_bmp585_read(const io_i2c_dev_t *i2c, struct dev_bmp585_sample *out)
{
    ULYSSES_ASSERT(out != NULL, "Sample pointer is NULL");

    // One burst read so temperature and pressure come from the same measurement (section 4.5.1)
    uint8_t buf[6];
    enum io_status status =
        io_i2c_mem_read(i2c, BMP585_REG_TEMP_DATA_XLSB, buf, sizeof(buf), BMP585_TIMEOUT_MS);
    if (status != IO_STATUS_OK) {
        return status;
    }

    out->temp_c = (float)to_int24(&buf[0]) / 65536.0f;
    out->pressure_pa = (float)to_int24(&buf[3]) / 64.0f;

    return IO_STATUS_OK;
}
