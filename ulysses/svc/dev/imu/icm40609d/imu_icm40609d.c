#include "imu_icm40609d.h"
#include "dev/imu/imu.h"
#include "ulysses/io/delay.h"
#include "ulysses/io/spi.h"
#include "ulysses/status.h"
#include "ulysses/lib/assert.h"
#include "ulysses/svc/dev/imu.h"

#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdatomic.h>

// SPI read/write macros
#define SPI_READ(reg) ((reg) | 0x80U)
#define SPI_WRITE(reg) ((reg) & 0x7FU)

// ICM40609D specific constants
#define ICM_WHO_AM_I_VALUE (0x3BU)
#define ICM_TIMEOUT_MS (10U)
#define ICM_STARTUP_DELAY_MS    (50U)
#define ICM_RAW_MIN_VALID (-32766)

// Measurement conversion constants
#define ICM_ACCEL_MAX_FSR_G    (32.0f)
#define ICM_GYRO_MAX_FSR_DPS   (2000.0f)
#define ICM_RAW_HALF_RANGE     (32768.0f)

// Physical conversion constants
#define GRAVITY_MPS2           (9.80665f)
#define DEG_TO_RAD             (3.141592653589793f / 180.0f)

// Register addresses
#define ICM_REG_DEVICE_CONFIG (0x11U)
#define ICM_REG_ACCEL_DATA_X1 (0x1FU)
#define ICM_REG_INT_STATUS (0x2DU)
#define ICM_REG_PWR_MGMT0 (0x4EU)
#define ICM_REG_GYRO_CONFIG0 (0x4FU)
#define ICM_REG_ACCEL_CONFIG0 (0x50U)
#define ICM_REG_WHO_AM_I (0x75U)

// Sensor configuration values
#define ICM_FSR_SHIFT      (5U)   
#define ICM_FSR_MASK       (0x07U << ICM_FSR_SHIFT)
#define ICM_ODR_MASK       (0x0FU)
#define ICM_FSR_ODR_MASK   (ICM_FSR_MASK | ICM_ODR_MASK)

// Power management configuration values
#define ICM_PWR_MGMT0_MASK      (0x2FU)
// Temperature sensor 
#define ICM_TEMP_ENABLE         (0x00U << 5)
#define ICM_TEMP_DISABLE        (0x01U << 5)
// Gyroscope power mode
#define ICM_GYRO_MODE_OFF       (0x00U << 2)
#define ICM_GYRO_MODE_STANDBY   (0x01U << 2)
#define ICM_GYRO_MODE_LN        (0x03U << 2)
// Accelerometer power mode
#define ICM_ACCEL_MODE_OFF      (0x00U << 0)
#define ICM_ACCEL_MODE_LP       (0x02U << 0)
#define ICM_ACCEL_MODE_LN       (0x03U << 0)

// =====================================================

// Async measurement read command
static const uint8_t sample_tx[ICM_TRANSFER_LEN] = {
    SPI_READ(ICM_REG_ACCEL_DATA_X1)
};

// =====================================================

// Private helper functions

// Multi register access
static enum uls_status read_regs(io_spi_dev_t *spi, uint8_t reg, uint8_t *data, uint16_t len)
{
    ULYSSES_ASSERT(data != NULL, "Invalid buffer");
    ULYSSES_ASSERT(len > 0 && len <= ICM_SAMPLE_LEN, "Invalid payload length");

    uint8_t tx[ICM_TRANSFER_LEN] = {0};
    uint8_t rx[ICM_TRANSFER_LEN];
    tx[0] = SPI_READ(reg);

    enum uls_status status = io_spi_transfer(spi, tx, rx, len + 1, ICM_TIMEOUT_MS);

    if (status == ULS_STATUS_OK) {
        memcpy(data, &rx[1], len);
    }

    return status;
}

static enum uls_status write_regs(io_spi_dev_t *spi, uint8_t reg, const uint8_t *data, uint16_t len)
{
    ULYSSES_ASSERT(data != NULL, "Invalid buffer");
    ULYSSES_ASSERT(len > 0 && len <= ICM_SAMPLE_LEN, "Invalid payload length");

    uint8_t tx[ICM_TRANSFER_LEN];

    tx[0] = SPI_WRITE(reg);
    memcpy(&tx[1], data, len);

    return io_spi_transmit(spi, tx, len + 1, ICM_TIMEOUT_MS);
}

// Single register access
static enum uls_status read_reg(io_spi_dev_t *spi, uint8_t reg, uint8_t *data)
{
    return read_regs(spi, reg, data, 1);
}

static enum uls_status write_reg(io_spi_dev_t *spi, uint8_t reg, uint8_t data)
{
    return write_regs(spi, reg, &data, 1);
}

static enum uls_status verify_reg(io_spi_dev_t *spi, uint8_t reg, uint8_t expected_value, uint8_t mask)
{
    uint8_t actual_value; 

    enum uls_status status = read_reg(spi, reg, &actual_value); 
    if (status != ULS_STATUS_OK) {
        return status;
    }

    if ((expected_value & mask) != (actual_value & mask)) {
        return ULS_STATUS_ERROR;
    }

    return ULS_STATUS_OK;
}

static enum uls_status configure_reg(io_spi_dev_t *spi, uint8_t reg, uint8_t value, uint8_t mask)
{
    enum uls_status status = write_reg(spi, reg, value);
    if (status != ULS_STATUS_OK) {
        return status;
    }

    return verify_reg(spi, reg, value, mask);
}

// Check for identity, and functionality of read_reg()
static enum uls_status verify_identity(io_spi_dev_t *spi)
{
    return verify_reg(spi, ICM_REG_WHO_AM_I, ICM_WHO_AM_I_VALUE, 0xFFU);
}

// Reset the device 
static enum uls_status reset_device(io_spi_dev_t *spi)
{
    enum uls_status status = write_reg(spi, ICM_REG_DEVICE_CONFIG, 0x01U);

    if (status != ULS_STATUS_OK) {
        return status;
    }

    io_spin_delay_ms(1);

    return ULS_STATUS_OK;
}

static int16_t combine_bytes(uint8_t high, uint8_t low)
{   
    uint16_t result = (uint16_t)high << 8; 
    result |= low;

    return (int16_t)result;
}

static void init_scale(
    const struct dev_imu_icm40609d_config *config, 
    struct dev_imu_icm40609d_data *data)
{
    float accel_fsr_g = ICM_ACCEL_MAX_FSR_G / (1U << config->accel_fsr); // Full scale range in g
    float gyro_fsr_dps = ICM_GYRO_MAX_FSR_DPS / (1U << config->gyro_fsr); // Full scale range in degrees per second

    data->accel_scale = accel_fsr_g * GRAVITY_MPS2 / ICM_RAW_HALF_RANGE;
    data->gyro_scale = gyro_fsr_dps * DEG_TO_RAD / ICM_RAW_HALF_RANGE;
}

// Data parsing
static enum uls_status parse_sample(
    const struct dev_imu_icm40609d_data *data, 
    const uint8_t *payload, 
    struct dev_imu_sample *sample)
{   
    struct dev_imu_sample temp_sample;

    for (size_t i = 0; i < 3; i++) {

        // Combine bytes and check for invalid readings
        int16_t raw_accel = combine_bytes(payload[2 * i], payload[2 * i + 1]);
        int16_t raw_gyro = combine_bytes(payload[2 * i + 6], payload[2 * i + 7]);
        if (raw_accel < ICM_RAW_MIN_VALID || raw_gyro < ICM_RAW_MIN_VALID) {
            return ULS_STATUS_ERROR;
        }
        
        temp_sample.accel_m_s2[i] = raw_accel * data->accel_scale;

        temp_sample.gyro_rad_s[i] = raw_gyro * data->gyro_scale;
    }

    *sample = temp_sample;
    return ULS_STATUS_OK;
}

// Blocking read
static enum uls_status icm40609d_read(dev_imu_t *imu, struct dev_imu_sample *sample)
{
    const struct dev_imu_icm40609d_config *config = imu->config;
    struct dev_imu_icm40609d_data *data = imu->data;

    // Acquire ownership atomically 
    if (atomic_exchange(&data->busy, true)) {
        return ULS_STATUS_BUSY;
    }

    uint8_t payload[ICM_SAMPLE_LEN];

    // Read measurement registers
    enum uls_status status = read_regs(config->spi, ICM_REG_ACCEL_DATA_X1, payload, ICM_SAMPLE_LEN);
    
    // Parse the raw data into physical units if read was successful
    if (status == ULS_STATUS_OK) {
        status = parse_sample(data, payload, sample);
    }

    // Release ownership regardless of read success or failure
    atomic_store(&data->busy, false);

    return status;
}

static void spi_read_callback(enum uls_status status, void *user_data)
{
    dev_imu_t *imu = user_data;
    struct dev_imu_icm40609d_data *data = imu->data;

    dev_imu_cb_t cb = data->cb;
    void *cb_user_data = data->cb_user_data;

    struct dev_imu_sample sample;

    // Parse sample 
    if (status == ULS_STATUS_OK) {
        status = parse_sample(data, &data->rx_data[1], &sample);
    }
    
    // Clear callback
    data->cb = NULL;
    data->cb_user_data = NULL;

    // Release ownership atomically
    atomic_store(&data->busy, false);

    // Notify caller
    cb(status, status == ULS_STATUS_OK ? &sample : NULL, cb_user_data);
}

// Non-blocking read
static enum uls_status icm40609d_read_async(dev_imu_t *imu, dev_imu_cb_t cb, void *user_data)
{
    const struct dev_imu_icm40609d_config *config = imu->config;
    struct dev_imu_icm40609d_data *data = imu->data;

    // Acquire ownership atomically 
    if (atomic_exchange(&data->busy, true)) {
        return ULS_STATUS_BUSY;
    }

    // Store callback and user data
    data->cb = cb;
    data->cb_user_data = user_data;

    // Start async SPI transfer
    enum uls_status status = io_spi_transfer_it(config->spi, sample_tx, data->rx_data, ICM_TRANSFER_LEN, spi_read_callback, imu);

    // Error handler 
    if (status != ULS_STATUS_OK) {
        // Clear up
        data->cb = NULL;
        data->cb_user_data = NULL;

        // Release ownership atomically
        atomic_store(&data->busy, false);
    }

    return status;
}

// ======================================================

// Operation table

static const struct dev_imu_ops icm40609d_ops = {
    .read = icm40609d_read,
    .read_async = icm40609d_read_async,
};

// ======================================================

// Public API
enum uls_status dev_imu_icm40609d_init(dev_imu_t *imu, const struct dev_imu_icm40609d_config *config,
                                      struct dev_imu_icm40609d_data *data)
{
    ULYSSES_ASSERT(imu != NULL, "Invalid IMU instance"); 
    ULYSSES_ASSERT(config != NULL, "Invalid config");
    ULYSSES_ASSERT(config->spi != NULL, "Invalid SPI device"); 
    ULYSSES_ASSERT(data != NULL, "Invalid data buffer");

    ULYSSES_ASSERT((unsigned)config->accel_fsr <= ICM_ACCEL_FSR_4G, "Invalid accelerometer full scale range");
    ULYSSES_ASSERT((unsigned)config->gyro_fsr <= ICM_GYRO_FSR_15_625DPS, "Invalid gyroscope full scale range");
    ULYSSES_ASSERT((unsigned)config->odr <= ICM_ODR_500HZ && config->odr != 0, "Invalid output data rate");

    // Software initialization
    imu->config = config;
    imu->data = data;
    imu->ops = NULL;

    atomic_init(&data->busy, false);
    data->cb = NULL;
    data->cb_user_data = NULL;

    // Hardware initialization
    enum uls_status status = reset_device(config->spi);
    if (status != ULS_STATUS_OK) {
        return status;
    }

    status = verify_identity(config->spi);
    if (status != ULS_STATUS_OK) {
        return status;
    }

    // Configure accelerometer FSR and ODR
    uint8_t accel_config = (config->accel_fsr << ICM_FSR_SHIFT) | config->odr;
    status = configure_reg(config->spi, ICM_REG_ACCEL_CONFIG0, accel_config, ICM_FSR_ODR_MASK);
    if (status != ULS_STATUS_OK) {
        return status;
    }

    // Configure gyroscope FSR and ODR
    uint8_t gyro_config = (config->gyro_fsr << ICM_FSR_SHIFT) | config->odr;
    status = configure_reg(config->spi, ICM_REG_GYRO_CONFIG0, gyro_config, ICM_FSR_ODR_MASK);
    if (status != ULS_STATUS_OK) {
        return status;
    }

    // Power management configruation: Temp enable, LN mode for both gyro and accel
    uint8_t pwr_config = ICM_TEMP_ENABLE | ICM_GYRO_MODE_LN | ICM_ACCEL_MODE_LN;
    status = write_reg(config->spi, ICM_REG_PWR_MGMT0, pwr_config);
    if (status != ULS_STATUS_OK) {
        return status; 
    }

    io_spin_delay_ms(ICM_STARTUP_DELAY_MS); 

    status = verify_reg(config->spi, ICM_REG_PWR_MGMT0, pwr_config, ICM_PWR_MGMT0_MASK);
    if (status != ULS_STATUS_OK) {
        return status;
    }

    // Initialize scale factors for converting raw readings to physical units
    init_scale(config, data);

    // Install device operations
    imu->ops = &icm40609d_ops;

    return ULS_STATUS_OK;
}
