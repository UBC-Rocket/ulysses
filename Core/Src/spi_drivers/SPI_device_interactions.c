#include "SPI_device_interactions.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

bmi088_accel_sample_queue_t bmi088_acc_sample_ring;
bmi088_gyro_sample_queue_t bmi088_gyro_sample_ring;
ms5611_sample_queue_t ms5611_sample_ring;

bmi088_accel_t accel;
bmi088_gyro_t gyro;

spi_job_queue_t jobq_spi_2;
volatile bool bmi088_accel_ready = false;
volatile bool bmi088_gyro_ready = false;

uint8_t bmi088_accel_init(SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port,
                       uint16_t cs_pin,
                       bmi088_accel_t *dev)
{
    uint8_t tx[16], rx[16];

    /* --- 1. Soft reset --- */
    size_t n = bmi088_accel_build_softreset(tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(600000);

    /* --- 2. Verify chip ID --- */
    n = bmi088_accel_build_read_reg(BMI088_ACC_CHIP_ID_REG, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if (rx[2] != BMI088_ACC_CHIP_ID_VALUE)
        return 1;

    /* --- 3. Power on accelerometer --- */
    n = bmi088_accel_build_power_conf(BMI088_ACC_PWR_ACTIVE, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, &tx[2], 2, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);
    
    n = bmi088_accel_build_read_reg(BMI088_ACC_PWR_CONF, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if(rx[2] != 0)
        return 2;

    n = bmi088_accel_build_read_reg(BMI088_ACC_PWR_CTRL, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if(rx[2] != 4)
        return 3;

    /* --- 4. Set measurement range (±24 g) --- */
    n = bmi088_accel_build_range(BMI088_ACC_RANGE_3G, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    /* --- 5. Configure ODR = 800 Hz, normal bandwidth --- */
    n = bmi088_accel_build_odr_bw(BMI088_ACC_ODR_800_HZ,
                                  BMI088_ACC_BWP_NORMAL,
                                  tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    n = bmi088_accel_build_read_reg(BMI088_ACC_CONF, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    n = bmi088_accel_build_read_reg(BMI088_ACC_RANGE, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 3, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    /* --- 6. Configure INT1 electrical behaviour --- */
    n = bmi088_accel_build_int_pin_conf(BMI088_ACC_INT1,
                                        BMI088_ACC_INT_PUSH_PULL,
                                        BMI088_ACC_INT_ACTIVE_HIGH,
                                        false, // pulse (not latched)
                                        tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* --- 7. Map data-ready event to INT1 --- */
    n = bmi088_accel_build_int_event_map(BMI088_ACC_INT1,
                                         BMI088_ACC_INT_EVENT_DATA_READY,
                                         tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* --- 8. (Optional) small startup delay --- */

    n = bmi088_accel_build_read_reg(BMI088_ACC_STATUS_REG, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    if((rx[2] & 0x80) == 0)
        return 4;

    n = bmi088_accel_build_read(BMI088_ACC_READ_DATA_XYZ, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(50000);

    bmi088_accel_sample_t sample;
    bmi088_accel_parse_data_xyz(&rx[2], &sample, dev);

    return 0;
}


/* -------------------------------------------------------------------------- */
/* Done callback: called after SPI DMA completes                              */
/* -------------------------------------------------------------------------- */
static void bmi088_accel_done(spi_job_t *job,
                              const uint8_t *rx_buf,
                              void *arg)
{
    (void)job;
    (void)arg;

    // Parse accelerometer XYZ data (skip address byte)
    bmi088_accel_sample_t sample;
    if (bmi088_accel_parse_data_xyz(&rx_buf[2], &sample, &accel)) {
        sample.t_us = job->t_sample;
        bmi088_acc_sample_queue(&bmi088_acc_sample_ring,  &sample);
    }
}

/* -------------------------------------------------------------------------- */
/* Accelerometer interrupt entry point                                        */
/* -------------------------------------------------------------------------- */
void bmi088_accel_interrupt(void)
{
    const uint32_t now = timestamp_us();

    spi_job_t job;

    job.cs_port  = BMI_ACC_Chip_Select_GPIO_Port;
    job.cs_pin   = BMI_ACC_Chip_Select_Pin;

    job.len      = bmi088_accel_build_read(BMI088_ACC_READ_DATA_XYZ, job.tx); 
    job.t_sample = now;
    job.type     = SPI_XFER_TXRX;
    job.done     = bmi088_accel_done;
    job.done_arg = NULL;
    job.sensor   = SENSOR_ID_ACCEL;

    spi_submit_job(job, &jobq_spi_2);

}


bmi088_gyro_t gyro_dev;
uint8_t gyro_tx[8];
bmi088_gyro_sample_t gyro_sample_ring; // You can expand to a queue later

/* -------------------------------------------------------------------------- */
/* Gyroscope Initialization                                                   */
/* -------------------------------------------------------------------------- */
uint8_t bmi088_gyro_init(SPI_HandleTypeDef *hspi,
                         GPIO_TypeDef *cs_port,
                         uint16_t cs_pin,
                         bmi088_gyro_t *dev)
{
    uint8_t tx[16], rx[16];
    size_t n;

    /* --- 1. Soft reset --- */
    n = bmi088_gyro_build_softreset(tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(30000); // per datasheet ≥30 ms

    /* --- 2. Verify chip ID --- */
    n = bmi088_gyro_build_read(BMI088_GYRO_READ_CHIP_ID, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(5000);

    if (rx[1] != BMI088_GYRO_CHIP_ID_VALUE)
        return 1;

    /* --- 3. Power on gyroscope --- */
    n = bmi088_gyro_build_power_conf(BMI088_GYRO_PWR_NORMAL, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(80000); // PLL lock time ≥80 ms

    /* --- 4. Configure range ±500 dps --- */
    n = bmi088_gyro_build_range(BMI088_GYRO_RANGE_500DPS, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(5000);

    /* --- 5. Configure ODR 400 Hz / BW 116 Hz --- */
    n = bmi088_gyro_build_odr(BMI088_GYRO_ODR_400_HZ_BW_116, tx, dev);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(5000);

    /* --- 6. Configure INT3 electrical behavior (push-pull, active high) --- */
    n = bmi088_gyro_build_int_pin_conf(BMI088_GYRO_INT3,
                                       BMI088_GYRO_INT_PUSH_PULL,
                                       BMI088_GYRO_INT_ACTIVE_HIGH,
                                       tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(5000);

    /* --- 7. Map data-ready event to INT3 --- */
    n = bmi088_gyro_build_int_event_map(BMI088_GYRO_INT3,
                                        BMI088_GYRO_INT_EVENT_DATA_READY,
                                        tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);   // first frame (INT_CTRL)
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx + 2, 2, HAL_MAX_DELAY); // second frame (IO_MAP)
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(1000);

    /* --- 8. Verify first reading --- */
    n = bmi088_gyro_build_read(BMI088_GYRO_READ_DATA_XYZ, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    bmi088_gyro_sample_t s;
    if (!bmi088_gyro_parse_data_xyz(rx, &s, dev))
        return 2;

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Gyroscope DMA done callback                                                */
/* -------------------------------------------------------------------------- */
static void bmi088_gyro_done(spi_job_t *job,
                             const uint8_t *rx_buf,
                             void *arg)
{
    (void)job;
    (void)arg;

    bmi088_gyro_sample_t s;
    if (bmi088_gyro_parse_data_xyz(&rx_buf[1], &s, &gyro)) {
        s.t_us = job->t_sample;
        bmi088_gyro_sample_queue(&bmi088_gyro_sample_ring, &s);
    }
}

/* -------------------------------------------------------------------------- */
/* Gyroscope interrupt entry point                                            */
/* -------------------------------------------------------------------------- */
void bmi088_gyro_interrupt(void)
{
    const uint32_t now = timestamp_us();

    spi_job_t job;

    job.cs_port  = BMI_GYRO_Chip_Select_GPIO_Port;
    job.cs_pin   = BMI_GYRO_Chip_Select_Pin;

    job.len      = bmi088_gyro_build_read(BMI088_GYRO_READ_DATA_XYZ, job.tx); 
    job.t_sample = now;
    job.type     = SPI_XFER_TXRX;
    job.done     = bmi088_gyro_done;
    job.done_arg = NULL;
    job.sensor   = SENSOR_ID_GYRO;

    spi_submit_job(job, &jobq_spi_2);
}

uint8_t ms5611_init(SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    ms5611_t *dev)
{
    uint8_t tx[4], rx[4];

    /* --- 1. Send reset command --- */
    size_t n = ms5611_build_reset(tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    delay_us(1000);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* --- 2. Wait ≥2.8 ms for PROM reload --- */
    delay_us(3000);

    /* --- 3. Read PROM[0..7] --- */
    for (uint8_t i = 0; i < 8; i++) {
        n = ms5611_build_prom_read(i, tx);

        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
        delay_us(1000);
        HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
        delay_us(1000);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
        delay_us(1000);

        if (!ms5611_parse_prom_word(&rx[1], &dev->C[i])) {
            return 1; // parse error
        }
    }

    /* --- 4. Validate PROM CRC --- */
    if (!ms5611_check_crc(dev->C)) {
        return 2; // CRC mismatch (invalid PROM)
    }

    /* --- 5. Clear previous raw readings --- */
    dev->D1_raw = 0;
    dev->D2_raw = 0;
    dev->TEMP_centi = 0;
    dev->P_centi_mbar = 0;

    return 0; // success
}

// ---- BARO poller private ----
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} ms5611_hw_t;

static ms5611_hw_t g_baro_hw;
static ms5611_poller_t *g_baro = NULL;
static const uint32_t ms5611_osr_delay_us_tbl[] = {
    [MS5611_OSR_256]  = 540,
    [MS5611_OSR_512]  = 1060,
    [MS5611_OSR_1024] = 2080,
    [MS5611_OSR_2048] = 4130,
    [MS5611_OSR_4096] = 8220
};

// Forward callbacks
static void ms5611_cb_read_d1(spi_job_t *job, const uint8_t *rx, void *arg);
static void ms5611_cb_read_d2(spi_job_t *job, const uint8_t *rx, void *arg);

// ---- Init poller: reset + PROM + CRC, set timing, arm state machine ----
void ms5611_poller_init(ms5611_poller_t *p,
                        SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin,
                        ms5611_osr_t osr, uint32_t odr_hz)
{
    memset(p, 0, sizeof(*p));
    ms5611_sample_ring.head = 0;
    ms5611_sample_ring.tail = 0;
    g_baro_hw.hspi = hspi;
    g_baro_hw.cs_port = cs_port;
    g_baro_hw.cs_pin = cs_pin;

    // 1) Reset
    uint8_t tx[4], rx[4];
    size_t n = ms5611_build_reset(tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); 
    delay_us(1000);
    HAL_SPI_Transmit(hspi, tx, n, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(3000); // >= 2.8 ms

    // 2) PROM read + CRC
    for (uint8_t i = 0; i < 8; ++i) {
        n = ms5611_build_prom_read(i, tx);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); 
        delay_us(1000);
        HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
        ms5611_parse_prom_word(&rx[1], &p->dev.C[i]);
        delay_us(200);
    }
    // If CRC fails, you may choose to retry reset once.
    (void)ms5611_check_crc(p->dev.C);

    ms5611_poller_set_rate(p, osr, odr_hz);

    p->state           = MS_IDLE;
    p->t_cycle_start_us= timestamp_us();
    p->t_next_action_us= p->t_cycle_start_us; // ready to start anytime
    p->fresh           = false;
    p->seq             = 0;

    g_baro = p; // keep a global pointer for ISR/callback access
}

// ---- Change rate/OSR at runtime ----
void ms5611_poller_set_rate(ms5611_poller_t *p, ms5611_osr_t osr, uint32_t odr_hz)
{
    p->osr       = osr;
    p->conv_us   = ms5611_osr_delay_us_tbl[osr];
    p->odr_hz    = odr_hz ? odr_hz : 50;
    p->period_us = (1000000UL + p->odr_hz/2) / p->odr_hz; // rounded
}

// ---- Submit helpers (non-blocking) ----
static void ms5611_submit_convert_d1(void)
{
    ms5611_poller_t *p = g_baro;
    if (!p) return;

    spi_job_t j = {0};
    j.cs_port = g_baro_hw.cs_port; j.cs_pin = g_baro_hw.cs_pin;
    j.len     = ms5611_build_convert_d1(p->osr, j.tx);
    j.type    = SPI_XFER_TX; j.sensor = SENSOR_ID_BARO;
    spi_submit_job(j, &jobq_spi_2);
}

static void ms5611_submit_convert_d2(void)
{
    ms5611_poller_t *p = g_baro;
    if (!p) return;

    spi_job_t j = {0};
    j.cs_port = g_baro_hw.cs_port; j.cs_pin = g_baro_hw.cs_pin;
    j.len     = ms5611_build_convert_d2(p->osr, j.tx);
    j.type    = SPI_XFER_TX; j.sensor = SENSOR_ID_BARO;
    spi_submit_job(j, &jobq_spi_2);
}

static void ms5611_submit_read_d1(void)
{
    if (!g_baro) return;

    spi_job_t j = {0};
    j.cs_port = g_baro_hw.cs_port; j.cs_pin = g_baro_hw.cs_pin;
    j.len     = ms5611_build_adc_read(j.tx);
    j.type    = SPI_XFER_TXRX; j.sensor = SENSOR_ID_BARO;
    j.done    = ms5611_cb_read_d1;
    spi_submit_job(j, &jobq_spi_2);
}

static void ms5611_submit_read_d2(void)
{
    if (!g_baro) return;

    spi_job_t j = {0};
    j.cs_port = g_baro_hw.cs_port; j.cs_pin = g_baro_hw.cs_pin;
    j.len     = ms5611_build_adc_read(j.tx);
    j.t_sample= timestamp_us();
    j.type    = SPI_XFER_TXRX; j.sensor = SENSOR_ID_BARO;
    j.done    = ms5611_cb_read_d2;
    spi_submit_job(j, &jobq_spi_2);
}

// ---- DMA done callbacks (update raw values; no blocking) ----
static void ms5611_cb_read_d1(spi_job_t *job, const uint8_t *rx, void *arg)
{
    (void)job; (void)arg;
    ms5611_poller_t *p = g_baro;
    if (!p) return;

    ms5611_parse_adc_result(&rx[1], &p->dev.D1_raw);
}

static void ms5611_cb_read_d2(spi_job_t *job, const uint8_t *rx, void *arg)
{
    (void)arg;
    ms5611_poller_t *p = g_baro;
    if (!p) return;

    ms5611_parse_adc_result(&rx[1], &p->dev.D2_raw);
    ms5611_compute(&p->dev);
    p->fresh = true;
    p->seq++;
    ms5611_sample_t sample = {
        .t_us           = job ? job->t_sample : 0U,
        .d1_raw         = p->dev.D1_raw,
        .d2_raw         = p->dev.D2_raw,
        .temp_centi     = p->dev.TEMP_centi,
        .pressure_centi = p->dev.P_centi_mbar,
        .seq            = p->seq
    };
    ms5611_sample_queue(&ms5611_sample_ring, &sample);
}

// ---- 1 kHz tick: advance state when time has come; never block ----
void ms5611_poller_tick_1khz(ms5611_poller_t *p)
{
    const uint32_t now = timestamp_us();

    // keep sample cadence anchored to t_cycle_start_us
    switch (p->state) {
        case MS_IDLE:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                p->t_cycle_start_us = now;
                ms5611_submit_convert_d1();
                p->t_next_action_us = now + p->conv_us;  // when D1 is ready
                p->state = MS_WAIT_D1;
            }
            break;

        case MS_WAIT_D1:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                ms5611_submit_read_d1();
                // immediately start D2 conversion after read job is queued?
                // Safer: queue conversion in next state for bus fairness:
                p->t_next_action_us = now; // as soon as bus allows; no wait needed
                p->state = MS_READ_D1;
            }
            break;

        case MS_READ_D1:
            // Once READ_D1 job completes, callback filled D1_raw.
            // Start D2 conversion when our scheduled moment arrives (no extra wait).
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                ms5611_submit_convert_d2();
                p->t_next_action_us = now + p->conv_us; // when D2 is ready
                p->state = MS_WAIT_D2;
            }
            break;

        case MS_WAIT_D2:
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                ms5611_submit_read_d2();
                // After D2 read finishes (callback), we’ll mark fresh.
                // Schedule next cycle boundary to hit the chosen ODR.
                p->t_next_action_us = p->t_cycle_start_us + p->period_us;
                p->state = MS_READ_D2;
            }
            break;

        case MS_READ_D2:
            // Wait until the cycle period elapses then go idle (ready for next cycle)
            if ((int32_t)(now - p->t_next_action_us) >= 0) {
                p->state = MS_IDLE;
            }
            break;
    }
}

// ---- Fetch latest sample without blocking; returns true if fresh ----
bool ms5611_fetch_latest(ms5611_poller_t *p, ms5611_t *out, uint32_t *seq)
{
    if (!p->fresh) return false;
    if (out) *out = p->dev;
    if (seq) *seq = p->seq;
    p->fresh = false;
    return true;
}
