#include "BMI088_accel.h"
#include "BMI088_gyro.h"
#include "MS5611_baro.h"
#include "SPI_queue.h"
#include "main.h"
#include "stm32h5xx_hal.h"

#ifndef SPI_DEVICE_INTERACTIONS
#define SPI_DEVICE_INTERACTIONS

#define BMI088_ACC_CHIP_ID_VALUE 0x1E
#define BMI088_GYRO_CHIP_ID_VALUE  0x0F

extern bmi088_accel_sample_queue_t bmi088_acc_sample_ring;
extern bmi088_gyro_sample_queue_t bmi088_gyro_sample_ring;
extern ms5611_sample_queue_t ms5611_sample_ring;

extern spi_job_queue_t jobq_spi_1;
extern spi_job_queue_t jobq_spi_2;

extern bmi088_accel_t accel;
extern bmi088_gyro_t gyro;

extern volatile bool bmi088_accel_ready;
extern volatile bool bmi088_gyro_ready;

uint8_t bmi088_accel_init(SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port,
                       uint16_t cs_pin,
                       bmi088_accel_t *dev);

void bmi088_accel_interrupt(void);

uint8_t bmi088_gyro_init(SPI_HandleTypeDef *hspi,
                         GPIO_TypeDef *cs_port,
                         uint16_t cs_pin,
                         bmi088_gyro_t *dev);

void bmi088_gyro_interrupt(void);

/**
 * @brief Initialize MS5611 barometer.
 * 
 * Sequence:
 *  1. Send Reset (0x1E)
 *  2. Wait ≥2.8 ms (PROM reload)
 *  3. Read PROM[0..7] (each 16-bit)
 *  4. Validate CRC-4
 *
 * @param hspi    SPI handle
 * @param cs_port Chip select GPIO port
 * @param cs_pin  Chip select pin
 * @param dev     Device struct to populate
 * @return uint8_t 0 = OK, nonzero = error code
 */
uint8_t ms5611_init(SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port,
                    uint16_t cs_pin,
                    ms5611_t *dev);

// --- BARO (MS5611) non-blocking poller API ---
typedef struct {
    // config
    ms5611_osr_t osr;
    uint32_t     odr_hz;           // target baro rate (e.g., 50)
    uint32_t     conv_us;          // OSR-dependent typ delay (e.g., 8220)
    uint32_t     period_us;        // 1e6/odr_hz

    // timing
    uint32_t     t_next_action_us; // when to do the next SPI action
    uint32_t     t_cycle_start_us; // start of current D1+D2 cycle

    // state
    enum { MS_IDLE, MS_CONV_D1, MS_WAIT_D1, MS_READ_D1,
           MS_CONV_D2, MS_WAIT_D2, MS_READ_D2, MS_READY } state;

    // data
    ms5611_t     dev;              // holds C[], D1/D2, outputs
    uint8_t      rx_buf[4];        // scratch if needed
    bool         fresh;            // toggles true when a new P/T is computed
    uint32_t     seq;              // monotonically increasing sample sequence
} ms5611_poller_t;

void ms5611_poller_init(ms5611_poller_t *p,
                        SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin,
                        ms5611_osr_t osr, uint32_t odr_hz);

void ms5611_poller_tick_1khz(ms5611_poller_t *p);      // call from EKF @1kHz
bool ms5611_fetch_latest(ms5611_poller_t *p, ms5611_t *out, uint32_t *seq); // non-blocking

// (optional) change ODR/OSR at runtime
void ms5611_poller_set_rate(ms5611_poller_t *p, ms5611_osr_t osr, uint32_t odr_hz);


#endif //DEVICE_INTERACTIONS
