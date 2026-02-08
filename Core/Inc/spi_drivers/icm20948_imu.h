#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h> 

/* -------------------------------------------------------------------------- */
/* Register Map (Checked against Datasheet DS-000189)                         */
/* -------------------------------------------------------------------------- */
#define ICM20948_REG_BANK_SEL       0x7F // [cite: 3878]

/* Bank 0 - Data & Power */
#define ICM20948_REG_WHO_AM_I       0x00 // [cite: 3903]
#define ICM20948_REG_PWR_MGMT_1     0x06 // [cite: 3931]
#define ICM20948_REG_ACCEL_XOUT_H   0x2D // [cite: 4060]

/* Bank 2 - Configuration */
#define ICM20948_REG_GYRO_CONFIG_1  0x01 // [cite: 4703]
#define ICM20948_REG_ACCEL_CONFIG   0x14 // [cite: 4837]

#define ICM20948_SAMPLE_Q_SIZE      16

/* -------------------------------------------------------------------------- */
/* Data Types                                                                 */
/* -------------------------------------------------------------------------- */

typedef struct {
    float ax, ay, az;       // Units: g
    float gx, gy, gz;       // Units: deg/s
    
    float pitch_deg;        // Complementary Filtered
    float roll_deg;         // Complementary Filtered
    float yaw_deg;          // Gyro Integrated (subject to drift)
    
    uint32_t timestamp_us;
} icm20948_sample_t;

typedef struct {
    // Configuration
    float accel_scale;      
    float gyro_scale;       
    uint8_t current_bank;

    // Integration State
    float last_pitch;
    float last_roll;
    float last_yaw;
    uint32_t last_ts_us;
    bool  first_sample;
} icm20948_t;

typedef struct {
    icm20948_sample_t samples[ICM20948_SAMPLE_Q_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} icm20948_sample_queue_t;

/* -------------------------------------------------------------------------- */
/* Logic Prototypes                                                           */
/* -------------------------------------------------------------------------- */
size_t icm20948_build_bank_sel(uint8_t bank, uint8_t *tx_buf, icm20948_t *dev);
size_t icm20948_build_write(uint8_t reg, uint8_t val, uint8_t *tx_buf);
size_t icm20948_build_read(uint8_t reg, uint8_t *tx_buf);

/* Modified: Reads only 12 bytes (Accel+Gyro) */
size_t icm20948_build_read_6axis(uint8_t *tx_buf);

/* Note: 'dev' is no longer const because we update its state (timestamp/angles) */
bool icm20948_parse_6axis(const uint8_t *rx_buf, icm20948_sample_t *out, icm20948_t *dev);
void icm20948_init_struct(icm20948_t *dev);

static inline bool icm_queue_empty(icm20948_sample_queue_t *q) {
    return q->head == q->tail;
}

static inline bool icm_queue_full(icm20948_sample_queue_t *q) {
    return ((q->head + 1U) % ICM20948_SAMPLE_Q_SIZE) == q->tail;
}

static inline bool icm_queue_push(icm20948_sample_queue_t *q, const icm20948_sample_t *sample) {
    if (icm_queue_full(q)) return false;
    q->samples[q->head] = *sample;
    q->head = (uint8_t)((q->head + 1U) % ICM20948_SAMPLE_Q_SIZE);
    return true;
}

static inline bool icm_queue_pop(icm20948_sample_queue_t *q, icm20948_sample_t *sample) {
    if (icm_queue_empty(q)) return false;
    *sample = q->samples[q->tail];
    q->tail = (uint8_t)((q->tail + 1U) % ICM20948_SAMPLE_Q_SIZE);
    return true;
}

#endif
