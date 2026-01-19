#include "icm20948_imu.h"
#include <string.h>

#define SPI_RD(a)       ((uint8_t)((a) | 0x80))
#define SPI_WR(a)       ((uint8_t)((a) & 0x7F))
#define RAD_TO_DEG      57.29578f
#define COMP_FILTER_ALPHA 0.98f

void icm20948_init_struct(icm20948_t *dev) {
    dev->accel_scale  = 1.0f / 2048.0f; // Default 16G [cite: 3144]
    dev->gyro_scale   = 1.0f / 16.4f;   // Default 2000dps [cite: 3128]
    dev->current_bank = 0xFF;           // Force initial bank update
    
    // Reset state
    dev->last_pitch   = 0.0f;
    dev->last_roll    = 0.0f;
    dev->last_yaw     = 0.0f;
    dev->last_ts_us   = 0;
    dev->first_sample = true;
}

size_t icm20948_build_bank_sel(uint8_t bank, uint8_t *tx_buf, icm20948_t *dev) {
    if (dev && dev->current_bank == bank) return 0;
    tx_buf[0] = SPI_WR(ICM20948_REG_BANK_SEL);
    tx_buf[1] = (bank << 4);
    if (dev) dev->current_bank = bank;
    return 2;
}

size_t icm20948_build_write(uint8_t reg, uint8_t val, uint8_t *tx_buf) {
    tx_buf[0] = SPI_WR(reg);
    tx_buf[1] = val;
    return 2;
}

size_t icm20948_build_read(uint8_t reg, uint8_t *tx_buf) {
    tx_buf[0] = SPI_RD(reg);
    tx_buf[1] = 0x00;
    return 2;
}

/* * Simplified Burst Read:
 * Reads 12 bytes starting at ACCEL_XOUT_H (0x2D).
 * Order: Accel X, Y, Z, Gyro X, Y, Z.
 */
size_t icm20948_build_read_6axis(uint8_t *tx_buf) {
    tx_buf[0] = SPI_RD(ICM20948_REG_ACCEL_XOUT_H); 
    memset(&tx_buf[1], 0, 12); // Send 12 dummy bytes
    return 13; // 1 cmd + 12 data
}

bool icm20948_parse_6axis(const uint8_t *rx_buf, icm20948_sample_t *out, icm20948_t *dev) {
    if (!rx_buf || !out || !dev) return false;
    
    const uint8_t *d = &rx_buf[1]; // Skip command byte

    // 1. Raw Data Extraction (Big Endian) [cite: 4060, 4114]
    int16_t rax = (int16_t)((d[0] << 8) | d[1]);
    int16_t ray = (int16_t)((d[2] << 8) | d[3]);
    int16_t raz = (int16_t)((d[4] << 8) | d[5]);

    int16_t rgx = (int16_t)((d[6] << 8) | d[7]);
    int16_t rgy = (int16_t)((d[8] << 8) | d[9]);
    int16_t rgz = (int16_t)((d[10] << 8) | d[11]);

    // 2. Scale Conversion
    out->ax = (float)rax * dev->accel_scale;
    out->ay = (float)ray * dev->accel_scale;
    out->az = (float)raz * dev->accel_scale;

    out->gx = (float)rgx * dev->gyro_scale;
    out->gy = (float)rgy * dev->gyro_scale;
    out->gz = (float)rgz * dev->gyro_scale;

    // 3. Time Delta Calculation
    // (Assuming external mechanism provides timestamp in sample struct before calling parse, 
    //  or we calculate it here based on a system timer. 
    //  For this driver, I assume 'out->timestamp_us' is populated by the user's SPI job)
    float dt = 0.0f;
    if (dev->first_sample) {
        dt = 0.001f; // Default small dt for first sample
        dev->first_sample = false;
    } else {
        // Handle wrapping or jumps if necessary
        dt = (float)(out->timestamp_us - dev->last_ts_us) / 1000000.0f;
        if (dt <= 0.0f) dt = 0.001f; 
    }
    dev->last_ts_us = out->timestamp_us;

    // 4. Orientation Estimation
    
    // A. Accelerometer Angles (Static reference)
    // Pitch: Rotation around Y-axis
    // Roll:  Rotation around X-axis
    float accel_pitch = atan2f(out->ax, sqrtf(out->ay * out->ay + out->az * out->az)) * RAD_TO_DEG;
    float accel_roll  = atan2f(out->ay, out->az) * RAD_TO_DEG;

    // B. Complementary Filter for Pitch & Roll
    // Filter combines high-pass Gyro integration with low-pass Accel
    dev->last_pitch = COMP_FILTER_ALPHA * (dev->last_pitch + out->gy * dt) + 
                      (1.0f - COMP_FILTER_ALPHA) * accel_pitch;
                      
    dev->last_roll  = COMP_FILTER_ALPHA * (dev->last_roll + out->gx * dt) + 
                      (1.0f - COMP_FILTER_ALPHA) * accel_roll;

    // C. Yaw Integration (Gyro only - subject to drift)
    // Since we don't have a Mag, we just integrate Z-axis gyro
    dev->last_yaw   = dev->last_yaw + (out->gz * dt);

    // 5. Output
    out->pitch_deg = dev->last_pitch;
    out->roll_deg  = dev->last_roll;
    out->yaw_deg   = dev->last_yaw;

    return true;
}