#include "icm20948_imu.h"
#include "SPI_device_interactions.h" 
#include "main.h"
#include <stm32h563xx.h>

/* Helper: SPI Transmit */
static void icm_tx(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t *tx, size_t len) {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, tx, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    delay_us(5);
}

/* Helper: Bank Switch */
static void icm_bank(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, icm20948_t *dev, uint8_t bank) {
    uint8_t tx[2];
    if(icm20948_build_bank_sel(bank, tx, dev)) {
        icm_tx(hspi, cs_port, cs_pin, tx, 2);
    }
}

/* Helper: Register Write */
static void icm_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint8_t val) {
    uint8_t tx[2];
    icm20948_build_write(reg, val, tx);
    icm_tx(hspi, cs_port, cs_pin, tx, 2);
}

/* * Simplified Init: 
 * Resets chip, Checks ID, Sets up Accel/Gyro. 
 * Ignores Magnetometer completely.
 */
uint8_t icm20948_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, icm20948_t *dev) {
    uint8_t tx[16], rx[16];
    size_t n;
    
    icm20948_init_struct(dev);

    /* 1. Reset (Bank 0) */
    icm_bank(hspi, cs_port, cs_pin, dev, 0);
    icm_write(hspi, cs_port, cs_pin, ICM20948_REG_PWR_MGMT_1, 0x80); // Reset bit
    HAL_Delay(100);
    
    icm_write(hspi, cs_port, cs_pin, ICM20948_REG_PWR_MGMT_1, 0x01); // Auto Clock
    HAL_Delay(50);

    /* 2. Check ID */
    n = icm20948_build_read(ICM20948_REG_WHO_AM_I, tx);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    
    if(rx[1] != 0xEA) return 1; // Error: Chip ID mismatch

    /* 3. Configure Accel & Gyro (Bank 2) */
    icm_bank(hspi, cs_port, cs_pin, dev, 2);
    
    // Gyro Config: ±2000dps, DLPF Enabled (Low Pass Filter)
    icm_write(hspi, cs_port, cs_pin, ICM20948_REG_GYRO_CONFIG_1, 0x07); 
    
    // Accel Config: ±16g, DLPF Enabled
    icm_write(hspi, cs_port, cs_pin, ICM20948_REG_ACCEL_CONFIG, 0x07);

    /* 4. Return to Bank 0 for Data Reading */
    icm_bank(hspi, cs_port, cs_pin, dev, 0);
    
    return 0; // Success
}

/* * Simplified Read:
 * Reads 12 bytes, parses, pushes to queue.
 */
uint8_t icm20948_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, icm20948_t *dev) {
    uint8_t tx[15], rx[15]; // Buffer size reduced (1 command + 12 data)
    
    // Ensure we are in Bank 0
    icm_bank(hspi, cs_port, cs_pin, dev, 0);
    
    size_t n = icm20948_build_read_6axis(tx);
    
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, n, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    icm20948_sample_t s;
    s.timestamp_us = micros();
    if(icm20948_parse_6axis(rx, &s, dev)) {
        icm_queue_push(&icm_sample_ring, &s);
        return 0;
    }
    return 1;
}