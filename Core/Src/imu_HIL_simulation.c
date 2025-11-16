#include "imu_HIL_simulation.h"

void config_imu_HIL(SPI_HandleTypeDef* SPI_Handle) {
    IMU_HIL* imu = (IMU_HIL*) malloc(sizeof(IMU_HIL));
    imu->bus_handle = SPI_Handle;
}

void update_imu_HIL_stream() {
    // read simulated accelerometer data
    uint8_t addr = 0x80 | 0x02;   // read ACC_X_LSB
    uint8_t buf[6];

    CS_ACC_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, 6, HAL_MAX_DELAY);
    CS_ACC_HIGH();

    int16_t ax = (buf[1] << 8) | buf[0];
    int16_t ay = (buf[3] << 8) | buf[2];
    int16_t az = (buf[5] << 8) | buf[4];

    // read simulated gyroscope data
    uint8_t addr = 0x80 | 0x02;
    uint8_t buf[6];

    CS_GYR_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, 6, HAL_MAX_DELAY);
    CS_GYR_HIGH();

    int16_t gx = (buf[1] << 8) | buf[0];
    int16_t gy = (buf[3] << 8) | buf[2];
    int16_t gz = (buf[5] << 8) | buf[4];
}
