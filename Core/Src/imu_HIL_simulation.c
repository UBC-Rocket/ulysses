#include "imu_HIL_simulation.h"

IMU_HIL_t* config_imu_HIL(SPI_HandleTypeDef* SPI_Handle) {
    IMU_HIL_t* imu = (IMU_HIL_t*) malloc(sizeof(IMU_HIL_t));
    imu->bus_handle = SPI_Handle;

    return imu;
}

void update_imu_HIL_stream(IMU_HIL_t* imu) {
    // read simulated accelerometer data
    uint8_t addr = 0x80 | 0x02;   // read ACC_X_LSB
    uint8_t buf_accel[6];

    HAL_GPIO_WritePin(CS_ACC_PORT, CS_ACC_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(imu->bus_handle, &addr, 1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_ACC_PORT, CS_ACC_PIN, GPIO_PIN_SET);
    }
    if (HAL_SPI_Receive(imu->bus_handle, buf_accel, 6, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_ACC_PORT, CS_ACC_PIN, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(CS_ACC_PORT, CS_ACC_PIN, GPIO_PIN_SET);

    imu->acc_x = (buf_accel[1] << 8) | buf_accel[0];
    imu->acc_y = (buf_accel[3] << 8) | buf_accel[2];
    imu->acc_z = (buf_accel[5] << 8) | buf_accel[4];

    // read simulated gyroscope data
    uint8_t addr = 0x80 | 0x02;
    uint8_t buf_gyro[6];

    HAL_GPIO_WritePin(CS_GYR_PORT, CS_GYR_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(imu->bus_handle, &addr, 1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_GYR_PORT, CS_GYR_PIN, GPIO_PIN_SET);
    }
    if (HAL_SPI_Receive(imu->bus_handle, buf_gyro, 6, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_GYR_PORT, CS_GYR_PIN, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(CS_GYR_PORT, CS_GYR_PIN, GPIO_PIN_SET);

    imu->gyro_x = (buf_gyro[1] << 8) | buf_gyro[0];
    imu->gyro_y = (buf_gyro[3] << 8) | buf_gyro[2];
    imu->gyro_z = (buf_gyro[5] << 8) | buf_gyro[4];
}
