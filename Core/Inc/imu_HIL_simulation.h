#ifndef IMU_HIL_SIMULATION_H_
#define IMU_HIL_SIMULATION_H_

#include <stdint.h>
#include "stm32u5xx_hal.h"
#include "main.h"

void config_imu_HIL(SPI_HandleTypeDef* SPI_Handle);
void update_imu_HIL_stream();

typedef struct IMU_HIL {

    uint32_t config_flags;
    uint32_t status_flags;

    SPI_HandleTypeDef* bus_handle;

    int* acc_z;
    int* acc_y;
    int* acc_x;

    uint8_t acc_iterator;
    uint8_t acc_buffer_full;

    int* gyro_z;
    int* gyro_y;
    int* gyro_x;

    // Note: Using on-demand conversion instead of pre-calculated arrays

    int* sensor_time;

    uint8_t gyro_iterator;
    uint8_t gyro_buffer_full;

} IMU_HIL;