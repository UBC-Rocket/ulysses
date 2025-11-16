#ifndef IMU_HIL_SIMULATION_H_
#define IMU_HIL_SIMULATION_H_

#include <stdint.h>
#include "stm32u5xx_hal.h"
#include "main.h"

#define CS_ACC_PIN   GPIO_PIN_4
#define CS_ACC_PORT  GPIOA
#define CS_GYR_PIN   GPIO_PIN_5
#define CS_GYR_PORT  GPIOA

void config_imu_HIL(SPI_HandleTypeDef* SPI_Handle);
void update_imu_HIL_stream();

typedef struct IMU_HIL_t {

    SPI_HandleTypeDef* bus_handle;

    int acc_z;
    int acc_y;
    int acc_x;

    int gyro_z;
    int gyro_y;
    int gyro_x;

} IMU_HIL_t;