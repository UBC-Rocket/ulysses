#include <stdbool.h>
#include "cmsis_os2.h"
#include "spi_drivers/SPI_queue.h"
#include "spi_drivers/SPI_device_interactions.h"
#include "stm32h5xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "algorithms/ekf.h"
#include "debug/log.h"

#define PI 3.141593
#define GRAV 9.807
#define FUSION_VECTOR_SAMPLE_SIZE 32

// External variable declarations
extern SPI_HandleTypeDef hspi2;
extern spi_job_queue_t jobq_spi_2;

extern bmi088_accel_t accel;
extern bmi088_gyro_t gyro;

static ms5611_poller_t baro_poll;

void quat_to_euler(float q[4], float e[3]) {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    // Roll (x-axis rotation)
    float sinr = 2.0f * (w*x + y*z);
    float cosr = 1.0f - 2.0f * (x*x + y*y);
    e[0] = atan2f(sinr, cosr) * 180 / PI;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w*y - z*x);
    if (fabsf(sinp) >= 1.0f)
        e[1] = copysignf(M_PI / 2.0f, sinp) * 180 / PI; // clamp at ±90°
    else
        e[1] = asinf(sinp) * 180 / PI;
}

void update_bias(float g_bias[3], float g_data_raw[3], float a_bias[3], float a_data_raw[3], int64_t ticks)
{
    if (ticks == 0) {
        for (int i = 0; i < 3; i++) g_bias[i] = g_data_raw[i];
        for (int i = 0; i < 2; i++) a_bias[i] = a_data_raw[i];

        a_bias[2] = a_data_raw[2] - 1;
    }

    for (int i = 0; i < 3; i++) {
        g_bias[i] = (g_bias[i]) * (((float)ticks - 1) / (float)ticks) + (g_data_raw[i]) * (1 / (float)ticks);
    }

    for (int i = 0; i < 2; i++) {
        a_bias[i] = (a_bias[i]) * (((float)ticks - 1) / (float)ticks) + (a_data_raw[i]) * (1 / (float)ticks);
    }

    a_bias[2] = (a_bias[2]) * (((float)ticks - 1) / (float)ticks) + (a_data_raw[2] - 1) * (1 / (float)ticks);
}

void sensor_fusion_task_start(void *argument)
{
    jobq_spi_2.spi_bus = &hspi2;
    jobq_spi_2.spi_busy = false;
    jobq_spi_2.head = 0;
    jobq_spi_2.tail = 0;
    jobq_spi_2.last_submit_status = HAL_OK;

    uint32_t primask = __get_PRIMASK();  // save current state
    __disable_irq();

    uint8_t accel_status = bmi088_accel_init(&hspi2, BMI_ACC_Chip_Select_GPIO_Port, BMI_ACC_Chip_Select_Pin, &accel);
    
    if(accel_status == 0){
        bmi088_acc_sample_ring.head = 0;
        bmi088_acc_sample_ring.tail = 0;
        bmi088_accel_ready = true;
    } else {
        bmi088_accel_ready = false;
    }

    uint8_t gyro_status = bmi088_gyro_init(&hspi2, BMI_GYRO_Chip_Select_GPIO_Port, BMI_GYRO_Chip_Select_Pin, &gyro);

    if(gyro_status == 0){
        bmi088_gyro_sample_ring.head = 0;
        bmi088_gyro_sample_ring.tail = 0;
        bmi088_gyro_ready = true;
    } else {
        bmi088_gyro_ready = false;
    }

    ms5611_poller_init(&baro_poll, &hspi2, BARO1_CS_GPIO_Port, BARO1_CS_Pin,
                       MS5611_OSR_4096, 50);

    ms5611_sample_ring.head = 0;
    ms5611_sample_ring.tail = 0;

    __set_PRIMASK(primask);

    static bmi088_accel_sample_t accel_samples[16];
    static bmi088_gyro_sample_t gyro_samples[16];
    static ms5611_sample_t baro_samples[16];
    const TickType_t period_ticks = pdMS_TO_TICKS(1);

    float q[4][4] = {{0.000001, 0, 0, 0},
                    {0, 0.000001, 0, 0},
                    {0, 0, 0.000001, 0},
                    {0, 0, 0, 0.000001}};

    float m[3][3] = {{0.01, 0, 0},
                    {0, 0.01, 0},
                    {0, 0, 0.01}};

    init_ekf(q, m);

    float delta_time = 0;
    float last_tick = 0;
    
    uint64_t CALIBRATION = 2000;
    uint64_t ticks = 0;

    float a_bias[3] = {0, 0, 0};
    float g_bias[3] = {0, 0, 0};

    while (true) {
        TickType_t cycle_start = xTaskGetTickCount();

        uint8_t num_accel_samples = 0;
        uint8_t num_gyro_samples = 0;
        uint8_t num_baro_samples = 0;

        bmi088_accel_sample_t accel_sample;
        bmi088_gyro_sample_t gyro_sample;
        ms5611_sample_t baro_sample;

        while(num_accel_samples < FUSION_VECTOR_SAMPLE_SIZE){
            if(!bmi088_acc_sample_dequeue(&bmi088_acc_sample_ring, &accel_sample)) break;
            
            accel_samples[num_accel_samples] = accel_sample;

            num_accel_samples++;
        }

        while(num_gyro_samples < FUSION_VECTOR_SAMPLE_SIZE){
            if(!bmi088_gyro_sample_dequeue(&bmi088_gyro_sample_ring, &gyro_sample)) break;
            
            gyro_samples[num_gyro_samples] = gyro_sample;

            num_gyro_samples++;
        }

        while(num_baro_samples < FUSION_VECTOR_SAMPLE_SIZE){
            if(!ms5611_sample_dequeue(&ms5611_sample_ring, &baro_sample)) break;

            baro_samples[num_baro_samples] = baro_sample;

            num_baro_samples++;
        }

        ms5611_poller_tick_1khz(&baro_poll);

        uint8_t loops = 0;
        if (num_accel_samples > num_gyro_samples) loops = num_gyro_samples;
        else loops = num_accel_samples;
        
        for (uint8_t i = 0; i < loops; i++) {
            float g_data_raw[3] = {gyro_samples[i].gx * PI / 180, gyro_samples[i].gy * PI / 180, gyro_samples[i].gz * PI / 180};
            float a_data_raw[3] = {accel_samples[i].ax / -GRAV, accel_samples[i].ay / -GRAV, accel_samples[i].az / -GRAV};
            // accel should read [0, 0, 1] when sitting stationary upright

            if (last_tick != 0) {
                delta_time = (gyro_samples[i].t_us - last_tick) / 1000000;
            }

            last_tick = gyro_samples[i].t_us;
            
            // Calibration
            if (ticks < CALIBRATION) {
                ticks += 1;
                update_bias(g_bias, g_data_raw, a_bias, a_data_raw, ticks);

                continue;
            }

            // Calibration finished
            float g_data[3] = {g_data_raw[0] - g_bias[0], 
                               g_data_raw[1] - g_bias[1], 
                               g_data_raw[2] - g_bias[2]};

            float a_data[3] = {a_data_raw[0] - a_bias[0], 
                               a_data_raw[1] - a_bias[1], 
                               a_data_raw[2] - a_bias[2]};

            tick_ekf(delta_time, g_data, a_data);

            // logging
            if (ticks % 200 == 0) {
                float q[4];
                get_state_x(q);

                float e[2];
                quat_to_euler(q, e);

                DLOG_PRINT("Pitch: %f\nRoll: %f\n", e[0], e[1]);
            }

            ticks += 1;
        }

        TickType_t elapsed = xTaskGetTickCount() - cycle_start;
        if (elapsed < period_ticks) {
            vTaskDelay(period_ticks - elapsed);
        } else {
            taskYIELD();
        }
    }
}