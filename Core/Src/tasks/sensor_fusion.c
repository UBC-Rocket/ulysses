#include <stdbool.h>
#include "cmsis_os2.h"
#include "spi_drivers/SPI_queue.h"
#include "spi_drivers/SPI_device_interactions.h"
#include "stm32h5xx_hal.h"
#include "main.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

#define FUSION_VECTOR_SAMPLE_SIZE 32

// External variable declarations
extern SPI_HandleTypeDef hspi2;
extern spi_job_queue_t jobq_spi_2;

extern bmi088_accel_t accel;
extern bmi088_gyro_t gyro;

static ms5611_poller_t baro_poll;

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

        TickType_t elapsed = xTaskGetTickCount() - cycle_start;
        if (elapsed < period_ticks) {
            vTaskDelay(period_ticks - elapsed);
        } else {
            taskYIELD();
        }
    }
}
