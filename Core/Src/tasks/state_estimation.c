#include <stdbool.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "spi_drivers/SPI_queue.h"
#include "spi_drivers/SPI_device_interactions.h"
#include "stm32h5xx_hal.h"
#include "main.h"
#include "state_exchange.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "SD_logging/log_service.h"
#include "spi_drivers/SEN0306_mmwave.h"


#define FUSION_VECTOR_SAMPLE_SIZE 32

// External variable declarations
extern SPI_HandleTypeDef hspi2;
extern spi_job_queue_t jobq_spi_2;

extern bmi088_accel_t accel;
extern bmi088_gyro_t gyro;

static ms5611_poller_t baro_poll;

extern UART_HandleTypeDef huart4;

void state_estimation_task_start(void *argument)
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

    sen0306_init(&huart4);

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

        uint64_t cycle_start = xTaskGetTickCount(); 

        uint8_t num_accel_samples = 0;
        bmi088_accel_sample_t accel_sample;

        while(num_accel_samples < FUSION_VECTOR_SAMPLE_SIZE){
            if(!bmi088_acc_sample_dequeue(&bmi088_acc_sample_ring, &accel_sample)) break;

            accel_samples[num_accel_samples] = accel_sample;
            log_service_log_accel_sample(accel_sample.t_us,
                                         accel_sample.ax,
                                         accel_sample.ay,
                                         accel_sample.az);
                                         
            num_accel_samples++;
        }

        uint8_t num_gyro_samples = 0;
        bmi088_gyro_sample_t gyro_sample;

        while(num_gyro_samples < FUSION_VECTOR_SAMPLE_SIZE){
            if(!bmi088_gyro_sample_dequeue(&bmi088_gyro_sample_ring, &gyro_sample)) break;

            gyro_samples[num_gyro_samples] = gyro_sample;
            log_service_log_gyro_sample(gyro_sample.t_us,
                                        gyro_sample.gx,
                                        gyro_sample.gy,
                                        gyro_sample.gz);
            num_gyro_samples++;
        }

        uint8_t num_baro_samples = 0;
        ms5611_sample_t baro_sample;

        while(num_baro_samples < FUSION_VECTOR_SAMPLE_SIZE){
            if(!ms5611_sample_dequeue(&ms5611_sample_ring, &baro_sample)) break;

            baro_samples[num_baro_samples] = baro_sample;
            log_service_log_baro_sample(baro_sample.t_us,
                                        baro_sample.temp_centi,
                                        baro_sample.pressure_centi,
                                        baro_sample.seq);
            num_baro_samples++;
        }

        // --- NEW: Process Radar Data ---
        // 1. Get pointer to the absolute latest data from the circular buffer
        const sen0306_sample_t *radar_sample = sen0306_get_latest();
        
        // 2. Variable to ensure we don't process the same sample twice
        static uint32_t last_radar_timestamp = 0;
        
        uint16_t current_radar_dist = 0; // Default or hold value

        if (radar_sample != NULL) {
            // Check if this is a NEW sample (timestamp changed)
            if (radar_sample->timestamp_us != last_radar_timestamp) {
                last_radar_timestamp = radar_sample->timestamp_us;
                
                // Optional: Log it if you have a log function for it
                // log_service_log_radar(radar_sample->distance_cm, radar_sample->timestamp_ms);
            }
            
            // 3. Keep the latest distance for fusion
            current_radar_dist = radar_sample->distance_cm;
        }

        state_t fused_state = {0};
        fused_state.u_s = timestamp_us();
        state_exchange_publish_state(&fused_state);

        flight_state_t flight_state;
        state_exchange_get_flight_state(&flight_state);
        log_service_log_state(&fused_state, flight_state);

        ms5611_poller_tick_1khz(&baro_poll);

        TickType_t elapsed = xTaskGetTickCount() - cycle_start;
        if (elapsed < period_ticks) {
            vTaskDelay(period_ticks - elapsed);
        } else {
            taskYIELD();
        }
    }
}
