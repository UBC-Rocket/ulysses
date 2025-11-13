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

#ifdef DEBUG
#include "debug_uart.h"
#endif

#define FUSION_VECTOR_SAMPLE_SIZE 32

// External variable declarations
extern SPI_HandleTypeDef hspi2;
extern spi_job_queue_t jobq_spi_2;

extern bmi088_accel_t accel;
extern bmi088_gyro_t gyro;

static ms5611_poller_t baro_poll;

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

#ifdef DEBUG
        {
            static const uint8_t max_print = 4;
            char line[160];

            uint8_t accel_print = num_accel_samples < max_print ? num_accel_samples : max_print;
            for (uint8_t i = 0; i < accel_print; ++i) {
                const bmi088_accel_sample_t *sample = &accel_samples[i];
                uint32_t t_us = (uint32_t)(sample->t_us & 0xFFFFFFFFu);
                int32_t ax_milli = (int32_t)(sample->ax * 1000.0f);
                int32_t ay_milli = (int32_t)(sample->ay * 1000.0f);
                int32_t az_milli = (int32_t)(sample->az * 1000.0f);
                int32_t ax_frac = ax_milli % 1000;
                int32_t ay_frac = ay_milli % 1000;
                int32_t az_frac = az_milli % 1000;
                if (ax_frac < 0) ax_frac = -ax_frac;
                if (ay_frac < 0) ay_frac = -ay_frac;
                if (az_frac < 0) az_frac = -az_frac;
                int len = snprintf(line, sizeof(line),
                                   "ACC t=%lu ax=%ld.%03ld ay=%ld.%03ld az=%ld.%03ld\r\n",
                                   (unsigned long)t_us,
                                   (long)(ax_milli / 1000), (long)ax_frac,
                                   (long)(ay_milli / 1000), (long)ay_frac,
                                   (long)(az_milli / 1000), (long)az_frac);
                if (len > 0) {
                    debug_uart_write((const uint8_t *)line, (size_t)len);
                }
            }

            uint8_t gyro_print = num_gyro_samples < max_print ? num_gyro_samples : max_print;
            for (uint8_t i = 0; i < gyro_print; ++i) {
                const bmi088_gyro_sample_t *sample = &gyro_samples[i];
                uint32_t t_us = sample->t_us;
                int32_t gx_milli = (int32_t)(sample->gx * 1000.0f);
                int32_t gy_milli = (int32_t)(sample->gy * 1000.0f);
                int32_t gz_milli = (int32_t)(sample->gz * 1000.0f);
                int32_t gx_frac = gx_milli % 1000;
                int32_t gy_frac = gy_milli % 1000;
                int32_t gz_frac = gz_milli % 1000;
                if (gx_frac < 0) gx_frac = -gx_frac;
                if (gy_frac < 0) gy_frac = -gy_frac;
                if (gz_frac < 0) gz_frac = -gz_frac;
                int len = snprintf(line, sizeof(line),
                                   "GYR t=%lu gx=%ld.%03ld gy=%ld.%03ld gz=%ld.%03ld\r\n",
                                   (unsigned long)t_us,
                                   (long)(gx_milli / 1000), (long)gx_frac,
                                   (long)(gy_milli / 1000), (long)gy_frac,
                                   (long)(gz_milli / 1000), (long)gz_frac);
                if (len > 0) {
                    debug_uart_write((const uint8_t *)line, (size_t)len);
                }
            }

            uint8_t baro_print = num_baro_samples < max_print ? num_baro_samples : max_print;
            for (uint8_t i = 0; i < baro_print; ++i) {
                const ms5611_sample_t *sample = &baro_samples[i];
                int32_t temp_frac = sample->temp_centi % 100;
                int32_t press_frac = sample->pressure_centi % 100;
                if (temp_frac < 0) temp_frac = -temp_frac;
                if (press_frac < 0) press_frac = -press_frac;
                int len = snprintf(line, sizeof(line),
                                   "BAR t=%lu temp=%ld.%02ldC press=%ld.%02ldhPa\r\n",
                                   (unsigned long)sample->t_us,
                                   (long)(sample->temp_centi / 100), (long)temp_frac,
                                   (long)(sample->pressure_centi / 100), (long)press_frac);
                if (len > 0) {
                    debug_uart_write((const uint8_t *)line, (size_t)len);
                }
            }
        }
#endif

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
