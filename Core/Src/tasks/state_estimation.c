/**
 * @file state_estimation.c
 * @brief State estimation task - sensor fusion and state publishing.
 *
 * This task:
 *   - Receives sensor data via task notifications from DMA callbacks
 *   - Consumes samples from sensor ring buffers
 *   - Runs sensor fusion algorithm (TODO)
 *   - Publishes fused state via state_exchange
 *   - Drives the barometer polling state machine
 *
 * Note: Sensor initialization is now done in sensors_init() before the
 * scheduler starts. This task only processes sensor data.
 *
 * UBC Rocket, Jan 2026
 */

#include <stdbool.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "spi_drivers/SPI_queue.h"
#include "spi_drivers/SPI_device_interactions.h"
#include "spi_drivers/ms5611_poller.h"
#include "spi_drivers/ms5607_poller.h"
#include "sensors_init.h"
#include "stm32h5xx_hal.h"
#include "main.h"
#include "state_exchange.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"

#define FUSION_VECTOR_SAMPLE_SIZE 32

void state_estimation_task_start(void *argument)
{
    (void)argument;

    /*
     * Get pointers to barometer pollers - initialized in sensors_init().
     * All other sensor state (sample ring buffers, device configs) is
     * accessed via the globals declared in SPI_device_interactions.h.
     */
    ms5611_poller_t *baro_poller = sensors_get_baro_poller();
    ms5607_poller_t *baro2_poller = sensors_get_baro2_poller();

    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    uint32_t ISR_flags = 0;

    while (true) {
        xTaskNotifyWaitIndexed(0, 0, UINT32_MAX, &ISR_flags, period_ticks);
        if(ISR_flags != 0){

            if(ISR_flags & MS5611_BARO_SAMPLE_FLAG){
                ms5611_sample_t baro_sample;
                while (ms5611_sample_dequeue(&ms5611_sample_ring, &baro_sample)) {
                    log_service_log_baro_sample(baro_sample.t_us,
                                                baro_sample.temp_centi,
                                                baro_sample.pressure_centi,
                                                baro_sample.seq);
                }
            }

            if(ISR_flags & MS5607_BARO2_SAMPLE_FLAG){
                ms5607_sample_t baro2_sample;
                while (ms5607_sample_dequeue(&ms5607_sample_ring, &baro2_sample)) {
                    log_service_log_baro2_sample(baro2_sample.t_us,
                                                 baro2_sample.temp_centi,
                                                 baro2_sample.pressure_centi,
                                                 baro2_sample.seq);
                }
            }

            if(ISR_flags & BMI088_ACCEL_SAMPLE_FLAG){
                bmi088_accel_sample_t accel_sample;
                while (bmi088_acc_sample_dequeue(&bmi088_acc_sample_ring, &accel_sample)) {
                    log_service_log_accel_sample((uint32_t)accel_sample.t_us,
                                                 accel_sample.ax,
                                                 accel_sample.ay,
                                                 accel_sample.az);
                }
            }

            if(ISR_flags & BMI088_GYRO_SAMPLE_FLAG){
                bmi088_gyro_sample_t gyro_sample;
                while (bmi088_gyro_sample_dequeue(&bmi088_gyro_sample_ring, &gyro_sample)) {
                    log_service_log_gyro_sample(gyro_sample.t_us,
                                                gyro_sample.gx,
                                                gyro_sample.gy,
                                                gyro_sample.gz);
                }
            }

            if(ISR_flags & ICM40609_IMU2_SAMPLE_FLAG){
                icm40609_sample_t imu2_sample;
                while (icm40609_sample_dequeue(&icm40609_sample_ring, &imu2_sample)) {
                    log_service_log_imu2_sample((uint32_t)imu2_sample.t_us,
                                                imu2_sample.ax, imu2_sample.ay, imu2_sample.az,
                                                imu2_sample.gx, imu2_sample.gy, imu2_sample.gz,
                                                (int16_t)(imu2_sample.temp_c * 100.0f));
                    /* TODO: Fuse IMU2 data into state estimate */
                }
            }

            // TODO: other sensor intakes

            state_t fused_state = {0};
            state_exchange_publish_state(&fused_state);

            flight_state_t flight_state;
            state_exchange_get_flight_state(&flight_state);
            log_service_log_state(&fused_state, flight_state);

        }

        /* Drive the barometer polling state machines */
        ms5611_poller_tick(baro_poller);
        ms5607_poller_tick(baro2_poller);
    }
}
