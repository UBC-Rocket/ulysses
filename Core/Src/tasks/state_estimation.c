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
#include "ekf.h"
#include "state_estimation/body.h"
#include "debug/log.h"
#include "state_exchange.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "spi_drivers/gnss_radio_master.h"
#include "debug/log.h"

#define GRAV 9.807f
#define FUSION_VECTOR_SAMPLE_SIZE 32
#define PI 3.1415f

float EXPECTED_GRAVITY[3] = {0, 0, 1};

typedef struct {
    float lat;
    float lon;
    float alt;
    uint8_t fix_quality;
    uint8_t num_sats;
} gps_data_t;

/* Math Helpers: convert lat/lon/alt to meters relative to reference. Does not mutate gps[]. */
void longlat_to_meters(const float reference_point[3], const float gps[3], float relative_distance[3]) {
    const float R = 6371000.0f;
    float delta_lat = gps[0] - reference_point[0];
    float delta_lon = gps[1] - reference_point[1];
    relative_distance[0] = R * delta_lat;
    relative_distance[1] = R * delta_lon;
    relative_distance[2] = gps[2] - reference_point[2];  /* altitude in meters */
}

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

        // Counters for the current batch
        uint8_t num_accel_samples = 0;
        uint8_t num_gyro_samples = 0;
        uint8_t num_gps_samples = 0;
        uint8_t num_baro1_samples = 0;
        uint8_t num_baro2_samples = 0;

        if (ISR_flags != 0) {            
            if (ISR_flags & BMI088_ACCEL_SAMPLE_FLAG) {
                bmi088_accel_sample_t accel_sample;
                while (bmi088_acc_sample_dequeue(&bmi088_acc_sample_ring, &accel_sample)) {
                    log_service_log_accel_sample((uint32_t)accel_sample.t_us, accel_sample.ax, accel_sample.ay, accel_sample.az);
                    if (num_accel_samples < FUSION_VECTOR_SAMPLE_SIZE) {
                        accel_samples[num_accel_samples++] = accel_sample;
                    }
                }
            }

            if (ISR_flags & BMI088_GYRO_SAMPLE_FLAG) {
                bmi088_gyro_sample_t gyro_sample;
                while (bmi088_gyro_sample_dequeue(&bmi088_gyro_sample_ring, &gyro_sample)) {
                    log_service_log_gyro_sample(gyro_sample.t_us, gyro_sample.gx, gyro_sample.gy, gyro_sample.gz);
                    if (num_gyro_samples < FUSION_VECTOR_SAMPLE_SIZE) {
                        gyro_samples[num_gyro_samples++] = gyro_sample;
                    }
                }
            }

            if (ISR_flags & MS5611_BARO_SAMPLE_FLAG) {
                ms5611_sample_t baro_sample;
                while (ms5611_sample_dequeue(&ms5611_sample_ring, &baro_sample)) {
                    log_service_log_baro_sample(baro_sample.t_us, baro_sample.temp_centi, baro_sample.pressure_centi, baro_sample.seq);
                    if (num_baro1_samples < FUSION_VECTOR_SAMPLE_SIZE) {
                        baro1_heights[num_baro1_samples++] = pressure_to_height(baro_sample.pressure_centi);
                    }
                }
            }

            if (ISR_flags & MS5607_BARO2_SAMPLE_FLAG) {
                ms5607_sample_t baro2_sample;
                while (ms5607_sample_dequeue(&ms5607_sample_ring, &baro2_sample)) {
                    log_service_log_baro2_sample(baro2_sample.t_us, baro2_sample.temp_centi, baro2_sample.pressure_centi, baro2_sample.seq);
                    if (num_baro2_samples < FUSION_VECTOR_SAMPLE_SIZE) {
                        baro2_heights[num_baro2_samples++] = pressure_to_height(baro2_sample.pressure_centi);
                    }
                }
            }


            gps_data_t current_gps;
            float pos_meters[3];
            bool have_pos_meters_this_cycle = false;

            if (ISR_flags & GNSS_GPS_FIX_READY_FLAG) {
                gnss_gps_fix_t gps_fix;
                while (gnss_gps_dequeue(&gps_fix)) {
                    //TODO: integrate into kalman properly
                }
            }

            uint8_t imu_loops = (num_accel_samples < num_gyro_samples) ? num_accel_samples : num_gyro_samples;

            float h1 = -1;
            float h2 = -1;

            for (int i = 0; i < num_baro1_samples; i++) {
                h1 = h1 * (i / (i + 1)) + baro1_heights[i] * 1 / (i + 1);
            }

            for (int i = 0; i < num_baro2_samples; i++) {
                h2 = h2 * (i / (i + 1)) + baro2_heights[i] * 1 / (i + 1);
            }

            for (uint8_t i = 0; i < imu_loops; i++) {
                float g_data_raw[3] = {gyro_samples[i].gx, gyro_samples[i].gy, gyro_samples[i].gz};
                float a_data_raw[3] = {accel_samples[i].ax / -GRAV, accel_samples[i].ay / -GRAV, accel_samples[i].az / -GRAV};

                if (last_tick != 0) {
                    delta_time = (gyro_samples[i].t_us - last_tick) / 1000000.0f;
                }
                last_tick = gyro_samples[i].t_us;

                

                // Calibration
                if (ticks < CALIBRATION) {
                    ticks++;
                    update_bias(gyro_bias, g_data_raw, accel_bias, a_data_raw, EXPECTED_GRAVITY, ticks);
                    continue;
                }

                // EKF Update
                float g_data[3] = {g_data_raw[0] - gyro_bias[0], g_data_raw[1] - gyro_bias[1], g_data_raw[2] - gyro_bias[2]};
                float a_data[3] = {a_data_raw[0] - accel_bias[0], a_data_raw[1] - accel_bias[1], a_data_raw[2] - accel_bias[2]};

                tick_ekf_orientation(delta_time, g_data, a_data);

                /* Body EKF: use nav-frame acceleration and only when we have valid GPS this cycle. */
                float q[4], pos[3], vel[3];
                get_state(q, pos, vel);
                float a_nav[3];
                transform_accel_data(a_data, q, a_nav);
                if (have_pos_meters_this_cycle) {
                    tick_ekf_body(delta_time, a_nav, pos_meters);
                }
                get_state(q, pos, vel);

                /* Publish State */

                float e[3];
                quat_to_euler(q,e);

                state_t data = {
                    .pos = {pos[0], pos[1], pos[2]},
                    .vel = {vel[0], vel[1], vel[2]},
                    .omega_b = {g_data[0], g_data[1], g_data[2]},
                    .q_bn = {.w=q[0], .x=q[1], .y=q[2], .z=q[3]},
                    .u_s = gyro_samples[i].t_us
                };
                
                state_exchange_publish_state(&data);
                
                // Flight State Logging
                flight_state_t flight_state;
                state_exchange_get_flight_state(&flight_state);
                log_service_log_state(&data, flight_state);

                if (ticks % 400 == 0) {
                    DLOG_PRINT("%f, %f, %f, %f]deg\n", q[0], q[1], q[2], q[3]);
                }
                ticks++;
            }
        }

        ms5611_poller_tick(baro_poller);
        ms5607_poller_tick(baro2_poller);
    }
}