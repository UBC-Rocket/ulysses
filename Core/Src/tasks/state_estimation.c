/**
 * @file state_estimation.c
 * @brief State estimation task - Merged EKF logic with Event-Driven Architecture
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
#include "debug/log.h"
#include "state_exchange.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"

#define GRAV 9.807f
#define FUSION_VECTOR_SAMPLE_SIZE 32
#define PI 3.1415f

float EXPECTED_GRAVITY[3] = {0, 0, 1};

// Math Helpers
void longlat_to_meters(float reference_point[3], float gps[3], float relative_distance[3]) {
    gps[2] = reference_point[2]; 
    float R = 6371000; 
    float delta_longlat[2] = {gps[0] - reference_point[0], gps[1] - reference_point[1]};
    relative_distance[0] = R * delta_longlat[0];
    relative_distance[1] = R * delta_longlat[1];
}

void quat_to_euler(float q[4], float e[3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    
    // x-axis
    float sinr = 2.0f * (w*x + y*z);
    float cosr = 1.0f - 2.0f * (x*x + y*y);
    e[0] = atan2f(sinr, cosr) * 180 / (float)M_PI;

    // y-axis
    float sinp = 2.0f * (w*y - z*x);
    if (fabsf(sinp) >= 1.0f)
        e[1] = copysignf((float)M_PI / 2.0f, sinp) * 180 / (float)M_PI;
    else
        e[1] = asinf(sinp) * 180 / (float)M_PI;

    // z-axis
    float siny = 2.0f * (w*z + x*y);
    float cosy = 1.0f - 2.0f * (y*y + z*z);
    e[2] = atan2f(siny, cosy) * 180.0f / (float)M_PI;
}

void update_bias(float g_bias[3], float g_data_raw[3], float a_bias[3], float a_data_raw[3], float expected_g[3], int64_t ticks)
{
    if (ticks == 0) {
        for (int i = 0; i < 3; i++) g_bias[i] = g_data_raw[i];
        for (int i = 0; i < 3; i++) a_bias[i] = a_data_raw[i] - expected_g[i];
    }
    for (int i = 0; i < 3; i++) {
        g_bias[i] = (g_bias[i]) * (((float)ticks - 1) / (float)ticks) + (g_data_raw[i]) * (1.0f / (float)ticks);
        a_bias[i] = (a_bias[i]) * (((float)ticks - 1) / (float)ticks) + (a_data_raw[i] - expected_g[i]) * (1.0f / (float)ticks);
    }
}

void state_estimation_task_start(void *argument)
{
    (void)argument;

    ms5611_poller_t *baro_poller = sensors_get_baro_poller();
    ms5607_poller_t *baro2_poller = sensors_get_baro2_poller();

    static bmi088_accel_sample_t accel_samples[FUSION_VECTOR_SAMPLE_SIZE];
    static bmi088_gyro_sample_t gyro_samples[FUSION_VECTOR_SAMPLE_SIZE];
    // static ms5611_sample_t baro_samples[FUSION_VECTOR_SAMPLE_SIZE]; 

    float process_noise_quaternion[4][4] = {{0.01, 0, 0, 0}, {0, 0.01, 0, 0}, {0, 0, 0.01, 0}, {0, 0, 0, 0.01}};
    float measurement_noise_quaternion[3][3] = {{0.001, 0, 0}, {0, 0.001, 0}, {0, 0, 0.001}};
    float process_noise_body[6][6] = {
        {0.01, 0, 0, 0, 0, 0}, {0, 0.01, 0, 0, 0, 0}, {0, 0, 0.01, 0, 0, 0},
        {0, 0, 0, 0.001, 0, 0}, {0, 0, 0, 0, 0.001, 0}, {0, 0, 0, 0, 0, 0.001}
    };
    float measurement_noise_body[3][3] = {{15, 0, 0}, {0, 15, 0}, {0, 0, 15}};

    init_ekf(process_noise_quaternion, measurement_noise_quaternion, process_noise_body, measurement_noise_body, EXPECTED_GRAVITY);

    float delta_time = 0;
    float last_tick = 0;
    uint64_t CALIBRATION = 2000;
    uint64_t ticks = 0;

    float accel_bias[3] = {0, 0, 0};
    float gyro_bias[3] = {0, 0, 0};
    // float gps_bias[3] = {0, 0, 0}; 
    // uint64_t gps_ticks = 0;
    // float gps_reference_point[3];
    // bool have_gps_reference_point = 0;

    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    uint32_t ISR_flags = 0;

    while (true) {
        // Event-driven wait (New Branch)
        xTaskNotifyWaitIndexed(0, 0, UINT32_MAX, &ISR_flags, period_ticks);

        // Counters for the current batch
        uint8_t num_accel_samples = 0;
        uint8_t num_gyro_samples = 0;
        uint8_t num_gps_samples = 0; 

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
                    // get baro...
                }
            }

            if (ISR_flags & MS5607_BARO2_SAMPLE_FLAG) {
                ms5607_sample_t baro2_sample;
                while (ms5607_sample_dequeue(&ms5607_sample_ring, &baro2_sample)) {
                    log_service_log_baro2_sample(baro2_sample.t_us, baro2_sample.temp_centi, baro2_sample.pressure_centi, baro2_sample.seq);
                    // get baro 2...
                }
            }

            uint8_t imu_loops = (num_accel_samples < num_gyro_samples) ? num_accel_samples : num_gyro_samples;
            uint8_t body_loops = (num_gps_samples > num_accel_samples) ? num_gps_samples : num_accel_samples; // Simplified, assumes 0 gps currently

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

                if (body_loops > i) {
                    tick_ekf_body(delta_time, a_data, (float[3]){0, 0, 0});
                }

                // Publish State
                float q[4], pos[3], vel[3];
                get_state(q, pos, vel);

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

                if (ticks % 40 == 0) {
                    // DLOG_PRINT("%f, %f, %f, %f]deg\n", q[0], q[1], q[2], q[3]);
                }
                ticks++;
            }
        }

        ms5611_poller_tick(baro_poller);
        ms5607_poller_tick(baro2_poller);
    }
}