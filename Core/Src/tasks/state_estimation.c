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
#include <stdint.h>
#include "ekf.h"
#include "debug/log.h"
#include "state_exchange.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"
#include "SD_logging/log_service.h"
#include "SD_logging/log_service.h"

#define GRAV 9.807
#define FUSION_VECTOR_SAMPLE_SIZE 32

float EXPECTED_GRAVITY[3] = {1, 0, 0};

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
    e[0] = atan2f(sinr, cosr) * 180 / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w*y - z*x);
    if (fabsf(sinp) >= 1.0f)
        e[1] = copysignf(M_PI / 2.0f, sinp) * 180 / M_PI; // clamp at ±90°
    else
        e[1] = asinf(sinp) * 180 / M_PI;

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
        g_bias[i] = (g_bias[i]) * (((float)ticks - 1) / (float)ticks) + (g_data_raw[i]) * (1 / (float)ticks);
    }

    for (int i = 0; i < 3; i++) {
        a_bias[i] = (a_bias[i]) * (((float)ticks - 1) / (float)ticks) + (a_data_raw[i] - expected_g[i]) * (1 / (float)ticks);
    }
}

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

    float process_noise_quaternion[4][4] = 
                   {{0.000001, 0, 0, 0},
                    {0, 0.000001, 0, 0},
                    {0, 0, 0.000001, 0},
                    {0, 0, 0, 0.000001}};

    float measurement_noise_quaternion[3][3] = 
                   {{0.1, 0, 0},
                    {0, 0.1, 0},
                    {0, 0, 0.1}};

    float process_noise_body[6][6] = 
                   {{0.01, 0, 0, 0, 0, 0},
                    {0, 0.01, 0, 0, 0, 0},
                    {0, 0, 0.01, 0, 0, 0},
                    {0, 0, 0, 0.001, 0, 0},
                    {0, 0, 0, 0, 0.001, 0},
                    {0, 0, 0, 0, 0, 0.001}};

    float measurement_noise_body[3][3] = 
                   {{15, 0, 0},
                    {0, 15, 0},
                    {0, 0, 15}};

    init_ekf(process_noise_quaternion, 
            measurement_noise_quaternion,
            process_noise_body,
            measurement_noise_body,
            EXPECTED_GRAVITY);

    float delta_time = 0;
    float last_tick = 0;
    
    uint64_t CALIBRATION = 2000;
    uint64_t ticks = 0;

    float a_bias[3] = {0, 0, 0};
    float g_bias[3] = {0, 0, 0};

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

        state_t fused_state = {0};
        fused_state.u_s = timestamp_us();
        state_exchange_publish_state(&fused_state);

        flight_state_t flight_state;
        state_exchange_get_flight_state(&flight_state);
        log_service_log_state(&fused_state, flight_state);

        ms5611_poller_tick_1khz(&baro_poll);

        uint8_t loops = 0;
        if (num_accel_samples > num_gyro_samples) loops = num_gyro_samples;
        else loops = num_accel_samples;
        
        for (uint8_t i = 0; i < loops; i++) {
            float g_data_raw[3] = {gyro_samples[i].gx * M_PI / 180, gyro_samples[i].gy * M_PI / 180, gyro_samples[i].gz * M_PI / 180};
            float a_data_raw[3] = {accel_samples[i].ax / -GRAV, accel_samples[i].ay / -GRAV, accel_samples[i].az / -GRAV};
            // accel should read [0, 0, 1] when sitting stationary upright

            if (last_tick != 0) {
                delta_time = (gyro_samples[i].t_us - last_tick) / 1000000;
            }

            last_tick = gyro_samples[i].t_us;
            
            // Calibration
            if (ticks < CALIBRATION) {
                ticks += 1;
                update_bias(g_bias, g_data_raw, a_bias, a_data_raw, EXPECTED_GRAVITY, ticks);

                continue;
            }

            // Calibration finished
            float g_data[3] = {g_data_raw[0] - g_bias[0], 
                               g_data_raw[1] - g_bias[1], 
                               g_data_raw[2] - g_bias[2]};

            float a_data[3] = {a_data_raw[0] - a_bias[0], 
                               a_data_raw[1] - a_bias[1], 
                               a_data_raw[2] - a_bias[2]};

            tick_ekf(delta_time, g_data, a_data, (float[3]){0, 0, 0});

            float q[4];
            float pos[3];
            float vel[3];
            get_state(q, pos, vel);

            float e[3];
            quat_to_euler(q, e);
            quaternion_t new_quaternion = {
                .w = q[0],
                .x = q[1],
                .y = q[2],
                .z = q[3]
            };

            state_t data = {
                .pos = {0, 0, 0},
                .vel = {0, 0, 0},

                .omega_b = {g_data[0], g_data[1], g_data[2]},
                .q_bn = new_quaternion,

                .u_s = gyro_samples[i].t_us
            };

            state_exchange_publish_state(&data);

            // logging (optional)
            if (ticks % 100 == 0) {
                DLOG_PRINT("[%f, %f, %f]deg\n", e[0], e[1], e[2]);
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