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
     * Get pointer to barometer poller - initialized in sensors_init().
     * All other sensor state (sample ring buffers, device configs) is
     * accessed via the globals declared in SPI_device_interactions.h.
     */
    ms5611_poller_t *baro_poller = sensors_get_baro_poller();

    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    uint32_t ISR_flags = 0;

    while (true) {
        xTaskNotifyWaitIndexed(0, 0, UINT32_MAX, &ISR_flags, period_ticks);
        if(ISR_flags != 0){

            if(ISR_flags & MS5611_BARO_SAMPLE_FLAG){

            }

            if(ISR_flags & BMI088_ACCEL_SAMPLE_FLAG){

            }

            if(ISR_flags & BMI088_GYRO_SAMPLE_FLAG){

            }

            // TODO: other sensor intakes

            state_t fused_state = {0};
            state_exchange_publish_state(&fused_state);

            flight_state_t flight_state;
            state_exchange_get_flight_state(&flight_state);
            log_service_log_state(&fused_state, flight_state);

        }

        /* Drive the barometer polling state machine */
        ms5611_poller_tick(baro_poller);
    }
}
