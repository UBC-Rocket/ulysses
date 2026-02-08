/**
 * @file    state_exchange.c
 * @brief   Thread-safe exchange for state, flight_state, and control_output (mutex + latest copy).
 */
#include "state_exchange.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static StaticSemaphore_t state_mutex_buffer;
static StaticSemaphore_t flight_mutex_buffer;
static StaticSemaphore_t control_mutex_buffer;
static SemaphoreHandle_t state_mutex_handle = NULL;
static SemaphoreHandle_t flight_mutex_handle = NULL;
static SemaphoreHandle_t control_mutex_handle = NULL;

static state_t latest_state = {0};
static flight_state_t latest_flight_state = IDLE;
static control_output_t latest_control_output = {0};

static uint32_t state_seq = 0;
static uint32_t flight_state_seq = 0;
static uint32_t control_seq = 0;

static void ensure_initialized(void)
{
    if (state_mutex_handle == NULL) {
        state_mutex_handle = xSemaphoreCreateMutexStatic(&state_mutex_buffer);
    }

    if (flight_mutex_handle == NULL) {
        flight_mutex_handle = xSemaphoreCreateMutexStatic(&flight_mutex_buffer);
    }

    if (control_mutex_handle == NULL) {
        control_mutex_handle = xSemaphoreCreateMutexStatic(&control_mutex_buffer);
    }
}

void state_exchange_init(void)
{
    taskENTER_CRITICAL();
    ensure_initialized();
    taskEXIT_CRITICAL();
}

uint32_t state_exchange_publish_state(const state_t *state)
{
    if (!state) return state_seq;

    ensure_initialized();
    xSemaphoreTake(state_mutex_handle, portMAX_DELAY);
    latest_state = *state;
    state_seq++;
    uint32_t seq = state_seq;
    xSemaphoreGive(state_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_state(state_t *state_out)
{
    ensure_initialized();
    xSemaphoreTake(state_mutex_handle, portMAX_DELAY);
    if (state_out) {
        *state_out = latest_state;
    }
    uint32_t seq = state_seq;
    xSemaphoreGive(state_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_flight_state(flight_state_t flight_state)
{
    ensure_initialized();
    xSemaphoreTake(flight_mutex_handle, portMAX_DELAY);
    latest_flight_state = flight_state;
    flight_state_seq++;
    uint32_t seq = flight_state_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_flight_state(flight_state_t *flight_state_out)
{
    ensure_initialized();
    xSemaphoreTake(flight_mutex_handle, portMAX_DELAY);
    if (flight_state_out) {
        *flight_state_out = latest_flight_state;
    }
    uint32_t seq = flight_state_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_publish_control_output(const control_output_t *out)
{
    if (!out) return control_seq;

    ensure_initialized();
    xSemaphoreTake(control_mutex_handle, portMAX_DELAY);
    latest_control_output = *out;
    control_seq++;
    uint32_t seq = control_seq;
    xSemaphoreGive(control_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_control_output(control_output_t *out)
{
    ensure_initialized();
    xSemaphoreTake(control_mutex_handle, portMAX_DELAY);
    if (out) {
        *out = latest_control_output;
    }
    uint32_t seq = control_seq;
    xSemaphoreGive(control_mutex_handle);
    return seq;
}
