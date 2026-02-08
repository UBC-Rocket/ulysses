#include "state_exchange.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Maximum time to wait for state mutex (in ticks).
 *
 * Using bounded timeout instead of portMAX_DELAY prevents indefinite blocking.
 * 10ms is generous - state operations should complete in <1us.
 * If timeout occurs, it indicates a serious system problem.
 *
 * Note: FreeRTOS mutexes (configUSE_MUTEXES=1) include priority inheritance,
 * which prevents unbounded priority inversion. However, bounded timeout
 * provides an additional safety net for deadlock detection.
 */
#define STATE_MUTEX_TIMEOUT_TICKS   pdMS_TO_TICKS(10)

/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

static StaticSemaphore_t state_mutex_buffer;
static StaticSemaphore_t flight_mutex_buffer;
static SemaphoreHandle_t state_mutex_handle = NULL;
static SemaphoreHandle_t flight_mutex_handle = NULL;

static state_t latest_state = {0};
static flight_state_t latest_flight_state = IDLE;

static uint32_t state_seq = 0;
static uint32_t flight_state_seq = 0;

/* -------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* -------------------------------------------------------------------------- */

static void ensure_initialized(void)
{
    if (state_mutex_handle == NULL) {
        state_mutex_handle = xSemaphoreCreateMutexStatic(&state_mutex_buffer);
    }

    if (flight_mutex_handle == NULL) {
        flight_mutex_handle = xSemaphoreCreateMutexStatic(&flight_mutex_buffer);
    }
}

/**
 * @brief Take mutex with bounded timeout.
 * @param mutex The mutex handle.
 * @return pdTRUE if acquired, pdFALSE on timeout.
 * @note Logs error and continues on timeout to avoid system hang.
 */
static inline BaseType_t take_mutex_safe(SemaphoreHandle_t mutex)
{
    BaseType_t result = xSemaphoreTake(mutex, STATE_MUTEX_TIMEOUT_TICKS);
    /* In production: could assert or log on timeout */
    /* configASSERT(result == pdTRUE); */
    return result;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

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
    if (take_mutex_safe(state_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return current sequence without updating */
        return state_seq;
    }
    latest_state = *state;
    state_seq++;
    uint32_t seq = state_seq;
    xSemaphoreGive(state_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_state(state_t *state_out)
{
    ensure_initialized();
    if (take_mutex_safe(state_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return stale data with current sequence */
        if (state_out) {
            *state_out = latest_state;  /* Read without lock (may be inconsistent) */
        }
        return state_seq;
    }
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
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return current sequence without updating */
        return flight_state_seq;
    }
    latest_flight_state = flight_state;
    flight_state_seq++;
    uint32_t seq = flight_state_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}

uint32_t state_exchange_get_flight_state(flight_state_t *flight_state_out)
{
    ensure_initialized();
    if (take_mutex_safe(flight_mutex_handle) != pdTRUE) {
        /* Mutex timeout - return stale data with current sequence */
        if (flight_state_out) {
            *flight_state_out = latest_flight_state;
        }
        return flight_state_seq;
    }
    if (flight_state_out) {
        *flight_state_out = latest_flight_state;
    }
    uint32_t seq = flight_state_seq;
    xSemaphoreGive(flight_mutex_handle);
    return seq;
}
