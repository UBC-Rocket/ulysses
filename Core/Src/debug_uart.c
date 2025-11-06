#ifdef DEBUG

#include "debug_uart.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "main.h"
#include <string.h>

#define DEBUG_UART_QUEUE_LENGTH        512U
#define DEBUG_UART_TX_CHUNK_SIZE       128U
#define DEBUG_UART_TASK_STACK_WORDS    256U

extern UART_HandleTypeDef huart1;

static StaticQueue_t debug_uart_queue_struct;
static uint8_t debug_uart_queue_storage[DEBUG_UART_QUEUE_LENGTH];
static QueueHandle_t debug_uart_queue;

static StaticSemaphore_t debug_uart_mutex_struct;
static SemaphoreHandle_t debug_uart_mutex;

static StaticSemaphore_t debug_uart_tx_done_struct;
static SemaphoreHandle_t debug_uart_tx_done;

static StaticTask_t debug_uart_task_tcb;
static StackType_t debug_uart_task_stack[DEBUG_UART_TASK_STACK_WORDS];
static TaskHandle_t debug_uart_task_handle;

static uint8_t debug_uart_tx_buffer[DEBUG_UART_TX_CHUNK_SIZE];

static void debug_uart_task(void *argument);
static void debug_uart_flush_buffer(size_t length);

void debug_uart_init(void)
{
    if (debug_uart_queue) {
        return;
    }

    debug_uart_queue = xQueueCreateStatic(
        DEBUG_UART_QUEUE_LENGTH,
        sizeof(uint8_t),
        debug_uart_queue_storage,
        &debug_uart_queue_struct);

    debug_uart_mutex = xSemaphoreCreateMutexStatic(&debug_uart_mutex_struct);
    debug_uart_tx_done = xSemaphoreCreateBinaryStatic(&debug_uart_tx_done_struct);

    configASSERT(debug_uart_queue != NULL);
    configASSERT(debug_uart_mutex != NULL);
    configASSERT(debug_uart_tx_done != NULL);

    debug_uart_task_handle = xTaskCreateStatic(
        debug_uart_task,
        "DbgUart",
        DEBUG_UART_TASK_STACK_WORDS,
        NULL,
        tskIDLE_PRIORITY + 2,
        debug_uart_task_stack,
        &debug_uart_task_tcb);

    configASSERT(debug_uart_task_handle != NULL);
}

void debug_uart_write(const uint8_t *data, size_t length)
{
    if (!debug_uart_queue || data == NULL || length == 0U) {
        return;
    }

    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        xQueueSend(debug_uart_queue, &byte, portMAX_DELAY);
    }
}

void debug_uart_write_string(const char *str)
{
    if (!str) {
        return;
    }
    debug_uart_write((const uint8_t *)str, strlen(str));
}

static void debug_uart_task(void *argument)
{
    (void)argument;

    for (;;) {
        size_t bytes_collected = 0U;
        uint8_t byte;

        if (xQueueReceive(debug_uart_queue, &byte, portMAX_DELAY) == pdPASS) {
            debug_uart_tx_buffer[bytes_collected++] = byte;

            while ((bytes_collected < DEBUG_UART_TX_CHUNK_SIZE) &&
                   (xQueueReceive(debug_uart_queue, &byte, 0U) == pdPASS)) {
                debug_uart_tx_buffer[bytes_collected++] = byte;
            }

            debug_uart_flush_buffer(bytes_collected);
        }

        taskYIELD();
    }
}

static void debug_uart_flush_buffer(size_t length)
{
    if ((length == 0U) || (debug_uart_mutex == NULL)) {
        return;
    }

    if (xSemaphoreTake(debug_uart_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    if (xSemaphoreTake(debug_uart_tx_done, 0U) == pdTRUE) {
        /* Ensure semaphore is empty before starting a new transfer. */
    }

    if (HAL_UART_Transmit_DMA(&huart1, debug_uart_tx_buffer, (uint16_t)length) == HAL_OK) {
        xSemaphoreTake(debug_uart_tx_done, portMAX_DELAY);
    }

    xSemaphoreGive(debug_uart_mutex);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1) && (debug_uart_tx_done != NULL)) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(debug_uart_tx_done, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

#endif /* DEBUG */
