#ifndef SEN0306_MMWAVE_H
#define SEN0306_MMWAVE_H

#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// 8-byte mode for Distance-Only
#define SEN0306_FRAME_SIZE 8

typedef struct {
    uint16_t distance_cm;
    uint32_t timestamp_us;
} sen0306_sample_t;

#define SEN0306_QUEUE_LEN 16

typedef struct {
    sen0306_sample_t samples[SEN0306_QUEUE_LEN];
    uint8_t head;
    uint8_t tail;
} sen0306_queue_t;

typedef struct {
    UART_HandleTypeDef *huart;
    bool initialized;
    uint8_t parser_buf[SEN0306_FRAME_SIZE];
    uint16_t parser_idx;
    uint32_t samples_received;
    uint32_t parse_errors;
    uint32_t queue_overflows;
    sen0306_queue_t queue;
} sen0306_context_t;

extern sen0306_context_t sen0306_ctx;

// Public API
void sen0306_init(UART_HandleTypeDef *huart);
void process_byte(uint8_t byte);
void sen0306_process_available_data(void);
const sen0306_sample_t* sen0306_get_latest(void);

#endif // SEN0306_MMWAVE_H