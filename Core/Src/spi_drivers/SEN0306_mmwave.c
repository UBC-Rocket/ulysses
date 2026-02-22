#include "SEN0306_mmwave.h"
#include <string.h>

// Link to the hardware ring buffer defined in main.c
#define RING_BUF_SIZE 512
extern uint8_t ring_buf[RING_BUF_SIZE];
extern volatile uint16_t head;
extern volatile uint16_t tail;

// Global Context Instance
sen0306_context_t sen0306_ctx;

void sen0306_init(UART_HandleTypeDef *huart) {
    memset(&sen0306_ctx, 0, sizeof(sen0306_context_t));
    sen0306_ctx.huart = huart;
    sen0306_ctx.initialized = true;
    // Note: Hardware listening is now started in main.c
}

void process_byte(uint8_t byte) {
    // 1. Prevent buffer overflow
    if (sen0306_ctx.parser_idx >= SEN0306_FRAME_SIZE) {
        sen0306_ctx.parser_idx = 0;
    }

    sen0306_ctx.parser_buf[sen0306_ctx.parser_idx++] = byte;

    // 2. Align to the 0xFF 0xFF 0xFF header
    if (sen0306_ctx.parser_idx >= 3) {
        if (sen0306_ctx.parser_buf[0] != 0xFF || 
            sen0306_ctx.parser_buf[1] != 0xFF || 
            sen0306_ctx.parser_buf[2] != 0xFF) {
            // Not a header. Shift left and keep looking.
            memmove(sen0306_ctx.parser_buf, &sen0306_ctx.parser_buf[1], --sen0306_ctx.parser_idx);
            return;
        }
    }

    // 3. We have a full 8-byte frame. Validate the Tail.
    if (sen0306_ctx.parser_idx == SEN0306_FRAME_SIZE) {
        if (sen0306_ctx.parser_buf[5] == 0x00 && 
            sen0306_ctx.parser_buf[6] == 0x00 && 
            sen0306_ctx.parser_buf[7] == 0x00) {
            
            // Valid Frame: Extract Distance
            uint16_t dist = ((uint16_t)sen0306_ctx.parser_buf[3] << 8) | sen0306_ctx.parser_buf[4];

            // Add to circular queue
            uint8_t next_head = (sen0306_ctx.queue.head + 1) % SEN0306_QUEUE_LEN;
            sen0306_ctx.queue.samples[sen0306_ctx.queue.head].distance_cm = dist;
            sen0306_ctx.queue.samples[sen0306_ctx.queue.head].timestamp_us = HAL_GetTick();
            
            if (next_head != sen0306_ctx.queue.tail) {
                sen0306_ctx.queue.head = next_head;
            } else {
                sen0306_ctx.queue.tail = (sen0306_ctx.queue.tail + 1) % SEN0306_QUEUE_LEN;
                sen0306_ctx.queue.head = next_head;
                sen0306_ctx.queue_overflows++;
            }

            sen0306_ctx.samples_received++;
        } else {
            // Bad frame tail
            sen0306_ctx.parse_errors++;
        }
        
        // Reset parser for the next frame
        sen0306_ctx.parser_idx = 0; 
    }
}

// Drains the hardware buffer and feeds it to the parser
void sen0306_process_available_data(void) {
    while (tail != head) {
        uint8_t byte = ring_buf[tail];
        tail = (tail + 1) % RING_BUF_SIZE;
        process_byte(byte); 
    }
}

const sen0306_sample_t* sen0306_get_latest(void) {
    if (sen0306_ctx.queue.head == sen0306_ctx.queue.tail) return NULL;
    uint8_t latest_idx = (sen0306_ctx.queue.head == 0) ? (SEN0306_QUEUE_LEN - 1) : (sen0306_ctx.queue.head - 1);
    return &sen0306_ctx.queue.samples[latest_idx];
}