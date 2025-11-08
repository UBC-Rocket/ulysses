// gps.h
#pragma once
#include "stm32u5xx_hal.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef gps_init(UART_HandleTypeDef *huart_rx);
size_t            gps_read_line(uint8_t *dst, size_t maxlen);
void              gps_poll(void);

// Optional: user override to handle each valid NMEA sentence
void gps_on_sentence(const char* sentence, size_t len);

extern volatile uint32_t gps_debug_err_hits;
extern volatile uint32_t gps_debug_last_err;

#ifdef __cplusplus
}
#endif
