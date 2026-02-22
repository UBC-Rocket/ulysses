#include "main.h"
#include "SEN0306_mmwave.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2; // For debug output

void mmwave_simple_test(void) {
    // Initialize the sensor
    sen0306_init(&huart4);
    
    char msg[100];
    
    while(1) {
        HAL_Delay(500);
        
        const sen0306_sample_t* sample = sen0306_get_latest();
        
        if (sample != NULL) {
            snprintf(msg, sizeof(msg), "Distance: %u cm @ %lu us\r\n", 
                    sample->distance_cm, sample->timestamp_us);
        } else {
            snprintf(msg, sizeof(msg), "No data\r\n");
        }
        
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        
        uint32_t rx, err;
        sen0306_get_stats(&rx, &err);
        snprintf(msg, sizeof(msg), "RX=%lu ERR=%lu\r\n\r\n", rx, err);
        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}