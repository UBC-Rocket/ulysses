#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include "ulysses/io/delay.h"

void io_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // One extra tick so the wait is never shorter than requested
        vTaskDelay(pdMS_TO_TICKS(ms) + 1);
    } else {
        HAL_Delay(ms);
    }
}
