#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "ulysses/io/delay.h"

void io_spin_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
