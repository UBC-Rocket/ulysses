#include <stdbool.h>
#include "cmsis_os2.h"
#include "state_exchange.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h5xx_hal.h"
#include "state_estimation/state.h"
#include "mission_manager/mission_manager.h"

void controls_task_start(void *argument)
{
    (void)argument;

    state_t current_state = {0};
    flight_state_t flight_state = IDLE;

    for (;;) {
        // Wait for timer notification from TIM3 OC interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        state_exchange_get_state(&current_state);
        state_exchange_get_flight_state(&flight_state);

        // TODO: controls

        // TODO: actuator drivers esc, servos
    }
}
