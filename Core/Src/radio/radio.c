// In Core/Src/Drivers/radio.c

#include "Drivers/radio.h"
#include "main.h"
#include <string.h> 

#define RX_BUFFER_SIZE 64 // maximum message size we can receive

// --- Private Variables ---
static UART_HandleTypeDef* radio_uart_handle = NULL;
static uint8_t rx_buffer[RX_BUFFER_SIZE]; 
volatile bool message_received = false;
volatile uint16_t received_message_length = 0; // Store exact length of received message


// --- Public Functions ---

void radio_init(UART_HandleTypeDef* huart) {
    radio_uart_handle = huart;
}


HAL_StatusTypeDef radio_send(uint8_t* data, uint16_t length) {
    if (radio_uart_handle == NULL) {
        return HAL_ERROR;
    }
    // This simple wait is acceptable for a bare-metal test.
    // In the final RTOS, this should be handled by a semaphore or task notification.
    while(HAL_UART_GetState(radio_uart_handle) == HAL_UART_STATE_BUSY_TX);
    return HAL_UART_Transmit_DMA(radio_uart_handle, data, length);
}


HAL_StatusTypeDef radio_start_listening(void) {
    if (radio_uart_handle == NULL) {
        return HAL_ERROR;
    }
    message_received = false;
    // non-blocking and returns immediately, DMA and UART5 listen in the background
    // copy the received bytes into our rx_buffer, 
    return HAL_UARTEx_ReceiveToIdle_DMA(radio_uart_handle, rx_buffer, RX_BUFFER_SIZE);
}


bool radio_is_new_message_available(void) {
    return message_received;
}


uint16_t radio_get_message(uint8_t* buffer, uint16_t max_length) {
    if (!message_received) {
        return 0;
    }
    
    // stops CPU from listening to hardware interrupts
    __disable_irq();
    
    // copy received message stored in rx_buffer into user buffer passed in
    uint16_t len = received_message_length;
    uint16_t copy_length = (len < max_length) ? len : max_length;
    memcpy(buffer, rx_buffer, copy_length);
    // reset the flag
    message_received = false;

    __enable_irq();

    // Re-start the listening process for the next message
    radio_start_listening();

    return copy_length;
}


// --- Interrupt and DMA Callbacks ---

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == radio_uart_handle->Instance) {
        // Turn LED2 ON to indicate a successful transmission.
        HAL_GPIO_WritePin(STAT_LED_2_GPIO_Port, STAT_LED_2_Pin, GPIO_PIN_SET);
    }
}


// predefined ISR for UART5
// size automatically set by hardware
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == radio_uart_handle->Instance) {
        received_message_length = Size;
        message_received = true;
        // Turn OFF LED2 to signal we are ready for a new transmission.
        HAL_GPIO_WritePin(STAT_LED_2_GPIO_Port, STAT_LED_2_Pin, GPIO_PIN_RESET);
    }
}
