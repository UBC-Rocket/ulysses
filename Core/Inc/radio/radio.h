// In Core/Inc/drivers/radio.h

#ifndef RADIO_H
#define RADIO_H

#include "stm32u5xx_hal.h"
#include <stdbool.h> 


void radio_init(UART_HandleTypeDef* huart);

HAL_StatusTypeDef radio_send(uint8_t* data, uint16_t length);

/**
 * @brief Starts a non-blocking DMA receive for a specified number of bytes.
 *        This function should be called once to start the listening process.
 *        Data is received in the background.
 */
HAL_StatusTypeDef radio_start_listening(void);

/**
 * @brief Checks if a new message has been received.
 * @retval bool true if a new message is ready, false otherwise.
 */
bool radio_is_new_message_available(void);

/**
 * @brief Gets the last received message and re-starts the listening process.
 * @param buffer A buffer to copy the received message into.
 * @param max_length The size of the provided buffer.
 * @return uint16_t The number of bytes in the received message.
 */
uint16_t radio_get_message(uint8_t* buffer, uint16_t max_length);

#endif // RADIO_H