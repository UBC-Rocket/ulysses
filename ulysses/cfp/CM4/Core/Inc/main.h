/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADIO_NRST_Pin GPIO_PIN_13
#define RADIO_NRST_GPIO_Port GPIOC
#define RADIO_BUSY_Pin GPIO_PIN_14
#define RADIO_BUSY_GPIO_Port GPIOC
#define RADIO_NSS_Pin GPIO_PIN_6
#define RADIO_NSS_GPIO_Port GPIOF
#define RADIO_SCK_Pin GPIO_PIN_7
#define RADIO_SCK_GPIO_Port GPIOF
#define RADIO_MISO_Pin GPIO_PIN_8
#define RADIO_MISO_GPIO_Port GPIOF
#define RADIO_MOSI_Pin GPIO_PIN_9
#define RADIO_MOSI_GPIO_Port GPIOF
#define STAT_LEDG_Pin GPIO_PIN_10
#define STAT_LEDG_GPIO_Port GPIOF
#define RTK_GNSS_INT_Pin GPIO_PIN_0
#define RTK_GNSS_INT_GPIO_Port GPIOC
#define SDCARD_CK_Pin GPIO_PIN_1
#define SDCARD_CK_GPIO_Port GPIOC
#define RTK_CORRECT_TX_Pin GPIO_PIN_2
#define RTK_CORRECT_TX_GPIO_Port GPIOA
#define RTK_CORRECT_RX_Pin GPIO_PIN_3
#define RTK_CORRECT_RX_GPIO_Port GPIOA
#define BUZZER_PWM_Pin GPIO_PIN_6
#define BUZZER_PWM_GPIO_Port GPIOA
#define STAT_LEDR_Pin GPIO_PIN_11
#define STAT_LEDR_GPIO_Port GPIOF
#define BMS_SCL_Pin GPIO_PIN_14
#define BMS_SCL_GPIO_Port GPIOF
#define BMS_SDA_Pin GPIO_PIN_15
#define BMS_SDA_GPIO_Port GPIOF
#define SD_CARD_DETECT_Pin GPIO_PIN_13
#define SD_CARD_DETECT_GPIO_Port GPIOE
#define SDCARD_D0_Pin GPIO_PIN_14
#define SDCARD_D0_GPIO_Port GPIOB
#define SDCARD_D1_Pin GPIO_PIN_15
#define SDCARD_D1_GPIO_Port GPIOB
#define RTK_GNSS_TX_Pin GPIO_PIN_8
#define RTK_GNSS_TX_GPIO_Port GPIOD
#define RTK_GNSS_RX_Pin GPIO_PIN_9
#define RTK_GNSS_RX_GPIO_Port GPIOD
#define GATE_DRIVER1_nFAULT_Pin GPIO_PIN_12
#define GATE_DRIVER1_nFAULT_GPIO_Port GPIOD
#define GATE_DRIVER2_nFAULT_Pin GPIO_PIN_13
#define GATE_DRIVER2_nFAULT_GPIO_Port GPIOD
#define GATE_DRIVER1_CS_Pin GPIO_PIN_7
#define GATE_DRIVER1_CS_GPIO_Port GPIOG
#define GATE_DRIVER2_CS_Pin GPIO_PIN_8
#define GATE_DRIVER2_CS_GPIO_Port GPIOG
#define ESC_MCU1_TX_Pin GPIO_PIN_6
#define ESC_MCU1_TX_GPIO_Port GPIOC
#define ESC_MCU1_RX_Pin GPIO_PIN_7
#define ESC_MCU1_RX_GPIO_Port GPIOC
#define FUEL_GAUGE_SDA_Pin GPIO_PIN_9
#define FUEL_GAUGE_SDA_GPIO_Port GPIOC
#define FUEL_GAUGE_SCL_Pin GPIO_PIN_8
#define FUEL_GAUGE_SCL_GPIO_Port GPIOA
#define RFD_TX_Pin GPIO_PIN_9
#define RFD_TX_GPIO_Port GPIOA
#define RFD_RX_Pin GPIO_PIN_10
#define RFD_RX_GPIO_Port GPIOA
#define RFD_CTS_Pin GPIO_PIN_11
#define RFD_CTS_GPIO_Port GPIOA
#define RFD_RTS_Pin GPIO_PIN_12
#define RFD_RTS_GPIO_Port GPIOA
#define RTK_GNSS_NRST_Pin GPIO_PIN_3
#define RTK_GNSS_NRST_GPIO_Port GPIOD
#define SDCARD_CMD_Pin GPIO_PIN_7
#define SDCARD_CMD_GPIO_Port GPIOD
#define SDCARD_D2_Pin GPIO_PIN_11
#define SDCARD_D2_GPIO_Port GPIOG
#define GATE_DRIVER_MISO_Pin GPIO_PIN_12
#define GATE_DRIVER_MISO_GPIO_Port GPIOG
#define GATE_DRIVER_SCK_Pin GPIO_PIN_13
#define GATE_DRIVER_SCK_GPIO_Port GPIOG
#define GATE_DRIVER_MOSI_Pin GPIO_PIN_14
#define GATE_DRIVER_MOSI_GPIO_Port GPIOG
#define SDCARD_D3_Pin GPIO_PIN_4
#define SDCARD_D3_GPIO_Port GPIOB
#define STAT_LEDO_Pin GPIO_PIN_5
#define STAT_LEDO_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOB
#define ESC_MCU2_RX_Pin GPIO_PIN_0
#define ESC_MCU2_RX_GPIO_Port GPIOE
#define ESC_MCU2_TX_Pin GPIO_PIN_1
#define ESC_MCU2_TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
