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
#define MAG_SCK_Pin GPIO_PIN_2
#define MAG_SCK_GPIO_Port GPIOE
#define BARO_INT_Pin GPIO_PIN_3
#define BARO_INT_GPIO_Port GPIOE
#define MAG_NSS_Pin GPIO_PIN_4
#define MAG_NSS_GPIO_Port GPIOE
#define MAG_MISO_Pin GPIO_PIN_5
#define MAG_MISO_GPIO_Port GPIOE
#define MAG_MOSI_Pin GPIO_PIN_6
#define MAG_MOSI_GPIO_Port GPIOE
#define IMU1_INT2_Pin GPIO_PIN_15
#define IMU1_INT2_GPIO_Port GPIOC
#define IMU2_MISO_Pin GPIO_PIN_2
#define IMU2_MISO_GPIO_Port GPIOC
#define IMU2_MOSI_Pin GPIO_PIN_3
#define IMU2_MOSI_GPIO_Port GPIOC
#define MaUWB1_TX_Pin GPIO_PIN_0
#define MaUWB1_TX_GPIO_Port GPIOA
#define MaUWB1_RX_Pin GPIO_PIN_1
#define MaUWB1_RX_GPIO_Port GPIOA
#define IMU3_NSS_Pin GPIO_PIN_4
#define IMU3_NSS_GPIO_Port GPIOA
#define IMU1_SCK_Pin GPIO_PIN_5
#define IMU1_SCK_GPIO_Port GPIOA
#define IMU1_MOSI_Pin GPIO_PIN_7
#define IMU1_MOSI_GPIO_Port GPIOA
#define IMU2_INT2_Pin GPIO_PIN_4
#define IMU2_INT2_GPIO_Port GPIOC
#define IMU1_INT1_Pin GPIO_PIN_5
#define IMU1_INT1_GPIO_Port GPIOC
#define LiDAR_EN_Pin GPIO_PIN_0
#define LiDAR_EN_GPIO_Port GPIOB
#define LiDAR_INT_Pin GPIO_PIN_1
#define LiDAR_INT_GPIO_Port GPIOB
#define IMU3_MOSI_Pin GPIO_PIN_2
#define IMU3_MOSI_GPIO_Port GPIOB
#define TTL_RX_Pin GPIO_PIN_7
#define TTL_RX_GPIO_Port GPIOE
#define TTL_TX_Pin GPIO_PIN_8
#define TTL_TX_GPIO_Port GPIOE
#define ESC1_PWM_Pin GPIO_PIN_9
#define ESC1_PWM_GPIO_Port GPIOE
#define TTL_LVL_SHIFT_EN_Pin GPIO_PIN_10
#define TTL_LVL_SHIFT_EN_GPIO_Port GPIOE
#define ESC2_PWM_Pin GPIO_PIN_11
#define ESC2_PWM_GPIO_Port GPIOE
#define TTL_TX_EN_Pin GPIO_PIN_12
#define TTL_TX_EN_GPIO_Port GPIOE
#define MAG_INT_Pin GPIO_PIN_15
#define MAG_INT_GPIO_Port GPIOE
#define LiDAR_SCL_Pin GPIO_PIN_10
#define LiDAR_SCL_GPIO_Port GPIOB
#define LiDAR_SDA_Pin GPIO_PIN_11
#define LiDAR_SDA_GPIO_Port GPIOB
#define IMU2_NSS_Pin GPIO_PIN_12
#define IMU2_NSS_GPIO_Port GPIOB
#define IMU2_SCK_Pin GPIO_PIN_13
#define IMU2_SCK_GPIO_Port GPIOB
#define GATE_DRIVER1_EN_Pin GPIO_PIN_10
#define GATE_DRIVER1_EN_GPIO_Port GPIOD
#define GATE_DRIVER2_EN_Pin GPIO_PIN_11
#define GATE_DRIVER2_EN_GPIO_Port GPIOD
#define IMU2_INT1_Pin GPIO_PIN_14
#define IMU2_INT1_GPIO_Port GPIOD
#define IMU3_INT2_Pin GPIO_PIN_15
#define IMU3_INT2_GPIO_Port GPIOD
#define IMU3_INT1_Pin GPIO_PIN_8
#define IMU3_INT1_GPIO_Port GPIOC
#define LIN_ACTUATOR1_PWM_Pin GPIO_PIN_15
#define LIN_ACTUATOR1_PWM_GPIO_Port GPIOA
#define IMU3_SCK_Pin GPIO_PIN_10
#define IMU3_SCK_GPIO_Port GPIOC
#define IMU3_MISO_Pin GPIO_PIN_11
#define IMU3_MISO_GPIO_Port GPIOC
#define MaUWB2_TX_Pin GPIO_PIN_12
#define MaUWB2_TX_GPIO_Port GPIOC
#define MaUWB2_RX_Pin GPIO_PIN_2
#define MaUWB2_RX_GPIO_Port GPIOD
#define IMU1_MISO_Pin GPIO_PIN_9
#define IMU1_MISO_GPIO_Port GPIOG
#define IMU1_NSS_Pin GPIO_PIN_10
#define IMU1_NSS_GPIO_Port GPIOG
#define LIN_ACTUATOR2_PWM_Pin GPIO_PIN_3
#define LIN_ACTUATOR2_PWM_GPIO_Port GPIOB
#define BARO_SCL_Pin GPIO_PIN_8
#define BARO_SCL_GPIO_Port GPIOB
#define BARO_SDA_Pin GPIO_PIN_9
#define BARO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
