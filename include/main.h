/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define AD8495_TC1_Pin GPIO_PIN_0
#define AD8495_TC1_GPIO_Port GPIOA
#define AD8495_TC2_Pin GPIO_PIN_1
#define AD8495_TC2_GPIO_Port GPIOA
#define PT0_Pin GPIO_PIN_2
#define PT0_GPIO_Port GPIOA
#define PT1_Pin GPIO_PIN_3
#define PT1_GPIO_Port GPIOA
#define PT2_Pin GPIO_PIN_4
#define PT2_GPIO_Port GPIOA
#define PT3_Pin GPIO_PIN_5
#define PT3_GPIO_Port GPIOA
#define PT4_Pin GPIO_PIN_6
#define PT4_GPIO_Port GPIOA
#define PT5_Pin GPIO_PIN_7
#define PT5_GPIO_Port GPIOA
#define STATUS_LED_Pin GPIO_PIN_0
#define STATUS_LED_GPIO_Port GPIOB
#define NCS3_Pin GPIO_PIN_11
#define NCS3_GPIO_Port GPIOB
#define NCS4_Pin GPIO_PIN_12
#define NCS4_GPIO_Port GPIOB
#define NCS_1_Pin GPIO_PIN_15
#define NCS_1_GPIO_Port GPIOA
#define NCS2_Pin GPIO_PIN_5
#define NCS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
