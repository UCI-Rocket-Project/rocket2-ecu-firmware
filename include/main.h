/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ALT_SCK_Pin GPIO_PIN_2
#define ALT_SCK_GPIO_Port GPIOE
#define ALT_nCS_Pin GPIO_PIN_3
#define ALT_nCS_GPIO_Port GPIOE
#define ALT_MISO_Pin GPIO_PIN_5
#define ALT_MISO_GPIO_Port GPIOE
#define ALT_MOSI_Pin GPIO_PIN_6
#define ALT_MOSI_GPIO_Port GPIOE
#define RADIO2_nRST_Pin GPIO_PIN_14
#define RADIO2_nRST_GPIO_Port GPIOC
#define RADIO2_nCS_Pin GPIO_PIN_15
#define RADIO2_nCS_GPIO_Port GPIOC
#define RADIO2_DIO1_Pin GPIO_PIN_9
#define RADIO2_DIO1_GPIO_Port GPIOI
#define RADIO2_DIO0_Pin GPIO_PIN_10
#define RADIO2_DIO0_GPIO_Port GPIOI
#define RADIO_nRST_Pin GPIO_PIN_11
#define RADIO_nRST_GPIO_Port GPIOI
#define RADIO_DIO1_Pin GPIO_PIN_0
#define RADIO_DIO1_GPIO_Port GPIOF
#define RADIO_DIO0_Pin GPIO_PIN_1
#define RADIO_DIO0_GPIO_Port GPIOF
#define RADIO1_DIO1_Pin GPIO_PIN_2
#define RADIO1_DIO1_GPIO_Port GPIOF
#define ECU_GSV_Pin GPIO_PIN_3
#define ECU_GSV_GPIO_Port GPIOF
#define RADIO1_nCS_Pin GPIO_PIN_4
#define RADIO1_nCS_GPIO_Port GPIOF
#define RADIO1_nRST_Pin GPIO_PIN_5
#define RADIO1_nRST_GPIO_Port GPIOF
#define RADIO_nCS_Pin GPIO_PIN_6
#define RADIO_nCS_GPIO_Port GPIOF
#define RADIO_SCK_Pin GPIO_PIN_7
#define RADIO_SCK_GPIO_Port GPIOF
#define RADIO_MISO_Pin GPIO_PIN_8
#define RADIO_MISO_GPIO_Port GPIOF
#define RADIO_MOSI_Pin GPIO_PIN_9
#define RADIO_MOSI_GPIO_Port GPIOF
#define PT0_Pin GPIO_PIN_10
#define PT0_GPIO_Port GPIOF
#define PT1_Pin GPIO_PIN_0
#define PT1_GPIO_Port GPIOC
#define PT2_Pin GPIO_PIN_1
#define PT2_GPIO_Port GPIOC
#define PT3_Pin GPIO_PIN_2
#define PT3_GPIO_Port GPIOC
#define PT4_Pin GPIO_PIN_3
#define PT4_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_0
#define GPS_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_1
#define GPS_TX_GPIO_Port GPIOA
#define PT5_Pin GPIO_PIN_2
#define PT5_GPIO_Port GPIOA
#define GPS_INT_Pin GPIO_PIN_2
#define GPS_INT_GPIO_Port GPIOH
#define MEM_nCS_Pin GPIO_PIN_3
#define MEM_nCS_GPIO_Port GPIOH
#define GPS_SCL_Pin GPIO_PIN_4
#define GPS_SCL_GPIO_Port GPIOH
#define GPS_SDA_Pin GPIO_PIN_5
#define GPS_SDA_GPIO_Port GPIOH
#define PT6_Pin GPIO_PIN_3
#define PT6_GPIO_Port GPIOA
#define ECU_BV_Pin GPIO_PIN_4
#define ECU_BV_GPIO_Port GPIOA
#define MEM_SCK_Pin GPIO_PIN_5
#define MEM_SCK_GPIO_Port GPIOA
#define MEM_MISO_Pin GPIO_PIN_6
#define MEM_MISO_GPIO_Port GPIOA
#define MEM_MOSI_Pin GPIO_PIN_7
#define MEM_MOSI_GPIO_Port GPIOA
#define SOLENOID0_FB_Pin GPIO_PIN_4
#define SOLENOID0_FB_GPIO_Port GPIOC
#define SOLENOID1_FB_Pin GPIO_PIN_5
#define SOLENOID1_FB_GPIO_Port GPIOC
#define SOLENOID2_FB_Pin GPIO_PIN_0
#define SOLENOID2_FB_GPIO_Port GPIOB
#define SOLENOID3_FB_Pin GPIO_PIN_1
#define SOLENOID3_FB_GPIO_Port GPIOB
#define MEM1_nCS_Pin GPIO_PIN_11
#define MEM1_nCS_GPIO_Port GPIOF
#define SOLENOID0_EN_Pin GPIO_PIN_13
#define SOLENOID0_EN_GPIO_Port GPIOF
#define SOLENOID1_EN_Pin GPIO_PIN_14
#define SOLENOID1_EN_GPIO_Port GPIOF
#define SOLENOID2_EN_Pin GPIO_PIN_15
#define SOLENOID2_EN_GPIO_Port GPIOF
#define SOLENOID3_EN_Pin GPIO_PIN_0
#define SOLENOID3_EN_GPIO_Port GPIOG
#define ALARM_Pin GPIO_PIN_9
#define ALARM_GPIO_Port GPIOE
#define ETH_RX_Pin GPIO_PIN_10
#define ETH_RX_GPIO_Port GPIOB
#define ETH_TX_Pin GPIO_PIN_11
#define ETH_TX_GPIO_Port GPIOB
#define ETH_nRST_Pin GPIO_PIN_12
#define ETH_nRST_GPIO_Port GPIOB
#define ETH_CTS_Pin GPIO_PIN_13
#define ETH_CTS_GPIO_Port GPIOB
#define ETH_RTS_Pin GPIO_PIN_14
#define ETH_RTS_GPIO_Port GPIOB
#define ETH_CP2_Pin GPIO_PIN_15
#define ETH_CP2_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_5
#define STATUS_LED_GPIO_Port GPIOG
#define USB_VBUS_SENS_Pin GPIO_PIN_9
#define USB_VBUS_SENS_GPIO_Port GPIOA
#define USB_CONN_D__Pin GPIO_PIN_11
#define USB_CONN_D__GPIO_Port GPIOA
#define USB_CONN_D_A12_Pin GPIO_PIN_12
#define USB_CONN_D_A12_GPIO_Port GPIOA
#define MCU_TMS_Pin GPIO_PIN_13
#define MCU_TMS_GPIO_Port GPIOA
#define MCU_TCK_Pin GPIO_PIN_14
#define MCU_TCK_GPIO_Port GPIOA
#define MCU_TDI_Pin GPIO_PIN_15
#define MCU_TDI_GPIO_Port GPIOA
#define TC0_SCK_Pin GPIO_PIN_10
#define TC0_SCK_GPIO_Port GPIOC
#define TC0_MISO_Pin GPIO_PIN_11
#define TC0_MISO_GPIO_Port GPIOC
#define TC0_nCS_Pin GPIO_PIN_12
#define TC0_nCS_GPIO_Port GPIOC
#define TC1_nCS_Pin GPIO_PIN_0
#define TC1_nCS_GPIO_Port GPIOD
#define IMU_INT4_Pin GPIO_PIN_5
#define IMU_INT4_GPIO_Port GPIOD
#define IMU_INT3_Pin GPIO_PIN_6
#define IMU_INT3_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_7
#define IMU_INT1_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_9
#define IMU_INT2_GPIO_Port GPIOG
#define IMU_nCS2_Pin GPIO_PIN_10
#define IMU_nCS2_GPIO_Port GPIOG
#define IMU_nCS1_Pin GPIO_PIN_11
#define IMU_nCS1_GPIO_Port GPIOG
#define IMU_MISO_Pin GPIO_PIN_12
#define IMU_MISO_GPIO_Port GPIOG
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOG
#define IMU_MOSI_Pin GPIO_PIN_14
#define IMU_MOSI_GPIO_Port GPIOG
#define MCU_TDO_Pin GPIO_PIN_3
#define MCU_TDO_GPIO_Port GPIOB
#define MAG_DRDY_Pin GPIO_PIN_5
#define MAG_DRDY_GPIO_Port GPIOB
#define MAG_INT_Pin GPIO_PIN_6
#define MAG_INT_GPIO_Port GPIOB
#define MAG_SDA_Pin GPIO_PIN_7
#define MAG_SDA_GPIO_Port GPIOB
#define MAG_SCL_Pin GPIO_PIN_8
#define MAG_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
