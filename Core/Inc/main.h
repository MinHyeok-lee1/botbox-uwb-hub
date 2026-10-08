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
#include "stm32g4xx_hal.h"

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
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_2
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_3
#define SW2_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_0
#define SW3_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_1
#define SW4_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define USART3_TX_RS485_Pin GPIO_PIN_10
#define USART3_TX_RS485_GPIO_Port GPIOB
#define USART3_RX_RS485_Pin GPIO_PIN_11
#define USART3_RX_RS485_GPIO_Port GPIOB
#define USART3_DE_RS485_Pin GPIO_PIN_14
#define USART3_DE_RS485_GPIO_Port GPIOB
#define LED_UART_Pin GPIO_PIN_9
#define LED_UART_GPIO_Port GPIOC
#define LED_CAN1_Pin GPIO_PIN_8
#define LED_CAN1_GPIO_Port GPIOA
#define LED_CAN2_Pin GPIO_PIN_9
#define LED_CAN2_GPIO_Port GPIOA
#define LED_RS485_Pin GPIO_PIN_10
#define LED_RS485_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
