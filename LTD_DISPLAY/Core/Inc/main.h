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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "usart.h"
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
#define KEY_DOWN_Pin GPIO_PIN_1
#define KEY_DOWN_GPIO_Port GPIOF
#define KEY_DOWN_EXTI_IRQn EXTI1_IRQn
#define KEY_SURE_Pin GPIO_PIN_2
#define KEY_SURE_GPIO_Port GPIOF
#define KEY_SURE_EXTI_IRQn EXTI2_IRQn
#define KEY_BACK_Pin GPIO_PIN_3
#define KEY_BACK_GPIO_Port GPIOF
#define KEY_BACK_EXTI_IRQn EXTI3_IRQn
#define KEY_UP_Pin GPIO_PIN_4
#define KEY_UP_GPIO_Port GPIOF
#define KEY_UP_EXTI_IRQn EXTI4_IRQn
#define OLED_NREST_Pin GPIO_PIN_5
#define OLED_NREST_GPIO_Port GPIOF
#define OLED_D_C_Pin GPIO_PIN_2
#define OLED_D_C_GPIO_Port GPIOA
#define _485_3_USART3_TX_Pin GPIO_PIN_8
#define _485_3_USART3_TX_GPIO_Port GPIOD
#define _485_3_USART3_RX_Pin GPIO_PIN_9
#define _485_3_USART3_RX_GPIO_Port GPIOD
#define P20MA_485_1_USART6_TX_Pin GPIO_PIN_6
#define P20MA_485_1_USART6_TX_GPIO_Port GPIOC
#define P20MA_485_1_USART6_RX_Pin GPIO_PIN_7
#define P20MA_485_1_USART6_RX_GPIO_Port GPIOC
#define MAIN_BOARD_UART5_TX_Pin GPIO_PIN_12
#define MAIN_BOARD_UART5_TX_GPIO_Port GPIOC
#define MAIN_BOARD_UART5_RX_Pin GPIO_PIN_2
#define MAIN_BOARD_UART5_RX_GPIO_Port GPIOD
#define P20MA_485_2_USART2_TX_Pin GPIO_PIN_5
#define P20MA_485_2_USART2_TX_GPIO_Port GPIOD
#define P20MA_485_2_USART2_RX_Pin GPIO_PIN_6
#define P20MA_485_2_USART2_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
