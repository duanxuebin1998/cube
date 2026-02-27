/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
/* USART2 485接收缓冲区 */
#define UART2_RX_BUF_SIZE 256
extern volatile uint8_t com2_rx_ready ;
extern volatile uint16_t UART2_RX_LEN;              // 接收一帧数据的长度
extern uint8_t UART2_RX_BUF[UART2_RX_BUF_SIZE];   // 接收数据缓冲区
 /** USART3 485接收缓冲区 */
#define UART3_RX_BUF_SIZE 512
extern volatile uint8_t com3_rx_ready ;
extern volatile uint16_t UART3_RX_LEN;              // 接收一帧数据的长度
extern uint8_t UART3_RX_BUF[UART3_RX_BUF_SIZE];   // 接收数据缓冲区

#define UART5_RX_BUF_SIZE 256
extern volatile uint16_t UART5_RX_LEN;              // 接收一帧数据的长度
extern uint8_t UART5_RX_BUF[UART5_RX_BUF_SIZE];   // 接收数据缓冲区

#define UART6_RX_BUF_SIZE 256
extern volatile uint8_t com1_rx_ready ;
extern volatile uint16_t UART6_RX_LEN;              // 接收一帧数据的长度
extern uint8_t UART6_RX_BUF[UART6_RX_BUF_SIZE];   // 接收数据缓冲区
/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

