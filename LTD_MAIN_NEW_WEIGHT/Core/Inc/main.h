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
#include "system_parameter.h"
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
#define ENCODE_SPI5_CS_Pin GPIO_PIN_6
#define ENCODE_SPI5_CS_GPIO_Port GPIOF
#define ENCODE_SPI5_SCK_Pin GPIO_PIN_7
#define ENCODE_SPI5_SCK_GPIO_Port GPIOF
#define ENCODE_SPI5_MISO_Pin GPIO_PIN_8
#define ENCODE_SPI5_MISO_GPIO_Port GPIOF
#define POWER_24V_TEST_Pin GPIO_PIN_2
#define POWER_24V_TEST_GPIO_Port GPIOC
#define POWER_12V_TEST_Pin GPIO_PIN_3
#define POWER_12V_TEST_GPIO_Port GPIOC
#define MOTOR_IREF_Pin GPIO_PIN_4
#define MOTOR_IREF_GPIO_Port GPIOA
#define FRAM_SPI4_CS_Pin GPIO_PIN_11
#define FRAM_SPI4_CS_GPIO_Port GPIOE
#define FRAM_SPI4_SCK_Pin GPIO_PIN_12
#define FRAM_SPI4_SCK_GPIO_Port GPIOE
#define FRAM_SPI4_MISO_Pin GPIO_PIN_13
#define FRAM_SPI4_MISO_GPIO_Port GPIOE
#define FRAM_SPI4_MOSI_Pin GPIO_PIN_14
#define FRAM_SPI4_MOSI_GPIO_Port GPIOE
#define MOTOR_SPI2_CS_Pin GPIO_PIN_12
#define MOTOR_SPI2_CS_GPIO_Port GPIOB
#define MOTOR_SPI2_SCK_Pin GPIO_PIN_13
#define MOTOR_SPI2_SCK_GPIO_Port GPIOB
#define MOTOR_SPI2_MOSI_Pin GPIO_PIN_14
#define MOTOR_SPI2_MOSI_GPIO_Port GPIOB
#define MOTOR_SPI2_MOSIB15_Pin GPIO_PIN_15
#define MOTOR_SPI2_MOSIB15_GPIO_Port GPIOB
#define DRV_ENN_Pin GPIO_PIN_10
#define DRV_ENN_GPIO_Port GPIOD
#define SENSOR_TXD6_Pin GPIO_PIN_6
#define SENSOR_TXD6_GPIO_Port GPIOC
#define SENSOR_RXD6_Pin GPIO_PIN_7
#define SENSOR_RXD6_GPIO_Port GPIOC
#define RELAY1_NC_Pin GPIO_PIN_8
#define RELAY1_NC_GPIO_Port GPIOC
#define RELAY1_NO_Pin GPIO_PIN_9
#define RELAY1_NO_GPIO_Port GPIOC
#define RELAY2_NC_Pin GPIO_PIN_8
#define RELAY2_NC_GPIO_Port GPIOA
#define DEBUG_TXD1_Pin GPIO_PIN_9
#define DEBUG_TXD1_GPIO_Port GPIOA
#define DEBUG_RXD1_Pin GPIO_PIN_10
#define DEBUG_RXD1_GPIO_Port GPIOA
#define RELAY2_NO_Pin GPIO_PIN_15
#define RELAY2_NO_GPIO_Port GPIOA
#define RELAY3_NC_Pin GPIO_PIN_10
#define RELAY3_NC_GPIO_Port GPIOC
#define RELAY3_NO_Pin GPIO_PIN_11
#define RELAY3_NO_GPIO_Port GPIOC
#define CPU2_TXD5_Pin GPIO_PIN_12
#define CPU2_TXD5_GPIO_Port GPIOC
#define RELAY4_NC_Pin GPIO_PIN_0
#define RELAY4_NC_GPIO_Port GPIOD
#define CPU2_RXD5_Pin GPIO_PIN_2
#define CPU2_RXD5_GPIO_Port GPIOD
#define RELAY4_NO_Pin GPIO_PIN_3
#define RELAY4_NO_GPIO_Port GPIOD
#define RXD2_485_Pin GPIO_PIN_5
#define RXD2_485_GPIO_Port GPIOD
#define TXD2_485_Pin GPIO_PIN_6
#define TXD2_485_GPIO_Port GPIOD
#define RELAY1_Pin GPIO_PIN_9
#define RELAY1_GPIO_Port GPIOG
#define RELAY2_Pin GPIO_PIN_10
#define RELAY2_GPIO_Port GPIOG
#define RELAY3_Pin GPIO_PIN_11
#define RELAY3_GPIO_Port GPIOG
#define RELAY4_Pin GPIO_PIN_12
#define RELAY4_GPIO_Port GPIOG
#define AD5421_SPI3_CS_Pin GPIO_PIN_3
#define AD5421_SPI3_CS_GPIO_Port GPIOB
#define AD5421_SPI3_MOSI_Pin GPIO_PIN_4
#define AD5421_SPI3_MOSI_GPIO_Port GPIOB
#define AD5421_SPI3_MOSIB5_Pin GPIO_PIN_5
#define AD5421_SPI3_MOSIB5_GPIO_Port GPIOB
#define AD5421_SPI3_CSB6_Pin GPIO_PIN_6
#define AD5421_SPI3_CSB6_GPIO_Port GPIOB
#define AD5421_LDAC_Pin GPIO_PIN_7
#define AD5421_LDAC_GPIO_Port GPIOB
#define AD5421_FAULT_Pin GPIO_PIN_8
#define AD5421_FAULT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern int16_t g_weight;//称重值
extern volatile int32_t g_encoder_count ;

//usart1接收数据缓冲区
// 存储接收到的数据
extern uint8_t received_buffer[64];
extern uint16_t buffer_index;  // 当前接收的数据索引
// 标志，表示有新命令待处理
extern volatile uint8_t new_command_ready;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
