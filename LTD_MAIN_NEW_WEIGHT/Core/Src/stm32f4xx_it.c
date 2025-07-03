/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi5_rx;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
//usart1接收数据缓冲区
uint8_t received_buffer[64];
uint16_t buffer_index = 0;  // 当前接收的数据索引
// 标志，表示有新命令待处理
volatile uint8_t new_command_ready = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
	uint8_t received_data;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	if (USART1 == huart1.Instance)  // 确保是USART1中断
	{
		tmp_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE);  // 获取IDLE标志位

		if (tmp_flag != RESET)  // 如果IDLE标志被置位
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);  // 清除IDLE标志
			HAL_UART_DMAStop(&huart1);  // 停止DMA接收

			temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  // 获取DMA中未传输的数据个数
			USART1_RX_LEN = USART1_RX_BUF_SIZE - temp;  // 计算已经接收到的数据个数

			// 将接收到的数据添加到接收缓冲区
			for (int i = 0; i < USART1_RX_LEN; i++)
			{
				received_data = USART1_RX_BUF[i];
				printf("receive %x!\n",received_data);
				// 判断是否遇到终止符 \r\n
				if (received_data == '\n' && buffer_index > 0
						&& received_buffer[buffer_index - 1] == '\r')
				{
					// 完整命令接收完毕，标记有新命令
					new_command_ready = 1;
					// 通过串口1重新发送接收到的所有数据
					HAL_UART_Transmit(&huart1, USART1_RX_BUF, USART1_RX_LEN,HAL_MAX_DELAY);  // 发送接收到的所有数据
					buffer_index = 0;  // 清空接收缓冲区，准备接收下一条命令
				}
				else
				{
                  // 存储接收到的数据
                  if (buffer_index < 64 - 1) {
                      received_buffer[buffer_index++] = received_data;
                  }
				}
			}
			HAL_UART_Receive_DMA(&huart1, USART1_RX_BUF, USART1_RX_BUF_SIZE); // 重新启用DMA接收
		}
	}
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
	uint8_t high_byte; // FF
	uint8_t low_byte;  // 69
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
  if (UART4 == huart4.Instance) {
  	tmp_flag = __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE); //获取IDLE标志位

  	if ((tmp_flag != RESET)) //idle标志被置位
  	{
  		__HAL_UART_CLEAR_IDLEFLAG(&huart4); //清除标志位
  		HAL_UART_DMAStop(&huart4);
  		temp = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx); // 获取DMA中未传输的数据个数
  		USART4_RX_LEN = USART4_RX_BUF_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
//  		printf("receive %x!\n",USART4_RX_BUF[0]);
  		high_byte = USART4_RX_BUF[19]; // FF
  		low_byte = USART4_RX_BUF[20];  // 69
  		weight = (high_byte << 8) | low_byte;

  		HAL_UART_Receive_DMA(&huart4, USART4_RX_BUF, USART4_RX_BUF_SIZE); //重新打开DMA接收
  	}
  	}
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi5_rx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
