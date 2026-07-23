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
#include "weight.h"
#include "hart.h"
#include "hostcommu.h"
#include "stateformodbus.h"
#include "iwdg.h"
#include "serial_command.h"
#include "../../Services/Relay/relay_output.h"
#include "../../Services/AoOutput/ao_output.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART4_WEIGHT_HIGH_IDX  19u
#define UART4_WEIGHT_LOW_IDX   20u
#define CPU2_TIM4_BUSINESS_DIVIDER  8U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static volatile uint8_t s_hart_frame_pending = 0U;
static volatile uint8_t s_hostcommu_frame_pending = 0U;
static uint8_t s_tim4_business_tick_count = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static void CPU2_ProcessDeferredUartFrames(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UART4_UpdateWeightFromFrame(uint16_t rx_len)
{
	uint8_t high_byte;
	uint8_t low_byte;

	if (rx_len <= UART4_WEIGHT_LOW_IDX) {
		return;
	}

	high_byte = USART4_RX_BUF[UART4_WEIGHT_HIGH_IDX];
	low_byte = USART4_RX_BUF[UART4_WEIGHT_LOW_IDX];
	g_weight = (int16_t)((high_byte << 8) | low_byte);
	weight_parament.current_weight = weight_parament.empty_weight - g_weight;
	Weight_Update(weight_parament.current_weight);
	Weight_MarkFrameReceived();
}

/*
 * 函数用途：严格校验CPU2板间Modbus请求帧的长度字段。
 * 调用场景：UART5 IDLE收帧后、进入业务解析前。
 * 关键约束：只拒绝畸形帧，不改变03、04、10功能码及合法帧业务语义。
 */
static bool CPU2_HostModbusFrameShapeIsValid(const uint8_t *frame, uint16_t length)
{
	uint16_t quantity;
	uint16_t expected_length;

	if ((frame == NULL) || (length < 2U)) {
		return false;
	}

	if ((frame[1] == FUNCTIONCODE_READ_HOLDREGISTER) ||
		(frame[1] == FUNCTIONCODE_READ_INPUTREGISTER)) {
		if (length != 8U) {
			return false;
		}
		quantity = ((uint16_t)frame[4] << 8) | (uint16_t)frame[5];
		return (quantity >= 1U) && (quantity <= 125U);
	}

	if (frame[1] == FUNCTIONCODE_WRITE_MULREGISTER) {
		uint8_t byte_count;

		if (length < 9U) {
			return false;
		}
		quantity = ((uint16_t)frame[4] << 8) | (uint16_t)frame[5];
		byte_count = frame[6];
		expected_length = (uint16_t)(9U + (uint16_t)byte_count);
		return (quantity >= 1U) &&
			(quantity <= 123U) &&
			((uint16_t)byte_count == (uint16_t)(quantity * 2U)) &&
			(length == expected_length);
	}

	/* 未支持功能码仍保留原异常应答流程，但必须具备固定请求头和CRC。 */
	return length == 8U;
}

/*
 * 函数用途：校验HART长短帧声明长度与DMA实际长度一致。
 * 调用场景：USART2 IDLE收帧后、进入既有HART解析器前。
 * 关键约束：不改变命令、地址或BCC判定，只阻止短帧读取DMA缓冲区旧数据。
 */
static bool CPU2_HartFrameShapeIsValid(const uint8_t *frame, uint16_t length)
{
	uint16_t expected_length;

	if ((frame == NULL) || (length == 0U) || (length > MAXREVEIVECNT)) {
		return false;
	}

	if (frame[0] == IS_SHORT_FRAME) {
		if (length < 5U) {
			return false;
		}
		expected_length = (uint16_t)frame[3] + 5U;
		return length == expected_length;
	}

	if (frame[0] == IS_LONG_FRAME) {
		if (length < 9U) {
			return false;
		}
		expected_length = (uint16_t)frame[7] + 9U;
		return length == expected_length;
	}

	return false;
}

/*
 * 函数用途：在最低优先级PendSV中执行HART和板间Modbus业务解析。
 * 调用场景：对应UART中断完成收帧并置位待处理标志后。
 * 关键约束：不依赖CPU2阻塞式主循环；UART中断只收帧和投递，业务应答顺序保持不变。
 */
static void CPU2_ProcessDeferredUartFrames(void)
{
	if (s_hostcommu_frame_pending != 0U) {
		s_hostcommu_frame_pending = 0U;
		if (CPU2_HostModbusFrameShapeIsValid(UART5_RX_BUF, UART5_RX_LEN)) {
			HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN);
		} else {
			UART5_RX_LEN = 0U;
			(void)CPU2_UartRestartRxDMA(&huart5);
		}
	}

	if (s_hart_frame_pending != 0U) {
		s_hart_frame_pending = 0U;
		USART2_TX_LEN = 0U;
		if (CPU2_HartFrameShapeIsValid(USART2_RX_BUF, USART2_RX_LEN)) {
			(void)HartCommunicationProcess(USART2_RX_BUF, USART2_TX_BUF, &USART2_TX_LEN);
		}

		if (USART2_TX_LEN > 0U) {
			HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET);
			if (HAL_UART_Transmit_DMA(&huart2, USART2_TX_BUF, USART2_TX_LEN) != HAL_OK) {
				(void)CPU2_UartRestartRxDMA(&huart2);
			}
		} else {
			(void)CPU2_UartRestartRxDMA(&huart2);
		}
	}
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi5_rx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

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
	while (1) {
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
  (void)AoOutput_ProcessPendingTimerRefresh();
  CPU2_ProcessDeferredUartFrames();
  CPU2_UartServicePendingRecovery();
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
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

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
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
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
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	/* 250 ms监督节拍只在没有被抢占ISR时喂狗，瞬时重叠可在下一节拍补喂。 */
	if ((SCB->ICSR & SCB_ICSR_RETTOBASE_Msk) != 0U) {
		HAL_IWDG_Refresh(&hiwdg);
	}

	/* 原继电器、AO和UART恢复任务保持2 s周期，避免改变既有业务时序。 */
	s_tim4_business_tick_count++;
	if (s_tim4_business_tick_count >= CPU2_TIM4_BUSINESS_DIVIDER) {
		s_tim4_business_tick_count = 0U;
		RelayOutput_Update();
		AoOutput_RequestTimerRefreshFromTim4Isr();
		CPU2_UartRecoveryPollFromTim4Isr();
	}
  /* USER CODE END TIM4_IRQn 1 */
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
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
			HAL_UART_DMAStop(&huart1);  // 停止DMA接收

			temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  // 获取DMA中未传输的数据个数
			USART1_RX_LEN = USART1_RX_BUF_SIZE - temp;  // 计算已经接收到的数据个数

			/* 中断只转交字节；完整帧、超长丢弃和延后报错由串口命令模块统一处理。 */
			for (int i = 0; i < USART1_RX_LEN; i++) {
				received_data = USART1_RX_BUF[i];
				SerialCommand_RxByteFromIsr(received_data);
			}
			(void)CPU2_UartRestartRxDMA(&huart1);
		}
	}
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	if (USART2 == huart2.Instance)  // 确保是USART2中断
			{
		tmp_flag = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);  // 获取IDLE标志位

		if (tmp_flag != RESET) {  // 如果IDLE标志被置位
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);  // 清除IDLE标志
			__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
			HAL_UART_DMAStop(&huart2);  // 停止DMA接收
			temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);  // 获取DMA中未传输的数据个数
			USART2_RX_LEN = USART2_RX_BUF_SIZE - temp;  // 计算已经接收到的数据个数

			/* 中断只投递完整候选帧，HART解析和应答由最低优先级PendSV执行。 */
			s_hart_frame_pending = 1U;
			SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
		}
	}
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
	if (UART4 == huart4.Instance) {
		tmp_flag = __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE); //获取IDLE标志位

		if ((tmp_flag != RESET)) //idle标志被置位
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart4); //清除标志位
			__HAL_UART_DISABLE_IT(&huart4, UART_IT_IDLE);
			HAL_UART_DMAStop(&huart4);
			temp = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx); // 获取DMA中未传输的数据个数
			USART4_RX_LEN = USART4_RX_BUF_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
//  		printf("receive %x!\n",USART4_RX_BUF[0]);
			UART4_UpdateWeightFromFrame(USART4_RX_LEN);
			(void)CPU2_UartRestartRxDMA(&huart4);
		}
	}
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
	uint32_t tmp_flag = 0;
	uint32_t temp;
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
	if (UART5 == huart5.Instance) {
		tmp_flag = __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE);  // 获取IDLE标志位

		if (tmp_flag != RESET) {  // 如果IDLE标志被置位
			__HAL_UART_CLEAR_IDLEFLAG(&huart5);  // 清除IDLE标志
			__HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
			HAL_UART_DMAStop(&huart5);  // 停止DMA接收
			temp = __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);  // 获取DMA中未传输的数据个数
			UART5_RX_LEN = UART5_RX_BUF_SIZE - temp;  // 计算已经接收到的数据个数
			/* 中断只投递完整候选帧，帧形校验、Modbus解析和应答由PendSV执行。 */
			s_hostcommu_frame_pending = 1U;
			SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
		}
	}
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
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

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
