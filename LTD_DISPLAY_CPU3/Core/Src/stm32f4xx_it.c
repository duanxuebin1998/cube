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
#include "cpu2_communicate.h"
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "exit.h"
#include "display_tankopera.h"
#include "DSM_communication.h"
#include "iwdg.h"
#include "Display.h"
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
#include "stm32f4xx.h"   // 换成你对应的芯片头文件
#include <stdio.h>
void Fault_Handler(const char *name); // 声明你原来的统一处理函数
void Fault_HandlerEx(const char *name, uint32_t *stack_addr)
{
    // 进异常时硬件自动压栈的寄存器
    uint32_t r0  = stack_addr[0];
    uint32_t r1  = stack_addr[1];
    uint32_t r2  = stack_addr[2];
    uint32_t r3  = stack_addr[3];
    uint32_t r12 = stack_addr[4];
    uint32_t lr  = stack_addr[5];
    uint32_t pc  = stack_addr[6];   // ★ 出错指令地址
    uint32_t psr = stack_addr[7];

    uint32_t msp = __get_MSP();
    uint32_t psp = __get_PSP();

    uint32_t cfsr  = SCB->CFSR;
    uint32_t hfsr  = SCB->HFSR;
    uint32_t mmfar = SCB->MMFAR;
    uint32_t bfar  = SCB->BFAR;

    __disable_irq();

    printf("\r\n=============================\r\n");
    printf("   !!! %s OCCURRED !!!\r\n", name);
    printf("-----------------------------\r\n");
    printf("  R0  = 0x%08lX\r\n", r0);
    printf("  R1  = 0x%08lX\r\n", r1);
    printf("  R2  = 0x%08lX\r\n", r2);
    printf("  R3  = 0x%08lX\r\n", r3);
    printf("  R12 = 0x%08lX\r\n", r12);
    printf("  LR  = 0x%08lX\r\n", lr);
    printf("  PC  = 0x%08lX  <-- 出错指令\r\n", pc);
    printf("  xPSR= 0x%08lX\r\n", psr);
    printf("  MSP = 0x%08lX\r\n", msp);
    printf("  PSP = 0x%08lX\r\n", psp);
    printf("-----------------------------\r\n");
    printf("  HFSR = 0x%08lX\r\n", hfsr);
    printf("  CFSR = 0x%08lX\r\n", cfsr);
    printf("  MMFAR= 0x%08lX\r\n", mmfar);
    printf("  BFAR = 0x%08lX\r\n", bfar);

    // 简单解码 CFSR，辅助判断
    uint8_t  mmfsr = (uint8_t)(cfsr & 0xFF);
    uint8_t  bfsr  = (uint8_t)((cfsr >> 8) & 0xFF);
    uint16_t ufsr  = (uint16_t)((cfsr >> 16) & 0xFFFF);

    printf("------------- Decode CFSR --------------\r\n");
    if (mmfsr) {
        printf("  MemManage Fault: 0x%02X\r\n", mmfsr);
    }
    if (bfsr) {
        printf("  Bus Fault     : 0x%02X\r\n", bfsr);
        if (bfsr & (1 << 7)) {
            printf("    -> BFAR valid, addr = 0x%08lX\r\n", bfar);
        }
    }
    if (ufsr) {
        printf("  Usage Fault   : 0x%04X\r\n", ufsr);
        if (ufsr & (1 << 9)) {
            printf("    -> DIVBYZERO: 除以 0 错误\r\n");
        }
        if (ufsr & (1 << 0)) {
            printf("    -> UNDEFINSTR: 非法指令/跳飞\r\n");
        }
    }
    printf("=========================================\r\n");

    // 继续调用你原来的统一处理逻辑（打印 HFSR/CFSR + 卡死/复位）
    Fault_Handler(name);
}

void Fault_Handler(const char *name)
{
    __disable_irq();  // 先关中断，防止再嵌套

    // 1. 打印基本信息（串口正常时）
    printf("\r\n=============================\r\n");
    printf("  !!! %s OCCURRED !!!\r\n", name);
    printf("  HFSR = 0x%08lX\r\n", SCB->HFSR);
    printf("  CFSR = 0x%08lX\r\n", SCB->CFSR);
    printf("  BFAR = 0x%08lX\r\n", SCB->BFAR);
    printf("  MMFAR= 0x%08lX\r\n", SCB->MMFAR);
    printf("=============================\r\n");

    // 2. TODO: 写入 FRAM/Flash，记录错误信息，方便下次上电分析
    // Fault_LogToFRAM(name, SCB->HFSR, SCB->CFSR, ...);

    // 3. 点亮告警灯（按你实际硬件改）
    // HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);

    // 4. 等待看门狗复位，或主动软复位
    while (1)
    {
        // 如果启用了独立看门狗，就不要在这里喂狗，让它超时复位
        // 如果没看门狗，也可以延时一会儿后软件复位：
        // for (volatile uint32_t i = 0; i < 1000000; i++);
         NVIC_SystemReset();
    }
}
static const char HardFaultName[] = "HardFault";
void HardFault_Handler(void) __attribute__((naked));

/* ===================== 通用：DMA + IDLE 一帧接收处理 ===================== */
static inline uint16_t uart_calc_dma_len(uint16_t buf_size, uint32_t dma_left)
{
    if (dma_left > buf_size) {
        return 0;
    }
    return (uint16_t)(buf_size - (uint16_t)dma_left);
}

/* ready 模式：len>0 置位 ready；len==0 重启 DMA */
static inline void uart_idle_rx_dma_ready(UART_HandleTypeDef *huart,
                                         DMA_HandleTypeDef  *hdma_rx,
                                         uint8_t            *rx_buf,
                                         uint16_t            rx_buf_size,
                                         volatile uint16_t  *rx_len,
                                         volatile uint8_t   *rx_ready,
                                         void (*set_recv_mode)(void))
{
    if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET) ||
        (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) == RESET)) {
        return;
    }

    /* 清 IDLE */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* 清错误标志，避免 ORE 导致后续异常 */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)  != RESET) __HAL_UART_CLEAR_FEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  != RESET) __HAL_UART_CLEAR_NEFLAG(huart);

    /* Stop DMA, compute length */
    HAL_UART_DMAStop(huart);
    uint32_t left = __HAL_DMA_GET_COUNTER(hdma_rx);
    uint16_t len  = uart_calc_dma_len(rx_buf_size, left);

    if (len > 0) {
        *rx_len   = len;
        *rx_ready = 1;    /* 主循环处理后再重启 DMA（你当前架构） */
    } else {
        *rx_len = 0;
        if (set_recv_mode) set_recv_mode();
        HAL_UART_Receive_DMA(huart, rx_buf, rx_buf_size);
    }
}

/* wait_response 模式：len>=min_len 解除等待；否则重启 DMA */
static inline void uart_idle_rx_dma_wait(UART_HandleTypeDef *huart,
                                        DMA_HandleTypeDef  *hdma_rx,
                                        uint8_t            *rx_buf,
                                        uint16_t            rx_buf_size,
                                        volatile uint16_t  *rx_len,
                                        volatile bool      *wait_response,
                                        uint16_t            min_len,
                                        void (*set_recv_mode)(void))
{
    if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET) ||
        (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) == RESET)) {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(huart);

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)  != RESET) __HAL_UART_CLEAR_FEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  != RESET) __HAL_UART_CLEAR_NEFLAG(huart);

    HAL_UART_DMAStop(huart);
    uint32_t left = __HAL_DMA_GET_COUNTER(hdma_rx);
    uint16_t len  = uart_calc_dma_len(rx_buf_size, left);

    *rx_len = len;

    if (len >= min_len) {
        *wait_response = false; /* 接收完成，解除等待 */
        /* 此时由你的上层逻辑决定何时重启 DMA */
    } else {
        if (set_recv_mode) set_recv_mode();
        HAL_UART_Receive_DMA(huart, rx_buf, rx_buf_size);
    }
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
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
    __asm volatile
    (
        "tst lr, #4                \n" // EXC_RETURN 的 bit2：0=MSP,1=PSP
        "ite eq                    \n"
        "mrseq r1, msp             \n" // r1 = stack_addr
        "mrsne r1, psp             \n"
        "ldr  r0, =HardFaultName   \n" // r0 = const char *name
        "b    Fault_HandlerEx      \n" // 直接跳到 C 函数，不再返回
    );
//	Fault_Handler("HardFault");
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
	Fault_Handler("MemManageFault");
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
	Fault_Handler("BusFault");
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
	Fault_Handler("UsageFault");
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
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_DOWN_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_SURE_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_BACK_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_UP_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

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
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
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

	// 读取按键状态（低电平为按下，根据电路决定）
	GPIO_PinState state = HAL_GPIO_ReadPin(KEY_SURE_GPIO_Port, KEY_SURE_Pin);

	if (state == GPIO_PIN_RESET) {
		button_press_counter++;

		if (button_press_counter >= REQUIRED_PRESS_COUNT) {
			button_press_counter = 0;  // 清零防止重复触发
			FlagofTankOpera = true;
			useKey();
			HAL_TIM_Base_Stop_IT(&htim1);
			keymenu[KEYNUM_IF_ENTER_MAINMENU].execute_opera();
		} else {
			HAL_TIM_Base_Start_IT(&htim1);
		}
	} else {
		// 一旦松开就清零
		FlagofTankOpera = false;
		button_press_counter = 0;
		HAL_TIM_Base_Stop_IT(&htim1);
	}

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	RefreshScreen();  //刷新屏幕
//	HAL_IWDG_Refresh(&hiwdg);
  /* USER CODE END TIM3_IRQn 1 */
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
	HAL_IWDG_Refresh(&hiwdg);
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    uart_idle_rx_dma_ready(&huart2, &hdma_usart2_rx,
                           UART2_RX_BUF, UART2_RX_BUF_SIZE,
                           &UART2_RX_LEN, &com2_rx_ready,
                           COM2_RecvMode /* 你已有的 inline 包装函数 */);
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
    uart_idle_rx_dma_ready(&huart3, &hdma_usart3_rx,
                           UART3_RX_BUF, UART3_RX_BUF_SIZE,
                           &UART3_RX_LEN, &com3_rx_ready,
                           COM3_RecvMode);
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
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
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
    uart_idle_rx_dma_wait(&huart5, &hdma_uart5_rx,
                          UART5_RX_BUF, UART5_RX_BUF_SIZE,
                          &UART5_RX_LEN, &wait_response,
                          4 /* min_len=4 等价于你原来的 >3 */,
                          NULL /* 如 UART5 也需要方向切换，这里可传 RS485_SET_RECV_MODE 的包装函数 */);

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

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
    uart_idle_rx_dma_ready(&huart6, &hdma_usart6_rx,
                           UART6_RX_BUF, UART6_RX_BUF_SIZE,
                           &UART6_RX_LEN, &com1_rx_ready,
                           COM1_RecvMode);
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
