/*
 * app_main.h
 *
 *  Created on: Nov 18, 2025
 *      Author: Duan Xuebin
 */

#ifndef APP_MAIN_H_
#define APP_MAIN_H_
#include "main.h"

/* 主程序初始化 */
void App_Init(void);

/* 主循环任务 */
void App_MainLoop(void);

/* UART接收异常或DMA收满时，由中断锁存事件并交给主循环恢复。 */
void CPU3_UartRxFaultFromISR(UART_HandleTypeDef *huart,
                            uint32_t error_code,
                            uint8_t overflow);

/* UART接收启动失败时投递统一退避恢复；中断和主循环均可调用。 */
void CPU3_UartScheduleRxRecovery(UART_HandleTypeDef *huart);

/* 前台完成一个有界阶段时报告进度；等待循环内部不得持续调用。 */
void CPU3_WatchdogReportProgress(void);

/* TIM4中断仅在前台健康进度推进后喂狗。 */
bool CPU3_WatchdogHealthAdvancedFromISR(void);

#endif /* APP_MAIN_H_ */
