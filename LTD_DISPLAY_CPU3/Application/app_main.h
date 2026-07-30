/*
 * app_main.h
 *
 *  Created on: Nov 18, 2025
 *      Author: Duan Xuebin
 */

#ifndef APP_MAIN_H_
/* APP_MAIN_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define APP_MAIN_H_
#include "main.h"

/**
 * @brief 初始化 CPU3 参数、显示、板间通信和三路外部串口调度。
 *
 * 先初始化异步日志并把 UART5 板间 RS485 收发器切回接收方向，然后初始化 OLED、立即绘制启动页并建立 RTC 时钟。
 * 随后从 FRAM 加载 CPU3 通信与显示参数，按当前配置重建三路外部串口并初始化 DSM 通信模块。
 * 外部串口保留一秒硬件稳定窗口，但只记录非阻塞就绪时刻，使屏幕任务和 CPU2 板间轮询可以立即进入主循环；各阶段持续上报看门狗健康进度。
 *
 * 主程序初始化。
 */
void App_Init(void);

/**
 * @brief 调度显示、参数保存、日志、CPU2 板间通信及三路外部协议任务。
 *
 * 每轮先上报看门狗进度，推进外部串口启动保护、接收异常恢复、协议切换和待执行串口重配置，再运行 SI 周期调度与 OLED 显示任务。
 * COM1、COM2、COM3 各自最多消费一帧已完成接收的数据；函数按端口当前协议分发请求，区分需要告警的处理失败和可忽略帧，并累计对应通信统计。
 * 处理成功且存在响应时启动或排队 UART DMA 发送；处理失败、发送启动失败或无需响应时立即恢复该端口 DMA 接收，避免外部串口停收。
 * 三路外部通信之后推进调试任务，并按 100 ms 门限轮询 CPU2，使持续外部流量不能永久饿死板间通信；最后服务异步日志，仅在本轮没有接收或轮询工作时延时 1 ms。
 *
 * 主循环任务。
 *
 * @note UART DMA 响应的最后一帧发送完成后，由 HAL_UART_TxCpltCallback 负责切回接收；主循环不得提前重启同一端口接收 DMA。
 */
void App_MainLoop(void);

/**
 * @brief UART错误或DMA收满后的统一中断出口。
 *
 * 中断只隔离端口、丢弃候选帧并投递恢复事件，DMA重启由主循环执行。
 *
 * UART接收异常或DMA收满时，由中断锁存事件并交给主循环恢复。
 *
 * @param huart 目标 UART 外设句柄。
 * @param error_code 待记录、转换或判断的错误码。该值是 HAL UART 错误位掩码，ISR 仅入队，主循环再按端口归并和打印。
 * @param overflow true 表示接收长度超过 DMA 缓冲容量，false 表示长度仍在范围内。
 */
void CPU3_UartRxFaultFromISR(UART_HandleTypeDef *huart,
                            uint32_t error_code,
                            uint8_t overflow);

/**
 * @brief 锁存一次DMA恢复请求；中断和主循环均可调用，本函数不打印、不循环重试。
 *
 * UART接收启动失败时投递统一退避恢复；中断和主循环均可调用。
 *
 * @param huart 目标 UART 外设句柄。
 */
void CPU3_UartScheduleRxRecovery(UART_HandleTypeDef *huart);

/**
 * @brief 前台完成一个有界阶段后推进健康代际；等待循环内部不得调用。
 *
 * 前台完成一个有界阶段时报告进度；等待循环内部不得持续调用。
 */
void CPU3_WatchdogReportProgress(void);

/**
 * @brief 看门狗只消费已经完成的前台健康进度，不再由定时器无条件续命。
 *
 * TIM4中断仅在前台健康进度推进后喂狗。
 *
 * @return true 表示前台健康代次自上次检查后已推进；没有新进度时返回 false。
 */
bool CPU3_WatchdogHealthAdvancedFromISR(void);

#endif /* APP_MAIN_H_ */
