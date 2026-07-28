/*
 * AS5145.h
 *
 * Created on: Dec 12, 2024
 * Author: Duan Xuebin
 */

#ifndef AS5145_H_
#define AS5145_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* AS5145 SSI总线复用SPI5，只接收4字节帧，不发送配置命令。 */
#define SSI hspi5
/* AS5145片选为PF6，低电平开始一帧，DMA完成或错误时必须恢复高电平。 */
#define SSI_CSN_PIN GPIO_PIN_6
#define SSI_CSN_PORT GPIOF
/* TIM1周期中断只发起下一次SPI5 DMA接收，不在定时器ISR解析帧。 */
#define ENCODER_TIM_HANDLE htim1

/* AS5145 18bit有效载荷的解析结果；原始帧只在PendSV中转换为本结构。 */
typedef struct
{
    uint16_t angle;   /* 12bit单圈绝对角，范围0至4095。 */
    uint8_t OCF;      /* Offset Compensation Finished，0表示上电补偿未完成。 */
    uint8_t COF;      /* CORDIC Overflow，1表示磁场或运算范围异常。 */
    uint8_t LIN;      /* Linearity Alarm，1表示线性度告警证据。 */
    uint8_t MagINCn;  /* 磁场增强告警位，低有效，保留用于诊断。 */
    uint8_t MagDECn;  /* 磁场减弱告警位，低有效，保留用于诊断。 */
    uint8_t parity;   /* 帧携带的偶校验位。 */
    uint8_t parity_ok;/* 1表示本地重算结果与帧校验位一致。 */
} SSI_Data_t;

extern SPI_HandleTypeDef SSI;
extern TIM_HandleTypeDef ENCODER_TIM_HANDLE;

/*
 * 启动接口会清空本次上电的队列、连续帧计数和故障锁存，再启动TIM1并主动读首帧。
 * 查询接口只读RAM；WaitFirstValidSample仅用于启动线程的有限等待。
 */
HAL_StatusTypeDef Start_Encoder_Collection_TIM(void);
uint32_t AS5145_GetLastError(void);
bool AS5145_HasValidSample(void);
uint32_t AS5145_WaitFirstValidSample(uint32_t timeout_ms);

/*
 * 函数用途：有界处理 SPI5 中断投递的编码器原始事件。
 * 调用场景：最低优先级 PendSV。
 * 关键约束：不打印、不调用 ErrorLog、不执行停机；每轮只处理固定数量事件。
 */
void AS5145_ProcessDeferred(void);

/*
 * 函数用途：为新的顶层正式业务过程解除编码器故障锁存。
 * 调用场景：ProcessMeasureCmd 进入正式测量初始化之前。
 * 关键约束：通信恢复和业务内部重试不得调用。
 */
void AS5145_ClearLatchedFaultForNewProcess(void);

bool AS5145_IsFaultLatched(void);

#endif