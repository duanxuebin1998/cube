/*
 * AS5145.h
 *
 * Created on: Dec 12, 2024
 * Author: Duan Xuebin
 * Description: Header file for AS5145 SPI-based rotary encoder.
 */

#ifndef AS5145_H_
#define AS5145_H_

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/* SPI Interface and CSN Pin Definitions */
#define SSI             hspi5               /* SPI interface used for AS5145 communication */
#define SSI_CSN_PIN     GPIO_PIN_6          /* Chip Select (CS) pin for AS5145 */
#define SSI_CSN_PORT    GPIOF               /* GPIO port for CS pin */
#define ENCODER_TIM_HANDLE  htim1              /* 编码器定时采集定时器 */

typedef enum {
	State_IDLE,
    RETRY_IN_PROGRESS,
    RETRY_FAILED
} EncoderState;
/* 数据结构体 */
typedef struct
{
	uint16_t angle;    /* 12位角度数据 (D11:D0) */
	uint8_t OCF;       /* 偏差补偿完成 */
	uint8_t COF;       /* CORDIC溢出 */
	uint8_t LIN;       /* 线性度报警 */
	uint8_t MagINCn;   /* 磁场增加 */
	uint8_t MagDECn;   /* 磁场减少 */
	uint8_t parity;    /* 偶校验位 */
	uint8_t parity_ok; /* 校验是否正确 (1: 正确，0: 错误) */
} SSI_Data_t;

/* External SPI Handle Declaration */
extern SPI_HandleTypeDef SSI;
extern TIM_HandleTypeDef ENCODER_TIM_HANDLE;

/**
 * @brief 启动编码器采集定时器，周期性读取 AS5145 角度。
 * @return HAL 状态码。
 */
HAL_StatusTypeDef Start_Encoder_Collection_TIM(void);
/**
 * @brief 执行AS5145 编码器中的 AS5145_GetLastError 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t AS5145_GetLastError(void);
/* 是否已经收到过一帧有效 SSI 数据；供启动门控判断编码器是否真正可用。 */
bool AS5145_HasValidSample(void);
/* 等待首帧有效 SSI 数据；会短延时轮询，只能在任务上下文调用，不能在中断里调用。 */
uint32_t AS5145_WaitFirstValidSample(uint32_t timeout_ms);

#endif /* AS5145_H_ */
