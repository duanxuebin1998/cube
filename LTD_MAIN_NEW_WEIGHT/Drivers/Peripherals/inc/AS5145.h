/*
 * AS5145.h
 *
 * Created on: Dec 12, 2024
 * Author: 1
 * Description: Header file for AS5145 SPI-based rotary encoder.
 */

#ifndef AS5145_H_
#define AS5145_H_

#include "main.h"

// SPI Interface and CSN Pin Definitions
#define SSI             hspi5               // SPI interface used for AS5145 communication
#define SSI_CSN_PIN     GPIO_PIN_6          // Chip Select (CS) pin for AS5145
#define SSI_CSN_PORT    GPIOF               // GPIO port for CS pin
#define ENCODER_TIM_HANDLE  htim1              //编码器定时采集定时器

typedef enum {
	State_IDLE,
    RETRY_IN_PROGRESS,
    RETRY_FAILED
} EncoderState;
// 数据结构体
typedef struct
{
	uint16_t angle;    // 12位角度数据 (D11:D0)
	uint8_t OCF;       // 偏差补偿完成
	uint8_t COF;       // CORDIC溢出
	uint8_t LIN;       // 线性度报警
	uint8_t MagINCn;   // 磁场增加
	uint8_t MagDECn;   // 磁场减少
	uint8_t parity;    // 偶校验位
	uint8_t parity_ok; // 校验是否正确 (1: 正确，0: 错误)
} SSI_Data_t;

// External SPI Handle Declaration
extern SPI_HandleTypeDef SSI;
extern TIM_HandleTypeDef ENCODER_TIM_HANDLE;

HAL_StatusTypeDef Start_Encoder_Collection_TIM(void);

#endif /* AS5145_H_ */
