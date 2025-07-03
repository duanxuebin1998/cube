/*
 * encoder.c
 *
 *  Created on: Mar 21, 2025
 *      Author: 1
 */

#include "AS5145.h"
#include "encoder.h"
#include "mb85rs2m.h"
volatile int32_t encoder_count = 0;            //编码计数值
volatile int32_t sensor_position = 0;            //传感器位置
static uint16_t prev_angle = 0;              // 上一次的角度值
static const uint16_t MAX_ANGLE = 4096;      // 最大角度值 (12位分辨率)

static void WriteEncoderData(int32_t encoder_count, uint16_t prev_angle); // 写入编码计数和上次角度值到 FRAM
static void ReadEncoderData(volatile int32_t *encoder_count,
		uint16_t *prev_angle);      // 从 FRAM 读取编码计数和上次角度值

// 在系统启动时初始化编码计数器
void Initialize_Encoder(void)
{
	// 从 FRAM 读取编码计数和上次角度
//	ReadEncoderData(&encoder_count, &prev_angle);

	//判断位置是否发生变化

	Start_Encoder_Collection_TIM(); //启动编码器定时采集
}

// 更新编码计数
void Update_Encoder_Count(uint16_t current_angle)
{
	int16_t delta = current_angle - prev_angle;

	// 处理跨零点情况
	if (delta > (MAX_ANGLE / 2))
	{
		delta -= MAX_ANGLE;
	}
	else if (delta < -(MAX_ANGLE / 2))
	{
		delta += MAX_ANGLE;
	}

	encoder_count += delta;
	prev_angle = current_angle; // 更新上一角度值

	// 每次更新后，写入编码值到 FRAM
	WriteEncoderData(encoder_count, prev_angle);
}


// 写入编码计数和上次角度值到 FRAM
static void WriteEncoderData(int32_t encoder_count, uint16_t prev_angle)
{
	WriteSingleData(encoder_count, FRAM_ANGLE_ADDRESS); // 写入编码计数到 FRAM 地址 0x0000
	WriteSingleData(prev_angle, FRAM_ENCODER_ADDRESS); // 写入上次角度值到 FRAM 地址 0x0004
}
// 从 FRAM 读取编码计数和上次角度值
static void ReadEncoderData(volatile int32_t *encoder_count,
		uint16_t *prev_angle)
{
	*encoder_count = ReadSingleData(FRAM_ANGLE_ADDRESS); // 从 FRAM 地址 0x0000 读取编码计数
	*prev_angle = ReadSingleData(FRAM_ENCODER_ADDRESS); // 从 FRAM 地址 0x0004 读取上次角度值
}

