/*
 * encoder.c
 *
 *  Created on: Mar 21, 2025
 *      Author: 1
 */

#include "AS5145.h"
#include "encoder.h"
#include "mb85rs2m.h"
#include <stdio.h>
#include <stdlib.h>
//全局变量
volatile int32_t g_encoder_count = 0;            //编码计数值
volatile int32_t g_encoder_saved = 0;
static uint16_t prev_angle = 0;              // 上一次的角度值
static const uint16_t MAX_ANGLE = 4096;      // 最大角度值 (12位分辨率)

static void WriteEncoderData(int32_t encoder_count, uint16_t prev_angle); // 写入编码计数和上次角度值到 FRAM
static void ReadEncoderData(volatile int32_t *encoder_count, uint16_t *prev_angle);      // 从 FRAM 读取编码计数和上次角度值
// 在系统启动时初始化编码计数器
void Initialize_Encoder(void) {

	// 从 FRAM 读取编码计数和上次角度
	ReadEncoderData(&g_encoder_count, &prev_angle);
	g_encoder_saved = g_encoder_count; // 保存初始编码计数值
	//判断位置是否发生变化

	Start_Encoder_Collection_TIM(); //启动编码器定时采集
}

// 更新编码计数
void Update_Encoder_Count(uint16_t current_angle) {
	int16_t delta = current_angle - prev_angle;

	// 处理跨零点情况
	if (delta > (MAX_ANGLE / 2)) {
		delta -= MAX_ANGLE;
	} else if (delta < -(MAX_ANGLE / 2)) {
		delta += MAX_ANGLE;
	}

	g_encoder_count += delta;
	prev_angle = current_angle; // 更新上一角度值

	if ((abs((int)g_encoder_count - (int)g_encoder_saved) > 3)||(g_measurement.debug_data.sensor_position ==0 )) {
		WriteEncoderData(g_encoder_count, prev_angle); // 写入编码计数和上次角度值到 FRAM
		g_encoder_saved = g_encoder_count; // 更新保存的编码计数值
		update_sensor_height_from_encoder(); // 更新传感器高度测量
	}
}

// 写入编码计数和上次角度值到 FRAM
static void WriteEncoderData(int32_t encoder_count, uint16_t prev_angle) {
	WriteSingleData(encoder_count, FRAM_ANGLE_ADDRESS); // 写入编码计数到 FRAM 地址 0x0000
	WriteSingleData(prev_angle, FRAM_ENCODER_ADDRESS); // 写入上次角度值到 FRAM 地址 0x0004
}
// 从 FRAM 读取编码计数和上次角度值
static void ReadEncoderData(volatile int32_t *encoder_count, uint16_t *prev_angle) {
	// 从 FRAM 地址 0x0000 读取编码计数
	*encoder_count = ReadSingleData(FRAM_ANGLE_ADDRESS);

	// 从 FRAM 地址 0x0004 读取上次角度值
	*prev_angle = ReadSingleData(FRAM_ENCODER_ADDRESS);
	update_sensor_height_from_encoder(); // 更新传感器高度测量
}

/**
 * 根据编码器脉冲值更新传感器高度测量
 * @param encoder_value 编码器原始计数值
 *
 * 功能说明：
 * 1. 将编码器值取反
 * 2. 计算当前位置高度
 * 3. 更新测量系统中的高度和编码值
 */
void update_sensor_height_from_encoder(void) {
	// 更新调试信息：当前编码值取负（编码器方向取反）
	g_measurement.debug_data.current_encoder_value = -g_encoder_count;
	// 1. 计算转动的总圈数（含小数）
	float revolutions = (float) g_measurement.debug_data.current_encoder_value / (float) MAX_ANGLE;
	// 2. 计算尺带的收放长度
	float cable_length = g_deviceParams.encoder_wheel_circumference_mm * revolutions / 100;
	// 3. 计算当前传感器高度（油罐高度减去悬吊长度）
	float current_height = g_deviceParams.tankHeight - cable_length;
	// 更新调试数据（高度值）
	g_measurement.debug_data.cable_length = (int) cable_length; // 更新尺带长度
	g_measurement.debug_data.sensor_position = (int) current_height;
	// 返回传感器高度
	return;
}
//设置编码器零点
void set_encoder_zero(void) {
	printf("设置编码器零点，当前置零点前零点编码值 %ld\r\n", g_encoder_count);
	// 设置零点编码值
	g_encoder_count = 0;
	// 更新调试信息
	g_measurement.debug_data.current_encoder_value = -g_encoder_count;
	// 写入新的零点到 FRAM
	printf("编码器零点设置为 %ld\r\n", g_encoder_count);
}
