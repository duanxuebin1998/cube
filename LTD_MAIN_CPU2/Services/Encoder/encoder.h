/*
 * encoder.h
 *
 *  Created on: Mar 21, 2025
 *      Author: Duan Xuebin
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

extern volatile int32_t g_encoder_count ;            /* 编码计数值 */
/* 函数接口声明 */

void Initialize_Encoder(void); /* 在系统启动时初始化编码计数器 */
/* 编码器是否已经具备首帧有效位置，供电机启动门控使用。 */
bool Encoder_IsReady(void);
/* 等待编码器首帧有效位置；内部会阻塞等待，不能在中断上下文调用。 */
uint32_t Encoder_WaitReady(uint32_t timeout_ms);
/**
 * @brief 根据 AS5145 当前角度刷新编码器累计计数。
 * @param current_angle 当前单圈角度计数。
 */
void Update_Encoder_Count(uint16_t current_angle);
/**
 * @brief 根据编码器计数刷新传感器高度调试值。
 */
void update_sensor_height_from_encoder(void);
/**
 * @brief 强制按编码器计数刷新传感器高度调试值。
 */
void update_sensor_height_from_encoder_force(void); /* 根据编码器计数更新传感器高度测量值 */
/**
 * @brief 写入或设置本模块中的 set_encoder_zero 逻辑。
 */
void set_encoder_zero(void);
/**
 * @brief 执行本模块中的 encoder_set_cable_length_01mm 逻辑。
 *
 * @param cable_length_01mm 数据长度。
 */
void encoder_set_cable_length_01mm(int32_t cable_length_01mm); /* 按尺带长度(0.1mm)修正编码器计数 / / 设置编码器零点 */
/**
 * @brief 执行本模块中的 encoder_get_cable_length_01mm 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int32_t encoder_get_cable_length_01mm(void); /* 只按编码轮计算尺带长度，不改写全局位置 */
/**
 * @brief 执行本模块中的 encoder_get_sensor_position_01mm 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int32_t encoder_get_sensor_position_01mm(void); /* 只按编码轮计算传感器位置，不改写全局位置 */
#endif /* ENCODER_ENCODER_H_ */
