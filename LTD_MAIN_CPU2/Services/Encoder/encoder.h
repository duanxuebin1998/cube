/*
 * encoder.h
 *
 *  Created on: Mar 21, 2025
 *      Author: Duan Xuebin
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

extern volatile int32_t g_encoder_count ;            // 编码计数值
// 函数接口声明

void Initialize_Encoder(void);// 在系统启动时初始化编码计数器
void Update_Encoder_Count(uint16_t current_angle);
void update_sensor_height_from_encoder(void);
void update_sensor_height_from_encoder_force(void); // 根据编码器计数更新传感器高度测量值
void set_encoder_zero(void);
void encoder_set_cable_length_01mm(int32_t cable_length_01mm); // 按尺带长度(0.1mm)修正编码器计数 // 设置编码器零点
int32_t encoder_get_cable_length_01mm(void); // 只按编码轮计算尺带长度，不改写全局位置
int32_t encoder_get_sensor_position_01mm(void); // 只按编码轮计算传感器位置，不改写全局位置
#endif /* ENCODER_ENCODER_H_ */
