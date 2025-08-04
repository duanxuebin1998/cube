/*
 * encoder.h
 *
 *  Created on: Mar 21, 2025
 *      Author: 1
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

extern volatile int32_t g_encoder_count ;            // 编码计数值
// 函数接口声明

void Initialize_Encoder(void);// 在系统启动时初始化编码计数器
void Update_Encoder_Count(uint16_t current_angle);
void set_encoder_zero(void); // 设置编码器零点
#endif /* ENCODER_ENCODER_H_ */
