/*
 * motor_ctrl.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 */

#ifndef INC_MOTOR_CTRL_H_
#define INC_MOTOR_CTRL_H_

#include "main.h"
#include "../../Peripherals/inc/TMC5130.h"

#define MOTOR_DIRECTION_DOWN 0
#define MOTOR_DIRECTION_UP 1

extern uint32_t velocity;

void motor_Init(void); //电机初始化
void motorMoveNoWait(float mm, int dir);
void motorMove_up(void);
void motorMove_down(void);

void motor_text(void);

//电机紧急刹车
void motorQuickStop(void);
//电机停止
void motorSlowStop(void);

//motorMoveForward(int distance) // 前进指定距离
//motorMoveBackward(int distance) // 后退指定距离
//motorMoveToPosition(int position) // 移动到指定位置
//motorSetVelocity(int velocity) // 设置速度
//motorAdjustSpeed(float factor) // 调整速度
//
//motorGetStatus() // 获取电机状态
//motorIsRunning() // 检查电机是否正在运行
//motorGetPosition() // 获取当前位置
//motorGetVelocity() // 获取当前速度
//
//motorInit() // 初始化电机
//motorDeinit() // 释放电机资源
//motorSetParameters(params_t *params) // 设置电机参数

#endif /* INC_MOTOR_CTRL_H_ */
