/*
 * motor_ctrl.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 */

#ifndef INC_MOTOR_CTRL_H_
#define INC_MOTOR_CTRL_H_

//#include "main.h"
#include "TMC5130.h"

#define MOTOR_DIRECTION_DOWN 0
#define MOTOR_DIRECTION_UP 1

extern uint32_t velocity;

uint32_t motor_Init(void); //电机初始化
uint32_t motorMoveNoWait(float mm, int dir);// 电机移动不等待
uint32_t motorMoveAndWaitUntilStop(float mm, int dir);// 电机移动并等待停止
uint32_t motorMoveWithBacklash(float mm, int dir);// 电机移动带回差补偿
uint32_t motorMove_up(void);// 电机向上移动
uint32_t motorMove_down(void);// 电机向下移动
uint32_t motorQuickStop(void);// 电机快速停止
uint32_t motorSlowStop(void);// 电机慢速停止
void motor_text(void);
void MotorLostStep_Init(void);// 丢步检测初始化
uint32_t Motor_CheckLostStep_AutoTiming(int32_t currentPos);// 电机丢步检测
uint32_t motorMoveToPositionOneShot(float target_mm);// 电机移动到指定位置（单次调用）
//uint32_t stpr_wait_until_stop(TMC5130TypeDef *tmc5130);// 等待电机停止
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
