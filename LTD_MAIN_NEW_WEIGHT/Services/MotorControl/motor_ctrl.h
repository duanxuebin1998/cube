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

uint32_t motorQuickStop(void);// 电机快速停止
uint32_t motorSlowStop(void);// 电机慢速停止
void MotorLostStep_Init(void);// 丢步检测初始化
uint32_t Motor_CheckLostStep_AutoTiming(int32_t currentPos);// 电机丢步检测

uint32_t motorMove_up(void);// 电机向上移动
uint32_t motorMove_down(void);// 电机向下移动
uint32_t motorMoveNoWait(float mm, int dir);// 电机移动不等待
uint32_t motorMoveAndWaitUntilStop(float mm, int dir);// 电机移动并等待停止
uint32_t motorMoveToPositionOneShot(float target_mm);// 电机移动到指定位置（单次调用）
bool stpr_isMoving(TMC5130TypeDef *tmc5130);


#endif /* INC_MOTOR_CTRL_H_ */
