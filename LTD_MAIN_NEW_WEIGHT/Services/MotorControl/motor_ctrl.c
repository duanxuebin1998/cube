/*
 * motor_ctrl.c
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 */

#include "motor_ctrl.h"
#include "spi.h"
#include <stdio.h>
#include <stdlib.h>
#include "../../Peripherals/inc/TMC5130.h"

uint32_t velocity = 1600 * 2 * 32;

void motor_Init(void)
{
	stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 25);
	stpr_enableDriver(&stepper);
}

// 控制电机移动函数
void motorMoveNoWait(float mm, int dir)
{
	if (mm < 0)
	{
		return; // 距离必须为正值，直接返回
	}
	printf("startp%.2f\n", mm);
	// 计算对应的步数
	int32_t ticks = (int32_t) ((mm * 30 * 1600 * 32) / 600); // 转换为步数
	if (dir == MOTOR_DIRECTION_UP)
	{
		ticks = -ticks; // 如果方向向上，步数取负值
	}

	// 调用实际的步进电机控制函数
	stpr_moveBy(&stepper, &ticks, velocity); // 最大速度为 1600 * 2 * 32
}
void motorMoveAndWaitUntilStop(float mm, int dir)
{
	motorMoveNoWait(mm, dir);
	HAL_Delay(1000);
	stpr_waitMove(&stepper);
}
void motorMove_up(void)
{
	printf("start up\n");
	motorMoveNoWait(100000, MOTOR_DIRECTION_UP);
}
void motorMove_down(void)
{
	printf("start down\n");
	motorMoveNoWait(100000, MOTOR_DIRECTION_DOWN);
}
void motor_text(void)
{
	printf("motor text start\n");
	while (1)
	{
		stpr_enableDriver(&stepper); //使能电机
//		stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
		motorMoveNoWait(1000, MOTOR_DIRECTION_DOWN);
		HAL_Delay(1000);
		printf("start down\n");
		stpr_waitMove(&stepper);
		printf("down over!\n");
		HAL_Delay(1000);
		stpr_moveTo(&stepper, 0, velocity);
		printf("start up to zero\n");
		HAL_Delay(1000);
		stpr_waitMove(&stepper);
		printf("上行结束\n");
		HAL_Delay(1000);
		stpr_disableDriver(&stepper); //使能电机
	}
}

//电机紧急刹车
void motorQuickStop(void)
{
	printf("急刹前{encoder}%d\t{weight}%d\r\n", (int) encoder_count, weight);
	stpr_stop(&stepper);
	stpr_disableDriver(&stepper); //使能电机
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	stpr_enableDriver(&stepper); //使能电机
	printf("急刹后{encoder}%d\t{weight}%d\r\n", (int) encoder_count, weight);
}

//电机停止
void motorSlowStop(void)
{
	stpr_stop(&stepper);
}

