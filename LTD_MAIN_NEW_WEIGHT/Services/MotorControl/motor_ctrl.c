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
#define VELOCITY_MAX 2
uint32_t velocity = 1600 * VELOCITY_MAX * 32;

uint32_t motor_Init(void)
{
	stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 19);
	stpr_enableDriver(&stepper);
	return NO_ERROR; // 初始化电机，返回无错误状态
}

// 控制电机移动函数
uint32_t motorMoveNoWait(float mm, int dir)
{
	if (mm < 0)
	{
		return NO_ERROR; // 距离必须为正值，直接返回 duan
	}
	printf("start move %.2fmm, dir %d\n", mm, dir);
	// 计算对应的步数
	int32_t ticks = (int32_t) ((mm * 30 * 1600 * 32) / 600); // 转换为步数
	if (dir == MOTOR_DIRECTION_UP)
	{
		ticks = -ticks; // 如果方向向上，步数取负值
	}

	// 调用实际的步进电机控制函数
	stpr_moveBy(&stepper, &ticks, velocity); // 最大速度为 1600 * 2 * 32
	return NO_ERROR; // 返回无错误状态
}
uint32_t motorMoveAndWaitUntilStop(float mm, int dir)
{
	uint32_t ret;
	motorMoveNoWait(mm, dir);
	HAL_Delay(1000);
	ret = stpr_waitMove(&stepper);
	CHECK_ERROR(ret); // 等待电机移动完成，并检查是否有错误
	return NO_ERROR; // 等待电机移动完成
}
/**
 * @brief 电机带回差补偿的移动函数
 *        上行：正常走
 *        下行：多走20mm，再往上走20mm克服回差
 * @param mm   目标移动距离 (mm)
 * @param dir  方向：0=下行，1=上行
 * @return uint32_t 错误码
 */
uint32_t motorMoveWithBacklash(float mm, int dir)
{
    uint32_t ret;

    if (dir == MOTOR_DIRECTION_UP) {
        // 上行：正常走
        ret = motorMoveAndWaitUntilStop(mm, dir);
        CHECK_ERROR(ret);
    } else {
        // 下行：先比目标多走20mm
        ret = motorMoveAndWaitUntilStop(mm + 20.0f, dir);
        CHECK_ERROR(ret);

        // 再往上走20mm，消除回差
        ret = motorMoveAndWaitUntilStop(20.0f, 1);
        CHECK_ERROR(ret);
    }

    return NO_ERROR;
}

uint32_t motorMove_up(void)
{
	motorMoveNoWait(100000, MOTOR_DIRECTION_UP);
	return NO_ERROR; // 向上移动，返回无错误状态
}
uint32_t motorMove_down(void)
{
	motorMoveNoWait(100000, MOTOR_DIRECTION_DOWN);
	return NO_ERROR; // 向下移动，返回无错误状态
}
void motorRun(float distance_mm,      		// 移动距离（毫米）
		int direction,          			// 运动方向
		bool enable_step_loss_detection,  	// 是否启用丢步检测
		bool enable_weight_detection      	// 是否启用称重检测
		)
{

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
uint32_t motorQuickStop(void)
{
	printf("急刹前{encoder}%d\t{weight}%d\r\n", (int) g_encoder_count , g_weight);
	stpr_stop(&stepper);
	stpr_disableDriver(&stepper); //使能电机
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	stpr_enableDriver(&stepper); //使能电机
	printf("急刹后{encoder}%d\t{weight}%d\r\n", (int) g_encoder_count , g_weight);
	return NO_ERROR; // 返回无错误状态
}

//电机停止
uint32_t motorSlowStop(void)
{
	stpr_stop(&stepper);

	return NO_ERROR; // 返回无错误状态
}

