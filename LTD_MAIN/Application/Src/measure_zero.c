/*
 * OriginCalibration.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */

/*
 * measureTankHeight.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */

#include "weight.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure_zero.h"

int32_t zero_value = 100000000;

static int SearchZeroRough();
static int SearchZeroPrecise();

/**
 * @brief 搜索零点位置
 *
 * 该函数通过粗找和细找两个阶段，完成零点位置的测量。
 * 粗找阶段通过多次尝试确定零点的大致位置，细找阶段则精确调整到零点位置。
 * 最终记录零点位置的编码值，并输出相关信息。
 *
 * @return int 返回状态码：
 *             - 0: 零点测量成功
 *             - 1: 粗找零点失败（超过最大尝试次数）
 *
 * @note 函数内部会调用以下辅助函数：
 *       - get_stable_weight(): 获取稳定重量
 *       - determine_weight_status(): 判断重量状态
 *       - SearchZeroRough(): 粗找零点
 *       - SearchZeroPrecise(): 细找零点
 *       - motorMoveNoWait(): 非阻塞电机移动
 *       - stpr_waitMove(): 等待电机移动完成
 *
 * @note 输出信息包括：
 *       - 零点测量进度
 *       - 零点位置的编码值
 *       - 零点位置的计算值（以毫米为单位）
 */
int SearchZero(void) // V1.120
{
	int try_times = 0;
	get_stable_weight();
	//第一次长距离下行
	while (determine_weight_status() != ZERO)
	{
		try_times++;
		SearchZeroRough();
		HAL_Delay(5000);
		if (try_times > 3)
			return 1; //寻找罐底三次尝试失败
	}
	//
	printf("零点测量\t粗找零点完成\r\n");
	motorMoveNoWait(200, MOTOR_DIRECTION_DOWN);
	HAL_Delay(5000);
	stpr_waitMove(&stepper);
	//精确寻找零点
	SearchZeroPrecise();
	printf("零点测量\t第一次细找零点完成\r\n");
	motorMoveNoWait(100, MOTOR_DIRECTION_DOWN);
	HAL_Delay(5000);
	stpr_waitMove(&stepper);
	SearchZeroPrecise();

	zero_value = encoder_count;
	printf("零点测量完成 \t当前编码值\t %ld\t（0.1MM)\r\n", zero_value);
	printf("{zero_value}%ld,%f\r\n", zero_value,
			(float) zero_value * 95.0 / 4096.0);
	motorMoveNoWait(400, MOTOR_DIRECTION_DOWN);//向下移动400mm
	HAL_Delay(5000);//等待5秒
	stpr_waitMove(&stepper);//等待移动完成
	return 0;
}

static int SearchZeroRough()
{
//	int v1flag = 0;
	motorMove_up();
	while (determine_weight_status() != ZERO)
	{
		printf("{encoder}%d\t{weight}%d\r\n", (int) encoder_count, weight);
//		if ((labs(encoder_count - bottom_value) < 4096)&&(v1flag == 0)) {
//			stpr_setVelocity(&stepper, 16 * 32 * 20);
//			v1flag = 1;
//			printf("{Velocity}%d\r\n", 20);
//		}
	}
	zero_value = encoder_count;
	motorQuickStop();
	return 0;
}
static int SearchZeroPrecise()
{
	int v1flag = 0;
	int v2flag = 0;
//	get_stable_weight();
	motorMove_up();
	while (determine_weight_status() != ZERO)
	{
		printf("{encoder}%d\t{weight}%d\r\n", (int) encoder_count, weight);
		if ((labs(encoder_count - zero_value) < 4096) && (v1flag == 0))
		{
			stpr_setVelocity(&stepper, 16 * 32 * 40);
			printf("{Velocity}%d\r\n", 40);
			v1flag = 1;
		}
		if ((labs(encoder_count - zero_value) < 100) && (v2flag == 0))
		{
			stpr_setVelocity(&stepper, 16 * 32 * 2);
			printf("{Velocity}%d\r\n", 2);
			v2flag = 1;
		}
	}
	zero_value = encoder_count;
	motorQuickStop();
	return 0;
}

