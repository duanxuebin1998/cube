/*
 * measureTankHeight.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */

#include "measure_tank_height.h"
#include "weight.h"
#include <stdio.h>
#include <stdlib.h>

int32_t bottom_value = 100000000;

static int SearchBottomRough();
static int SearchBottomPrecise();

uint32_t SearchBottom(void)
{
	int try_times = 0;
	get_stable_weight();
	//第一次长距离下行
	while (determine_weight_status() != BOTTOM)
	{
		try_times++;
		SearchBottomRough();
		if (try_times > 3)
			return update_and_encode_fault(FAULT_MEASUREMENT,
					MEASUREMENT_WEIGHT_DOWN_FAIL, 1, FAULT_SEVERITY_ERROR); //寻找罐底三次尝试失败
	}
	//
	motorMoveNoWait(200, MOTOR_DIRECTION_UP);
	HAL_Delay(5000);
	stpr_waitMove(&stepper); //等待电机停止
	//精确寻找罐底
	SearchBottomPrecise();

	motorMoveNoWait(100, MOTOR_DIRECTION_UP);	//上行重新寻找
	HAL_Delay(5000);
	stpr_waitMove(&stepper);	//等 待
	SearchBottomPrecise();

	bottom_value = encoder_count;
	printf("罐底测量完成 \t罐底\t %ld\t（0.1MM)\r\n", bottom_value);
	printf("{bottom_value}%ld,%f\r\n", bottom_value,
			(float) bottom_value * 95.0 / 4096.0);
	motorMoveNoWait(400, MOTOR_DIRECTION_UP);
	HAL_Delay(5000);
	stpr_waitMove(&stepper);
	return NO_ERROR;
}

static int SearchBottomRough()
{
//	int v1flag = 0;
	motorMove_down();
	while (determine_weight_status() != BOTTOM)
	{
		printf("{encoder}%d\t{weight}%d\r\n", (int) encoder_count, weight);
//		if ((labs(encoder_count - bottom_value) < 4096)&&(v1flag == 0))
//		{
//			stpr_setVelocity(&stepper, 16 * 32 * 20);
//			v1flag = 1;
//			printf("{Velocity}%d\r\n", 20);
//		}
	}
	bottom_value = encoder_count;
	motorQuickStop();
	return NO_ERROR;
}
static int SearchBottomPrecise()
{
	int v1flag = 0;
	int v2flag = 0;
	get_stable_weight();
	motorMove_down();
	while (determine_weight_status() != BOTTOM)
	{
		printf("{encoder}%d\t{weight}%d\r\n", (int) encoder_count, weight);
		if ((labs(encoder_count - bottom_value) < 4096) && (v1flag == 0))
		{
			stpr_setVelocity(&stepper, 16 * 32 * 40);
			printf("{Velocity}%d\r\n", 40);
			v1flag = 1;
		}
		if ((labs(encoder_count - bottom_value) < 100) && (v2flag == 0))
		{
			stpr_setVelocity(&stepper, 16 * 32 * 2);
			printf("{Velocity}%d\r\n", 2);
			v2flag = 1;
		}
	}
	bottom_value = encoder_count;
	motorQuickStop();
	return NO_ERROR;
}

