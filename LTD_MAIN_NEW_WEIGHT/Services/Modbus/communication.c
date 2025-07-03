/*
 * communication.c
 *
 *  Created on: Apr 10, 2025
 *      Author: 1
 */
#include "communication.h"
#include "measure.h"
#include "motor_ctrl.h"
#include <stdio.h>
#include <stdlib.h>

void motor_step_up_text(void)
{
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
//		stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
	printf("start up\n");
	for (i = 0; i < 24000; i++)
	{
		ticks = -4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) encoder_count, weight);
		HAL_Delay(100);
		printf("%d\t", weight);
		HAL_Delay(100);
		printf("%d\r\n", weight);
	}
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}
void motor_step_down_text(void)
{
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
//		stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
	printf("start down\n");
	for (i = 0; i < 24000; i++)
	{
		ticks = 4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) encoder_count, weight);
		HAL_Delay(100);
		printf("%d\t", weight);
		HAL_Delay(100);
		printf("%d\r\n", weight);
	}
	printf("down over!\n");
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}

void motor_step_text(void)
{
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
//		stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
	printf("start down\n");
	for (i = 0; i < 24000; i++)
	{
		ticks = 4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) encoder_count, weight);
		HAL_Delay(100);
		printf("%d\t", weight);
		HAL_Delay(100);
		printf("%d\r\n", weight);
	}
	printf("down over!\n");
	printf("start up\n");
	for (i = 0; i < 24000; i++)
	{
		ticks = -4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) encoder_count, weight);
		HAL_Delay(100);
		printf("%d\t", weight);
		HAL_Delay(100);
		printf("%d\r\n", weight);
	}
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}

// 处理接收到的命令
void process_command(uint8_t *command)
{
	printf("Command received\n");
//	stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
//	stpr_enableDriver(&stepper);
	if (command[0] == 'A')
	{
		// 处理命令
		if (command[1] == '0')
		{
			// 刹车操作
			motorQuickStop();
		}
		else if (command[1] == '+')
		{
			int mm = atoi((char*) &command[2]);  // 获取数字
			printf("start up%d\n", mm);
			motorMoveNoWait((float) mm, MOTOR_DIRECTION_UP);  // 电机上行
		}
		else if (command[1] == '-')
		{
			int mm = atoi((char*) &command[2]);  // 获取数字
			printf("start down%d\n", mm);
			motorMoveNoWait((float) mm, MOTOR_DIRECTION_DOWN);  // 电机下行
		}
	}
	if (command[0] == 'B')
	{
		printf("motor text start\n");
		while (1)
		{
			stpr_enableDriver(&stepper);  //使能电机
//						stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
					//		stpr_moveTo(&stepper, -30 * 1600 * 32, 1600 * 2 * 32);
			motorMoveNoWait(200000, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			printf("start down\n");
			while (abs(encoder_count) < 2400000)
			{
				printf("{encoder}%d\t{weight}%d\r\n", (int) encoder_count,
						weight);
				HAL_Delay(50);
			}
			printf("down over!\n");
			HAL_Delay(1000);
			motorMoveNoWait(200000, MOTOR_DIRECTION_UP);
			printf("start up to zero\n");
			HAL_Delay(1000);
			while (abs(encoder_count) > 4096)
			{
				printf("{encoder}%d\t{weight}%d\r\n", (int) encoder_count,
						weight);
				HAL_Delay(50);
			}
			motorQuickStop();
			printf("上行结束\n");
			HAL_Delay(1000);
			stpr_disableDriver(&stepper); //使能电机
		}
	}
	if (command[0] == 'C')
	{
		printf("***电机4步进分辨率测试***\r\n");
		motor_step_text();
	}
	if (command[0] == 'D')
	{
		printf("***电机4步进下行触底测试***\r\n");
		motor_step_down_text();
	}
	if (command[0] == 'E')
	{
		printf("***电机4步进上行碰零点测试***\r\n");
		motor_step_up_text();
	}
	if (command[0] == 'F')
	{
		printf("***罐底测量重复性测试***\r\n");
		while (1)
		{
			SearchBottom();
		}
	}
	if (command[0] == 'G')
	{
		printf("***罐底测量单次测试***\r\n");
		SearchBottom();
	}
	if (command[0] == 'H')
	{
		printf("***零点测量重复性测试***\r\n");
		while (1)
		{
			MeasureZero();
			SearchBottom();
		}
	}
	if (command[0] == 'I')
	{
		printf("***零点测试单次测试***\r\n");
		MeasureZero();

	}
	if (command[0] == 'J')
	{
		printf("***液位测量重复性测试***\r\n");

	}
	if (command[0] == 'K')
	{
		printf("***液位测量单次测试***\r\n");

	}
	if (command[0] == 'L')
	{
		printf("***空气中称重阈值***\r\n");

	}
	if (command[0] == 'M')
	{
		printf("***电机高温测试***\r\n");
		while (1)
		{
			motorMoveNoWait(100000, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			stpr_waitMove(&stepper);
		}

	}
	if (command[0] == 'N')
	{
		printf("motor text start\n");
		while (1)
		{
			stpr_enableDriver(&stepper);  //使能电机
//						stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
					//		stpr_moveTo(&stepper, -30 * 1600 * 32, 1600 * 2 * 32);
			motorMoveNoWait(300, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			printf("start down\n");
			HAL_Delay(1000);
			printf("down over!\n");
			stpr_waitMove(&stepper);
			motorMoveNoWait(300, MOTOR_DIRECTION_UP);
			printf("start up to zero\n");
			HAL_Delay(1000);
			stpr_waitMove(&stepper);
			printf("上行结束\n");
			HAL_Delay(1000);
			stpr_disableDriver(&stepper); //使能电机
		}
	}
}
