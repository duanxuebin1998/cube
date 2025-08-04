/*
 * test.c
 * 测试函数，用来临时测试某些功能
 *  Created on: Jul 22, 2025
 *      Author: 1
 */

#include "test.h"
#include "measure.h"
#include "motor_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include "system_parameter.h"
#include "measure_tank_height.h"
#include "weight.h"
#include "system_parameter.h"

//电机小步进上行测试
void motor_step_up_text(void) {
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
	printf("start up\n");
	for (i = 0; i < 24000; i++) {
		ticks = -4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, g_weight);
		HAL_Delay(100);
		printf("%d\t", g_weight);
		HAL_Delay(100);
		printf("%d\r\n", g_weight);
	}
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}
//电机小步进下行测试
void motor_step_down_text(void) {
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
	printf("start down\n");
	for (i = 0; i < 24000; i++) {
		ticks = 4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, g_weight);
		HAL_Delay(100);
		printf("%d\t", g_weight);
		HAL_Delay(100);
		printf("%d\r\n", g_weight);
	}
	printf("down over!\n");
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}
//电机步进测试
void motor_step_text(void) {
	int i = 0;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
	printf("start down\n");
	for (i = 0; i < 24000; i++) {
		ticks = 4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, g_weight);
		HAL_Delay(100);
		printf("%d\t", g_weight);
		HAL_Delay(100);
		printf("%d\r\n", g_weight);
	}
	printf("down over!\n");
	printf("start up\n");
	for (i = 0; i < 24000; i++) {
		ticks = -4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, g_weight);
		HAL_Delay(100);
		printf("%d\t", g_weight);
		HAL_Delay(100);
		printf("%d\r\n", g_weight);
	}
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}

///*********************** 测试函数 ***********************/
void Test_Params_Storage(void) {
	// 备份原始参数
	DeviceParameters original = g_deviceParams;

	// 测试写读校验
	g_deviceParams.tankHeight = 1234;// 测试数据
	save_device_params();//存储

	if (load_device_params()) {
		if (g_deviceParams.tankHeight != 1234) {
			printf("数据加载失败");
			// 数据验证失败处理
		} else {
			printf("数据加载成功: tankHeight = %lu", g_deviceParams.tankHeight);
		}
	} else {
		printf("CRC校验失败");
	}

	// 恢复原始参数
	g_deviceParams = original;
	save_device_params();//存储
}
