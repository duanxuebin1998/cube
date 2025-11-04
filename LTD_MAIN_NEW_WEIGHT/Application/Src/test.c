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
		printf("%d\t{传感器位置}%.1f\t{称重值}%d\r\n",  i,(float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
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
		printf("%d\t{传感器位置}%.1f\t{称重值}%d\r\n",  i,(float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
	}
	printf("down over!\n");
	stpr_disableDriver(&stepper); //使能电机
	printf("motor text over\n");
}
//电机步进测试
void motor_step_text(void) {
	int i = 0;
	float density, viscosity, temp;
	int32_t ticks = 4 * 32;
	printf("motor STEP text start\n");
	stpr_enableDriver(&stepper); //使能电机
	printf("start down\n");
	for (i = 0; i < 24000; i++) {
		ticks = 4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
		if (Read_Density_Temp(&density, &viscosity, &temp) == 0) {
			printf("密度: %.3f  粘度: %.3f  温度: %.3f ℃\r\n", density, viscosity, temp);
		} else {
			printf("读取密度/温度失败！\r\n");
		}
	}
	printf("down over!\n");
	printf("start up\n");
	for (i = 0; i < 24000; i++) {
		ticks = -4 * 32;
		stpr_moveBy(&stepper, &ticks, velocity);
		HAL_Delay(2000);
		printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\t", weight_parament.current_weight);
		HAL_Delay(100);
		printf("%d\r\n", weight_parament.current_weight);
		if (Read_Density_Temp(&density, &viscosity, &temp) == 0) {
			printf("密度: %.3f  粘度: %.3f  温度: %.3f ℃\r\n", density, viscosity, temp);
		} else {
			printf("读取密度/温度失败！\r\n");
		}
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
void motor_text(void) {
	printf("motor text start\n");
	while (1) {
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
