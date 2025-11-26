/*
 * test.c
 * 测试函数，用来临时测试某些功能
 *  Created on: Jul 22, 2025
 *      Author: 1
 */

#include <ltd_sensor_communication.h>
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
	printf("4步进测试\n");
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

	}
	printf("8步进测试\n");
		printf("start down\n");
		for (i = 0; i < 12000; i++) {
			ticks = 8 * 32;
			stpr_moveBy(&stepper, &ticks, velocity);
			HAL_Delay(2000);
			printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
			HAL_Delay(100);
			printf("%d\t", weight_parament.current_weight);
			HAL_Delay(100);
			printf("%d\r\n", weight_parament.current_weight);
		}
		printf("down over!\n");
		printf("start up\n");
		for (i = 0; i < 12000; i++) {
			ticks = -8 * 32;
			stpr_moveBy(&stepper, &ticks, velocity);
			HAL_Delay(2000);
			printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
			HAL_Delay(100);
			printf("%d\t", weight_parament.current_weight);
			HAL_Delay(100);
			printf("%d\r\n", weight_parament.current_weight);

		}
		printf("40步进测试\n");
			printf("start down\n");
			for (i = 0; i < 2400; i++) {
				ticks = 40 * 32;
				stpr_moveBy(&stepper, &ticks, velocity);
				HAL_Delay(2000);
				printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
				HAL_Delay(100);
				printf("%d\t", weight_parament.current_weight);
				HAL_Delay(100);
				printf("%d\r\n", weight_parament.current_weight);
			}
			printf("down over!\n");
			printf("start up\n");
			for (i = 0; i < 2400; i++) {
				ticks = -40 * 32;
				stpr_moveBy(&stepper, &ticks, velocity);
				HAL_Delay(2000);
				printf("%d\t{encoder}%d\t{weight}%d\t", i, (int) g_encoder_count, weight_parament.current_weight);
				HAL_Delay(100);
				printf("%d\t", weight_parament.current_weight);
				HAL_Delay(100);
				printf("%d\r\n", weight_parament.current_weight);

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
#include <ltd_sensor_communication.h>
#include <stdio.h>

/**
 * @brief  测试V2协议通讯与关键参数读取
 * @note   可在初始化完成后调用，例如 main() 或 sensor init 后
 */
void DSM_V2_Test_AllParams(void)
{
    printf("\r\n===== DSM V2 通讯测试开始 =====\r\n");

//    // 1. 切换到液位模式
//    int ret = DSM_V2_SwitchToLevelMode();
//    if (ret == NO_ERROR)
//        printf("切换液位模式成功\r\n");
//    else {
//        printf("切换液位模式失败，错误码 %d\r\n", ret);
//        return; // 通讯异常，后面读也没意义
//    }

    // 2. 定义变量
    float ver = 0, temp = 0, rho = 0, mu = 0, nu = 0;
    uint32_t freq = 0, sensor_id = 0;

    // 3. 依次读取各参数
//    if (DSM_V2_Read_SoftwareVersion(&ver) == NO_ERROR)
//        printf("软件版本: %.3f\r\n", ver);
//    else printf("读取软件版本失败\r\n");

    if (DSM_V2_Read_Temperature(&temp) == NO_ERROR)
    {
        printf("温度值: %.3f ℃\r\n", temp);
        g_measurement.single_point_monitoring.temperature = (int)(temp*100)+20000;
    }
    else printf("读取温度失败\r\n");

    if (DSM_V2_Read_Density(&rho) == NO_ERROR){
    	g_measurement.single_point_monitoring.density = (int)(rho*10);
    	printf("密度值: %.3f\r\n", rho);
    }

    else printf("读取密度失败\r\n");

    if (DSM_V2_Read_DynamicViscosity(&mu) == NO_ERROR)
        printf("动力粘度: %.3f\r\n", mu);
    else printf("读取动力粘度失败\r\n");

    if (DSM_V2_Read_KinematicViscosity(&nu) == NO_ERROR)
        printf("运动粘度: %.3f\r\n", nu);
    else printf("读取运动粘度失败\r\n");
//
//    if (DSM_V2_Read_LevelFrequency(&freq) == NO_ERROR)
//        printf("液位频率: %lu Hz\r\n", (unsigned long)freq);
//    else printf("读取液位频率失败\r\n");
//
//    if (DSM_V2_Read_SensorID(&sensor_id) == NO_ERROR)
//        printf("传感器号: %lu\r\n", (unsigned long)sensor_id);
//    else printf("读取传感器号失败\r\n");

    printf("===== DSM V2 通讯测试结束 =====\r\n\r\n");
}

