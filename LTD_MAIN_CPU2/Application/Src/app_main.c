/*
 * @FilePath     : \CUBE\LTD_MAIN_CPU2\Application\Src\app_main.c
 * @Description  : 主函数
 * @Author       : Aubon
 * @Date         : 2026-02-03 14:06:14
 * @LastEditors  : Duan Xuebin
 * @LastEditTime : 2026-03-17 11:34:14
 * Copyright 2026 Aubon, All Rights Reserved. 
 * 2026-02-03 14:06:14
 */

#include "app_main.h"
#include "main.h"
#include "system_parameter.h"
#include "motor_ctrl.h"
#include "measure.h"
#include "encoder.h"
#include "hart.h"
#include "hostcommu.h"
#include "test.h"
#include "ad5421.h"
#include "sensor.h"

// 初始化函数
void App_Init(void) {
	printf("LTD restart!\n");
	HAL_Delay(1000); // 延时1000ms
	Initialize_Encoder(); // 初始化编码器
	motor_Init(); //电机初始化
	HartInit(); // 初始化AD5421
	init_device_params(); // 初始化设备参数
	weight_init();
	HostCommuInit(); // 初始化Modbus通信
	AD5421_SetCurrent(6.0); // 设置初始电流为4mA
	motor_Init(); //电机初始化
	fault_info_init(); // 初始化故障信息
	DetectSensorType(); // 检测传感器类型
	g_deviceParams.command = CMD_NONE; // 清除命令
	g_measurement.device_status.zero_point_status=1; // 设置零点状态为需要回零点
	if (g_deviceParams.powerOnDefaultCommand != CMD_NONE) {
		g_deviceParams.command = DefaultCmd_To_MeasureCmd(g_deviceParams.powerOnDefaultCommand); // 上电默认命令
		printf("上电默认命令：%d\r\n", g_deviceParams.command);
	}
	//测试函数
//	Test_main(); // 测试函数
//	motor_text(); //电机测试
}
// 主循环任务
void App_MainLoop(void) {
	// 如果有新的命令
	if (new_command_ready) {
		new_command_ready = 0;  // 重置标志，避免重复处理

		// 处理接收到的命令
		process_command(received_buffer);
	}
	if (g_deviceParams.command != CMD_NONE) {
		printf("当前命令：%d\r\n", g_deviceParams.command);
		g_measurement.device_status.current_command = g_deviceParams.command; // 更新当前命令
		g_deviceParams.command = CMD_NONE; // 清除命令
		ProcessMeasureCmd(g_measurement.device_status.current_command); // 处理测量命令
		g_measurement.device_status.current_command =CMD_NONE; // 重置当前命令
		WIRELESS_PrintInfo(01); // 打印无线传感器信息
		WIRELESS_PrintInfo(02); // 打印无线传感器信息
		//
	}
//		DSM_V2_Test_AllParams(); // 二代传感器测试函数
//		Sensor_Test(); // 传感器测试
//		Test_FRAM_ReadWrite();
//		printf("{encoder}%d\r\n{weight}%d\r\n", (int) g_encoder_count, g_weight);
//		printf("位置%d", g_measurement.debug_data.sensor_position);
//		HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET);
//		HAL_UART_Transmit_DMA(&huart2, "123456", 6);  // 通过UART发送响应
	process_device_params_deferred_tasks();
	HAL_Delay(50); // 延时50ms
}
