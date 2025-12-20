/*
 * app_main.c
 *
 *  Created on: Mar 13, 2025
 *      Author: Duan Xuebin
 */

#include "app_main.h"
#include "main.h"
#include "system_parameter.h"
#include "measure.h"
#include "encoder.h"
#include "hart.h"
#include "hostcommu.h"
//测试用
#include "sensor.h"
#include <mb85rs2m.h>
#include "my_crc.h"
#include "test.h"
#include "motor_ctrl.h"
#include "ad5421.h"
void Test_main(void); // 测试函数声明
// 初始化函数
void App_Init(void) {
	printf("LTD demo restart!\n");
	Initialize_Encoder(); // 初始化编码器
	motor_Init(); //电机初始化
	HartInit(); // 初始化AD5421
	init_device_params(); // 初始化设备参数
	g_deviceParams.command = CMD_NONE; // 清除命令
	g_measurement.device_status.zero_point_status=1; // 设置零点状态为需要回零点
	weight_init();
	HostCommuInit(); // 初始化Modbus通信
	//测试函数
//	Test_main(); // 测试函数
	AD5421_SetCurrent(6.0); // 设置初始电流为4mA
	motor_Init(); //电机初始化
	fault_info_init(); // 初始化故障信息
	DetectSensorType(); // 检测传感器类型
}
// 主循环任务
void App_MainLoop(void) {


	while (1) {
		// 如果有新的命令
		if (new_command_ready) {
			new_command_ready = 0;  // 重置标志，避免重复处理

			// 处理接收到的命令
			process_command(received_buffer);
		}
		if (g_deviceParams.command != CMD_NONE) {
//			g_measurement.device_status.zero_point_status = 0; // 重置零点状态，表示不需要回零点
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
//		DSMSendcommand3times(DSM_POWER, strlen(DSM_POWER));
//		DSMSendcommand3times(DSM_SENSORGET, strlen(DSM_SENSORGET));
//		Probe_EnableWaterSensor();
//		printf("{encoder}%d\r\n{weight}%d\r\n", (int) g_encoder_count, g_weight);
//		printf("位置%d", g_measurement.debug_data.sensor_position);
//		HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET);
//		HAL_UART_Transmit_DMA(&huart2, "123456", 6);  // 通过UART发送响应
//		 DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
		HAL_Delay(50); // 延时50ms
	}
//
//	switch (g_measurement.device_status.current_command)
//	{
//	case COMMAND_FINDOIL_START:/*液位跟随*/
//	{
//		printf("***液位跟随开始***\r\n");
////				MeasureOilFollow();
//		break;
//	}
//	case COMMAND_FINDBOTTOM_START:/*罐底测量*/
//	{
//		printf("***罐底测量开始***\r\n");
//		CMD_MeasureBottom();
//		printf("***罐底测量结束***\r\n");
//	}
//	case COMMAND_BACKZERO_START:/*零点测量*/
//	{
//		printf("***回零点开始***\r\n");
//		MeasureZero();
//		printf("***回零点结束***\r\n");
//		break;
//	}
//
//	default:
//		printf("未知命令\r\n");
//		break;
//
//	}
}
//测试主函数
void Test_main(void) {
	Test_FRAM_ReadWrite(); //测试FRAM读写
//	motor_text();
	Test_Params_Storage(); //测试参数存储
	CRC32_HAL_Test(); //CRC校验测试
}
