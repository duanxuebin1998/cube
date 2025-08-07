/*
 * app_main.c
 *
 *  Created on: Mar 13, 2025
 *      Author: 1
 */

#include "app_main.h"
#include "main.h"
#include "system_parameter.h"
#include "measure.h"
#include "encoder.h"
//测试用
#include "sensor.h"
#include <mb85rs2m.h>
#include "my_crc.h"
#include "test.h"
#include "motor_ctrl.h"
void Test_main(void); // 测试函数声明
// 初始化函数
void App_Init(void) {
	printf("LTD demo restart!\n");
	Initialize_Encoder(); // 初始化编码器
	motor_Init(); //电机初始化
	init_device_params(); // 初始化设备参数
	g_measurement.device_status.zero_point_status == 1;// 设置零点状态为需要回零点
	weight_init();
	HostCommuInit(); // 初始化Modbus通信
	//测试函数
//	Test_main(); // 测试函数
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
//		Test_FRAM_ReadWrite();
////		DSMSendcommand3times(DSM_POWER, strlen(DSM_POWER));
		printf("{encoder}%d\r\n{weight}%d\r\n", (int) g_encoder_count, g_weight);
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
//		MeasureBottom();
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
	test_macro(); // 测试宏函数
}
int test_macro(void) {
	CHECK_ERROR(0x10001); // 测试宏函数
	 fault_info_init(); // 初始化故障信息
	return NO_ERROR; // 返回无错误状态
}// 测试宏函数
