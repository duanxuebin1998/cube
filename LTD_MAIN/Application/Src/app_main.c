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
#include "communication.h"
#include "encoder.h"
//测试用
#include "sensor.h"

// 初始化函数
void App_Init(void)
{
	printf("LTD demo restart!\n");
	Initialize_Encoder();
	motor_Init(); //电机初始化
//	Test_FRAM_ReadWrite();
//	motor_text();
//	motorMoveAndWaitUntilStop(1000.0, MOTOR_DIRECTION_DOWN);
}

// 主循环任务
void App_MainLoop(void)
{
//	motorMoveNoWait(1000.0, MOTOR_DIRECTION_UP);
//	HAL_Delay(1000);
//	stpr_waitMove(&stepper);
//	Start_Read_SSI_Data(); // 开始一次DMA读取
//	HAL_Delay(50);
//	Update_Encoder_Count(currentangle);

	while (1)
	{
		// 如果有新的命令
		if (new_command_ready)
		{
			new_command_ready = 0;  // 重置标志，避免重复处理

			// 处理接收到的命令
			process_command(received_buffer);
		}
//		Test_FRAM_ReadWrite();
//		DSMSendcommand3times(DSM_POWER, strlen(DSM_POWER));
		printf("{encoder}%d\r\n{weight}%d\r\n", (int) encoder_count, weight);
		HAL_Delay(50);
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
