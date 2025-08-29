/*
 * measure.c
 *
 *  Created on: Mar 20, 2025
 *      Author: duan
 */

#include "measure.h"
#include "system_parameter.h"
#include "measure_zero.h"
#include "measure_tank_height.h"
#include "motor_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include "test.h"
#include "sensor.h"
void ProcessMeasureCmd(CommandType command)
{
	switch (command) {
	case CMD_BACK_ZERO:
		printf("执行回零点指令\n");
		MeasureZero();
		break;
//	case CMD_FIND_OIL:
//		printf("执行寻找液位指令\n");
//		SearchOilLevel();
//		break;
//	case CMD_FIND_WATER:
//		printf("执行寻找水位指令\n");
//		SearchWaterLevel();
//		break;
	case CMD_FIND_BOTTOM:
		printf("执行寻找罐底指令\n");
		MeasureBottom();
		break;
//	case CMD_MEASURE_SINGLE:
//		printf("执行单点测量指令\n");
//		SinglePointMeasurement();
//		break;
//	case CMD_MONITOR_SINGLE:
//		printf("执行单点监测指令\n");
//		SinglePointMonitoring();
//		break;
//	case CMD_SYNTHETIC:
//		printf("执行综合测量指令\n");
//		SyntheticMeasurement();
//		break;
//	case CMD_MEASURE_DISTRIBUTED:
//		printf("执行分布测量指令\n");
//		DensityDistributionMeasurement();
//		break;
//	case CMD_MEASURE_DENSITY_METER:
//		printf("执行密度每米测量指令\n");
//		DensityMeterMeasurement();
//		break;
//	case CMD_MEASURE_DENSITY_RANGE:
//		printf("执行液位区间测量指令\n");
//		DensityRangeMeasurement();
//		break;
//	case CMD_CALIBRATE_ZERO:
//		printf("执行标定零点指令\n");
//		CalibrateZeroPoint();
//		break;
//	case CMD_CALIBRATE_OIL:
//		printf("执行标定液位指令\n");
//		CalibrateOilLevel();
//		break;
//	case CMD_READ_PARAM:
//		printf("执行读取参数指令\n");
//		ReadDeviceParameters();
//		break;
//	case CMD_MOVE_UP:
//		printf("电机上行操作\n");
//		motorMoveNoWait(100, MOTOR_DIRECTION_UP);
//		break;
//	case CMD_MOVE_DOWN:
//		printf("电机下行操作\n");
//		motorMoveNoWait(100, MOTOR_DIRECTION_DOWN);
//		break;
	default:
		printf("未知命令类型: %d\n", command);
	}
}
// 处理接收到的命令
void process_command(uint8_t *command) {
	printf("Command received\n");
//	stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
//	stpr_enableDriver(&stepper);
	if (command[0] == 'A') {
		// 处理命令
		if (command[1] == '0') {
			// 刹车操作
			motorQuickStop();
		} else if (command[1] == '+') {
			int mm = atoi((char*) &command[2]);  // 获取数字
			printf("start up%d\n", mm);
			motorMoveNoWait((float) mm, MOTOR_DIRECTION_UP);  // 电机上行
		} else if (command[1] == '-') {
			int mm = atoi((char*) &command[2]);  // 获取数字
			printf("start down%d\n", mm);
			motorMoveNoWait((float) mm, MOTOR_DIRECTION_DOWN);  // 电机下行
		}
	}
	if (command[0] == 'B') {//
		printf("motor text start\n");
		printf("***编码值清零***\r\n");
		g_encoder_count = 0; // 重置编码器计数
		while (1) {
			stpr_enableDriver(&stepper);  //使能电机
//						stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
					//		stpr_moveTo(&stepper, -30 * 1600 * 32, 1600 * 2 * 32);
			motorMoveNoWait(200000, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			printf("start down\n");
			while (abs(g_encoder_count) < 20000) {
				printf("{encoder}%d\t{weight}%d\r\n", (int) g_encoder_count, g_weight);
				DSMSendcommand3times(DSM_POWER, strlen(DSM_POWER));
				HAL_Delay(50);
			}
			printf("down over!\n");
			HAL_Delay(1000);
			motorMoveNoWait(200000, MOTOR_DIRECTION_UP);
			printf("start up to zero\n");
			HAL_Delay(1000);
			while (abs(g_encoder_count) >1000) {
				printf("{encoder}%d\t{weight}%d\r\n", (int) g_encoder_count, g_weight);
				DSMSendcommand3times(DSM_POWER, strlen(DSM_POWER));
				HAL_Delay(50);
			}
			motorQuickStop();
			printf("上行结束\n");
			stpr_disableDriver(&stepper); //使能电机
		}
	}
	if (command[0] == 'C') {
		printf("***电机4步进分辨率测试***\r\n");
		motor_step_text();
	}
	if (command[0] == 'D') {
		printf("***电机4步进下行触底测试***\r\n");
		motor_step_down_text();
	}
	if (command[0] == 'E') {
		printf("***电机4步进上行碰零点测试***\r\n");
		motor_step_up_text();
	}
	if (command[0] == 'F') {
		printf("***罐底测量重复性测试***\r\n");
		while (1) {
			SearchBottom();
		}
	}
	if (command[0] == 'G') {
		printf("***罐底测量单次测试***\r\n");
		SearchBottom();
	}
	if (command[0] == 'H') {
		printf("***零点/罐底测量重复性测试***\r\n");
		while (1) {
			MeasureZero();
			SearchBottom();
		}
	}
	if (command[0] == 'I') {
		printf("***零点测试单次测试***\r\n");
		MeasureZero();

	}
	if (command[0] == 'J') {
		printf("***液位测量重复性测试***\r\n");

	}
	if (command[0] == 'K') {
		printf("***液位测量单次测试***\r\n");

	}
	if (command[0] == 'L') {
		printf("***编码值清零***\r\n");
		g_encoder_count = 0; // 重置编码器计数
	}
	if (command[0] == 'M') {
		printf("***电机高温测试***\r\n");
		while (1) {
			motorMoveNoWait(100000, MOTOR_DIRECTION_DOWN);
			HAL_Delay(1000);
			stpr_waitMove(&stepper);
		}

	}
	if (command[0] == 'N') {
		printf("motor text start\n");
		while (1) {
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
	if (command[0] == 'O') //获取空载称重
			{
		printf("Get empty weight\n");
		get_empty_weight();

	}
}

static int MeasureStart(void)
{
	fault_info_init();//故障初始化清零
	g_measurement.device_status.error_code = NO_ERROR;//故障代码清零
	return NO_ERROR;
}

//罐底零点主函数
void MeasureZero(void)
{
	int ret = 0;
	g_measurement.device_status.device_state = STATE_BACKZEROING;
    ret = MeasureStart();
	if ((ret != NO_ERROR)&& (ret != STATE_SWITCH))
	{
		g_measurement.device_status.device_state = STATE_ERROR;
		return;
	}
	//开始回零点
	ret = SearchZero();
	if ((ret != NO_ERROR)&& (ret != STATE_SWITCH))
	{
		g_measurement.device_status.device_state = STATE_ERROR;
		return;
	}

	g_measurement.device_status.device_state = STATE_STANDBY;
	return;
}


//罐底测量主函数
void MeasureBottom(void)
{
	int ret = 0;
	g_measurement.device_status.device_state = STATE_FINDBOTTOM;
    ret = MeasureStart();
	if ((ret != NO_ERROR)&& (ret != STATE_SWITCH))
	{
		g_measurement.device_status.device_state = STATE_ERROR;
		return;
	}
	//开始测量罐高
	ret = SearchBottom();
	if ((ret != NO_ERROR)&& (ret != STATE_SWITCH))
	{
		g_measurement.device_status.device_state = STATE_ERROR;
		return;
	}

	g_measurement.device_status.device_state = STATE_FINDBOTTOM_OVER;
	return;
}
