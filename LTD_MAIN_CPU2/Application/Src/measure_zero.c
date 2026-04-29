/*
 * measure_zero.c
 *
 *  用于测量和校准设备的零点位置
 *  包含粗略和精确两步寻找零点的过程
 *  依赖外部的重量传感器和电机控制接口
 */

#include "weight.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure_zero.h"
#include "system_parameter.h"
#include "motor_ctrl.h"
#include "measure_tank_height.h"
#include "measure_water_level.h"
#include "encoder.h"

#define ZERO_SEARCH_RETRY_MAX  3  // 可通过宏配置最大重试次数
#define ZERO_ROUGH_SLOW_DISTANCE_01MM     6000  // 粗找零点提前 600mm 降到低速
#define ZERO_PRECISE_MEDIUM_DISTANCE_01MM 6000  // 精找零点提前 600mm 降到中速
#define ZERO_PRECISE_SLOW_DISTANCE_01MM   1000  // 精找零点提前 100mm 降到低速

// 全局变量，记录零点的编码器数值
int32_t zero_position;

// 内部函数声明：粗略和精确寻找零点
static int SearchZeroRough();
static int SearchZeroPrecise();

/* 电机记步时，零点位置由 XACTUAL/电机基准维护，不再用编码轮零点偏差报警。
 * 标定零点流程本身也跳过该检查，避免标定过程被旧零点拦截。 */
static uint8_t Zero_ShouldCheckDeviation(void)
{
	return (!MotorCtrl_IsPositionSourceMotor()) &&
	       (g_measurement.device_status.device_state != STATE_FINDZEROING);
}

/**
 * @brief 主零点搜索流程
 *        先进行多次粗略找零点，成功后再进行两次精确找零点（均带可配置重试机制）
 *        最终记录零点编码器值
 * @return 0表示成功，1表示三次粗找零点失败
 */

#ifndef ZERO_SEARCH_RETRY_MAX
#define ZERO_SEARCH_RETRY_MAX  3  // 可通过宏配置最大重试次数
#endif

int SearchZero(void) {
	uint32_t ret;
	uint8_t try_times = 0;

	fault_info_init(); // 清除故障信息
	printf("零点测量\t开始\r\n");

	if (weight_parament.stable_weight >weight_parament.full_weight+2000) { // 如果当前超重
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret);
		printf("零点测量\t脱离零点完成\r\n");
	}

	printf("零点测量\t初始重量：%d\r\n", weight_parament.stable_weight);

	/*************** 粗找阶段 - 带重试机制 ***************/
	try_times = 0;
	while (try_times < ZERO_SEARCH_RETRY_MAX) {
		try_times++;
		fault_info_init();
		printf("零点测量\t粗找零点第%d次尝试\r\n", try_times);

		ret = SearchZeroRough();
		CHECK_COMMAND_SWITCH(ret);

		if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
			printf("零点测量\tt零点偏差超过阈值\r\n");
			ret = MEASUREMENT_ZERO_OUT_OF_RANGE;
			if (try_times >= ZERO_SEARCH_RETRY_MAX) {
				printf("零点测量\t粗找零点失败(尝试%d次)\r\n", try_times);
				CHECK_ERROR(ret);
			}
			ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
			CHECK_ERROR(ret);
			continue;
		}

		HAL_Delay(3000);

		if (check_zero_point_status() == ZERO)
			break;

		if (ret != NO_ERROR) {
			printf("零点测量\t粗找失败[%d]:0x%X\r\n", try_times, (unsigned int) ret);
			ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
			CHECK_ERROR(ret);
			continue;
		}
	}
	CHECK_ERROR(ret);
	if (try_times > ZERO_SEARCH_RETRY_MAX) {
		printf("零点测量\t粗找零点失败(尝试%d次)\r\n", try_times);
		RETURN_ERROR(MEASUREMENT_WEIGHT_UP_FAIL);
	}
	printf("零点测量\t粗找零点完成\t位置%ld\r\n", zero_position);

	/*************** 精找阶段 - 第一次精确找零点 ***************/
	ret = MotorCtrl_MoveAndWait(200.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
	CHECK_ERROR(ret);

	try_times = 0;
	while (try_times < ZERO_SEARCH_RETRY_MAX) {
		try_times++;
		printf("零点测量\t第一次精找第%d次尝试\r\n", try_times);
		ret = SearchZeroPrecise();

		if (ret == NO_ERROR) {
			printf("零点测量\t第一次精找完成\t位置%ld\r\n", zero_position);
			break;
		} else {
			printf("零点测量\t第一次精找失败[%d]:0x%X\r\n", try_times, (unsigned int) ret);
			if (try_times < ZERO_SEARCH_RETRY_MAX) {
				ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
				CHECK_ERROR(ret);
				HAL_Delay(1000);
			}
		}
	}
	CHECK_ERROR(ret);

//	/*************** 精找阶段 - 第二次精确找零点 ***************/
//	ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
//	CHECK_ERROR(ret);
//	printf("零点测量\t电机下行完成，准备第二次精找\r\n");
//
//	try_times = 0;
//	while (try_times < ZERO_SEARCH_RETRY_MAX) {
//		try_times++;
//		printf("零点测量\t第二次精找第%d次尝试\r\n", try_times);
//		ret = SearchZeroPrecise();
//
//		if (ret == NO_ERROR) {
//			printf("零点测量\t第二次精找完成\t位置%ld\r\n", zero_position);
//			break;
//		} else {
//			printf("零点测量\t第二次精找失败[%d]:0x%X\r\n", try_times, (unsigned int) ret);
//			if (try_times < ZERO_SEARCH_RETRY_MAX) {
//				ret = MotorCtrl_MoveAndWait(50.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
//				CHECK_ERROR(ret);
//				HAL_Delay(1000);
//			}
//		}
//	}
//	CHECK_ERROR(ret);

	/*************** 最终校验与记录 ***************/
	printf("零点测量完成 \t当前编码值\t %ld\r\n", g_encoder_count);
	printf("{zero_value}%ld mm", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");

	if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
		printf("零点测量\t编码值异常，可能需要重新校准\r\n");
		RETURN_ERROR(MEASUREMENT_ZERO_OUT_OF_RANGE);
	} else {
		set_encoder_zero();
		/* 普通回零继续保持电机记步基准；只有标定零点才清除电机侧基准并切回编码轮。 */
		if (g_measurement.device_status.device_state == STATE_FINDZEROING) {
			MotorCtrl_ResetDrumReferenceForZeroCalibration();
		}
		printf("零点测量\t编码器零点设置成功\r\n");
		ret = MotorCtrl_MoveNoWait(10, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());//脱离零点
		CHECK_ERROR(ret);
		HAL_Delay(3000);
		printf("零点测量\t向下移动\t下行距离\t%ld\r\n", g_deviceParams.findZeroDownDistance/10);
		ret = MotorCtrl_MoveAndWait((float)g_deviceParams.findZeroDownDistance/10.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret);
		read_zero_capacitance();//读取零点电容值
		if(g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_GYRO)
		{
			Bottom_SaveGyroZeroRef();//保存陀螺仪零点参考
		}
		/* 只有“标定零点”流程才强制恢复编码轮记步。
		 * 普通回零点只重建零点基准，不改变当前记步模式；如果回零前已经是电机记步，回零后继续保持电机记步。 */
		if (g_measurement.device_status.device_state == STATE_FINDZEROING) {
			ret = MotorCtrl_SwitchPositionSourceToEncoder();
			CHECK_ERROR(ret);
		}
		printf("零点测量\t电机下行完成，流程结束\r\n");
	}

	g_measurement.device_status.zero_point_status = 0;
	return NO_ERROR;
}


/**
 * @brief 粗略寻找零点
 *        电机上行，直到重量状态为ZERO
 *        记录此时编码器值为零点
 * @return 总是返回0
 */
static int SearchZeroRough() {
	uint32_t ret;
    uint32_t speed_x100;
    int32_t distance_to_zero_01mm;

	// 循环直到重量状态为ZERO
    MotorCtrl_LostStepInit();// 重置丢步检测计数器
	while (check_zero_point_status() != ZERO) {
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        distance_to_zero_01mm = abs(g_measurement.debug_data.cable_length);
        if (distance_to_zero_01mm < ZERO_ROUGH_SLOW_DISTANCE_01MM) {
            speed_x100 = 100;
        }

		ret = MotorCtrl_MoveUp(speed_x100);  // 启动电机向下运动
		CHECK_ERROR(ret); // 检查上行是否成功
		//丢步检测
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
		// 实时打印编码器和重量信息
		printf("零点测量\t长距离寻找零点\t{传感器位置}%.1f", (float) (g_measurement.debug_data.sensor_position) / 10.0); MotorCtrl_PrintPositionRefs(); printf("\t距零点=%.1fmm\t速度(0.01m/min)=%lu\t", (double)distance_to_zero_01mm / 10.0, (unsigned long)speed_x100);
		ret = MotorCtrl_MoveUp(speed_x100); // 电机上行
		CHECK_ERROR(ret); // 检查上行是否成功
	}
	zero_position = g_measurement.debug_data.cable_length;
	ret = MotorCtrl_QuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功
	return NO_ERROR; // 返回无错误状态
}

/**
 * @brief 精确寻找零点
 *        电机上行，靠近零点时逐步降低速度，提高精度
 *        记录此时编码器值为零点
 * @return 总是返回0
 */
static int SearchZeroPrecise() {
	uint32_t ret;
    uint32_t speed_x100;
    MotorCtrl_LostStepInit();// 重置丢步检测计数器
	while (check_zero_point_status() != ZERO) {
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        if ((g_measurement.debug_data.cable_length - zero_position) < ZERO_PRECISE_SLOW_DISTANCE_01MM) {
            speed_x100 = 40;
        }
        else if ((g_measurement.debug_data.cable_length - zero_position) < ZERO_PRECISE_MEDIUM_DISTANCE_01MM) {
            speed_x100 = 100;
        }

		ret = MotorCtrl_MoveUp(speed_x100);  // 启动电机向下运动
		CHECK_ERROR(ret); // 检查上行是否成功

		if ((g_measurement.debug_data.cable_length < (zero_position - (int32_t)g_deviceParams.max_zero_deviation_distance)) &&
            Zero_ShouldCheckDeviation()) {
			printf("精找零点超出范围\t尺带长度\t%ld", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");
			ret = MotorCtrl_QuickStop(); // 到达零点后快速停止电机
			CHECK_ERROR(ret); // 检查快速停止是否成功
			RETURN_ERROR(MEASUREMENT_WEIGHT_UP_FAIL); // 检查快速停止是否成功
		}
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功

		printf("零点测量\t精确寻找零点\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t速度(0.01m/min)\t%lu\t", (unsigned long)g_measurement.debug_data.motor_speed);
	}
	printf("精找零点完成\t尺带长度\t%ld", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");
	zero_position = g_measurement.debug_data.cable_length;
	ret = MotorCtrl_QuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功

	if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
		printf("零点测量\t零点偏差超过阈值\r\n");//TODO：需要把阈值打印出来
		ret = MEASUREMENT_ZERO_OUT_OF_RANGE;
		CHECK_ERROR(ret);
	}

	return NO_ERROR; // 返回无错误状态
}
