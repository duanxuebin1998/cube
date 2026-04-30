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
#define ZERO_ROUGH_SLOW_DISTANCE_01MM     2000  // 粗找零点提前 200mm 降到低速
#define ZERO_ROUGH_SLOW_SPEED_X100         50    // 粗找接近零点后降到 0.50m/min
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
    uint8_t rough_ok = 0;

	fault_info_init(); // 清除故障信息
	printf("零点测量    开始\r\n");

	if (weight_parament.stable_weight >weight_parament.full_weight+2000) { // 如果当前超重
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret);
		printf("零点测量    脱离零点完成\r\n");
	}

	printf("零点测量    初始重量：%d\r\n", weight_parament.stable_weight);

    /*************** 粗找阶段 - 带重试机制 ***************/
    try_times = 0;
    rough_ok = 0;
    while (try_times < ZERO_SEARCH_RETRY_MAX) {
        try_times++;
        fault_info_init();
        printf("零点测量    粗找零点第%d次尝试\r\n", try_times);

        ret = SearchZeroRough();
        CHECK_COMMAND_SWITCH(ret);

        if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
            printf("零点测量    粗找后零点偏差超过阈值 | cable=%ld | limit=%lu\r\n",
                   (long)g_measurement.debug_data.cable_length,
                   (unsigned long)g_deviceParams.max_zero_deviation_distance);
            ret = MEASUREMENT_ZERO_OUT_OF_RANGE;
        } else if (ret == NO_ERROR) {
            /* SearchZeroRough() 已经在检测到 ZERO 后停机；粗找只负责进入零点区域，
             * 后续精找会再次确认，不再用 3 秒后的称重波动否定本次粗找。 */
            rough_ok = 1;
            break;
        } else {
            printf("零点测量    粗找失败[%d]:0x%X\r\n", try_times, (unsigned int) ret);
        }

        if (try_times >= ZERO_SEARCH_RETRY_MAX) {
            printf("零点测量    粗找零点失败(尝试%d次) | last_error=0x%X\r\n", try_times, (unsigned int)ret);
            CHECK_ERROR(ret);
            break;
        }

        ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);
    }
    if (!rough_ok) {
        RETURN_ERROR(ret);
    }
	printf("零点测量    粗找零点完成    位置%ld\r\n", zero_position);

	/*************** 精找阶段 - 第一次精确找零点 ***************/
	ret = MotorCtrl_MoveAndWait(200.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
	CHECK_ERROR(ret);

	try_times = 0;
	while (try_times < ZERO_SEARCH_RETRY_MAX) {
		try_times++;
		printf("零点测量    第一次精找第%d次尝试\r\n", try_times);
		ret = SearchZeroPrecise();

		if (ret == NO_ERROR) {
			printf("零点测量    第一次精找完成    位置%ld\r\n", zero_position);
			break;
		} else {
			printf("零点测量    第一次精找失败[%d]:0x%X\r\n", try_times, (unsigned int) ret);
			if (try_times < ZERO_SEARCH_RETRY_MAX) {
				ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
				CHECK_ERROR(ret);
				HAL_Delay(1000);
			}
		}
	}
	CHECK_ERROR(ret);

	/*************** 最终校验与记录 ***************/
	printf("零点测量完成     当前编码值     %ld\r\n", g_encoder_count);
	printf("{zero_value}%ld mm", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");

	if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
		printf("零点测量    编码值异常，可能需要重新校准\r\n");
		RETURN_ERROR(MEASUREMENT_ZERO_OUT_OF_RANGE);
	} else {
		set_encoder_zero();
		MotorCtrl_ResetDrumReferenceForZeroCalibration();
		ret = MotorCtrl_SwitchPositionSourceToEncoder();
		CHECK_ERROR(ret);
		printf("零点测量    编码器零点设置成功，已清除电机记步并切回编码轮记步\r\n");
		ret = MotorCtrl_MoveNoWait(10, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());//脱离零点
		CHECK_ERROR(ret);
		HAL_Delay(3000);
		printf("零点测量    向下移动    下行距离    %ld\r\n", g_deviceParams.findZeroDownDistance/10);
		ret = MotorCtrl_MoveAndWait((float)g_deviceParams.findZeroDownDistance/10.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret);
		read_zero_capacitance();//读取零点电容值
		if(g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_GYRO)
		{
			Bottom_SaveGyroZeroRef();//保存陀螺仪零点参考
		}
		printf("零点测量    电机下行完成，流程结束\r\n");
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
    uint32_t last_speed_x100 = 0U;

    // 循环直到重量状态为ZERO
    MotorCtrl_LostStepInit();// 重置丢步检测计数器
    while (check_zero_point_status() != ZERO) {
        MotorCtrl_PollRuntimePosition();
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        distance_to_zero_01mm = abs(g_measurement.debug_data.cable_length);
        /* 靠近零点时降低粗找速度，减少撞零点后的惯性冲击。 */
        if (distance_to_zero_01mm < ZERO_ROUGH_SLOW_DISTANCE_01MM) {
            speed_x100 = ZERO_ROUGH_SLOW_SPEED_X100;
        }
        if (speed_x100 != last_speed_x100) {
            printf("零点测量    粗找速度切换 | 距零点=%.1fmm | speed=%.2f m/min\r\n",
                   (double)distance_to_zero_01mm / 10.0,
                   (double)speed_x100 / 100.0);
            last_speed_x100 = speed_x100;
        }

        ret = MotorCtrl_MoveUp(speed_x100);
        CHECK_ERROR(ret);
        //丢步检测
        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
        CHECK_ERROR(ret);
        // 实时打印编码器和重量信息
        printf("零点测量    长距离寻找零点    {传感器位置}%.1f", (float) (g_measurement.debug_data.sensor_position) / 10.0); MotorCtrl_PrintPositionRefs(); printf("    距零点=%.1fmm    速度(0.01m/min)=%lu    ", (double)distance_to_zero_01mm / 10.0, (unsigned long)speed_x100);
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
			printf("精找零点超出范围    尺带长度    %ld", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");
			ret = MotorCtrl_QuickStop(); // 到达零点后快速停止电机
			CHECK_ERROR(ret); // 检查快速停止是否成功
			RETURN_ERROR(MEASUREMENT_WEIGHT_UP_FAIL); // 检查快速停止是否成功
		}
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功

		printf("零点测量    精确寻找零点    {传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("    速度(0.01m/min)    %lu    ", (unsigned long)g_measurement.debug_data.motor_speed);
	}
	printf("精找零点完成    尺带长度    %ld", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");
	zero_position = g_measurement.debug_data.cable_length;
	ret = MotorCtrl_QuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功

	if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
		printf("零点测量    零点偏差超过阈值\r\n");//TODO：需要把阈值打印出来
		ret = MEASUREMENT_ZERO_OUT_OF_RANGE;
		CHECK_ERROR(ret);
	}

	return NO_ERROR; // 返回无错误状态
}
