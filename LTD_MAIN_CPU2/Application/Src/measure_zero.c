/*
 * measure_zero.c
 *
 *  用于测量和校准设备的零点位置
 *  包含粗略和精确两步寻找零点的过程
 *  依赖外部的扭力传感器和电机控制接口
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
#include "error_log.h"
#include "sensor.h"

#define ZERO_SEARCH_RETRY_MAX  3  /* 可通过宏配置最大重试次数 */
#define ZERO_ROUGH_SLOW_DISTANCE_01MM     2000  /* 粗找零点提前 200mm 降到低速 */
#define ZERO_ROUGH_SLOW_SPEED_X100         50    /* 粗找接近零点后降到 0.50m/min */
#define ZERO_PRECISE_MEDIUM_DISTANCE_01MM 6000  /* 精找零点提前 600mm 降到中速 */
#define ZERO_PRECISE_SLOW_DISTANCE_01MM   1000  /* 精找零点提前 100mm 降到低速 */

/* 全局变量，记录零点的编码器数值 */
int32_t zero_position; /* 最近一次找零确认的电机零点位置计数，供后续位置换算使用。 */

/* 内部函数声明：粗略和精确寻找零点 */
static int SearchZeroRough();
static int SearchZeroPrecise();
static uint32_t Zero_MoveDownWithoutWeightGuard(const char *phase_name, float distance_mm);

/**
 * @brief 在明确跳过扭力保护的场景下向下移动传感器。
 *
 * @param phase_name 用于日志标识当前阶段的只读文字。
 * @param distance_mm 距离。
 * @return 返回 MotorCtrl_MoveAndWait 的实际结果；NO_ERROR 表示无扭力保护下行完成，其他值为命令切换、参数、驱动、位置或到位错误。
 */
static uint32_t Zero_MoveDownWithoutWeightGuard(const char *phase_name, float distance_mm)
{
    uint32_t ret;

    printf("零点测量    %s开始    下行距离=%.1fmm    当前扭力=%ld", phase_name, (double)distance_mm, (long)weight_parament.current_weight);
    MotorCtrl_PrintPositionRefs();
    printf("\r\n");
    ret = MotorCtrl_MoveBlockingNoDetectQuiet(distance_mm,
                                          MOTOR_DIRECTION_DOWN,
                                          MotorCtrl_GetDefaultSpeedX100());
    if (ret == NO_ERROR) {
        Weight_RebaseStableWeight();
        printf("零点测量    %s完成    下行距离=%.1fmm    当前扭力=%ld", phase_name, (double)distance_mm, (long)weight_parament.current_weight);
        MotorCtrl_PrintPositionRefs();
        printf("\r\n");
    }
    return ret;
}


/**
 * @brief 电机记步时，零点位置由 XACTUAL/电机基准维护，不再用编码轮零点偏差报警。
 *
 * 标定零点流程本身也跳过该检查，避免标定过程被旧零点拦截。
 *
 * @return 1 表示当前使用编码轮位置源且不在找零状态，需要执行零点偏差检查；否则返回 0。
 */
static uint8_t Zero_ShouldCheckDeviation(void)
{
	return (!MotorCtrl_IsPositionSourceMotor()) &&
	       (g_measurement.device_status.device_state != STATE_FINDZEROING);
}

/**
 * @brief 回零内部重试只处理搜索类失败；电机驱动掉电/复位交给外层自动恢复重新初始化。
 *
 * @param error_code 待记录、转换或判断的错误码。该值用于判断零点流程失败是否属于允许执行电机驱动恢复的错误集合。
 * @return 1 表示错误码属于 TMC 通信、配置丢失、欠压、禁用、未初始化，或运行、停止、到位等待超时，应交给外层电机驱动恢复；0 表示不属于该恢复集合，仍由回零内部或其它上层逻辑处理。
 */
static uint8_t Zero_IsMotorDriverRecoveryError(uint32_t error_code)
{
    switch (error_code) {
    case MOTOR_TMC_COMM_ERROR:
    case MOTOR_TMC_CONFIG_LOST:
    case MOTOR_CHARGE_PUMP_UNDER_VOLTAGE:
    case MOTOR_DISABLED:
    case MOTOR_DRIVER_NOT_INITIALIZED:
    case MOTOR_RUN_TIMEOUT:
    case MOTOR_STOP_WAIT_TIMEOUT:
    case MOTOR_ARRIVAL_WAIT_TIMEOUT:
        return 1U;
    default:
        return 0U;
    }
}



#ifndef ZERO_SEARCH_RETRY_MAX
#define ZERO_SEARCH_RETRY_MAX  3  /* 可通过宏配置最大重试次数 */
#endif

/**
 * @brief 在有限重试次数内依次执行零点粗找和精找，成功后重建编码器及卷筒位置基准。
 *
 * 初始扭力过大时先下行脱离零点；粗找和精找分别最多尝试 ZERO_SEARCH_RETRY_MAX 次，并记录失败重试、恢复成功及驱动恢复错误。
 * 最终校验零点偏差后，保存编码器零点、重置电机卷筒参考，按配置切回编码轮位置源，再下行脱离并在传感器支持时保存水位零电容和陀螺仪零基准。
 *
 * @return NO_ERROR 表示粗找、精找、零点保存及后续脱离动作全部完成；STATE_SWITCH
 *         表示被新命令正常打断；其他值为运动、驱动、丢步、零点范围、位置保存或可选传感器基准读取错误码。
 */
int SearchZero(void) {
	uint32_t ret;
	uint8_t try_times = 0;
    uint8_t rough_ok = 0;

	fault_info_init(); /* 清除故障信息 */
	printf("零点测量    开始\r\n");

	if (weight_parament.stable_weight >weight_parament.full_weight+2000) { /* 如果当前超重 */
		ret = Zero_MoveDownWithoutWeightGuard("初始脱离零点", 100.0f);
		CHECK_ERROR(ret);
	}

	printf("零点测量    初始扭力：%d\r\n", weight_parament.stable_weight);

    /* ************** 粗找阶段 - 带重试机制 ************** */
    try_times = 0;
    rough_ok = 0;
    while (try_times < ZERO_SEARCH_RETRY_MAX) {
        try_times++;
        fault_info_init();

        ret = SearchZeroRough();
        CHECK_COMMAND_SWITCH(ret);

        if (ret == NO_ERROR) {
            if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
                printf("零点测量    粗找后零点偏差超过阈值 | cable=%ld | limit=%lu\r\n",
                       (long)g_measurement.debug_data.cable_length,
                       (unsigned long)g_deviceParams.max_zero_deviation_distance);
                ret = MEASUREMENT_ZERO_OUT_OF_RANGE;
            } else {
                /* SearchZeroRough() 已经在检测到 ZERO 后停机；粗找只负责进入零点区域，
                 * 后续精找会再次确认，不再用 3 秒后的扭力波动否定本次粗找。 */
                rough_ok = 1;
                break;
            }
        }

        /* 保持原有粗找重试语义：粗找失败或偏差超限都先记录重试，再执行一次退让动作。 */
        ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                       ERROR_LOG_OP_SEARCH_ZERO_ROUGH,
                       ERROR_LOG_REASON_SEARCH_FAIL,
                       (uint32_t)try_times,
                       (uint32_t)ZERO_SEARCH_RETRY_MAX,
                       ret);

        /* 粗找遇到电机驱动类故障时不继续位置退让和测量重试，立即退出本轮命令，交由自动恢复重新初始化驱动。 */
        if (Zero_IsMotorDriverRecoveryError(ret)) {
            printf("零点测量    粗找检测到电机驱动故障，退出本轮命令等待自动恢复 | 错误码=0x%08lX\r\n",
                   (unsigned long)ret);
            return (int)ret;
        }
        if (try_times >= ZERO_SEARCH_RETRY_MAX) {
            CHECK_ERROR(ret);
            break;
        }

        ret = Zero_MoveDownWithoutWeightGuard("粗找退让", 100.0f);
        CHECK_ERROR(ret);
    }
    if (!rough_ok) {
        RETURN_ERROR(ret);
    }
	if (try_times > 1U) {
		ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
		                 ERROR_LOG_OP_SEARCH_ZERO_ROUGH,
		                 ERROR_LOG_REASON_RECOVER_OK,
		                 (uint32_t)try_times,
		                 (uint32_t)ZERO_SEARCH_RETRY_MAX);
	}

	/* ************** 精找阶段 - 第一次精确找零点 ************** */
	ret = Zero_MoveDownWithoutWeightGuard("精找前退让", 200.0f);
	CHECK_ERROR(ret);

	try_times = 0;
	while (try_times < ZERO_SEARCH_RETRY_MAX) {
		try_times++;
		ret = SearchZeroPrecise();

		if (ret == NO_ERROR) {
			if (try_times > 1U) {
				ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
				                 ERROR_LOG_OP_SEARCH_ZERO_PRECISE,
				                 ERROR_LOG_REASON_RECOVER_OK,
				                 (uint32_t)try_times,
				                 (uint32_t)ZERO_SEARCH_RETRY_MAX);
			}
			break;
		} else if (ret == STATE_SWITCH) {
			break;
		} else {
			ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
			               ERROR_LOG_OP_SEARCH_ZERO_PRECISE,
			               ERROR_LOG_REASON_SEARCH_FAIL,
			               (uint32_t)try_times,
			               (uint32_t)ZERO_SEARCH_RETRY_MAX,
			               ret);
			/* 精找遇到电机驱动类故障同样立即退出；只有普通测量失败才允许执行退让后再次尝试。 */
			if (Zero_IsMotorDriverRecoveryError(ret)) {
				printf("零点测量    精找检测到电机驱动故障，退出本轮命令等待自动恢复 | 错误码=0x%08lX\r\n",
				       (unsigned long)ret);
				return (int)ret;
			}
			if (try_times < ZERO_SEARCH_RETRY_MAX) {
				ret = Zero_MoveDownWithoutWeightGuard("精找退让", 100.0f);
				CHECK_ERROR(ret);
				HAL_Delay(1000);
			}
		}
	}
	CHECK_ERROR(ret);

	/* ************** 最终校验与记录 ************** */
	printf("零点测量完成     当前编码值     %ld\r\n", g_encoder_count);
	printf("{zero_value}%ld mm", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");

	if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
		RETURN_ERROR(MEASUREMENT_ZERO_OUT_OF_RANGE);
	} else {
		ret = set_encoder_zero();
		CHECK_ERROR(ret);
		ret = MotorCtrl_ResetDrumReferenceForZeroCalibration();
		CHECK_ERROR(ret);
		if (g_deviceParams.position_source_auto_switch == POSITION_SOURCE_AUTO_SWITCH_ENABLE) {
			ret = MotorCtrl_SwitchPositionSourceToEncoder();
			CHECK_ERROR(ret);
			printf("零点测量    编码器零点设置成功，已清除电机记步并切回编码轮记步\r\n");
		} else {
			printf("零点测量    编码器零点设置成功，按参数保持当前记步模式\r\n");
		}
		ret = Zero_MoveDownWithoutWeightGuard("标零后脱离", 10.0f); /* 脱离零点 */
		CHECK_ERROR(ret);
		ret = Zero_MoveDownWithoutWeightGuard("标零后下行", (float)g_deviceParams.findZeroDownDistance/10.0f);
		CHECK_ERROR(ret);
		if (Sensor_SupportsWaterCapChannel() != 0) {
			/* 零点电容仅在传感器声明水位电容能力时读取，不支持时不能阻断回零点。 */
			ret = read_zero_capacitance(); /* 读取零点电容值 */
			CHECK_ERROR(ret);
		} else {
			printf("零点测量    当前传感器类型不支持零点电容读取，已跳过\r\n");
		}
		if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_GYRO) {
			if (Sensor_SupportsGyroChannel() != 0) {
				/* 陀螺仪基准仅在传感器声明姿态能力时读取，不支持时不阻断回零点。 */
				ret = Bottom_SaveGyroZeroRef(); /* 保存陀螺仪零点参考 */
				CHECK_ERROR(ret);
			} else {
				printf("零点测量    当前传感器类型不支持陀螺仪基准读取，已跳过\r\n");
			}
		}
		printf("零点测量    电机下行完成，流程结束\r\n");
	}

	g_measurement.device_status.zero_point_status = 0;
	return NO_ERROR;
}


/**
 * @brief 粗略寻找零点
 *        电机上行，直到扭力状态为ZERO
 *        记录此时编码器值为零点
 * @return 总是返回0
 */
static int SearchZeroRough() {
    uint32_t ret;
    uint32_t speed_x100;
    int32_t distance_to_zero_01mm;
    uint32_t last_speed_x100 = 0U;

    /* 循环直到扭力状态为ZERO */
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    while (check_zero_point_status() != ZERO) {
        ret = MotorCtrl_PollRuntimePosition(); /* 循环中同步运行期位置，同时识别 TMC 掉电或配置丢失。 */
        CHECK_ERROR(ret);
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        distance_to_zero_01mm = abs(g_measurement.debug_data.cable_length);
        /* 靠近零点时降低粗找速度，减少撞零点后的惯性冲击。 */
        if (distance_to_zero_01mm < ZERO_ROUGH_SLOW_DISTANCE_01MM) {
            speed_x100 = ZERO_ROUGH_SLOW_SPEED_X100;
        }
        if (speed_x100 != last_speed_x100) {
            printf("零点测量    粗找速度切换 | 距零点：%.1fmm | speed=%.2f m/min\r\n",
                   (double)distance_to_zero_01mm / 10.0,
                   (double)speed_x100 / 100.0);
            last_speed_x100 = speed_x100;
        }

        ret = MotorCtrl_MoveUp(speed_x100);
        CHECK_ERROR(ret);
        /* 丢步检测 */
        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
        CHECK_ERROR(ret);
        /* 实时打印编码器和扭力信息 */
        printf("零点测量    长距离寻找零点    {传感器位置}%.1f", (float) (g_measurement.debug_data.sensor_position) / 10.0); MotorCtrl_PrintPositionRefs(); printf("    距零点：%.1fmm    速度(0.01m/min)=%lu    ", (double)distance_to_zero_01mm / 10.0, (unsigned long)speed_x100);
    }
    zero_position = g_measurement.debug_data.cable_length;
    ret = MotorCtrl_QuickStop(); /* 到达零点后快速停止电机 */
    CHECK_ERROR(ret); /* 检查快速停止是否成功 */
    return NO_ERROR; /* 返回无错误状态 */
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
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
	while (check_zero_point_status() != ZERO) {
        ret = MotorCtrl_PollRuntimePosition(); /* 循环中同步运行期位置，同时识别 TMC 掉电或配置丢失。 */
        CHECK_ERROR(ret);
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        if ((g_measurement.debug_data.cable_length - zero_position) < ZERO_PRECISE_SLOW_DISTANCE_01MM) {
            speed_x100 = 40;
        }
        else if ((g_measurement.debug_data.cable_length - zero_position) < ZERO_PRECISE_MEDIUM_DISTANCE_01MM) {
            speed_x100 = 100;
        }

		ret = MotorCtrl_MoveUp(speed_x100);  /* 启动电机向下运动 */
		CHECK_ERROR(ret); /* 检查上行是否成功 */

		if ((g_measurement.debug_data.cable_length < (zero_position - (int32_t)g_deviceParams.max_zero_deviation_distance)) &&
            Zero_ShouldCheckDeviation()) {
			printf("精找零点超出范围    尺带长度    %ld", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");
			ret = MotorCtrl_QuickStop(); /* 到达零点后快速停止电机 */
			CHECK_ERROR(ret); /* 检查快速停止是否成功 */
			RETURN_ERROR(MEASUREMENT_WEIGHT_UP_FAIL); /* 检查快速停止是否成功 */
		}
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); /* 检查丢步检测是否成功 */

		printf("零点测量    精确寻找零点    {传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("    速度(0.01m/min)    %lu    ", (unsigned long)g_measurement.debug_data.motor_speed);
	}
	printf("精找零点完成    尺带长度    %ld", g_measurement.debug_data.cable_length); MotorCtrl_PrintPositionRefs(); printf("\r\n");
	zero_position = g_measurement.debug_data.cable_length;
	ret = MotorCtrl_QuickStop(); /* 到达零点后快速停止电机 */
	CHECK_ERROR(ret); /* 检查快速停止是否成功 */

	if ((abs(g_measurement.debug_data.cable_length) > g_deviceParams.max_zero_deviation_distance) && Zero_ShouldCheckDeviation()) {
		printf("零点测量    零点偏差超过阈值\r\n"); /* TODO：需要把阈值打印出来 */
		ret = MEASUREMENT_ZERO_OUT_OF_RANGE;
		CHECK_ERROR(ret);
	}

	return NO_ERROR; /* 返回无错误状态 */
}
