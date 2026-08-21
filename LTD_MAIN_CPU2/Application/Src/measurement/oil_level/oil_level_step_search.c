/*
 * 文件职责：方法 0 和方法 1 使用的旧步进搜索实现。
 * 本文件只保留旧步进搜索算法，公共位置、边界和盲区流程由独立模块提供。
 * 结果发布和 AO 处理统一由 oil_level_runtime.c 提供，本文件不重复实现。
 */
#include "oil_level_internal.h"
#include "abortable_delay.h"
#include "fault_manager.h"
#include "motor_ctrl.h"
#include "sensor_service.h"
#include "system_parameter.h"
#include "weight.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief 从空气侧向下搜索油相并记录油侧频率端点。
 * @return NO_ERROR 表示已确认进入油相；命令切换、运动、传感器或安全检查错误原样返回。
 */
int SearchOil(void)
{

	uint32_t ret;
    Level_StateTypeDef level_state;

	printf("液位测量\t初始扭力：%d\r\n", weight_parament.stable_weight);

	/* 向上运行保证传感器全部在空气 */
	if (g_measurement.debug_data.cable_length > 1000) { /* 如果尺带长度大于200mm，先将电机上行到安全位置 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查上行是否成功 */
	}
	/* 长距离下行寻找油面 */
	MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
	/* 持续监控扭力状态，直到检测到液位 */
    while (1) {
        ret = determine_level_status_motion(&level_state);
        CHECK_ERROR(ret);
        if (level_state == OIL) {
            break;
        }
		ret = MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());  /* 启动电机向下运动 */
		CHECK_ERROR(ret); /* 检查上行是否成功 */
		/* 实时输出编码器位置和扭力值（用于调试） */
		printf("液位测量\t长距离寻找液位\t{传感器位置}%.1f", (float) (g_measurement.debug_data.sensor_position) / 10.0); MotorCtrl_PrintPositionRefs(); printf("\t{扭力值}%d\r\n", weight_parament.current_weight);
		CHECK_ERROR(ret);
		/* 丢步检测 */
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); /* 检查丢步检测是否成功 */

		/* 扭力检测 */
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); /* 检查碰撞检测是否成功 */

		/* 盲区检测 */
		if (g_measurement.debug_data.sensor_position < g_deviceParams.blindZone) {
			printf("超声波找液位\t到达位置下限\r\n");
			return OilLevel_StopBeforeReturn(MEASUREMENT_OILLEVEL_LOW, "到达液位下限");
		}
	}
	ret = MotorCtrl_SlowStop(); /* 到达零点后慢速停止电机 */
	CHECK_ERROR(ret); /* 检查慢速停止是否成功 */
	if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
		/* 向下运行保证传感器全部在油 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查下行是否成功 */
	}
	/* 取油中频率 */
	ret = OilLevel_ReadAverageFrequencyWithRetry(&g_measurement.oil_measurement.oil_frequency, "油中端点");
	CHECK_ERROR(ret);  /* 检查获取油中频率是否成功 */
	printf("液位测量\t油中频率：%ld\r\n", g_measurement.oil_measurement.oil_frequency);

	printf("液位测量\t传感器已在液位中，向上寻找空气\r\n");
	ret = SearchAir();
	CHECK_ERROR(ret); /* 检查寻找空气是否成功 */

	return NO_ERROR;
}

/**
 * @brief 搜索空气中的零点位置并获取空气中频率值
 *
 * 该函数通过控制电机上行，持续监控扭力状态，直到检测到液位（从油中进入空气）。
 * 到达零点后，慢速停止电机，并确保传感器全部位于空气中，然后获取空气中频率值。
 * 如果传感器位置超出盲区范围，还会将传感器下移至油中进行后续操作。
 *
 * @return int 返回状态码：
 *             - NO_ERROR: 操作成功
 *             - 其他错误码: 具体错误状态
 *
 * @note 函数内部会调用以下辅助函数：
 *       - determine_level_status_motion(): 判断当前液位状态（空气/油中）
 *       - MotorCtrl_MoveUp(): 以指定速度控制电机上行
 *       - MotorCtrl_CheckLostStepAutoTiming(): 自动定时检测电机丢步
 *       - CheckWeightCollision(): 检测扭力碰撞
 *       - MotorCtrl_SlowStop(): 慢速停止电机
 *       - MotorCtrl_MoveAndWait(): 移动电机并等待停止
 *       - OilLevel_ReadAverageFrequency(): 获取平均频率值
 *
 * @note 输出信息包括：
 *       - 初始扭力值
 *       - 传感器位置和扭力值（实时调试信息）
 *       - 空气中频率值
 */
int SearchAir() {
	uint32_t ret;
    Level_StateTypeDef level_state;
	printf("液位测量\t初始扭力：%d\r\n", weight_parament.stable_weight);

	/* 持续监控扭力状态，直到检测到液位 */
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* 待完善：电机可能存在上行停止重新加速 */
    while (1) {
        ret = determine_level_status_motion(&level_state);
        CHECK_ERROR(ret);
        if (level_state == AIR) {
            break;
        }
		ret = MotorCtrl_MoveUp(MotorCtrl_GetDefaultSpeedX100());  /* 启动电机向下运动 */
		CHECK_ERROR(ret); /* 检查上行是否成功 */
		/* 实时输出编码器位置和扭力值（用于调试） */
		printf("液位测量\t长距离寻找空气\t{传感器位置}%.1f", (float) (g_measurement.debug_data.sensor_position) / 10.0); MotorCtrl_PrintPositionRefs(); printf("\t{扭力值}%d\r\n", weight_parament.current_weight);
		CHECK_ERROR(ret);
		/* 丢步检测 */
		ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); /* 检查丢步检测是否成功 */
/* 扭力检测 */
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); /* 检查碰撞检测是否成功 */
	}
	ret = MotorCtrl_SlowStop(); /* 到达零点后慢速停止电机 */
	CHECK_ERROR(ret); /* 检查慢速停止是否成功 */
	/* 向上运行保证传感器全部在空气 */
	if (g_measurement.debug_data.cable_length > 1000) { /* 如果尺带长度大于200mm，先将电机上行到安全位置 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查上行是否成功 */
	}
	/* 取空气中频率 */
	ret = OilLevel_ReadAverageFrequencyWithRetry(&g_measurement.oil_measurement.air_frequency, "空气端点");
	CHECK_ERROR(ret);  /* 检查获取空气中频率是否成功 */
	printf("液位测量\t空气中频率：%ld\r\n", g_measurement.oil_measurement.air_frequency);
	if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
		/* 向下运行保证传感器全部在油 */
		ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
		CHECK_ERROR(ret); /* 检查下行是否成功 */
	}

	return NO_ERROR;
}

/**
 * @brief 确定液位状态。
 *
 * 该函数通过获取当前液位模式的频率值，判断传感器当前所处的状态（空气中或油中）。
 * 根据频率值与阈值的比较，返回相应的液位状态枚举值。
 * OIL: 传感器在油中（频率低于下限阈值）。
 *
 * @param state_out 用于返回液位频率相对上下阈值的判定状态。
 * @param allow_mode_recovery 允许模式。
 * @return Level_StateTypeDef 返回液位状态。
 * @note 函数内部会调用 OilLevel_ReadValidatedFrequency 获取当前频率值。
 * @note 输出信息包括当前频率值和传感器状态（空气中或油中）。
 */
static uint32_t determine_level_status_internal(Level_StateTypeDef *state_out, uint8_t allow_mode_recovery) {
    uint32_t ret;
    uint32_t current_frequency = 0U;
    const char *mode_text;

    if (state_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    mode_text = allow_mode_recovery ? "静态" : "运动";
    if (allow_mode_recovery) {
        ret = OilLevel_ReadValidatedFrequency(&g_measurement.oil_measurement.current_frequency);
    } else {
        /* 运动中不执行模式恢复；统一入口会按V4主动快照或交互R04读取，并拒绝未知类型。 */
        ret = SensorService_ReadLevelFrequency(&current_frequency);
    }

    CHECK_COMMAND_SWITCH(ret);
    if (ret != NO_ERROR) {
        return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
    }

    if (!allow_mode_recovery) {
        g_measurement.oil_measurement.current_frequency = current_frequency;
    }

    printf("液位状态检测[%s]\t当前频率\t%ld\t", mode_text, g_measurement.oil_measurement.current_frequency);
    if (OilLevel_IsCurrentFrequencyInAir() != 0U) {
        printf("传感器在空气中\r\n");
        *state_out = AIR;
    } else {
        printf("传感器在油中\r\n");
        *state_out = OIL;
    }
    return NO_ERROR;
}


/**
 * @brief 根据运动过程中的传感器频率判断浮子位于空气还是油中。
 *
 * @param state_out 用于返回液位频率相对上下阈值的判定状态。
 * @return NO_ERROR 表示已读取运动过程频率并通过 state_out 写入 AIR 或 OIL；空输出指针、命令切换、传感器通信或读数错误返回对应错误码。
 */
uint32_t determine_level_status_motion(Level_StateTypeDef *state_out) {
    return determine_level_status_internal(state_out, 0U);
}
/**
 * @brief 按旧步进算法计算本轮移动方向和步长。
 *
 * @details 调用场景：SearchOilPrecise() 每次读取频率后调用。
 * @note 关键约束：保持原超大偏差、接近端点和标准偏差四段判定顺序。
 *
 * @param per_mm_frequency 频率。
 * @param frequency_error 频率故障。
 * @param over_time 用于返回超过跟随频率阈值的累计采样时间。
 * @param lower_time 用于返回低于跟随频率阈值的累计采样时间。
 * @param run_length 用于返回本轮精确跟随需要移动的距离，单位 mm。
 * @param dir 本轮精确找油移动方向输出指针；函数根据频率误差写入 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN。
 */
static void OilLevel_ComputePreciseStep(float per_mm_frequency,
                                        float frequency_error,
                                        int *over_time,
                                        int *lower_time,
                                        float *run_length,
                                        uint32_t *dir)
{
    if ((over_time == NULL) || (lower_time == NULL) || (run_length == NULL) || (dir == NULL)) {
        return;
    }

    if ((frequency_error > 500.0f) ||
        ((g_measurement.oil_measurement.air_frequency - g_measurement.oil_measurement.current_frequency) < 200U)) {
        (*over_time)++;
        *lower_time = 0;
        *run_length = 4.0f * (float)(*over_time) + frequency_error / per_mm_frequency - 4.0f;
        *dir = MOTOR_DIRECTION_DOWN;
    } else if (frequency_error > (float)FrequencyLevel_GetCompatThresholdHz(g_deviceParams.oilLevelThreshold)) {
        *over_time = 0;
        *lower_time = 0;
        *run_length = frequency_error / per_mm_frequency;
        *dir = MOTOR_DIRECTION_DOWN;
    } else if ((frequency_error < -500.0f) ||
               ((g_measurement.oil_measurement.current_frequency - g_measurement.oil_measurement.oil_frequency) < 200U)) {
        (*lower_time)++;
        *over_time = 0;
        *run_length = -(frequency_error / per_mm_frequency) + 4.0f * (float)(*lower_time) - 4.0f;
        *dir = MOTOR_DIRECTION_UP;
    } else if (frequency_error < -(float)FrequencyLevel_GetCompatThresholdHz(g_deviceParams.oilLevelThreshold)) {
        *over_time = 0;
        *lower_time = 0;
        *run_length = -(frequency_error / per_mm_frequency);
        *dir = MOTOR_DIRECTION_UP;
    } else {
        *run_length = 0.0f;
        *over_time = 0;
        *lower_time = 0;
    }
}

/**
 * @brief 判断旧步进精找是否已处于可稳定计数区间。
 *
 * @details 调用场景：SearchOilPrecise() 计算步长后决定是否累计 followTime。
 * @note 关键约束：保持原端点 200Hz 保护，避免停在空气或油中端点附近。
 *
 * @param frequency_error 频率故障。
 * @return 1 表示旧步进精找已处于可稳定计数区间；0 表示旧步进精找尚未处于可稳定计数区间。
 */
static uint8_t OilLevel_IsPreciseStable(float frequency_error)
{
    if ((fabsf(frequency_error) <= (float)FrequencyLevel_GetCompatThresholdHz(g_deviceParams.oilLevelThreshold)) &&
        ((g_measurement.oil_measurement.air_frequency - g_measurement.oil_measurement.current_frequency) > 200U) &&
        ((g_measurement.oil_measurement.current_frequency - g_measurement.oil_measurement.oil_frequency) > 200U)) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 执行旧步进精找本轮电机移动。
 *
 * @details 调用场景：SearchOilPrecise() 稳定判断后仍需移动时调用。
 * @note 关键约束：保持原小步长折半和最小 0.1mm 移动口径。
 *
 * @param run_length 本轮精确跟随计划移动的距离，单位 mm。
 * @param dir 电机或扫描方向编码；使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN，决定本轮步进移动的正负方向。
 * @return 无需移动时返回 NO_ERROR；需要移动时返回 MotorCtrl_MoveAndWait 的结果，包括命令切换、参数、驱动、位置或到位错误。
 */
static uint32_t OilLevel_RunPreciseMove(float run_length, uint32_t dir)
{
    if (run_length == 0.0f) {
        return NO_ERROR;
    }

    if (run_length < 5.0f) {
        run_length = run_length / 2.0f;
    }
    if (run_length < 0.3f) {
        run_length = 0.1f;
    }
    printf("频率跟随\t电机移动\t距离%f\t方向%s\r\n", run_length, (dir == MOTOR_DIRECTION_UP) ? "上" : "下");

    return MotorCtrl_MoveAndWait(run_length, dir, MotorCtrl_GetDefaultSpeedX100());
}

/**
 * @brief 执行旧步进精找每轮结束后的当前位置更新和盲区等待。
 *
 * @details 调用场景：SearchOilPrecise() 完成本轮移动和超限检查后调用。
 * @note 关键约束：命令切换直接透传；其他错误由上层按旧逻辑统一停机返回。
 *
 * @return 返回本轮位置更新、边界检查和盲区等待结果；NO_ERROR 表示可继续，其他值原样保留切换、边界或传感器错误。
 */
static int OilLevel_HandlePrecisePositionUpdate(void)
{
    int ret = OilLevel_UpdatePositionAndCheckBounds();
    if (ret == MEASUREMENT_OILLEVEL_LOW) {
        ret = OilLevel_WaitForBlindZone();
        CHECK_COMMAND_SWITCH(ret);
    }
    return ret;
}

/**
 * @brief 通过频率跟随策略定位液位界面。
 *
 * 算法原理：1. 持续比较当前频率与目标频率(空气/油频率均值)。
 * 2. 根据频率偏差计算电机移动距离：- 小偏差：按线性关系移动。
 * 大偏差：采用补偿步长 (4 * overTime + ...)。
 * 3. 达到稳定条件（连续10次频率波动<阈值）时退出。
 * 特殊处理：- 死循环保护：超过100次循环强制退出。
 * 超限保护：连续加速移动仍无法跟踪时报错。
 *
 * @param per_mm_Frequency 每毫米对应的频率变化量（频率-位置换算系数）。
 * @return 执行状态码。
 */
int SearchOilPrecise(float per_mm_Frequency)
{
    int ret;
    uint32_t dir = MOTOR_DIRECTION_DOWN;
    float runlenth = 0.0f;
    int followTime = 0;
    int overTime = 0;
    int lowerTime = 0;
    int loopTime = 0;

    printf("进入频率跟随区间\r\n");

    while (followTime < 10) {
        float frequency_error;

        ret = (int)AbortableDelay_CommandSwitch(2000U, 100U);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        ret = OilLevel_ReadValidatedFrequency(&g_measurement.oil_measurement.current_frequency);
        if (ret != NO_ERROR) {
            return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
        }

        if (loopTime++ > 100) {
            printf("频率跟随可能陷入循环，跳出频率跟随\r\n");
            break;
        }

        frequency_error = OilLevel_GetFrequencyDifference();
        printf("当前频率%ld\t频率阈值%ld\t阈值差%f\r\n",
               g_measurement.oil_measurement.current_frequency,
               g_measurement.oil_measurement.follow_frequency,
               (double)frequency_error);

        OilLevel_ComputePreciseStep(per_mm_Frequency, frequency_error, &overTime, &lowerTime, &runlenth, &dir);

        if (OilLevel_IsPreciseStable(frequency_error) != 0U) {
            followTime++;
            runlenth = 0.0f;
            printf("频率跟随\t电机不动作\t等待频率稳定\t频率稳定次数%d\r\n", followTime);
        } else {
            followTime = 0;
        }

        ret = (int)OilLevel_RunPreciseMove(runlenth, dir);
        if (ret != NO_ERROR) {
            return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
        }

        if ((overTime > MAX_TIMES_WHEN_FRE_FOLLOW) || (lowerTime > MAX_TIMES_WHEN_FRE_FOLLOW)) {
            return OilLevel_StopBeforeReturn(MEASUREMENT_OVERSPEED, "探头频率异常");
        }

        ret = OilLevel_HandlePrecisePositionUpdate();
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
        }
    }

    return NO_ERROR;
}
/**
 * @brief 用当前尺带长度和液位标定值修正罐高，刷新位置并持久化设备参数。
 *
 * 该函数用于执行液位标定流程，包括：
 * - 根据缆线长度和标定液位计算并设置罐体高度
 * - 标定完成后清零标定液位值
 * - 更新传感器高度数据
 * - 保存设备参数
 *
 * @note 函数内部会调用以下辅助函数：
 *       - MotorCtrl_RefreshPositionFromActiveSource(): 按当前记步源刷新当前位置
 *       - save_device_params(): 保存设备参数
 *
 * @note 输出信息包括：
 *       - 标定开始提示
 *       - 标定完成后的罐体高度值
 */
