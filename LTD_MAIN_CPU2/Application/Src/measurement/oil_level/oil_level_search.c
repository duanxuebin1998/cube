/*
 * 文件职责：液位搜索总入口和测量方式分流。
 * 本文件负责搜索流程编排，不重复实现结果字段和 AO 发布。
 * 结果发布和 AO 处理统一由 oil_level_runtime.c 提供，本文件不重复实现。
 */
#include "oil_level_internal.h"
#include "error_log.h"
#include "fault_manager.h"
#include "measure_zero.h"
#include "motor_ctrl.h"
#include "sensor_service.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"
#include "weight.h"
#include <stdint.h>
#include <stdio.h>

/**
 * @brief 搜索并跟随液位
 *
 * 该函数执行液位测量的完整流程，包括回零点检查、液位搜索和液位跟随。
 * 首先检查设备是否需要回零点，如需要则先执行回零点操作。
 * 然后进行液位搜索，最多尝试3次。搜索成功后，根据设备状态执行液位位置修正。
 * 最后进入液位跟随状态，同样最多尝试3次，完成液位的持续跟踪。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 操作成功完成
 *                  - 其他错误码: 操作失败，具体错误码由 SearchZero、SearchOilLevel 或 FollowOilLevel 返回
 *
 * @note 函数执行流程：
 *       1. 检查并执行回零点操作（如果设备需要）
 *       2. 搜索液位（最多3次尝试）
 *       3. 修正液位位置（如果处于校准状态）
 *       4. 跟随液位（最多3次尝试）
 *
 * @note 每次失败后会延迟1秒后重试，如果检测到状态切换（STATE_SWITCH）则中止当前步骤
 */
uint32_t SearchAndFollowOilLevel(void) {
	uint32_t ret;
	uint8_t try_times;
	printf("液位测量\t开始\r\n");
	/* 设备标记需要回零且启用了故障自动回零时，必须先重建零点基准，再开始液位搜索和跟随。 */
	if ((g_measurement.device_status.zero_point_status == 1)&&(g_deviceParams.error_auto_back_zero==1)){
		printf("液位测量\t设备需要回零点\r\n");
		ret = SearchZero();  /* 如果设备需要回零点，先执行回零点测量 */
		CHECK_ERROR(ret);  /* 检查回零点是否成功 */
		printf("液位测量\t回零点完成\r\n");
	}
	printf("液位测量\t初始扭力：%d\r\n", weight_parament.stable_weight);
	/* ************** 步骤 1: 搜索液位 ************** */
	try_times = 0;
	while (try_times < 3) {
		try_times++;

		ret = SearchOilLevel();

		if (ret == NO_ERROR) {
			if (try_times > 1U) {
				ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
				                 ERROR_LOG_OP_SEARCH_OIL_LEVEL,
				                 ERROR_LOG_REASON_RECOVER_OK,
				                 try_times,
				                 3U);
			}
			break;
		} else if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		} else {
			ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
			               ERROR_LOG_OP_SEARCH_OIL_LEVEL,
			               ERROR_LOG_REASON_SEARCH_FAIL,
			               try_times,
			               3U,
			               ret);
			(void)MotorCtrl_SlowStop();
			HAL_Delay(1000);
		}
	}
	if (ret != NO_ERROR) {
		CHECK_ERROR(ret);
	}
	/* 修正液位位置 */
	if (g_measurement.device_status.device_state == STATE_CALIBRATIONOILING) {
		CorrectOilLevelProcess();
	}
	/* ************** 步骤 2: 跟随液位 ************** */
	g_measurement.device_status.device_state = STATE_FLOWOIL; /* 切换到液位跟随状态 */
	try_times = 0;
	while (try_times < 3) {
		try_times++;

		ret = FollowOilLevel();

		if (ret == NO_ERROR) {
			if (try_times > 1U) {
				ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
				                 ERROR_LOG_OP_FOLLOW_OIL_LEVEL,
				                 ERROR_LOG_REASON_RECOVER_OK,
				                 try_times,
				                 3U);
			}
			break;
		} else if (ret == STATE_SWITCH) {
			/* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
			return STATE_SWITCH;
		} else {
			ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
			               ERROR_LOG_OP_FOLLOW_OIL_LEVEL,
			               ERROR_LOG_REASON_FOLLOW_FAIL,
			               try_times,
			               3U,
			               ret);
			HAL_Delay(1000);
		}
	}
	if (ret != NO_ERROR) {
		CHECK_ERROR(ret);
	}

	printf("液位流程\t全部完成\r\n");
	return NO_ERROR;
}

/**
 * @brief 清理单次找液位开始前的命中和稳定状态。
 *
 * @details 调用场景：SearchOilLevel() 的方法 0/1/4/未知路径，以及方法 5 直接闭环入口。
 * @note 关键约束：只清运行态标志和故障缓存，不修改目标频率和测量方法。
 */
void OilLevel_ResetSearchRuntimeState(void)
{
    OilLevelRuntime_ClearStableState();
    fault_info_init();
}

/**
 * @brief 处理不需要空气/油中粗找的直接找液位方式。
 *
 * @details 调用场景：SearchOilLevel() 入口按测量方法分流方法 2、3、5 时调用。
 * @note 关键约束：方法 5 保持固定频率速度闭环，方法 0/1/4 继续交给后续粗找和精找。
 *
 * @param handled 用于返回当前液位查找方式是否已由直接查找分支处理。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t OilLevel_TryRunDirectSearchMethod(uint8_t *handled)
{
    OilLevelStrategySelection selection;

    if (handled == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    OilLevelStrategy_Select(g_deviceParams.liquidLevelMeasurementMethod, &selection);
    *handled = selection.direct_search;
    if (selection.direct_search == 0U) {
        return NO_ERROR;
    }

    switch (selection.family) {
    case OIL_LEVEL_ALGORITHM_DENSITY_CLOSED_LOOP:
        g_measurement.device_status.device_state = STATE_FINDOIL;
        return DensityLevel_RunClosedLoop(OIL_LEVEL_CLOSED_LOOP_SEARCH);
    case OIL_LEVEL_ALGORITHM_UNSUPPORTED:
        printf("液位测量\t方式3为超声预留\r\n");
        return PARAM_FEATURE_UNSUPPORTED;
    case OIL_LEVEL_ALGORITHM_FREQUENCY_CLOSED_LOOP:
        if (selection.fixed_frequency_target == 0U) {
            return SYSTEM_CALL_CONDITION_ERROR;
        }
        g_measurement.device_status.device_state = STATE_FINDOIL;
        OilLevel_ResetSearchRuntimeState();
        return FrequencyLevel_RunFixedClosedLoop(OIL_LEVEL_CLOSED_LOOP_SEARCH);
    default:
        *handled = 0U;
        return NO_ERROR;
    }
}

/**
 * @brief 按既有重试策略启用液位模式。
 *
 * @details 调用场景：SearchOilLevel() 中方法 0/1/4/未知路径进入频率粗找前调用。
 * @note 关键约束：命令切换直接透传；普通错误在本函数内按 CHECK_ERROR 统一出口处理。
 *
 * @return NO_ERROR 表示液位模式在允许次数内启用成功；STATE_SWITCH 表示重试等待被新命令打断，重试耗尽时返回最后一次模式切换或传感器通信错误。
 */
static uint32_t OilLevel_EnableLevelModeWithRetry(void)
{
    uint32_t ret = MEASUREMENT_OILLEVEL_NOTFOUND;
    int mode_try_times = 0;

    while (mode_try_times < 3) {
        mode_try_times++;

        ret = SensorService_EnableLevelMode();
        if (ret == NO_ERROR) {
            if (mode_try_times > 1) {
                ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                                 ERROR_LOG_OP_ENABLE_LEVEL_MODE,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)mode_try_times,
                                 3U);
            }
            break;
        } else if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        } else {
            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_ENABLE_LEVEL_MODE,
                           ERROR_LOG_REASON_MODE_FAIL,
                           (uint32_t)mode_try_times,
                           3U,
                           ret);
            HAL_Delay(500);
        }
    }

    if (ret != NO_ERROR) {
        CHECK_ERROR(ret);
    }

    return NO_ERROR;
}

/**
 * @brief 完成方法 0/1/4/未知路径共用的空气/油中端点粗找。
 *
 * @details 调用场景：SearchOilLevel() 启用液位模式成功后调用。
 * @note 关键约束：入口读频率失败最多重试 3 次；运动安全错误和端点搜索错误保持原有 CHECK_ERROR 出口。
 *
 * @return NO_ERROR 表示空气端和油中端的粗找频率均已确认；STATE_SWITCH 表示用户命令切换，其他传感器、运动或边界错误由粗找检查宏原样返回。
 */
static uint32_t OilLevel_RunFrequencyCoarseSearch(void)
{
    uint32_t ret;
    uint32_t last_coarse_ret = MEASUREMENT_OILLEVEL_NOTFOUND;
    uint8_t coarse_found = 0U;
    int coarse_try_times = 0;

    while (coarse_try_times < 3) {
        coarse_try_times++;
        fault_info_init();
        ret = OilLevel_ReadAverageFrequency(&g_measurement.oil_measurement.current_frequency);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            last_coarse_ret = ret;
            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_READ_LEVEL_FREQ,
                           ErrorLog_GetReasonByCode(ret),
                           (uint32_t)coarse_try_times,
                           3U,
                           ret);
            HAL_Delay(500);
            continue;
        }
        if (OilLevel_IsCurrentFrequencyInOil() != 0U) {
            /* 当前传感器在盲区以上 100mm 时，先下行确保传感器完全在油中。 */
            if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
                ret = MotorCtrl_MoveAndWait(100.0, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
                CHECK_ERROR(ret);
            }

            ret = OilLevel_ReadAverageFrequencyWithRetry(&g_measurement.oil_measurement.oil_frequency, "油中端点");
            CHECK_ERROR(ret);
            printf("液位测量\t油中频率：%ld\r\n", g_measurement.oil_measurement.oil_frequency);

            printf("液位测量\t传感器已在液位中，向上寻找空气\r\n");
            ret = SearchAir();
            CHECK_ERROR(ret);
            coarse_found = 1U;
            break;
        } else if (OilLevel_IsCurrentFrequencyInAir() != 0U) {
            printf("液位测量\t传感器在空气中，向下寻找液位\r\n");
            ret = SearchOil();
            CHECK_ERROR(ret);
            coarse_found = 1U;
            break;
        }
    }

    if (coarse_found == 0U) {
        RETURN_ERROR(last_coarse_ret);
    }
    if (coarse_try_times > 1) {
        ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                         ERROR_LOG_OP_SEARCH_OIL_LEVEL,
                         ERROR_LOG_REASON_RECOVER_OK,
                         (uint32_t)coarse_try_times,
                         3U);
    }
    printf("液位测量\t粗找液位完成\r\n");

    return NO_ERROR;
}

/**
 * @brief 根据测量方法解析本次找液位目标频率。
 *
 * @details 调用场景：SearchOilLevel() 粗找空气/油中端点完成后调用。
 * @note 关键约束：方法 0/4 使用端点中点；方法 1 保持固定目标但仍走步进精找。
 */
static void OilLevel_ResolveSearchTargetFrequency(void)
{
    switch (g_deviceParams.liquidLevelMeasurementMethod) {
    case OIL_LEVEL_METHOD_RELATIVE_FREQ:
    case OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ:
        g_measurement.oil_measurement.follow_frequency =
                (g_measurement.oil_measurement.air_frequency + g_measurement.oil_measurement.oil_frequency) / 2U;
        break;
    case OIL_LEVEL_METHOD_FIXED_FREQ:
        /* 方法1固定频率步进法：目标固定，但仍使用 SearchOilPrecise 步进精找。 */
        g_measurement.oil_measurement.follow_frequency = g_deviceParams.oilLevelFrequency;
        break;
    default:
        g_measurement.oil_measurement.follow_frequency = g_deviceParams.oilLevelFrequency;
        break;
    }
    printf("液位查找\t目标频率：%lu Hz\r\n",
           (unsigned long)g_measurement.oil_measurement.follow_frequency);
}

/**
 * @brief 按解析后的目标执行精找或相对频率速度闭环搜索。
 *
 * @details 调用场景：SearchOilLevel() 目标频率确定后调用。
 * @note 关键约束：方法 4 的闭环函数会自行记录结果；方法 0/1 继续使用 SearchOilPrecise 离散步进。
 *
 * @param result_recorded 用于返回本轮液位查找结果是否已写入测量状态。
 * @return NO_ERROR 表示选定的步进精找或频率闭环搜索完成；STATE_SWITCH 表示新命令打断，其他值为目标频率、传感器读取、位置边界或电机运动错误。
 */
static uint32_t OilLevel_RunResolvedSearchMethod(uint8_t *result_recorded)
{
    uint32_t ret;

    if (result_recorded == NULL) {
        RETURN_ERROR(SYSTEM_CALL_CONDITION_ERROR);
    }
    *result_recorded = 0U;

    switch (g_deviceParams.liquidLevelMeasurementMethod) {
    case OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ:
        ret = FrequencyLevel_RunClosedLoop(OIL_LEVEL_CLOSED_LOOP_SEARCH);
        if (ret != NO_ERROR) {
            CHECK_ERROR(ret);
        }
        *result_recorded = 1U;
        break;
    case OIL_LEVEL_METHOD_RELATIVE_FREQ:
    case OIL_LEVEL_METHOD_FIXED_FREQ:
    default:
        ret = SearchOilPrecise(100);
        /* 方法0/1为旧步进频率方式，保留 SearchOilPrecise 离散步进路径。 */
        if (ret != NO_ERROR) {
            CHECK_ERROR(ret);
        }
        break;
    }

    return NO_ERROR;
}

/**
 * @brief 提交传统步进找液位流程找到的最终位置。
 *
 * @details 共享运行时服务统一更新测量结果、密度
 * 缓存、SI 标志和 AO 样本；找液位专用诊断信息
 * 保留在本模块中，使本模块只负责找液位过程的日志输出。
 *
 * @return 始终返回 NO_ERROR；AO 刷新失败由运行时服务记录，
 *         不改变原有找液位流程的返回约定。
 */
static uint32_t OilLevel_RecordSearchResult(void)
{
    /* 找液位流程已经到达稳定位置。 */
    OilLevelRuntime_CommitLevel(g_measurement.debug_data.sensor_position, "液位测量");
    printf("液位测量\t液位：%lu(0.1mm)\r\n",
           (unsigned long)g_measurement.oil_measurement.oil_level);

    return NO_ERROR;
}
/**
 * @brief 搜索液位高度
 *
 * 该函数通过粗找和精找两个阶段完成液位高度的测量。粗找阶段判断传感器当前状态（在油中或在空气中），
 * 并执行相应的搜索操作；精找阶段根据设定的目标频率进行精确液位定位。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 液位测量成功
 *                  - MEASUREMENT_WEIGHT_DOWN_FAIL: 粗找液位失败（超过最大尝试次数）
 *                  - 其他错误码: 具体错误状态
 *
 * @note 函数执行流程：
 *       1. 方法 2/3/5 先直接分流：密度速度闭环、超声预留、固定频率速度闭环
 *       2. 方法 0/1/4/未知配置复位运行态并启用液位模式
 *       3. 粗找阶段：判断传感器在油中或空气中，执行 SearchAir() 或 SearchOil()
 *       4. 解析目标频率：相对法取端点中点，固定频率步进法取参数频率
 *       5. 执行搜索：方法 4 走频率速度闭环，方法 0/1 走 SearchOilPrecise() 离散步进
 *       6. 步进路径记录最终液位位置；速度闭环路径由闭环函数记录结果
 *
 * @note 依赖函数：
 *       - OilLevel_TryRunDirectSearchMethod(): 处理方法 2/3/5 直接入口
 *       - OilLevel_EnableLevelModeWithRetry(): 启用液位模式并保留重试
 *       - OilLevel_RunFrequencyCoarseSearch(): 执行空气/油中端点粗找
 *       - OilLevel_ResolveSearchTargetFrequency(): 解析本次目标频率
 *       - OilLevel_RunResolvedSearchMethod(): 执行步进精找或方法 4 速度闭环
 *       - OilLevel_RecordSearchResult(): 记录步进精找成功结果
 */
uint32_t SearchOilLevel(void)
{
    uint32_t ret;
    uint8_t direct_handled = 0U;
    uint8_t result_recorded = 0U;

    /* 新一轮搜索开始即失效旧液位，只有成功记录结果后才重新发布。 */
    OilLevelRuntime_InvalidateLevelSample();
    ret = OilLevel_TryRunDirectSearchMethod(&direct_handled);
    if ((ret != NO_ERROR) || (direct_handled != 0U)) {
        return ret;
    }

    OilLevel_ResetSearchRuntimeState();

    ret = OilLevel_EnableLevelModeWithRetry();
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = OilLevel_RunFrequencyCoarseSearch();
    if (ret != NO_ERROR) {
        return ret;
    }

    OilLevel_ResolveSearchTargetFrequency();

    ret = OilLevel_RunResolvedSearchMethod(&result_recorded);
    if (ret != NO_ERROR) {
        return ret;
    }
    if (result_recorded != 0U) {
        return NO_ERROR;
    }

    return OilLevel_RecordSearchResult();
}
