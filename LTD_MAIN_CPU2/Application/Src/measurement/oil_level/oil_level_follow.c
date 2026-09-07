/*
 * 文件职责：液位跟随入口和运行方式选择。
 * 本文件负责选择配置的算法族，不重复实现各算法的闭环细节。
 * 结果发布和 AO 处理统一由 oil_level_runtime.c 提供，本文件不重复实现。
 */
#include "oil_level_internal.h"
#include "oil_level_reference_refresh.h"
#include "abortable_delay.h"
#include "fault_manager.h"
#include "system_parameter.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief 液位跟随函数，用于持续监测并跟踪液位变化
 *
 * 该函数通过超声波频率信号实现液位的实时监测与跟随。当液位稳定时，电机保持静止；
 * 当检测到液位变动时，重新执行精确搜索并调整传感器位置，确保液位值准确更新。
 *
 * @return uint32_t 返回操作状态码：
 *             - NO_ERROR: 操作成功
 *             - 其他错误码: 具体错误状态
 *
 * @note 函数内部包含无限循环，持续监测液位状态，仅在发生错误时退出
 * @note 液位稳定判断基于频率波动阈值（oilLevelHysteresisThreshold）
 * @note 当液位过低进入盲区时，会调用 OilLevel_WaitForBlindZone 等待液位恢复
 */
static uint32_t FollowOilLevelConfigured(void) {
	uint32_t ret;
	/* 切换到跟随状态 */
	g_measurement.device_status.device_state = STATE_FLOWOIL;

	    OilLevelStrategySelection selection;

    OilLevelStrategy_Select(g_deviceParams.liquidLevelMeasurementMethod, &selection);
    switch (selection.family) {
    case OIL_LEVEL_ALGORITHM_FREQUENCY_CLOSED_LOOP:
        if (selection.fixed_frequency_target != 0U) {
            return FrequencyLevel_RunFixedClosedLoop(OIL_LEVEL_CLOSED_LOOP_FOLLOW);
        }
        return FrequencyLevel_RunClosedLoop(OIL_LEVEL_CLOSED_LOOP_FOLLOW);
    case OIL_LEVEL_ALGORITHM_DENSITY_CLOSED_LOOP:
        return DensityLevel_RunClosedLoop(OIL_LEVEL_CLOSED_LOOP_FOLLOW);
    case OIL_LEVEL_ALGORITHM_UNSUPPORTED:
        printf("液位跟随\t方式3为超声预留\r\n");
        return PARAM_FEATURE_UNSUPPORTED;
    default:
        break;
    }

	/* 液位跟随主循环 */

	while (1) {
		printf("液位跟随\t");
		ret = OilLevelReferenceRefresh_Poll(false);
		if (ret != NO_ERROR) { return ret; }

		/* 获取当前频率 */
		ret = OilLevel_ReadValidatedFrequency(&g_measurement.oil_measurement.current_frequency);
		CHECK_ERROR(ret);  /* 检查开启液位模式是否成功 */
		/* 稳定性判断（频率波动在阈值内） */
		if (fabs(OilLevel_GetFrequencyDifference()) < FrequencyLevel_GetCompatThresholdHz(g_deviceParams.oilLevelHysteresisThreshold)) {
			ret = OilLevelReferenceRefresh_Poll(g_measurement.oil_measurement.liquid_stable != 0U);
			if (ret != NO_ERROR) { return ret; }
			/* 液位稳定时电机不动作，直接打印寄存器中保存的液位值 */
			printf("液位稳定,电机不动作\t");
			printf("液位跟随\t液位值为%ld (0.1mm)", g_measurement.oil_measurement.oil_level);
			ret = (int)AbortableDelay_CommandSwitch(3000U, 100U);
			CHECK_COMMAND_SWITCH(ret);
		} else {
			/* 检测到液位变动，先按滞后时间确认，避免瞬时波动触发重找。 */
			printf("识别到液位变动\t");
			if (OilLevel_ConfirmFollowDeviation(&ret) == 0U) {
				CHECK_COMMAND_SWITCH(ret);
				CHECK_ERROR(ret);
				continue;
			}
			/* 0/1跟随重找期间保持跟随态，按设计允许中间液位继续发布到 AO 和继电器。 */
			ret = SearchOilPrecise(100);
			if (ret != NO_ERROR) {
				return OilLevel_StopBeforeReturn((uint32_t)ret, "液位流程故障");
			} else {
				ret = OilLevel_UpdatePositionAndCheckBounds();
				if (ret == MEASUREMENT_OILLEVEL_LOW) {
					/* 处理盲区状态 */
					ret = OilLevel_WaitForBlindZone();
					CHECK_COMMAND_SWITCH(ret);
					CHECK_ERROR(ret);
				} else {
					CHECK_ERROR(ret);
				}
			}
		}
	}
}

/* 所有方法共用刷新生命周期；任何返回都解除保持并丢弃本轮候选，取消不累计失败。 */
uint32_t FollowOilLevel(void)
{
    uint32_t ret;
    OilLevelReferenceRefresh_EnterFollow();
    ret = FollowOilLevelConfigured();
    return OilLevelReferenceRefresh_LeaveFollow(ret);
}
