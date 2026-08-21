/*
 * 文件职责：液位算法共用的测频、运动和状态辅助函数。
 * 本文件提供通用辅助，不重复实现液位结果和 AO 发布。
 * 结果发布和 AO 处理统一由 oil_level_runtime.c 提供，本文件不重复实现。
 */
#include "oil_level_internal.h"
#include "abortable_delay.h"
#include "encoder.h"
#include "error_log.h"
#include "motor_ctrl.h"
#include "sensor_comm_diagnostics.h"
#include "sensor_service.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @brief 将跟随滞回确认时间限制到支持范围并换算为毫秒。
 *
 * @return 返回 0 至 60000 ms 的确认窗口；参数为 0 时表示不延时确认。
 */
uint32_t OilLevel_GetFollowChangeConfirmTimeMs(void)
{
    if (g_deviceParams.oilLevelHysteresisTime == 0U) {
        return 0U;
    }
    if (g_deviceParams.oilLevelHysteresisTime > OIL_LEVEL_HYSTERESIS_TIME_MAX_S) {
        return OIL_LEVEL_HYSTERESIS_TIME_MAX_S * 1000U;
    }
    return g_deviceParams.oilLevelHysteresisTime * 1000U;
}

/**
 * @brief 液位跟随检测到偏差后，连续确认偏差是否持续存在。
 *
 * @details 方法 0/1 的旧步进跟随准备重新精找前调用；等待、读频率和命令切换结果均通过 ret_code 透传。
 *
 * @param ret_code 输出本次确认过程的整机结果码；指针无效时函数直接返回 0。
 * @return 1 表示偏差持续到确认窗口结束，应重新搜索；0 表示回到稳定区、被打断或发生读取错误。
 */
uint8_t OilLevel_ConfirmFollowDeviation(uint32_t *ret_code)
{
    uint32_t confirm_time_ms; /* 本轮需要持续确认的总时间，单位 ms。 */
    uint32_t start_tick;      /* 确认窗口起始系统节拍。 */
    uint32_t threshold_hz;    /* 兼容参数换算后的频率偏差阈值，单位 Hz。 */

    if (ret_code == NULL) {
        return 0U;
    }

    *ret_code = NO_ERROR;
    confirm_time_ms = OilLevel_GetFollowChangeConfirmTimeMs();
    if (confirm_time_ms == 0U) {
        return 1U;
    }

    threshold_hz = FrequencyLevel_GetCompatThresholdHz(g_deviceParams.oilLevelHysteresisThreshold);
    start_tick = HAL_GetTick();
    printf("液位跟随\t偏差超阈值\t连续确认%lu秒\r\n",
           (unsigned long)g_deviceParams.oilLevelHysteresisTime);

    while (1) {
        uint32_t elapsed_ms = HAL_GetTick() - start_tick; /* 确认窗口已耗时。 */
        uint32_t remain_ms; /* 确认窗口剩余时间。 */
        uint32_t wait_ms;   /* 本轮可被命令切换打断的等待时间。 */
        float frequency_error; /* 当前频率相对跟随目标的有符号偏差。 */

        if (elapsed_ms >= confirm_time_ms) {
            break;
        }
        remain_ms = confirm_time_ms - elapsed_ms;
        wait_ms = (remain_ms > OIL_LEVEL_FOLLOW_CONFIRM_POLL_MS) ?
                  OIL_LEVEL_FOLLOW_CONFIRM_POLL_MS : remain_ms;

        /* 分段等待，保证新的设备命令可以及时中断确认过程。 */
        *ret_code = AbortableDelay_CommandSwitch(wait_ms, 50U);
        if (*ret_code != NO_ERROR) {
            return 0U;
        }

        /* 使用公共可靠读频接口，保持所有液位方式的错误语义一致。 */
        *ret_code = OilLevel_ReadValidatedFrequency(&g_measurement.oil_measurement.current_frequency);
        if (*ret_code != NO_ERROR) {
            return 0U;
        }

        frequency_error = OilLevel_GetFrequencyDifference();
        if (fabsf(frequency_error) < (float)threshold_hz) {
            printf("液位跟随\t确认期间回到稳定区\t取消重找\r\n");
            return 0U;
        }
    }

    printf("液位跟随\t连续偏差确认完成\t重找液位\r\n");
    return 1U;
}

/**
 * @brief 液位流程故障退出前统一撤销 AO 样本并慢停电机。
 *
 * @param error_code 准备返回给上层的整机错误码或 STATE_SWITCH。
 * @param reason 用于诊断输出的只读原因文字，可为 NULL。
 * @return 原样返回 error_code；NO_ERROR 不触发停机副作用。
 */
uint32_t OilLevel_StopBeforeReturn(uint32_t error_code, const char *reason)
{
    /* 退出副作用统一交给运行服务，当前函数仅保留旧接口兼容。 */
    return OilLevelRuntime_StopAndInvalidate(
            error_code,
            reason,
            OIL_LEVEL_RUNTIME_DOMAIN_GENERIC);
}

/**
 * @brief 将当前传感器位置立即同步到液位测量结果和密度分布液位缓存。
 *
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 */
void OilLevel_SyncCurrentPositionToResult(const char *reason)
{
    /* 校准仅提供当前位置，统一提交由运行服务完成。 */
    OilLevelRuntime_RecordCurrentPosition(reason, OIL_LEVEL_RUNTIME_DOMAIN_SYNC);
}

/**
 * @brief 液位跟随打印液位值时，同时输出当前记步来源和两套尺带长度，便于现场比对。
 */
void OilLevel_PrintFollowPositionInfo(void)
{
    double motor_cable_mm;
    const double encoder_cable_mm = (double)encoder_get_cable_length_01mm() / 10.0;

    if (MotorCtrl_IsPositionSourceMotor()) {
        motor_cable_mm = (double)g_measurement.debug_data.cable_length / 10.0;
    } else {
        motor_cable_mm = (double)g_measurement.debug_data.motor_distance / 10.0;
    }

    printf("\t{编码模式}%s\t{电机记步尺带长度}%.1f\t{编码记步尺带长度}%.1f",
           MotorCtrl_IsPositionSourceMotor() ? "电机记步" : "编码轮记步",
           motor_cable_mm,
           encoder_cable_mm);
}

/**
 * @brief 集中计算当前频率和跟随目标频率的偏差。
 *
 * @details 调用场景：旧步进精找、盲区等待和兼容跟随循环中读取频率差。
 * @note 关键约束：只读全局测量缓存，不触发传感器读取或状态改变。
 *
 * @return 返回当前油位频率减去跟随目标频率的有符号偏差，单位 Hz；正值表示当前频率高于目标，负值表示低于目标。
 */
float OilLevel_GetFrequencyDifference(void)
{
    return (float)g_measurement.oil_measurement.current_frequency -
           (float)g_measurement.oil_measurement.follow_frequency;
}

/**
 * @brief 判断当前平均频率是否属于空气侧。
 *
 * @details 调用场景：粗找入口和液位状态判定。
 * @note 关键约束：保持原空气侧判定口径，等于阈值时不判为空气。
 *
 * @return 1 表示当前平均频率属于空气侧；0 表示当前平均频率不属于空气侧。
 */
uint8_t OilLevel_IsCurrentFrequencyInAir(void)
{
    return (g_measurement.oil_measurement.current_frequency > g_deviceParams.oilLevelFrequency) ? 1U : 0U;
}

/**
 * @brief 判断当前平均频率是否属于油中侧。
 *
 * @details 调用场景：粗找入口判断探头是否已在油中。
 * @note 关键约束：保持原油中侧判定口径，等于阈值时不判为油中。
 *
 * @return 1 表示当前平均频率属于油中侧；0 表示当前平均频率不属于油中侧。
 */
uint8_t OilLevel_IsCurrentFrequencyInOil(void)
{
    return (g_measurement.oil_measurement.current_frequency < g_deviceParams.oilLevelFrequency) ? 1U : 0U;
}


/**
 * @brief 按方向和目标速度启动、停止或更新液位闭环速度运动。
 *
 * @param dir 电机方向，使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN。
 * @param speed_x100 目标速度，单位 0.01 m/min；0 表示停止当前速度运动。
 * @param active_dir 输入并更新当前已下发方向。
 * @param active_speed_x100 输入并更新当前已下发速度。
 * @return NO_ERROR 表示状态已满足；其他值为电机控制或参数错误。
 */
uint32_t LevelVelocity_StartOrUpdateMotion(int dir,
                                                 uint32_t speed_x100,
                                                 int *active_dir,
                                                 uint32_t *active_speed_x100)
{
    uint32_t ret;
    uint32_t old_speed;
    uint32_t delta_speed;

    if ((active_dir == NULL) || (active_speed_x100 == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    if (speed_x100 == 0U) {
        if (*active_dir != OIL_LEVEL_DIRECTION_NONE) {
            ret = MotorCtrl_SlowStop();
            if (ret != NO_ERROR) {
                return ret;
            }
            *active_dir = OIL_LEVEL_DIRECTION_NONE;
            *active_speed_x100 = 0U;
        }
        return NO_ERROR;
    }

    if ((*active_dir != OIL_LEVEL_DIRECTION_NONE) && (*active_dir != dir)) {
        ret = MotorCtrl_SlowStop();
        if (ret != NO_ERROR) {
            return ret;
        }
        *active_dir = OIL_LEVEL_DIRECTION_NONE;
        *active_speed_x100 = 0U;
    }

    old_speed = *active_speed_x100;
    delta_speed = (speed_x100 >= old_speed) ? (speed_x100 - old_speed) : (old_speed - speed_x100);
    if ((*active_dir == dir) && (delta_speed < DENSITY_LEVEL_SPEED_DELTA_X100)) {
        return NO_ERROR;
    }

    ret = MotorCtrl_StartVelocity(dir, speed_x100);
    if (ret != NO_ERROR) {
        return ret;
    }

    *active_dir = dir;
    *active_speed_x100 = speed_x100;
    return NO_ERROR;
}

/**
 * @brief 综合显示状态和驱动器运动位判断电机是否已经停止。
 *
 * @return 1 表示两个运动来源都确认静止；0 表示仍在运动或驱动状态读取失败。
 * @note 驱动读取失败按仍在运动处理，避免频率恢复时误启动微动。
 */
uint8_t OilLevel_IsMotorStopped(void)
{
    uint32_t motor_state = MotorCtrl_GetDisplayState();
    bool is_moving = true;
    uint32_t ret;

    if ((motor_state == 1U) || (motor_state == 2U)) {
        return 0U;
    }

    ret = MotorCtrl_IsDriverMoving(&stepper, &is_moving);
    if (ret != NO_ERROR) {
        return 0U;
    }
    return is_moving ? 0U : 1U;
}

/*
 * 函数用途：在液位频率连续异常后执行受电机状态约束的模式恢复。
 * 调用场景：可靠液位频率读取完成三次无效值重试后。
 * 关键约束：静止时上行1mm并按密度稳定、液位稳定顺序恢复；等待可被命令切换打断。
 */
uint32_t OilLevel_RecoverLevelFrequencyWhenStopped(void)
{
	uint32_t ret;

	if (!OilLevel_IsMotorStopped()) {
		ret = SensorService_EnableLevelMode();
		if (ret != NO_ERROR) {
			return ret;
		}
		return NO_ERROR;
	}

	ret = MotorCtrl_MoveAndWait(OIL_LEVEL_FREQ_RECOVERY_LIFT_MM,
	                            MOTOR_DIRECTION_UP,
	                            MotorCtrl_GetDefaultSpeedX100());
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = SensorService_EnableDensityMode();
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = AbortableDelay_CommandSwitch(OIL_LEVEL_DENSITY_MODE_SETTLE_MS, 100U);
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = SensorService_EnableLevelMode();
	if (ret != NO_ERROR) {
		return ret;
	}

	return NO_ERROR;
}

/*
 * 函数用途：读取并校验液位频率，在连续无效时执行最多三轮受控恢复。
 * 调用场景：液位搜索和跟随需要一个可靠整数Hz频率时。
 * 关键约束：每轮三次无效值不等同串口重试；通信错误立即传播，恢复耗尽返回频率异常。
 */
uint32_t OilLevel_ReadValidatedFrequency(volatile uint32_t *frequency_out) {
	if (frequency_out == NULL) {
		return SYSTEM_CALL_CONDITION_ERROR;   /* 比设备通信错误更合理 */
	}

	uint32_t ret;
	uint32_t hz = 0;
	const int MAX_INVALID_FREQ_RETRY = 3;
	const int MAX_MODE_SWITCH_RECOVERY = 3;
	int mode_switch_recovery_count = 0;

	while (1) {
		for (int attempt = 0; attempt < MAX_INVALID_FREQ_RETRY; attempt++) {
			ret = SensorService_ReadLevelFrequency(&hz);

			if (ret != NO_ERROR) {
				return SensorComm_DiagnoseTimeout(ret, "读取液位频率");  /* 读取失败直接返回错误码 */
			}

			if (hz != 0 && hz <= 6500) {
				*frequency_out = hz;
				printf("液位频率: %lu Hz\r\n", (unsigned long)*frequency_out);
				return NO_ERROR;
			}

            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_READ_LEVEL_FREQ,
                           ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                           (uint32_t)(attempt + 1),
                           MAX_INVALID_FREQ_RETRY,
                           SONIC_FREQ_ABNORMAL);
			ret = AbortableDelay_CommandSwitch(1000U, 100U);
			if (ret != NO_ERROR) {
				return ret;
			}
		}

		if (mode_switch_recovery_count >= MAX_MODE_SWITCH_RECOVERY) {
			return SONIC_FREQ_ABNORMAL;
		}

		mode_switch_recovery_count++;
        ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                       ERROR_LOG_OP_SWITCH_MODE,
                       ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                       (uint32_t)mode_switch_recovery_count,
                       MAX_MODE_SWITCH_RECOVERY,
                       SONIC_FREQ_ABNORMAL);
		ret = OilLevel_RecoverLevelFrequencyWhenStopped();
		if (ret != NO_ERROR) {
			return ret;
		}
        ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                         ERROR_LOG_OP_SWITCH_MODE,
                         ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                         (uint32_t)mode_switch_recovery_count,
                         MAX_MODE_SWITCH_RECOVERY);
	}
}

/*
 * 函数用途：对十次可靠液位频率去除两大两小后计算中间六次平均值。
 * 调用场景：液位跟随建立抗瞬态干扰的基准频率。
 * 关键约束：采样间隔可被新命令打断；任一次可靠读取失败即停止并传播错误。
 */
uint32_t OilLevel_ReadAverageFrequency(volatile uint32_t *frequency_out) {
	if (frequency_out == NULL) {
		return SYSTEM_CALL_CONDITION_ERROR;
	}

	uint32_t values[10];
	uint32_t ret;

	for (int i = 0; i < 10; i++) {
		ret = OilLevel_ReadValidatedFrequency(&values[i]);
		if (ret != NO_ERROR) {
			return ret;
		}
		printf("第 %d 次液位频率: %lu Hz\r\n", i + 1, (unsigned long) values[i]);
		ret = AbortableDelay_CommandSwitch(2000U, 100U); /* 2 秒间隔，可被命令切换打断 */
		if (ret != NO_ERROR) {
			return ret;
		}
	}

	/* 冒泡排序（升序） */
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9 - i; j++) {
			if (values[j] > values[j + 1]) {
				uint32_t tmp = values[j];
				values[j] = values[j + 1];
				values[j + 1] = tmp;
			}
		}
	}

	/* 去掉两个最大与两个最小 */
	float sum = 0.0f;
	for (int i = 2; i < 8; i++) {
		sum += (float) values[i];
	}

	float avg = sum / 6.0f;
	uint32_t avg_u32 = (avg >= 0.0f) ? (uint32_t) (avg + 0.5f) : 0u;
	*frequency_out = avg_u32;

	printf("液位频率平均值(去极值): %lu Hz\r\n", (unsigned long) *frequency_out);
	return NO_ERROR;
}

/**
 * @brief 端点频率需要写入油中/空气基准，读取失败时做局部重试。
 *
 * @details 调用场景：SearchOilLevel、SearchOil、SearchAir 记录端点平均频率时调用。
 * @note 关键约束：只重试传感器平均读数；命令切换和延时中断直接透传。
 *
 * @param frequency_out 用于返回读取或平均后的液位通道频率。
 * @param stage_text 写入重试和失败日志的测量阶段名称，不参与频率计算。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t OilLevel_ReadAverageFrequencyWithRetry(volatile uint32_t *frequency_out,
                                                       const char *stage_text)
{
    uint32_t try_times;
    uint32_t last_ret = SYSTEM_CALL_CONDITION_ERROR;

    if (frequency_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    for (try_times = 1U; try_times <= 3U; try_times++) {
        uint32_t read_ret = OilLevel_ReadAverageFrequency(frequency_out);
        if (read_ret == NO_ERROR) {
            if (try_times > 1U) {
                ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                                 ERROR_LOG_OP_READ_LEVEL_FREQ,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 try_times,
                                 3U);
            }
            return NO_ERROR;
        }
        if (read_ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }

        last_ret = read_ret;
        printf("液位测量\t%s读取平均频率失败\t第%lu/3次\t错误码=0x%08lX\r\n",
               (stage_text != NULL) ? stage_text : "端点频率",
               (unsigned long)try_times,
               (unsigned long)read_ret);
        ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                       ERROR_LOG_OP_READ_LEVEL_FREQ,
                       ErrorLog_GetReasonByCode(read_ret),
                       try_times,
                       3U,
                       read_ret);
        if (try_times < 3U) {
            uint32_t delay_ret = AbortableDelay_CommandSwitch(500U, 100U);
            if (delay_ret != NO_ERROR) {
                return delay_ret;
            }
        }
    }

    return last_ret;
}

/**
 * @brief 获取频率闭环死区，参数未配置时使用保守默认值。
 *
 * @param raw_threshold 兼容参数中的频率阈值 x10 定点值，单位 0.1 Hz；0 表示未配置。
 * @return raw_threshold 为 0 时返回 0；否则把 0.1 Hz 定点阈值四舍五入换算为整数 Hz。
 */
