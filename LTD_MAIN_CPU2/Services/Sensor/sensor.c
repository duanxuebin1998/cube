/*
 * @FilePath     : \CUBE\LTD_MAIN_CPU2\Services\Sensor\sensor.c
 * @Description  : 传感器通信相关函数
 * @Author       : Aubon
 * @Date         : 2024-02-23 10:20:27
 * @LastEditors  : Duan Xuebin
 * @LastEditTime : 2026-05-12 11:06:48
 * Copyright 2024 Aubon, All Rights Reserved.
 * 2024-02-23 10:20:27
 */

#include "sensor.h"
#include "error_log.h"
#include "AS5145.h"
#include "measure_tank_height.h"
#include "measure.h"
#include "motor_ctrl.h"
#include "abortable_delay.h"

#define WIRELESS_HOST_ADDR 1U
#define WIRELESS_SLAVE_ADDR 2U
#define SENSOR_LEVEL_FREQ_RECOVERY_LIFT_MM 1.0f
#define SENSOR_DENSITY_MODE_SETTLE_MS 3000U

static uint32_t Sensor_PositionToU01mmClamped(void);


/**
 * @brief 把轻量无线探测错误映射成对应节点错误码。
 *
 * 仅用于通信超时后的链路归因，不打印日志；STATE_SWITCH 等非超时返回值保持原样向上传递。
 */
static uint32_t Sensor_MapWirelessProbeError(uint32_t ret, uint32_t timeout_code)
{
    if (ret == NO_ERROR) {
        return NO_ERROR;
    }
    if (ret == SENSOR_DEVICE_COMM_TIMEOUT) {
        return timeout_code;
    }
    return ret;
}

/**
 * @brief 在 LTD 和 DSM 探测结果中选择最终识别错误。
 *
 * 优先保留协议格式、校验等具体错误；只有两路都是无响应时才归并为传感器通信超时。
 */
static uint32_t Sensor_SelectProbeError(uint32_t ltd_ret, uint32_t dsm_ret)
{
    if ((ltd_ret != NO_ERROR) && (ltd_ret != SENSOR_DEVICE_COMM_TIMEOUT)) {
        return ltd_ret;
    }
    if ((dsm_ret != NO_ERROR) && (dsm_ret != SENSOR_DEVICE_COMM_TIMEOUT)) {
        return dsm_ret;
    }
    if ((ltd_ret == SENSOR_DEVICE_COMM_TIMEOUT) || (dsm_ret == SENSOR_DEVICE_COMM_TIMEOUT)) {
        return SENSOR_DEVICE_COMM_TIMEOUT;
    }
    if (ltd_ret != NO_ERROR) {
        return ltd_ret;
    }
    return dsm_ret;
}

/**
 * @brief 记录传感器识别阶段的最终错误。
 *
 * NO_ERROR 和 STATE_SWITCH 都不是故障，不写入全局 error_code，避免命令切换被误报。
 */
static void Sensor_SetCommDetectError(uint32_t err)
{
    if ((err != NO_ERROR) && (err != STATE_SWITCH)) {
        g_measurement.device_status.error_code = err;
    }
}


/**
 * @brief 在传感器无响应后执行滑环/无线链路诊断。
 *
 * 该函数只在普通任务上下文调用，允许打印错误报警；诊断过程中若收到命令切换，
 * 立即返回 STATE_SWITCH，不把切换动作当作通信故障。
 */
static uint32_t Sensor_DiagnoseCommTimeout(uint32_t ret, const char *context)
{
    uint32_t host_ret;
    uint32_t slave_ret;
    uint32_t diag_ret;
    const char *op_context = (context != NULL) ? context : "未知操作";
    char detail[96];

    if (ret != SENSOR_DEVICE_COMM_TIMEOUT) {
        return ret;
    }

    snprintf(detail, sizeof(detail), "原操作：%s", op_context);
    // 错误	阶段：错误报警	模块：传感器	操作：通信诊断	原因：ErrorLog_GetReasonByCode(ret)	处理：继续尝试	详情：detail
    ErrorLog_WarnDetail(ERROR_LOG_MODULE_SENSOR,
                        ERROR_LOG_OP_COMM_DIAG,
                        ErrorLog_GetReasonByCode(ret),
                        ERROR_LOG_ACTION_CONTINUE,
                        detail);

    host_ret = WIRELESS_ProbeNode(WIRELESS_HOST_ADDR);
    if (host_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    diag_ret = Sensor_MapWirelessProbeError(host_ret, WIRELESS_HOST_COMM_TIMEOUT);
    if (diag_ret != NO_ERROR) {
        snprintf(detail, sizeof(detail), "原操作：%s,节点：主机,地址：%u", op_context, (unsigned)WIRELESS_HOST_ADDR);
        // 错误	阶段：错误报警	模块：滑环通信	操作：链路诊断	原因：ErrorLog_GetReasonByCode(diag_ret)	处理：继续尝试	详情：detail
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_SLIPRING_COMM,
                            "链路诊断",
                            ErrorLog_GetReasonByCode(diag_ret),
                            ERROR_LOG_ACTION_CONTINUE,
                            detail);
        return diag_ret;
    }

    slave_ret = WIRELESS_ProbeNode(WIRELESS_SLAVE_ADDR);
    if (slave_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    diag_ret = Sensor_MapWirelessProbeError(slave_ret, WIRELESS_SLAVE_COMM_TIMEOUT);
    if (diag_ret != NO_ERROR) {
        snprintf(detail, sizeof(detail), "原操作：%s,节点：从机,地址：%u", op_context, (unsigned)WIRELESS_SLAVE_ADDR);
        // 错误	阶段：错误报警	模块：滑环通信	操作：链路诊断	原因：ErrorLog_GetReasonByCode(diag_ret)	处理：继续尝试	详情：detail
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_SLIPRING_COMM,
                            "链路诊断",
                            ErrorLog_GetReasonByCode(diag_ret),
                            ERROR_LOG_ACTION_CONTINUE,
                            detail);
        return diag_ret;
    }
    return SENSOR_DEVICE_COMM_TIMEOUT;
}

/**
 * @brief 识别传感器前检查主机和从机无线链路。
 *
 * 用完整信息读取确认链路是否可用，失败时映射为主机或从机通信超时，便于现场定位。
 */
static uint32_t Sensor_ProbeWirelessLink(void)
{
    uint32_t ret;

    ret = WIRELESS_PrintInfo(WIRELESS_HOST_ADDR);
    if (ret != NO_ERROR) {
        ret = Sensor_MapWirelessProbeError(ret, WIRELESS_HOST_COMM_TIMEOUT);
        return ret;
    }

    ret = WIRELESS_PrintInfo(WIRELESS_SLAVE_ADDR);
    if (ret != NO_ERROR) {
        ret = Sensor_MapWirelessProbeError(ret, WIRELESS_SLAVE_COMM_TIMEOUT);
        return ret;
    }

    return NO_ERROR;
}

/**
 * @brief 解析 DSM 一代字符串编号中的数字部分。
 *
 * CN 指令返回通常形如 N2009924H，系统参数只能保存 uint32_t，因此这里只提取连续数字并保存为传感器编号。
 */
static uint32_t Sensor_ParseDsmTextId(const char *id_text, uint32_t *sensor_id_out)
{
    uint32_t value = 0U;
    uint8_t has_digit = 0U;

    if ((id_text == NULL) || (sensor_id_out == NULL)) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    while (*id_text != '\0') {
        if ((*id_text >= '0') && (*id_text <= '9')) {
            value = (value * 10U) + (uint32_t)(*id_text - '0');
            has_digit = 1U;
        }
        id_text++;
    }

    if (has_digit == 0U) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    *sensor_id_out = value;
    return NO_ERROR;
}

/**
 * @brief 用传感器编号读数探测 LTD/V2 传感器。
 *
 * 识别阶段直接读取编号，成功时回填 sensorID，避免后续再额外发一次编号读取命令。
 */
static uint32_t Sensor_ProbeLtdSensor(uint32_t *sensor_id_out)
{
    uint32_t sensor_id = 0U;
    uint32_t ret = DSM_V2_Read_SensorID(&sensor_id);

    if ((ret == NO_ERROR) && (sensor_id_out != NULL)) {
        *sensor_id_out = sensor_id;
    }
    return ret;
}

/**
 * @brief 用振动管编号探测 DSM 一代传感器。
 *
 * CN 编号读取用于识别协议并保存编号；识别成功后仍切到密度模式，保持初始化后的运行模式语义不变。
 */
static uint32_t Sensor_ProbeDsmSensor(uint32_t *sensor_id_out)
{
    char id_text[RCVBUFFLEN] = {0};
    uint32_t sensor_id = 0U;
    uint32_t ret;

    ret = Read_VibrationTube_ID(id_text, sizeof(id_text));
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = Sensor_ParseDsmTextId(id_text, &sensor_id);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = DSM_EnableDensityMode();
    if (ret != NO_ERROR) {
        return ret;
    }

    if (sensor_id_out != NULL) {
        *sensor_id_out = sensor_id;
    }
    return NO_ERROR;
}
static int Sensor_SupportsAuxDsmChannels(void)
{
    return (g_deviceParams.sensorType == DSM_SENSOR);
}
/**
 * @brief 自动识别传感器类型（DSM 一代 / DSM_V2 / SIL）
 *
 * @return uint32_t 错误码或 NO_ERROR
 */

uint32_t DetectSensorType(void) {
	uint32_t ret = NO_ERROR;
	uint32_t ltd_ret;
	uint32_t dsm_ret;
	uint32_t sensor_id = 0U;

	printf("========== 传感器识别开始 ==========\r\n");
	printf("[1/3] 检查无线链路\r\n");

	ret = Sensor_ProbeWirelessLink();
	if (ret != NO_ERROR) {
		Sensor_SetCommDetectError(ret);
		return ret;
	}
	printf("无线链路正常\r\n");

	printf("[2/3] 尝试LTD协议\r\n");
	ltd_ret = Sensor_ProbeLtdSensor(&sensor_id);
	if (ltd_ret == NO_ERROR) {
		g_deviceParams.sensorType = LTD_SENSOR;
		g_deviceParams.sensorID = sensor_id;
		/* 传感器类型属于系统参数，这里是运行期自动识别场景，
		 * 如果不立即保存，CPU3 后续就看不到这次变更，
		 * 下次重启也会丢掉新的 sensorType。 */
		save_device_params();
		printf("识别成功：LTD传感器 | 编号=%lu\r\n", (unsigned long)sensor_id);
		printf("====================================\r\n");
		return NO_ERROR;
	}

	// 错误	阶段：错误报警	模块：传感器	操作：通信诊断	原因：ErrorLog_GetReasonByCode(ltd_ret)	处理：继续尝试
	ErrorLog_Warn(ERROR_LOG_MODULE_SENSOR,
	              ERROR_LOG_OP_COMM_DIAG,
	              ErrorLog_GetReasonByCode(ltd_ret),
	              ERROR_LOG_ACTION_CONTINUE);
	dsm_ret = Sensor_ProbeDsmSensor(&sensor_id);
	if (dsm_ret == NO_ERROR) {
		g_deviceParams.sensorType = DSM_SENSOR;
		g_deviceParams.sensorID = sensor_id;
		/* 同上：DSM 识别成功后也要立即落盘，
		 * 这样才能触发 parameter_update_flag，让 CPU3 补读最新的系统参数。 */
		save_device_params();
		printf("识别成功：DSM传感器 | 编号=%lu | 密度模式握手成功\r\n", (unsigned long)sensor_id);
		printf("====================================\r\n");
		return NO_ERROR;
	}

	// 错误	阶段：错误报警	模块：传感器	操作：通信诊断	原因：ErrorLog_GetReasonByCode(dsm_ret)	处理：继续尝试
	ErrorLog_Warn(ERROR_LOG_MODULE_SENSOR,
	              ERROR_LOG_OP_COMM_DIAG,
	              ErrorLog_GetReasonByCode(dsm_ret),
	              ERROR_LOG_ACTION_CONTINUE);
	ret = Sensor_SelectProbeError(ltd_ret, dsm_ret);
	Sensor_SetCommDetectError(ret);
	return ret;
}
uint32_t EnableDensityMode(void) {
	uint32_t ret;
	if (g_deviceParams.sensorType == DSM_SENSOR) {
		ret = DSM_EnableDensityMode();
	} else {
		ret = DSM_V2_SwitchToDensityMode();
	}
	return Sensor_DiagnoseCommTimeout(ret, "切换密度模式");
}

static uint32_t Sensor_PrepareLtdDensityModeForPartParams(void)
{
	uint32_t ret = EnableDensityMode();
	if (ret != NO_ERROR) {
		return ret;
	}
	printf("读取部件参数\tLTD已切换密度模式，等待%lu ms稳定\r\n",
	       (unsigned long)SENSOR_DENSITY_MODE_SETTLE_MS);
	return AbortableDelay_CommandSwitch(SENSOR_DENSITY_MODE_SETTLE_MS, 50U);
}

uint32_t EnableLevelMode(void) {
	uint32_t ret;

	ret = MotorCtrl_SlowStop();
	if (ret != NO_ERROR) {
		return ret;
	}

	if (g_deviceParams.sensorType == DSM_SENSOR) {
		ret = DSM_EnableLevelMode();
	} else {
		ret = DSM_V2_SwitchToLevelMode();
	}
	ret = Sensor_DiagnoseCommTimeout(ret, "切换液位模式");
	if (ret == NO_ERROR) {
		printf("切换液位模式成功，等待%lu ms稳定\r\n", (unsigned long)SENSOR_LEVEL_MODE_SETTLE_MS);
		ret = AbortableDelay_CommandSwitch(SENSOR_LEVEL_MODE_SETTLE_MS, 100U);
	}
	return ret;
}

// 读取一次并以整数 Hz 返回。
// 这里的循环是业务层“等待有效频率”，不是底层串口通信重试；
// 真正的通信重试统一收敛在各协议层，所有 UART6 传感器/无线协议统一使用 UART6_COMM_MAX_RETRY。
// 如果频率连续 3 次为 0 或大于 6500Hz，且电机静止，则上行 1mm 后切密度/液位模式恢复；
// 若多轮恢复后仍无有效频率，则返回 SONIC_FREQ_ABNORMAL。

/**
 * @brief 判断电机是否已经停止，供液位频率恢复动作使用。
 *
 * 先看上层显示状态，再读取驱动运动状态；该函数会访问 TMC5130，不允许在中断中调用。
 */
static uint8_t Sensor_IsMotorStopped(void)
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

/**
 * @brief 电机停止时尝试恢复液位频率有效读数。
 *
 * 仅在频率连续异常且电机静止时执行：微动上行后切密度/液位模式重新稳定；
 * 等待过程使用可打断延时，命令切换时直接返回 STATE_SWITCH。
 */
static uint32_t Sensor_RecoverLevelFrequencyWhenStopped(void)
{
	uint32_t ret;

	if (!Sensor_IsMotorStopped()) {
		ret = EnableLevelMode();
		if (ret != NO_ERROR) {
			return ret;
		}
		return NO_ERROR;
	}

	ret = MotorCtrl_MoveAndWait(SENSOR_LEVEL_FREQ_RECOVERY_LIFT_MM,
	                            MOTOR_DIRECTION_UP,
	                            MotorCtrl_GetDefaultSpeedX100());
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = EnableDensityMode();
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = AbortableDelay_CommandSwitch(SENSOR_DENSITY_MODE_SETTLE_MS, 100U);
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = EnableLevelMode();
	if (ret != NO_ERROR) {
		return ret;
	}

	return NO_ERROR;
}

uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency_out) {
	if (frequency_out == NULL) {
		return PARAM_ADDRESS_OVERFLOW;   // 比设备通信错误更合理
	}

	uint32_t ret;
	uint32_t hz = 0;
	const int MAX_INVALID_FREQ_RETRY = 3;
	const int MAX_MODE_SWITCH_RECOVERY = 3;
	int mode_switch_recovery_count = 0;

	while (1) {
		for (int attempt = 0; attempt < MAX_INVALID_FREQ_RETRY; attempt++) {
			if (g_deviceParams.sensorType == DSM_SENSOR) {
				ret = Read_Level_Frequency(&hz);
			} else {
				ret = DSM_V2_Read_LevelFrequency(&hz);
			}

			if (ret != NO_ERROR) {
				return Sensor_DiagnoseCommTimeout(ret, "读取液位频率");  // 读取失败直接返回错误码
			}

			if (hz != 0 && hz <= 6500) {
				*frequency_out = hz;
				printf("液位频率: %lu Hz\r\n", (unsigned long)*frequency_out);
				return NO_ERROR;
			}

            // 错误	阶段：错误重试	模块：传感器	操作：读取液位频率	原因：ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL)	尝试：(attempt + 1)/MAX_INVALID_FREQ_RETRY	错误码：SONIC_FREQ_ABNORMAL	错误名：ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL)
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
        // 错误	阶段：错误重试	模块：传感器	操作：切换模式	原因：ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL)	尝试：mode_switch_recovery_count/MAX_MODE_SWITCH_RECOVERY	错误码：SONIC_FREQ_ABNORMAL	错误名：ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL)
        ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                       ERROR_LOG_OP_SWITCH_MODE,
                       ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                       (uint32_t)mode_switch_recovery_count,
                       MAX_MODE_SWITCH_RECOVERY,
                       SONIC_FREQ_ABNORMAL);
		ret = Sensor_RecoverLevelFrequencyWhenStopped();
		if (ret != NO_ERROR) {
			return ret;
		}
        // 错误	阶段：重试成功	模块：传感器	操作：切换模式	原因：ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL)	尝试：mode_switch_recovery_count/MAX_MODE_SWITCH_RECOVERY
        ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                         ERROR_LOG_OP_SWITCH_MODE,
                         ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                         (uint32_t)mode_switch_recovery_count,
                         MAX_MODE_SWITCH_RECOVERY);
	}
}
/**
 * @brief 获取液位跟随频率的平均值
 *        （10 次采样，2s 间隔，去 2 大 2 小，取中间 6 次均值）
 */
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency_out) {
	if (frequency_out == NULL) {
		return PARAM_ADDRESS_OVERFLOW;
	}

	uint32_t values[10];
	uint32_t ret;

	for (int i = 0; i < 10; i++) {
		ret = DSM_Get_LevelMode_Frequence(&values[i]);
		if (ret != NO_ERROR) {
			return ret;
		}
		printf("第 %d 次液位频率: %lu Hz\r\n", i + 1, (unsigned long) values[i]);
		ret = AbortableDelay_CommandSwitch(2000U, 100U); // 2 秒间隔，可被命令切换打断
		if (ret != NO_ERROR) {
			return ret;
		}
	}

	// 冒泡排序（升序）
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9 - i; j++) {
			if (values[j] > values[j + 1]) {
				uint32_t tmp = values[j];
				values[j] = values[j + 1];
				values[j + 1] = tmp;
			}
		}
	}

	// 去掉两个最大与两个最小
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

uint32_t Read_Density_text(float *frequency, float *density, float *temp) {
	if (frequency == NULL || temp == NULL || density == NULL) {
		return PARAM_ADDRESS_OVERFLOW;
	}
	*frequency = 5500.123f;
	*density = 800.5f;
	*temp = -180.52f;
	printf("密度: %.2f  频率: %.3f  温度: %.3f ℃\r\n", *density, *frequency, *temp);

	return 0;
}

static void Apply_Fixed_DensityTemp_Correction(float *density, float *temp)
{
    if ((density)&&(*density>200.0)) {
        *density = (*density) + ((float)g_deviceParams.densityCorrection-10000.0f) / 10.0f;
    }

    if (temp) {
        *temp = (*temp) + ((float)g_deviceParams.temperatureCorrection-1000.0f)/ 10.0f;
    }
}

uint32_t Read_Density(float *frequency, float *density, float *temp) {
	if (frequency == NULL || temp == NULL || density == NULL) {
		return PARAM_ADDRESS_OVERFLOW;
	}
	float hz_45, hz_225;
	uint32_t ret = NO_ERROR;
	if (g_deviceParams.sensorType == DSM_SENSOR) {
		ret = DSM_Read_Frequency_Density_Temp(frequency, density, temp);
	} else {
		ret = DSM_V2_Read_Temperature(temp);
		if (ret == NO_ERROR) {
			printf("温度值: %.3f ℃\r\n", *temp);
		} else {
			return Sensor_DiagnoseCommTimeout(ret, "读取LTD温度");
		}

		ret = DSM_V2_Read_Density(density);
		if (ret == NO_ERROR) {
			printf("密度值: %.3f\r\n", *density);
		} else {
			return Sensor_DiagnoseCommTimeout(ret, "读取LTD密度");
		}
		ret = DSM_V2_Read_DensityFrequency(frequency,&hz_45,&hz_225);
		if (ret == NO_ERROR) {
			printf("频率值: %.1f Hz\r\n45度扫频周期平方均值:  %.2f Hz\r\n22.5度扫频周期平方均值: %.2f Hz\r\n", *frequency,hz_45,hz_225);
		} else {
			return Sensor_DiagnoseCommTimeout(ret, "读取LTD频率");
		}
	}
	ret = Sensor_DiagnoseCommTimeout(ret, "读取密度");
	if (ret == NO_ERROR) {

	    /* ===== 固定系数修正 ===== */
	    float density_before = *density;
	    float temp_before    = *temp;

	    Apply_Fixed_DensityTemp_Correction(density, temp);//修正密度和温度

	    printf("原始密度: %.3f  修正后密度: %.3f\r\n",
	           density_before, *density);
	    printf("原始温度: %.3f ℃  修正后温度: %.3f ℃\r\n",
	           temp_before, *temp);
	    printf("频率: %.3f Hz\r\n", *frequency);

	    uint32_t density_raw = DENSITY_TO_RAW(*density);
	    uint32_t temp_raw    = TEMP_TO_RAW(*temp);
		uint32_t pos = Sensor_PositionToU01mmClamped();
		if(*density != 0)
		{
			g_measurement.single_point_monitoring.density = density_raw;
		}
		g_measurement.single_point_monitoring.temperature = temp_raw;
		g_measurement.single_point_monitoring.temperature_position = pos;

		if(*density !=0)
		{
			g_measurement.single_point_measurement.density = density_raw;
		}
		g_measurement.single_point_measurement.temperature = temp_raw;
		g_measurement.single_point_measurement.temperature_position = pos;

		//调试信息赋值
	    g_measurement.debug_data.temperature = temp_raw;
	    g_measurement.debug_data.frequency = *frequency;
	}

	return ret;
}

/**
 * @brief 将当前传感器位置写入无符号结果字段前钳位，避免负位置变成超大数。
 */
static uint32_t Sensor_PositionToU01mmClamped(void)
{
    int32_t pos_s = g_measurement.debug_data.sensor_position;

    if (pos_s <= 0) {
        return 0U;
    }

    return (uint32_t)pos_s;
}
uint32_t Sensor_ReadWaterCapacitance(float *cap_out)
{
    if (!Sensor_SupportsAuxDsmChannels()) {
        printf("当前传感器类型不支持读取水位电容\r\n");
        return PARAM_ERROR;
    }

    uint32_t ret = Read_Water_Capacitance(cap_out);
    return Sensor_DiagnoseCommTimeout(ret, "读取水位电容");
}

uint32_t Sensor_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg)
{
    if (!Sensor_SupportsAuxDsmChannels()) {
        printf("当前传感器类型不支持读取姿态角\r\n");
        return PARAM_ERROR;
    }

    uint32_t ret = Read_Gyro_Angle(angle_x_deg, angle_y_deg);
    return Sensor_DiagnoseCommTimeout(ret, "读取陀螺仪");
}
uint32_t Sensor_Test1(void) {
	float frequency = 5500.123f;
	float density = 800.5f;
	float temp = -180.52f;

	printf("密度: %.2f  频率: %.3f  温度: %.3f ℃\r\n", density, frequency, temp);

	uint32_t density_raw = DENSITY_TO_RAW(density);
	uint32_t temp_raw = TEMP_TO_RAW(temp);
	uint32_t pos = Sensor_PositionToU01mmClamped();

	g_measurement.single_point_monitoring.density = density_raw;
	g_measurement.single_point_monitoring.temperature = temp_raw;
	g_measurement.single_point_monitoring.temperature_position = pos;

	g_measurement.single_point_measurement.density = density_raw;
	g_measurement.single_point_measurement.temperature = temp_raw;
	g_measurement.single_point_measurement.temperature_position = pos;

	return NO_ERROR;
}

/* ================== 读取部件参数：适配层（你按工程实际替换实现） ================== */

/* 1) 编码器值：若你工程已有函数，替换这里即可 */
static int32_t Read_CurrentEncoderValue_Adapter(void)
{
    /* 你现在很多地方用 g_encoder_count；若你的“实际编码器”不同，替换这里 */
    return -(int32_t)g_encoder_count;
}

/* 2) 传感器位置/尺带长度：如果你已有统一换算函数，直接调用它
 *    下面给出最保守写法：优先使用现有 debug_data.sensor_position（如果别处已维护）
 *    如果你希望这里“主动计算”，就把 TODO 替换为你的长度/位置换算函数。
 */
static int32_t Calc_SensorPosition_Adapter(void)
{
    /* TODO: 替换为你工程中的“绝对位置mm/0.1mm”计算逻辑 */
    return g_measurement.debug_data.sensor_position;
}

static int32_t Calc_CableLength_Adapter(void)
{
    /* TODO: 替换为你工程中的“尺带长度”计算逻辑（允许负值则这里不要 abs） */
    return g_measurement.debug_data.cable_length;
}

/* 3) 电机步进/距离/速度/状态：按你工程的 stepper/TMC5130 接口改 */
static int32_t Read_MotorStep_Adapter(void)
{
    /* TODO: 若你有 stpr_getPosition() / XACTUAL 等寄存器，替换这里 */
    return g_measurement.debug_data.motor_step;
}

static int32_t Read_MotorDistance_0p1mm_Adapter(void)
{
    /* TODO: 若你有当前运动距离累计，替换这里 */
    return g_measurement.debug_data.motor_distance;
}

static uint32_t Read_MotorSpeed_Adapter(void)
{
    /* TODO: 若你有实际速度读取接口，替换这里 */
    return g_measurement.debug_data.motor_speed;
}

static uint32_t Read_MotorState_Adapter(void)
{
    /* TODO: 统一为 0停 1上 2下；如果你已有 g_measurement.debug_data.motor_state 维护，就直接返回 */
    return g_measurement.debug_data.motor_state;
}

/* 4) 称重：通常 weight_parament.current_weight 已是实时值 */
static uint32_t Read_CurrentWeight_Adapter(void)
{
    return (uint32_t)weight_parament.current_weight;
}

static uint32_t Read_WeightParam_Adapter(void)
{
    /* TODO: 若你有称重系数/滤波参数等，可填这里；没有就保持原值 */
    return g_measurement.debug_data.weight_param;
}

/* ================== CMD：读取部件参数 ================== */
/**
 * @brief 读取部件参数的共用实现。
 *
 * 命令入口调用时会进入读取状态；自动恢复检查调用时不改命令状态，
 * 但仍会先检查电机驱动健康，避免 24V 断电后继续报告无效位置。
 */
static uint32_t Sensor_ReadPartParamsInternal(uint8_t update_command_state)
{
    uint32_t ret = NO_ERROR;
    uint8_t is_ltd_sensor = (g_deviceParams.sensorType != DSM_SENSOR) ? 1U : 0U;

    float ax = 0.0f, ay = 0.0f;
    float freq = 0.0f, dens = 0.0f, temp = 0.0f;
    float cap = 0.0f;

    if (update_command_state) {
        ret = (uint32_t)MeasureStart();
        if (ret != NO_ERROR) {
            return ret;
        }
        g_measurement.device_status.device_state = STATE_READPARAMETERING;
    } else if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    /* ---------- 1) 位置类：编码器/位置/尺带长度/步进/距离 ---------- */
    ret = MotorCtrl_CheckDriverGstat();
    if (ret != NO_ERROR) {
        return ret;
    }

    MotorCtrl_RefreshDebugDrumState();
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (!MotorCtrl_IsPositionSourceMotor()) {
        ret = AS5145_GetLastError();
        if (ret != NO_ERROR) {
            return ret;
        }
    }

    g_measurement.debug_data.current_encoder_value = Read_CurrentEncoderValue_Adapter();
    g_measurement.debug_data.sensor_position       = Calc_SensorPosition_Adapter();
    g_measurement.debug_data.cable_length          = Calc_CableLength_Adapter();
    g_measurement.debug_data.motor_step            = Read_MotorStep_Adapter();
    g_measurement.debug_data.motor_distance        = Read_MotorDistance_0p1mm_Adapter();

    /* ---------- 2) 电机状态类：速度/状态 ---------- */
    g_measurement.debug_data.motor_speed = Read_MotorSpeed_Adapter();
    g_measurement.debug_data.motor_state = Read_MotorState_Adapter();

    /* ---------- 3) 称重类 ---------- */
    ret = Weight_CheckOwnCommunicationTimeout();

    if (ret != NO_ERROR) {
        return ret;
    }

    g_measurement.debug_data.current_weight = Read_CurrentWeight_Adapter();
    g_measurement.debug_data.weight_param   = Read_WeightParam_Adapter();

    /* ---------- 4) 姿态角（陀螺仪） ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (Sensor_SupportsAuxDsmChannels()) {
        ret = Sensor_ReadGyroAngle(&ax, &ay);
        if (ret != NO_ERROR) {
            return ret;
        } else {
            /* 你 Read_Gyro_Angle() 里已经写了 debug_data.angle_x/y，这里再确保一遍 */
            g_measurement.debug_data.angle_x = (int32_t)(ax * 100.0f);
            g_measurement.debug_data.angle_y = (int32_t)(ay * 100.0f);
        }
    } else {
        g_measurement.debug_data.angle_x = 0;
        g_measurement.debug_data.angle_y = 0;
        printf("读取部件参数\t当前传感器类型不支持姿态角读取，已跳过\r\n");
    }

    /* ---------- 5) 密度/温度/频率 ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (is_ltd_sensor) {
        ret = Sensor_PrepareLtdDensityModeForPartParams();
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
    }

    if (ret == NO_ERROR) {
        ret = Read_Density(&freq, &dens, &temp);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
    }

    if (ret != NO_ERROR) {
        if (is_ltd_sensor) {
            g_measurement.debug_data.frequency = 0U;
            printf("读取部件参数\tLTD密度/频率暂未读到，按部分成功处理，其他部件参数保留有效。错误码=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else {
            return ret;
        }
    } else {
        /* 你 Read_Density() 里已写 debug_data.temperature/frequency，这里保证一致 */
        g_measurement.debug_data.frequency    = (uint32_t)freq;   /* 若你要保留小数频率，可改成 ×100 或另存 */
        /* debug_data.temperature 在 Read_Density 内已 TEMP_TO_RAW，避免重复计算 */
    }

    /* ---------- 6) 水位电容/电压 ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (Sensor_SupportsAuxDsmChannels()) {
        ret = Sensor_ReadWaterCapacitance(&cap);
        if (ret != NO_ERROR) {
            return ret;
        } else {
            /* 水位电容值/电压值：你结构体写 uint32_t，这里约定 ×10 或 ×100 以保留小数
               若你工程已有“水位电容原始值”的标定口径，请按你的口径替换 */
            g_measurement.debug_data.water_level_voltage = (uint32_t)(cap * 10.0f); /* 例如：0.1单位 */
        }
    } else {
        g_measurement.debug_data.water_level_voltage = 0;
        printf("读取部件参数\t当前传感器类型不支持水位电容读取，已跳过\r\n");
    }

    /* ---------- 7) 预留接口：后续新增部件参数统一挂这里 ---------- */
    /* TODO:
       - 读电源电压/驱动电压
       - 读TMC5130错误寄存器(GSTAT/DRV_STATUS)
       - 读DSP/传感器版本号
       - 读温度2/环境温度
    */

    /* ---------- 8) 打印汇总：正常信息集中一行 ---------- */
    printf("读取部件参数完成 | 编码值=%ld 位置=%ld 缆长=%ld 步数=%ld 距离=%ld(0.1mm) "
           "| freq=%lu temp=%lu | cap=%lu | w=%lu | ax=%ld ay=%ld | mspd=%lu mstate=%lu\r\n",
           (long)g_measurement.debug_data.current_encoder_value,
           (long)g_measurement.debug_data.sensor_position,
           (long)g_measurement.debug_data.cable_length,
           (long)g_measurement.debug_data.motor_step,
           (long)g_measurement.debug_data.motor_distance,
           (unsigned long)g_measurement.debug_data.frequency,
           (unsigned long)g_measurement.debug_data.temperature,
           (unsigned long)g_measurement.debug_data.water_level_voltage,
           (unsigned long)g_measurement.debug_data.current_weight,
           (long)g_measurement.debug_data.angle_x,
           (long)g_measurement.debug_data.angle_y,
           (unsigned long)g_measurement.debug_data.motor_speed,
           (unsigned long)g_measurement.debug_data.motor_state);

    if (update_command_state) {
        g_measurement.device_status.device_state = STATE_READPARAMETEROVER;
    }

    return NO_ERROR;
}

uint32_t Sensor_CheckAllPartParams(void)
{
    return Sensor_ReadPartParamsInternal(0U);
}

void CMD_ReadPartParams(void)
{
    uint32_t ret = Sensor_ReadPartParamsInternal(1U);

    SET_ERROR(ret);
}
