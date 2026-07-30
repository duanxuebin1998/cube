/*
 * device_param_sync.c
 *
 *  Created on: 2025年12月12日
 *      Author: Duan Xuebin
 */


#include "device_param_sync.h"
#include "cpu3_debug_log.h"
#include "param_float32.h"
#include <string.h>

/* CPU3 参数元数据表项总数，由参数模块定义并供同步范围检查使用。 */
extern const int param_metaAmount;

/**
 * @brief 返回设备参数小数位对应的十进制缩放因子。
 *
 * @param point 参数元数据声明的小数位数；函数返回 10 的 point 次幂作为菜单值与协议值之间的缩放因子。
 * @return 计算后的业务数值。
 */
static float DeviceParams_DecimalScale(uint8_t point)
{
    float scale = 1.0f;
    for (uint8_t i = 0U; i < point; i++) {
        scale *= 10.0f;
    }
    return scale;
}

/**
 * @brief 把菜单元数据值还原为 CPU2 协议原始值；浮点参数按位复制 IEEE-754 表示。
 *
 * @param h 参数元数据。
 * @param meta_value 待转换的菜单显示值。
 * @return 返回菜单元数据值对应的 CPU2 32 位原始值；浮点字段保留 IEEE 754 位模式，整数移除显示偏移。
 */
static uint32_t DeviceParams_MetaValueToRaw(volatile struct ParameterMetadata *h,
                                            int32_t meta_value)
{
    if ((h != NULL) && (h->data_type == TYPE_FLOAT)) {
        float value = ((float)meta_value) / DeviceParams_DecimalScale(h->point);
        uint32_t raw;
        /* 把缩放后的 float 按 IEEE-754 Float32 位模式复制到协议原始值；memcpy 用于位级转换，避免违反严格别名规则。 */
        memcpy(&raw, &value, sizeof(raw));
        return raw;
    }
    if (h == NULL) {
        return 0U;
    }
    /* param_meta.val 保存菜单显示值，写回 CPU2 前恢复为协议原始值。 */
    return (uint32_t)(meta_value - (int32_t)h->offset);
}

/**
 * @brief 将 CPU2 参数原始值转换为菜单显示值；浮点字段按 point 安全缩放，整数按照元数据 offset 补回显示偏移，参数无效或浮点转换失败时返回 false。
 *
 * @param h 目标参数的 ParameterMetadata 元数据项。
 * @param raw CPU2 参数快照中的 32 位原始值；浮点字段按 IEEE 754 解释，整数按元数据 offset 和小数位还原。
 * @param meta_value 用于接收按参数元数据类型解释后的 32 位值。
 * @return true 表示原始值已按元数据类型转换并写入 meta_value；h 或 meta_value 为空，或 Float32 字段无法按 point 安全缩放时返回 false。
 */
static bool DeviceParams_RawToMetaValue(volatile struct ParameterMetadata *h,
                                        uint32_t raw,
                                        int32_t *meta_value)
{
    if ((h == NULL) || (meta_value == NULL)) {
        return false;
    }
    if (h->data_type == TYPE_FLOAT) {
        return ParamFloat32_TryRawToScaledInt(raw, h->point, meta_value);
    }
    /* CPU2 下发的是协议原始值，缓存到菜单元数据时补回显示偏移。 */
    *meta_value = (int32_t)raw + (int32_t)h->offset;
    return true;
}

/* ==================== 内部：operanum → g_deviceParams 字段映射 ==================== */
/**
 * @brief 根据 operanum(COM_NUM_xxx) 找到 DeviceParameters 中对应字段的指针（适配新寄存器/新操作码）。
 *
 * @param operanum 参数操作号，用于定位 ParameterMetadata 项或同步策略。
 * @return 成功时返回 DeviceParameters 中与操作号对应字段的可写指针；操作号没有字段映射时返回 NULL。
 */
static volatile uint32_t* get_deviceparam_ptr_by_operanum(int operanum)
{
    switch (operanum) {

    /* ===== 指令 ===== */
    case COM_NUM_DEVICEPARAM_COMMAND:
        return (volatile uint32_t*)&g_deviceParams.command;

    /* ===== 基础参数（与新 HOLDREGISTER 对齐的字段） ===== */
    case COM_NUM_DEVICEPARAM_SENSORTYPE:
        return &g_deviceParams.sensorType;
    case COM_NUM_DEVICEPARAM_SENSORID:
        return &g_deviceParams.sensorID;
    case COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION:
        return &g_deviceParams.sensorSoftwareVersion;
    case COM_NUM_DEVICEPARAM_SOFTWAREVERSION:
        return &g_deviceParams.softwareVersion;
    case COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND:
        return (volatile uint32_t*)&g_deviceParams.powerOnDefaultCommand;
    case COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO:
        return &g_deviceParams.error_auto_back_zero;
    case COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT:
        return &g_deviceParams.error_stop_measurement;
    case COM_NUM_DEVICEPARAM_PROTOCOL_VERSION:
        return &g_deviceParams.protocolVersion;
    case COM_NUM_DEVICEPARAM_RESERVED2:
        return &g_deviceParams.fault_auto_recovery_retry_limit;
    case COM_NUM_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME:
        return &g_deviceParams.water_level_hysteresis_time_s;
    case COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH:
        return &g_deviceParams.position_source_auto_switch;

    /* ===== 电机与编码器参数 ===== */
    case COM_NUM_DEVICEPARAM_MOTOR_CURRENT:
        return &g_deviceParams.motor_current;
    case COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM:
        return &g_deviceParams.encoder_wheel_circumference_mm;
    case COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED:
        return &g_deviceParams.max_motor_speed;
    case COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM:
        return &g_deviceParams.first_loop_circumference_mm;
    case COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM:
        return &g_deviceParams.tape_thickness_mm;
    case COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE:
        return &g_deviceParams.position_count_mode;
    case COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC:
        return &g_deviceParams.motor_count_first_loop_circumference_mm;

    /* ===== 扭力参数 ===== */
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT:
        return &g_deviceParams.empty_weight_upper_limit;
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT:
        return &g_deviceParams.empty_weight_lower_limit;

    case COM_NUM_DEVICEPARAM_FULL_WEIGHT:
        return &g_deviceParams.full_weight;
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT:
        return &g_deviceParams.full_weight_upper_limit;
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT:
        return &g_deviceParams.full_weight_lower_limit;

    case COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO:
        return &g_deviceParams.weight_upper_limit_ratio;
    case COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO:
        return &g_deviceParams.weight_lower_limit_ratio;

    /* ===== 零点测量（新寄存器段） ===== */
    case COM_NUM_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO:
        return &g_deviceParams.zero_weight_threshold_ratio;
    case COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE:
        return &g_deviceParams.weight_ignore_zone;
    case COM_NUM_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE:
        return &g_deviceParams.max_zero_deviation_distance;
    case COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE:
        return &g_deviceParams.findZeroDownDistance;

    /* ===== 液位测量（新寄存器段） ===== */
    case COM_NUM_DEVICEPARAM_TANKHEIGHT:
        return &g_deviceParams.tankHeight;
    case COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF:
        return &g_deviceParams.liquid_sensor_distance_diff;
    case COM_NUM_DEVICEPARAM_BLINDZONE:
        return &g_deviceParams.blindZone;
    case COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD:
        return &g_deviceParams.oilLevelThreshold;
    case COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD:
        return &g_deviceParams.oilLevelHysteresisThreshold;
    case COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD:
        return &g_deviceParams.liquidLevelMeasurementMethod;
    case COM_NUM_DEVICEPARAM_OILLEVEL_FREQUENCY:
        return &g_deviceParams.oilLevelFrequency;
    case COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY:
        return &g_deviceParams.oilLevelDensity;

    /* ===== 水位测量参数（新寄存器段） ===== */
    case COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT:
        return &g_deviceParams.water_tank_height;
    case COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE:
        return &g_deviceParams.water_level_mode;
    case COM_NUM_DEVICEPARAM_WATER_BLINDZONE:
        return &g_deviceParams.waterBlindZone;
    case COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD:
        return &g_deviceParams.water_cap_threshold;
    case COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD:
        return &g_deviceParams.water_find_cap_threshold;
    case COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE:
        return &g_deviceParams.maxDownDistance;
    case COM_NUM_DEVICEPARAM_ZERO_CAP:
    	return &g_deviceParams.zero_cap;
    case COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD:
       	return &g_deviceParams.water_stable_threshold;
    case COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD:
        return &g_deviceParams.water_lag_cap_threshold;
    /* ===== 罐高/罐底测量（新寄存器段） ===== */
    case COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE:
        return &g_deviceParams.bottom_detect_mode;
    case COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD:
        return &g_deviceParams.bottom_angle_threshold;
    case COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD:
        return &g_deviceParams.bottom_weight_threshold;

    case COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG:
        return &g_deviceParams.refreshTankHeightFlag;
    case COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION:
        return &g_deviceParams.maxTankHeightDeviation;
    case COM_NUM_DEVICEPARAM_INITIAL_TANKHEIGHT:
        return &g_deviceParams.initialTankHeight;
    case COM_NUM_DEVICEPARAM_CURRENT_TANKHEIGHT:
        return &g_deviceParams.currentTankHeight;
    case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE:
        return &g_deviceParams.bottom_encoder_correction_enable;

    /* ===== 密度与温度修正 ===== */
    case COM_NUM_DEVICEPARAM_DENSITYCORRECTION:
        return &g_deviceParams.densityCorrection;
    case COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION:
        return &g_deviceParams.temperatureCorrection;

    /* ===== 分布/区间测量参数（新寄存器段） ===== */
    case COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT:
        return &g_deviceParams.requireBottomMeasurement;
    case COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT:
        return &g_deviceParams.requireWaterMeasurement;
    case COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY:
        return &g_deviceParams.requireSinglePointDensity;

    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER:
        return &g_deviceParams.spreadMeasurementOrder;
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE:
        return &g_deviceParams.spreadMeasurementMode;
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT:
        return &g_deviceParams.spreadMeasurementCount;
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE:
        return &g_deviceParams.spreadMeasurementDistance;
    case COM_NUM_DEVICEPARAM_SPREADTOPLIMIT:
        return &g_deviceParams.spreadTopLimit;
    case COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT:
        return &g_deviceParams.spreadBottomLimit;
    case COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME:
        return &g_deviceParams.spreadPointHoverTime;

    case COM_NUM_DEVICEPARAM_INTERVAL_TOPLIMIT:
        return &g_deviceParams.intervalMeasurementTopLimit;
    case COM_NUM_DEVICEPARAM_INTERVAL_BOTTOMLIMIT:
        return &g_deviceParams.intervalMeasurementBottomLimit;

    /* ===== Wartsila 密度区间测量参数 ===== */
    case COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT:
        return &g_deviceParams.wartsila_upper_density_limit;
    case COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT:
        return &g_deviceParams.wartsila_lower_density_limit;
    case COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL:
        return &g_deviceParams.wartsila_density_interval;
    case COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE:
        return &g_deviceParams.wartsila_max_height_above_surface;
    case COM_NUM_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL:
        return &g_deviceParams.wartsila_bottom_detect_interval;
    case COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT:
        return &g_deviceParams.bottom_encoder_correction_tank_height;

    /* ===== 继电器报警输出配置（四路） ===== */
    case COM_NUM_DEVICEPARAM_RELAY1_OPERATING_MODE:
        return &g_deviceParams.relayAlarm[0U].operating_mode;
    case COM_NUM_DEVICEPARAM_RELAY1_DIGITAL_SOURCE:
        return &g_deviceParams.relayAlarm[0U].digital_source;
    case COM_NUM_DEVICEPARAM_RELAY1_CONTACT_TYPE:
        return &g_deviceParams.relayAlarm[0U].contact_type;
    case COM_NUM_DEVICEPARAM_RELAY1_ALARM_MODE:
        return &g_deviceParams.relayAlarm[0U].alarm_mode;
    case COM_NUM_DEVICEPARAM_RELAY1_ERROR_VALUE:
        return &g_deviceParams.relayAlarm[0U].error_value;
    case COM_NUM_DEVICEPARAM_RELAY1_ALARM_SOURCE:
        return &g_deviceParams.relayAlarm[0U].alarm_source;
    case COM_NUM_DEVICEPARAM_RELAY1_HH_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[0U].HH_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY1_H_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[0U].H_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY1_L_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[0U].L_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY1_LL_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[0U].LL_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY1_ALARM_HYSTERESIS:
        return &g_deviceParams.relayAlarm[0U].alarm_hysteresis;
    case COM_NUM_DEVICEPARAM_RELAY1_DAMPING_FACTOR:
        return &g_deviceParams.relayAlarm[0U].damping_factor;
    case COM_NUM_DEVICEPARAM_RELAY1_CLEAR_ALARM:
        return &g_deviceParams.relayAlarm[0U].clear_alarm;
    case COM_NUM_DEVICEPARAM_RELAY2_OPERATING_MODE:
        return &g_deviceParams.relayAlarm[1U].operating_mode;
    case COM_NUM_DEVICEPARAM_RELAY2_DIGITAL_SOURCE:
        return &g_deviceParams.relayAlarm[1U].digital_source;
    case COM_NUM_DEVICEPARAM_RELAY2_CONTACT_TYPE:
        return &g_deviceParams.relayAlarm[1U].contact_type;
    case COM_NUM_DEVICEPARAM_RELAY2_ALARM_MODE:
        return &g_deviceParams.relayAlarm[1U].alarm_mode;
    case COM_NUM_DEVICEPARAM_RELAY2_ERROR_VALUE:
        return &g_deviceParams.relayAlarm[1U].error_value;
    case COM_NUM_DEVICEPARAM_RELAY2_ALARM_SOURCE:
        return &g_deviceParams.relayAlarm[1U].alarm_source;
    case COM_NUM_DEVICEPARAM_RELAY2_HH_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[1U].HH_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY2_H_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[1U].H_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY2_L_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[1U].L_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY2_LL_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[1U].LL_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY2_ALARM_HYSTERESIS:
        return &g_deviceParams.relayAlarm[1U].alarm_hysteresis;
    case COM_NUM_DEVICEPARAM_RELAY2_DAMPING_FACTOR:
        return &g_deviceParams.relayAlarm[1U].damping_factor;
    case COM_NUM_DEVICEPARAM_RELAY2_CLEAR_ALARM:
        return &g_deviceParams.relayAlarm[1U].clear_alarm;
    case COM_NUM_DEVICEPARAM_RELAY3_OPERATING_MODE:
        return &g_deviceParams.relayAlarm[2U].operating_mode;
    case COM_NUM_DEVICEPARAM_RELAY3_DIGITAL_SOURCE:
        return &g_deviceParams.relayAlarm[2U].digital_source;
    case COM_NUM_DEVICEPARAM_RELAY3_CONTACT_TYPE:
        return &g_deviceParams.relayAlarm[2U].contact_type;
    case COM_NUM_DEVICEPARAM_RELAY3_ALARM_MODE:
        return &g_deviceParams.relayAlarm[2U].alarm_mode;
    case COM_NUM_DEVICEPARAM_RELAY3_ERROR_VALUE:
        return &g_deviceParams.relayAlarm[2U].error_value;
    case COM_NUM_DEVICEPARAM_RELAY3_ALARM_SOURCE:
        return &g_deviceParams.relayAlarm[2U].alarm_source;
    case COM_NUM_DEVICEPARAM_RELAY3_HH_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[2U].HH_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY3_H_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[2U].H_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY3_L_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[2U].L_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY3_LL_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[2U].LL_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY3_ALARM_HYSTERESIS:
        return &g_deviceParams.relayAlarm[2U].alarm_hysteresis;
    case COM_NUM_DEVICEPARAM_RELAY3_DAMPING_FACTOR:
        return &g_deviceParams.relayAlarm[2U].damping_factor;
    case COM_NUM_DEVICEPARAM_RELAY3_CLEAR_ALARM:
        return &g_deviceParams.relayAlarm[2U].clear_alarm;
    case COM_NUM_DEVICEPARAM_RELAY4_OPERATING_MODE:
        return &g_deviceParams.relayAlarm[3U].operating_mode;
    case COM_NUM_DEVICEPARAM_RELAY4_DIGITAL_SOURCE:
        return &g_deviceParams.relayAlarm[3U].digital_source;
    case COM_NUM_DEVICEPARAM_RELAY4_CONTACT_TYPE:
        return &g_deviceParams.relayAlarm[3U].contact_type;
    case COM_NUM_DEVICEPARAM_RELAY4_ALARM_MODE:
        return &g_deviceParams.relayAlarm[3U].alarm_mode;
    case COM_NUM_DEVICEPARAM_RELAY4_ERROR_VALUE:
        return &g_deviceParams.relayAlarm[3U].error_value;
    case COM_NUM_DEVICEPARAM_RELAY4_ALARM_SOURCE:
        return &g_deviceParams.relayAlarm[3U].alarm_source;
    case COM_NUM_DEVICEPARAM_RELAY4_HH_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[3U].HH_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY4_H_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[3U].H_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY4_L_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[3U].L_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY4_LL_ALARM_VALUE:
        return &g_deviceParams.relayAlarm[3U].LL_alarm_value;
    case COM_NUM_DEVICEPARAM_RELAY4_ALARM_HYSTERESIS:
        return &g_deviceParams.relayAlarm[3U].alarm_hysteresis;
    case COM_NUM_DEVICEPARAM_RELAY4_DAMPING_FACTOR:
        return &g_deviceParams.relayAlarm[3U].damping_factor;
    case COM_NUM_DEVICEPARAM_RELAY4_CLEAR_ALARM:
        return &g_deviceParams.relayAlarm[3U].clear_alarm;

    /* ===== 协议20 AO配置 ===== */
    case COM_NUM_DEVICEPARAM_AO_WORK_MODE:
        return &g_deviceParams.ao_output.work_mode;
    case COM_NUM_DEVICEPARAM_AO_CURRENT_MODE:
        return &g_deviceParams.ao_output.current_mode;
    case COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE:
        return &g_deviceParams.ao_output.output_source;
    case COM_NUM_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X100:
        return (volatile uint32_t *)&g_deviceParams.ao_output.current_correction_mA_x100;
    case COM_NUM_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100:
        return &g_deviceParams.ao_output.fixed_current_mA_x100;
    case COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM:
        return (volatile uint32_t *)&g_deviceParams.ao_output.range_0_01mm;
    case COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM:
        return (volatile uint32_t *)&g_deviceParams.ao_output.range_100_01mm;
    case COM_NUM_DEVICEPARAM_AO_DAMPING_X10_S:
        return &g_deviceParams.ao_output.damping_x10_s;
    case COM_NUM_DEVICEPARAM_AO_FAULT_MODE:
        return &g_deviceParams.ao_output.fault_mode;
    case COM_NUM_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100:
        return &g_deviceParams.ao_output.fault_current_mA_x100;
    case COM_NUM_DEVICEPARAM_AO_ERROR_LEVEL:
        return &g_deviceParams.ao_output.error_level;
    case COM_NUM_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100:
        return &g_deviceParams.ao_output.power_on_current_mA_x100;
    case COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100:
        return &g_deviceParams.ao_output.simulation_current_mA_x100;

    /* ===== 指令参数（新寄存器段：注意“操作码”与“参数项”分开） ===== */
    case COM_NUM_DEVICEPARAM_CALIBRATE_OIL_LEVEL:
        return &g_deviceParams.calibrateOilLevel;
    case COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL:
        return &g_deviceParams.calibrateWaterLevel;
    case COM_NUM_DEVICEPARAM_CALIBRATE_TANK_HEIGHT:
        return &g_deviceParams.calibrateTankHeight;
    case COM_NUM_DEVICEPARAM_SP_MEAS_POSITION:
        return &g_deviceParams.singlePointMeasurementPosition;
    case COM_NUM_DEVICEPARAM_SP_MONITOR_POSITION:
        return &g_deviceParams.singlePointMonitoringPosition;
    case COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL:
        return &g_deviceParams.densityDistributionOilLevel;
    case COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE:
        return &g_deviceParams.motorCommandDistance;
    case COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME:
        return &g_deviceParams.oilLevelHysteresisTime;
    case COM_NUM_DEVICEPARAM_WATER_LEVEL_CORRECTION:
        /* 当前项目约定：水位修正与水位标定共用 calibrateWaterLevel。 */
        return &g_deviceParams.calibrateWaterLevel;
    case COM_NUM_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL:
        return &g_deviceParams.lastOilCorrectionLevel;
    case COM_NUM_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE:
        return &g_deviceParams.tankGasPhaseTemperature;
    case COM_NUM_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT:
        return &g_deviceParams.tapeExpansionCoefficient;
    case COM_NUM_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE:
        return &g_deviceParams.tapeCalibrationTemperature;
    case COM_NUM_DEVICEPARAM_SI_PROFILE_FIRST_POINT:
        return &g_deviceParams.si_profile_first_point;
    case COM_NUM_DEVICEPARAM_SI_PROFILE_INCREMENT:
        return &g_deviceParams.si_profile_increment;
    case COM_NUM_DEVICEPARAM_SI_PROFILE_DWELL_TIME:
        return &g_deviceParams.si_profile_dwell_time;
    case COM_NUM_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL:
        return &g_deviceParams.si_profile_bottom_detect_interval;

    /* 兼容：旧调试指令仍然可能直接用这些 operanum 取指针 */
    case COM_NUM_CAL_OIL:
    case COM_NUM_CORRECTION_OIL:
        return &g_deviceParams.calibrateOilLevel;
    case COM_NUM_CALIBRATE_WATER:
        return &g_deviceParams.calibrateWaterLevel;
    case COM_NUM_CALIBRATE_TANKHEIGHT:
        return &g_deviceParams.calibrateTankHeight;
    case COM_NUM_SINGLE_POINT:
        return &g_deviceParams.singlePointMeasurementPosition;
    case COM_NUM_SP_TEST:
        return &g_deviceParams.singlePointMonitoringPosition;

    /* ===== 元信息与 CRC ===== */
    case COM_NUM_DEVICEPARAM_PARAM_VERSION:
        return &g_deviceParams.param_version;
    case COM_NUM_DEVICEPARAM_STRUCT_SIZE:
        return &g_deviceParams.struct_size;
    case COM_NUM_DEVICEPARAM_MAGIC:
        return &g_deviceParams.magic;
    case COM_NUM_DEVICEPARAM_CRC:
        return &g_deviceParams.crc;

    default:
        return NULL;
    }
}

/**
 * @brief 取得一个元数据项准备同步到 CPU2 的协议原始值。
 *
 * @details 调用场景：DSM 批量同步预检和实际逐项同步共用。
 * @note 关键约束：只读项和不属于 DeviceParameters 的项返回“无需参与”，转换失败不得开始批量写入。
 *
 * @param h 目标参数的 ParameterMetadata 元数据项。
 * @param target_value 准备同步到 CPU2 的参数协议原始值。
 * @param participates 用于返回该参数是否参加本轮 CPU2 批量同步。
 * @return true 表示元数据项可同步，且当前 DeviceParameters 值已按协议原始格式写入输出；false 表示元数据/输出指针无效、字段不支持批量同步，或值无法按声明类型编码。
 */
static bool DeviceParams_TryGetSyncValue(volatile struct ParameterMetadata *h,
                                         int32_t *target_value,
                                         bool *participates)
{
    volatile uint32_t *p_dev;

    if ((h == NULL) || (target_value == NULL) || (participates == NULL)) {
        return false;
    }
    *participates = false;
    if (!h->authority_write) {
        return true;
    }

    if (h->operanum == COM_NUM_DEVICEPARAM_EMPTY_WEIGHT) {
        *target_value = g_deviceParams.empty_weight;
    } else {
        p_dev = get_deviceparam_ptr_by_operanum(h->operanum);
        if (p_dev == NULL) {
            return true;
        }
        if (!DeviceParams_RawToMetaValue(h, *p_dev, target_value)) {
            CPU3_LOG_WARNING("CPU2参数",
                             "参数值无法安全转换 名称=%s 原始值=0x%08lX",
                             h->name ? (char*)h->name : "noname",
                             (unsigned long)*p_dev);
            return false;
        }
    }
    *participates = true;
    return true;
}


/**
 * @brief 将一个双寄存器参数按其线格式下发 CPU2，并以合法应答判定成功。
 *
 * @param h 目标参数的 ParameterMetadata 元数据项。
 * @param target_value 准备同步到 CPU2 的参数协议原始值。
 * @return true 表示参数已获得 CPU2 合法写应答；元数据为空、寄存器宽度不支持或通信失败时返回 false。
 */

static bool DeviceParams_SendHoldValueToCPU2(volatile struct ParameterMetadata *h,
                                             int32_t target_value)
{
    uint16_t wire_regs[REG_STRIDE];
    uint32_t u32_temp;

    if (h == NULL) return false;

    /* 每个共享参数占两个寄存器，单寄存器字段不在本同步入口处理。 */
    if (h->rgstcnt != REG_STRIDE) {
        /* 如果以后有 1 寄存器参数，再单独处理 */
        CPU3_LOG_WARNING("CPU2参数",
                         "参数暂不支持同步 名称=%s 寄存器数=%u",
                         h->name ? (char*)h->name : "noname",
                         h->rgstcnt);
        return false;
    }

    /* TYPE_FLOAT 菜单值是显示缩放后的整数，下发前恢复为 IEEE754 原始位。 */
    u32_temp = DeviceParams_MetaValueToRaw(h, target_value);
    if (LtdModbus_HoldingWriteIsCommandArgumentOnly(h->startadd, h->rgstcnt)) {
        wire_regs[0] = (uint16_t)(u32_temp >> 16);
        wire_regs[1] = (uint16_t)(u32_temp & 0xFFFFU);
        return CPU2_CommWriteHoldingRegistersEx(h->startadd,
                                               h->rgstcnt,
                                               wire_regs) ==
               CPU2_MODBUS_RESULT_OK;
    }

    /* 普通字段保留原批量 ACK 路径，避免首项成功后使后续合法字段被本地快照门禁截断。 */
    return CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                                      h->startadd,
                                      h->rgstcnt,
                                      &u32_temp);
}

/**
 * @brief 比较一个参数的 CPU2 快照值与 CPU3 目标值，并在有差异时完成下发确认。
 *
 * @param h 目标参数的 ParameterMetadata 元数据项。
 * @return true 表示参数无需同步或新值已由 CPU2 确认；元数据解析或下发失败时返回 false。
 */

static bool DeviceParams_SyncOneHold(volatile struct ParameterMetadata *h)
{
    int32_t dev_val;
    bool participates;

    if (!DeviceParams_TryGetSyncValue(h, &dev_val, &participates)) {
        return false;
    }
    if (!participates) {
        return true;
    }

    if (h->val == dev_val) {
        /* 与 CPU2 当前值一致，不需要更新 */
        return true;
    }

    CPU3_LOG_INFO("CPU2参数",
                  "发现参数差异并准备下发 名称=%s CPU2=%d 本地=%ld",
                  h->name ? (char*)h->name : "noname",
                  h->val,
                  (long)dev_val);

    /* 先确认 CPU2 已接受新值，避免失败后本地缓存伪装成同步成功。 */
    if (!DeviceParams_SendHoldValueToCPU2(h, dev_val)) {
        return false;
    }

    h->val = dev_val;
    return true;
}

/**
 * @brief 批量同步通常由外部协议写入旧寄存器后触发。
 *
 * 命令只允许走一次性命令入口，禁止随参数批量同步重放。
 * 位置源模式和电机局部周长可能由 CPU2 在 YM 切换/标定流程中自动更新，如果 CPU3 本地缓存尚未补读完成，批量同步会把旧值覆盖回 CPU2。
 * 因此这些字段不参与批量同步；菜单单项读写仍然直接走对应保持寄存器。
 *
 * @param operanum 参数操作号，用于定位 ParameterMetadata 项或同步策略。
 * @return true 表示操作号是命令、位置计数模式或电机首圈计数，必须跳过通用批量同步；false 表示该字段可继续按元数据差异集判断是否同步。
 */
static bool DeviceParams_ShouldSkipBulkSync(int operanum)
{
    return (operanum == COM_NUM_DEVICEPARAM_COMMAND) ||
           (operanum == COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE) ||
           (operanum == COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC);
}

/**
 * @brief 在 DSM 批量同步发送首帧前检查完整差异集和状态门禁。
 *
 * @details 调用场景：DeviceParams_SyncAllToCPU2 每次批量同步开始时。
 * @note 关键约束：存在普通持久参数差异时按原状态白名单整体拒绝；只有命令前置参数差异时允许运行态写入。
 *
 * @return true 表示没有参数差异，或差异集均可编码、CPU2 通信可用，且普通持久参数差异满足当前设备状态写入门禁；false 表示任一元数据值无法编码、CPU2 不可用，或普通持久参数在当前运行状态禁止写入。
 */
static bool DeviceParams_BulkSyncPreflight(void)
{
    bool has_difference = false;
    bool has_ordinary_difference = false;

    for (uint32_t i = 0U; i < (uint32_t)param_metaAmount; ++i) {
        volatile struct ParameterMetadata *h = &param_meta[i];
        int32_t target_value;
        bool participates;

        if (DeviceParams_ShouldSkipBulkSync(h->operanum)) {
            continue;
        }
        if (!DeviceParams_TryGetSyncValue(h, &target_value, &participates)) {
            return false;
        }
        if (!participates || (h->val == target_value)) {
            continue;
        }
        has_difference = true;
        if (!LtdModbus_HoldingWriteIsCommandArgumentOnly(h->startadd, h->rgstcnt)) {
            has_ordinary_difference = true;
        }
    }

    if (!has_difference) {
        return true;
    }
    if (!CPU2_CommIsAvailable()) {
        return false;
    }
    if (has_ordinary_difference &&
        !DeviceState_AllowsPersistentParamWrite(
            g_measurement.device_status.device_state)) {
        return false;
    }
    return true;
}

/* ==================== 对外接口 ==================== */

/**
 * @brief 同步所有 DeviceParameters → CPU2。
 *
 * @return true 表示无需同步或全部差异参数已逐项写入 CPU2 并通过事务确认；false 表示预检失败，或任一参数的板间 Modbus 写入失败。
 */
bool DeviceParams_SyncAllToCPU2(void)
{
    if (!DeviceParams_BulkSyncPreflight()) {
        return false;
    }

    for (uint32_t i = 0U; i < (uint32_t)param_metaAmount; ++i) {
        if (DeviceParams_ShouldSkipBulkSync(param_meta[i].operanum)) {
            continue;
        }
        /* 需要判定一下是否是CPU2的可写参数 */
        if (!DeviceParams_SyncOneHold(&param_meta[i])) {
            return false;
        }
    }
    return true;
}
