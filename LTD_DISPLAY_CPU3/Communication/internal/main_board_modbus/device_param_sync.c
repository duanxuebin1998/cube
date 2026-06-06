/*
 * device_param_sync.c
 *
 *  Created on: 2025年12月12日
 *      Author: Duan Xuebin
 */


#include "device_param_sync.h"
#include <stdio.h>
#include <string.h>

extern const int param_metaAmount;

static float DeviceParams_DecimalScale(uint8_t point)
{
    float scale = 1.0f;
    for (uint8_t i = 0U; i < point; i++) {
        scale *= 10.0f;
    }
    return scale;
}

static uint32_t DeviceParams_MetaValueToRaw(volatile struct ParameterMetadata *h)
{
    if ((h != NULL) && (h->data_type == TYPE_FLOAT)) {
        float value = ((float)h->val) / DeviceParams_DecimalScale(h->point);
        uint32_t raw;
        memcpy(&raw, &value, sizeof(raw));
        return raw;
    }
    return (h != NULL) ? (uint32_t)((int32_t)h->val) : 0U;
}

static int32_t DeviceParams_RawToMetaValue(volatile struct ParameterMetadata *h, uint32_t raw)
{
    if ((h != NULL) && (h->data_type == TYPE_FLOAT)) {
        float value;
        memcpy(&value, &raw, sizeof(value));
        value *= DeviceParams_DecimalScale(h->point);
        return (int32_t)value;
    }
    return (int32_t)raw;
}

/* ==================== 内部：operanum → g_deviceParams 字段映射 ==================== */
/* 根据 operanum(COM_NUM_xxx) 找到 DeviceParameters 中对应字段的指针（适配新寄存器/新操作码） */
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

    /* ===== 称重参数 ===== */
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

    /* ===== 继电器方式2报警配置（四路） ===== */
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

    /* ===== 4–20mA / 报警 AO ===== */
    case COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA:
        return &g_deviceParams.CurrentRangeStart_mA;
    case COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA:
        return &g_deviceParams.CurrentRangeEnd_mA;
    case COM_NUM_DEVICEPARAM_ALARM_HIGH_AO:
        return &g_deviceParams.AlarmHighAO;
    case COM_NUM_DEVICEPARAM_ALARM_LOW_AO:
        return &g_deviceParams.AlarmLowAO;
    case COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA:
        return &g_deviceParams.InitialCurrent_mA;
    case COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA:
        return &g_deviceParams.AOHighCurrent_mA;
    case COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA:
        return &g_deviceParams.AOLowCurrent_mA;
    case COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA:
        return &g_deviceParams.FaultCurrent_mA;
    case COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA:
        return &g_deviceParams.DebugCurrent_mA;

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


/* ==================== 内部：把 param_meta[i].val 下发到 CPU2（10 功能码） ==================== */

static void DeviceParams_SendHoldValueToCPU2(volatile struct ParameterMetadata *h)
{
    if (h == NULL) return;

    if (!h->authority_write) {
        // 屏幕不可写的参数通常不需要同步到 CPU2
        return;
    }

    // CPU2_CombinatePackage_Send 是按 32bit + word swap 来发的，
    // 每个参数占两个寄存器，因此这里只支持 rgstcnt == 2 的情况。
    if (h->rgstcnt != 2) {
        // 如果以后有 1 寄存器参数，再单独处理
        printf("设备参数警告: %s 寄存器数=%u 暂不支持同步\n",
               h->name ? (char*)h->name : "noname", h->rgstcnt);
        return;
    }

    // TYPE_FLOAT 菜单值是显示缩放后的整数，下发前恢复为 IEEE754 原始位。
    uint32_t u32_temp = DeviceParams_MetaValueToRaw(h);

    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                               h->startadd,
                               h->rgstcnt,
                               &u32_temp);
}

/* ==================== 内部：同步一个 ParameterMetadata 项 ==================== */

static void DeviceParams_SyncOneHold(volatile struct ParameterMetadata *h)
{
    if (h == NULL) return;

    // g_deviceParams 中的源值
    int32_t dev_val;
    if (h->operanum == COM_NUM_DEVICEPARAM_EMPTY_WEIGHT) {
        dev_val = g_deviceParams.empty_weight;
    } else {
        volatile uint32_t *p_dev = get_deviceparam_ptr_by_operanum(h->operanum);
        if (p_dev == NULL) {
            // 不属于 DeviceParameters 的项（例如测量结果），跳过
            return;
        }
        dev_val = DeviceParams_RawToMetaValue(h, *p_dev);
    }

    if (h->val == dev_val) {
        // 与 CPU2 当前值一致，不需要更新
        return;
    }

    printf("设备参数差异: %s, CPU2=%d, 本地=%ld -> 更新并发送\r\n",
           h->name ? (char*)h->name : "noname",
           h->val, dev_val);

    // 1) 把 g_deviceParams 的值写回 param_meta[i].val
    h->val = dev_val;

    // 2) 按寄存器信息下发 10 指令给 CPU2
    DeviceParams_SendHoldValueToCPU2(h);
}

/* 批量同步通常由外部协议写入旧寄存器后触发。
 * 位置源模式和电机局部周长可能由 CPU2 在 YM 切换/标定流程中自动更新，
 * 如果 CPU3 本地缓存尚未补读完成，批量同步会把旧值覆盖回 CPU2。
 * 因此这两个字段不参与批量同步；菜单单项读写仍然直接走对应保持寄存器。 */
static bool DeviceParams_ShouldSkipBulkSync(int operanum)
{
    return (operanum == COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE) ||
           (operanum == COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC);
}

/* ==================== 对外接口 ==================== */

/* 同步所有 DeviceParameters → CPU2 */
void DeviceParams_SyncAllToCPU2(void)
{
    for (uint32_t i = 0; i < param_metaAmount; ++i) {
        if (DeviceParams_ShouldSkipBulkSync(param_meta[i].operanum)) {
            continue;
        }
        //需要判定一下是否是CPU2的可写参数
        DeviceParams_SyncOneHold(&param_meta[i]);
    }
}
