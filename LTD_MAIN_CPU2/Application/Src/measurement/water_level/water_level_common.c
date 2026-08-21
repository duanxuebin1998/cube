/**
 * @file water_level_common.c
 * @brief 水位搜索、跟随和标定共用的位置换算、状态判定与结果发布实现。
 */
#include "water_level_internal.h"
#include "fault_manager.h"
#include "system_parameter.h"
#include "motor_ctrl.h"
#include "measure_zero.h"
#include "sensor_service.h"
#include "error_log.h"
#include "AoOutput/ao_output.h"

#include <stdio.h>
#include <stdlib.h>
/* 慢速粗找和精找共享的最近水位参考，单位 0.1 mm；极小初值表示尚未建立。 */
static int32_t s_water_search_reference_01mm = -100000000;

/**
 * @brief 保存慢速搜索最近一次确定的水位参考。
 * @param level_01mm 搜索参考水位，单位 0.1 mm。
 */
void WaterLevel_SetSearchReference01mm(int32_t level_01mm)
{
    s_water_search_reference_01mm = level_01mm;
}

/**
 * @brief 读取慢速搜索最近一次确定的水位参考。
 * @return 搜索参考水位，单位 0.1 mm；尚未建立时返回内部无效初值。
 */
int32_t WaterLevel_GetSearchReference01mm(void)
{
    return s_water_search_reference_01mm;
}
/**
 * @brief 把水位电容原始位模式还原为单精度浮点值。
 *
 * @param raw 传感器水位电容的 IEEE 754 Float32 原始 32 位位模式。
 * @return 计算后的业务数值。
 */
float WaterLevel_CapRawToFloat(uint32_t raw)
{
    return raw / 1000.0f;
}

/**
 * @brief 用有符号计算当前水位，避免 water_tank_height 与负尺带长度混算成无符号大数。
 *
 * @return 返回由水位罐高和当前尺带长度计算的有符号水位，单位 0.1 mm；溢出时饱和到 INT32_MIN 或 INT32_MAX。
 */
int32_t WaterLevel_CalcFromCable(void)
{
    int64_t lvl = (int64_t)g_deviceParams.water_tank_height -
                  (int64_t)g_measurement.debug_data.cable_length;

    if (lvl > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (lvl < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)lvl;
}

/**
 * @brief 用有符号计算目标水位对应的尺带长度，供水位跟随移动距离使用。
 *
 * @param lvl_target_01mm 目标水位对应的位置，单位 0.1 mm。
 * @return 返回目标水位对应的有符号尺带长度，单位 0.1 mm；溢出时饱和到 INT32_MIN 或 INT32_MAX。
 */
int32_t WaterLevel_CableTargetFromLevel(int32_t lvl_target_01mm)
{
    int64_t cable_target = (int64_t)g_deviceParams.water_tank_height -
                           (int64_t)lvl_target_01mm;

    if (cable_target > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (cable_target < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)cable_target;
}
/**
 * @brief 把水位结果限制到协议允许上报的有效区间。
 *
 * @param lvl 待限制、保存或打印的水位值，单位 0.1 mm。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
uint32_t WaterLevel_ClampForReport(int32_t lvl)
{
    return (lvl > 0) ? (uint32_t)lvl : 0U;
}

/**
 * @brief 用当前尺带位置更新水位结果并打印定位信息。
 *
 * @param lvl 待限制、保存或打印的水位值，单位 0.1 mm。
 */
void WaterLevel_SetAndLog(int32_t lvl)
{
    uint32_t old_lvl = g_measurement.water_measurement.water_level;
    uint32_t report_lvl = WaterLevel_ClampForReport(lvl);

    g_measurement.water_measurement.water_level = report_lvl;
    WaterLevel_SetSearchReference01mm(lvl);
    AoOutput_PublishProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL, (int32_t)report_lvl, 1U);

    if (lvl < 0) {
        printf("水位更新\t计算水位为负：%ld(0.1mm)，按0上报\r\n", (long)lvl);
    }

    if (old_lvl != report_lvl)
    {
        printf("水位更新\t旧值=%.1fmm 新值=%.1fmm 缆长=%.1fmm\r\n",
               old_lvl / 10.0f,
               report_lvl / 10.0f,
               g_measurement.debug_data.cable_length / 10.0f);
    }
}

/**
 * @brief 根据当前尺带长度同步水位结果和传感器位置。
 */
void WaterLevel_SyncFromCable(void)
{
    int32_t lvl = WaterLevel_CalcFromCable();
    WaterLevel_SetAndLog(lvl);
}
/**
 * @brief 读取水位传感器零点电容并校验结果。
 * @return NO_ERROR 表示零点电容读取成功且数值有效；其他值为水位传感器通信、数据格式或零点电容有效性错误。
 */
uint32_t read_zero_capacitance(void)
{
    uint32_t ret;
    float    cap = 0.0f;

    ret = SensorService_ReadWaterCapacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }
    g_deviceParams.zero_cap = 10*cap;
    g_measurement.water_measurement.zero_capacitance = cap;

    printf("零点电容 = %.1f\r\n", cap);
    /* 参数存储 */
    save_device_params();
    return NO_ERROR;
}
/**
 * @brief 根据当前电容和零点电容判断水位探头是否接触水层。
 * @param water_state 输出水位状态。
 * @return NO_ERROR 表示判断完成，其他值表示参数或传感器异常。
 */
uint32_t WaterLevel_CheckStatus(WaterLevelState *water_state)
{
    uint32_t ret;
    float    cap = 0.0f;
    float    zero;
    float    th;

    if (water_state == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = SensorService_ReadWaterCapacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }

    zero = g_measurement.water_measurement.zero_capacitance;
    th   = zero + WaterLevel_CapRawToFloat(g_deviceParams.water_cap_threshold);

    /* 判定 */
    if (cap > th) {
        *water_state = WATER_LEVEL_STATE_DETECTED;
    } else {
        *water_state = WATER_LEVEL_STATE_NORMAL;
    }

    /* 单行、完整判定信息 */
    printf("[水位检查] 电容=%.1f  零点：%.1f  阈值=%.1f  电容%s阈值  -> %s\r\n",
           cap,
           zero,
           th,
           (cap > th) ? ">" : "<=",
           (*water_state == WATER_LEVEL_STATE_DETECTED) ? "WATER" : "NORMAL");

    g_measurement.water_measurement.current_capacitance = cap;

    return NO_ERROR;
}
