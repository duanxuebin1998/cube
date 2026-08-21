/**
 * @file water_level_internal.h
 * @brief 水位测量目录私有的状态判定、位置换算和结果发布接口。
 *
 * 本头文件只允许 water_level 目录内的搜索、跟随、标定和剖面测试实现使用。
 * Application 其它模块必须通过 measure_water_level.h 调用公开流程，不能依赖这里的内部状态。
 */

#ifndef APPLICATION_MEASUREMENT_WATER_LEVEL_INTERNAL_H
#define APPLICATION_MEASUREMENT_WATER_LEVEL_INTERNAL_H

#include "measure_water_level.h"

#include <stdint.h>

/* 状态翻转定位和丢失恢复使用的默认稳定窗口，单位 ms。 */
#define WATER_STABLE_WINDOW_DEFAULT_MS (30000U)

/** 水位探头相对水层的二态判定结果。 */
typedef enum {
    WATER_LEVEL_STATE_NORMAL = 0,  /* 探头未检测到水，允许继续下行搜索。 */
    WATER_LEVEL_STATE_DETECTED = 1 /* 探头已经检测到水，应停止或转入边界定位。 */
} WaterLevelState;

/**
 * @brief 把设备参数中的千分之一电容定点值换算为浮点电容值。
 * @param raw 电容定点值，缩放比例为 1000。
 * @return 换算后的电容值。
 */
float WaterLevel_CapRawToFloat(uint32_t raw);

/**
 * @brief 根据水位罐高和当前尺带长度计算有符号水位。
 * @return 当前水位，单位 0.1 mm；溢出时饱和到 int32_t 边界。
 */
int32_t WaterLevel_CalcFromCable(void);

/**
 * @brief 根据目标水位反算目标尺带长度。
 * @param level_target_01mm 目标水位，单位 0.1 mm。
 * @return 目标尺带长度，单位 0.1 mm；溢出时饱和到 int32_t 边界。
 */
int32_t WaterLevel_CableTargetFromLevel(int32_t level_target_01mm);

/**
 * @brief 把有符号水位限制为协议可发布的非负值。
 * @param level_01mm 有符号水位，单位 0.1 mm。
 * @return 可发布水位，单位 0.1 mm。
 */
uint32_t WaterLevel_ClampForReport(int32_t level_01mm);

/**
 * @brief 更新内部搜索参考、水位测量结果和 AO 过程样本。
 * @param level_01mm 当前水位，单位 0.1 mm。
 */
void WaterLevel_SetAndLog(int32_t level_01mm);

/** 根据当前尺带长度重新计算并发布水位结果。 */
void WaterLevel_SyncFromCable(void);

/**
 * @brief 保存慢速搜索最近一次确定的水位参考。
 * @param level_01mm 搜索参考水位，单位 0.1 mm。
 */
void WaterLevel_SetSearchReference01mm(int32_t level_01mm);

/**
 * @brief 读取慢速搜索最近一次确定的水位参考。
 * @return 搜索参考水位，单位 0.1 mm；尚未建立时返回内部无效初值。
 */
int32_t WaterLevel_GetSearchReference01mm(void);

/**
 * @brief 读取水位电容并判断探头是否接触水层。
 * @param water_state 输出水位二态判定结果。
 * @return NO_ERROR 表示判定完成；参数或传感器异常时返回原错误码。
 */
uint32_t WaterLevel_CheckStatus(WaterLevelState *water_state);

#endif /* APPLICATION_MEASUREMENT_WATER_LEVEL_INTERNAL_H */
