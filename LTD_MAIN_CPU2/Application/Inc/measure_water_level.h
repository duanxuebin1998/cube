/*
 * @FilePath     : \undefinedd:\CUBE\LTD_MAIN_CPU2\Application\Inc\measure_water_level.h
 * @Description  :
 * @Author       : Aubon
 * @Date         : 2025-12-19 15:55:33
 * @LastEditors  : Duan
 * @LastEditTime : 2025-12-26 09:16:22
 * Copyright 2025 Aubon, All Rights Reserved.
 * 2025-12-19 15:55:33
 */
/*
 * measure_water_level.h
 *
 *  Created on: 2025年12月19日
 *      Author: admin
 */

#ifndef INC_MEASURE_WATER_LEVEL_H_
#define INC_MEASURE_WATER_LEVEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* -------------------- 水位状态 -------------------- */
#ifndef NORMAL
#define NORMAL 0
#endif

#ifndef WATER
#define WATER  1
#endif

/* -------------------- 对外变量 -------------------- */
/* water_value 的物理意义以 .c 内实现为准（当前实现为：水位高度/或位置的缓存值） */
extern int32_t water_value;

/* -------------------- 对外接口 -------------------- */
/**
 * @brief 获取零点（空气区）电容值并写入 g_measurement.water_measurement.zero_capacitance
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t read_zero_capacitance(void);

/**
 * @brief 获取油区电容值并写入 g_measurement.water_measurement.oil_capacitance
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t read_oil_capacitance(void);

/**
 * @brief 读取当前电容并判断水位状态
 * @param water_state 输出：NORMAL / WATER
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t check_water_status(uint8_t *water_state);

/**
 * @brief 水位测量（粗找 + 精找）
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t SearchWaterLevel(void);

/**
 * @brief 水位跟随（持续监控电容并控制电机微调）
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t FollowWaterLevel(void);

/**
 * @brief 水位标定：基于当前水位位置 + 真值(g_deviceParams.calibrateWaterLevel) 修正 water_tank_height
 * @note  需要在 .c 中提供实现；若未启用标定，可不实现此函数
 * @return 错误码（NO_ERROR 表示成功）
 */
void CMD_CalibrateWaterLevel(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MEASURE_WATER_LEVEL_H_ */
