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
 * @brief 液面附近快速跟随（按“连续调用 motorMove_upWithSpeed/motorMove_downWithSpeed 直到状态翻转”的结构）
 *
 * 逻辑：
 *  - 若当前在水里(WATER)：持续上行直到变为 NORMAL（提出水面/到油里）
 *  - 若当前不在水里(NORMAL)：持续下行直到变为 WATER（进入水里）
 *  - 如此循环往复，实现“贴着液面”快速跟随
 *
 * 依赖：
 *  - motorMove_upWithSpeed()/motorMove_downWithSpeed(): 每次调用推动继续运动（你现有粗找就是这样用的）
 *  - check_water_status(): 返回 WATER / NORMAL
 *  - Motor_CheckLostStep_AutoTiming(): 丢步检测（可选但建议保留）
 * 退出条件：
 *   连续 60s 内 water_level 波动（max-min）不超过 50mm -> 认为稳定找到水位，退出
 */
uint32_t FindWaterLevel_FastByStateFlip_StableExit(void);
/**
 * @brief 快速水位跟随（基于电容阈值 + 滞回 + 自恢复）
 *
 * 核心思想：
 * 1. 以“空气电容 zero_capacitance”为基准，构造：
 *      - 进入水区阈值 th
 *      - 离开水区滞回阈值 th_low
 * 2. 根据当前电容与阈值关系，判断“偏水 / 偏空气 / 稳定区”
 * 3. 偏水则上行，偏空气则下行；稳定区不动作
 * 4. 根据偏差大小选择不同步长（大 / 中 / 小）
 * 5. 当出现“偏差很大但电容几乎不变化”的异常情况时，
 *    累计 lost_count，超过阈值后触发重新找水位
 *
 * 特点：
 * - 步进式运动（motorMoveAndWaitUntilStopWithSpeed）
 * - 带滞回，避免界面抖动
 * - 带自恢复机制，避免长期卡死在错误区域
 */
uint32_t FollowWaterLevel_fast(void);
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
