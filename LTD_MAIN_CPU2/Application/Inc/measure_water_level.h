/*
 * @FilePath     : \CUBE\LTD_MAIN_CPU2\Application\Inc\measure_water_level.h
 * @Description  :
 * @Author       : Aubon
 * @Date         : 2025-12-19 15:55:33
 * @LastEditors  : Duan Xuebin
 * @LastEditTime : 2026-03-26 13:42:14
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

#define WATER_STABLE_WINDOW_DEFAULT_MS (30000u)

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
 * @brief 水位传感器剖面测试
 *
 * 流程：
 * 1. 先执行一次 SearchWaterLevel()，定位当前水位界面
 * 2. 上行 50mm 作为测试起点
 * 3. 以 0.1mm 步进先下行 100mm，记录每个点位的位置和电容值
 * 4. 再以 0.1mm 步进上行 100mm，记录每个点位的位置和电容值
 * 5. 最后统一打印上行、下行两段扫描结果
 *
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t WaterSensorCapacitanceProfileTest(void);

/**
 * @brief 水位跟随（持续监控电容并控制电机微调）
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t FollowWaterLevel(void);

/**
 * @brief 快速找跟随点（基于状态翻转）
 *
 * 流程：
 * 1. 根据当前 WATER / NORMAL 状态决定上行或下行方向
 * 2. 每次检测到状态翻转时，记录一次翻转位置
 * 3. 从第二次翻转开始，用最近两次翻转位置的平均值估计当前液面
 * 4. 当估计值在 stable_win_ms 窗口内波动足够小时，认为快速找点完成
 *
 * @param stable_win_ms 稳定判定窗口时长（ms）
 * @return 错误码（NO_ERROR 表示成功）
 */
uint32_t FindWaterLevel_FastByStateFlip_StableExit(uint32_t stable_win_ms);

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
 * - 步进式运动（MotorCtrl_MoveAndWait）
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
