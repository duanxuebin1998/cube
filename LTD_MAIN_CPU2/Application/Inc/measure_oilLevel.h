/*
 * measure_oilLevel.h
 *
 *  Created on: Mar 28, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_OILLEVEL_H_
#define INC_MEASURE_OILLEVEL_H_

#include "system_parameter.h"
typedef enum {
	OIL, AIR
} Level_StateTypeDef;

#define frequency_difference ((float)g_measurement.oil_measurement.current_frequency-(float)g_measurement.oil_measurement.follow_frequency)
#define INAIR   (g_measurement.oil_measurement.current_frequency > g_deviceParams.oilLevelFrequency)
#define INOIL   (g_measurement.oil_measurement.current_frequency < g_deviceParams.oilLevelFrequency)
#define MAX_TIMES_WHEN_FRE_FOLLOW				15 /* 频率跟随时的最大加速次数 */

uint32_t determine_level_status(Level_StateTypeDef *state_out);
/**
 * @brief 执行液位测量中的 determine_level_status_motion 逻辑。
 *
 * @param state_out 状态值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t determine_level_status_motion(Level_StateTypeDef *state_out);
/**
 * @brief 执行液位测量中的 SearchOilLevel 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t SearchOilLevel(void); /* 寻找油面但不跟随 */
/**
 * @brief 执行液位测量中的 FollowOilLevel 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t FollowOilLevel(void); /* 在油面附近跟随油面 */
/**
 * @brief 执行液位测量中的 SearchAndFollowOilLevel 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t SearchAndFollowOilLevel(void); /* 寻找并跟随油面 */
/**
 * @brief 处理液位测量中的 CorrectOilLevelProcess 逻辑。
 */
void CorrectOilLevelProcess(void); /* 修正液位 */
#endif /* INC_MEASURE_OILLEVEL_H_ */
