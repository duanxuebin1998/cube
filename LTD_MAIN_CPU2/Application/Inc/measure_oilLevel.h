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
#define MAX_TIMES_WHEN_FRE_FOLLOW				15 /*频率跟随时的最大加速次数*/


uint32_t SearchOilLevel(void); //寻找油面但不跟随
uint32_t FollowOilLevel(void); //在油面附近跟随油面
uint32_t SearchAndFollowOilLevel(void); //寻找并跟随油面
void CorrectOilLevelProcess(void);//修正液位
#endif /* INC_MEASURE_OILLEVEL_H_ */
