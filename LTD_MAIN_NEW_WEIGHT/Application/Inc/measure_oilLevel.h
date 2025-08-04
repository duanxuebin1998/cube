/*
 * measure_oilLevel.h
 *
 *  Created on: Mar 28, 2025
 *      Author: 1
 */

#ifndef INC_MEASURE_OILLEVEL_H_
#define INC_MEASURE_OILLEVEL_H_

#include "system_parameter.h"

#define frequency_difference (measure_density_level.current_frequency-measure_density_level.follow_frequency)
#define STABILITYTHRESHOLD  15
#define OILWATERDETERMINATIONTHRESHOLD  5500
#define INAIR   (measure_density_level.current_frequency > OILWATERDETERMINATIONTHRESHOLD)
#define INOIL   (measure_density_level.current_frequency < OILWATERDETERMINATIONTHRESHOLD)
#define MAX_TIMES_WHEN_FRE_FOLLOW				15 /*频率跟随时的最大加速次数*/
#define MEASURE_DOWNLIMIT -1
struct measure_density_level
{
	float air_frequency;		//空气中频率
	float oil_frequency;		//油中频率
	float follow_frequency;		//液位跟随频率
	float current_frequency;	//当前频率
};
// 全局液位测量结构体，存储当前状态
struct measure_density_level measure_density_level;

int lookForLiquidLevelButNotFollow(void); //寻找油面但不跟随
int findAndFollowTheLiquidLevel(void); //寻找并跟随油面

#endif /* INC_MEASURE_OILLEVEL_H_ */
