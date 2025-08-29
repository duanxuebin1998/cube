/*
 * measure_oilLevel.h
 *
 *  Created on: Mar 28, 2025
 *      Author: 1
 */

#ifndef INC_MEASURE_OILLEVEL_H_
#define INC_MEASURE_OILLEVEL_H_

#include "system_parameter.h"

#define frequency_difference (g_measurement.oil_measurement.current_frequency-g_measurement.oil_measurement.follow_frequency)
#define STABILITYTHRESHOLD  15
#define OILWATERDETERMINATIONTHRESHOLD  5500
#define INAIR   (g_measurement.oil_measurement.current_frequency > OILWATERDETERMINATIONTHRESHOLD)
#define INOIL   (g_measurement.oil_measurement.current_frequency < OILWATERDETERMINATIONTHRESHOLD)
#define MAX_TIMES_WHEN_FRE_FOLLOW				15 /*频率跟随时的最大加速次数*/
#define MEASURE_DOWNLIMIT -1


int lookForLiquidLevelButNotFollow(void); //寻找油面但不跟随
int findAndFollowTheLiquidLevel(void); //寻找并跟随油面

#endif /* INC_MEASURE_OILLEVEL_H_ */
