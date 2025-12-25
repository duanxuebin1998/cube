/*
 * measure_density.h
 *
 *  Created on: Nov 8, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_DENSITY_H_
#define INC_MEASURE_DENSITY_H_

#include "app_main.h"
void CMD_MeasureDensitySpread(void);
void CMD_SinglePointMeasurement();
void CMD_SinglePointMonitoring();

uint32_t Density_SpreadMeasurement(DensityDistribution *dist);
uint32_t SinglePoint_ReadSensor(volatile DensityMeasurement *result);
#endif /* INC_MEASURE_DENSITY_H_ */
