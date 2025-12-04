/*
 * measure_density.h
 *
 *  Created on: Nov 8, 2025
 *      Author: admin
 */

#ifndef INC_MEASURE_DENSITY_H_
#define INC_MEASURE_DENSITY_H_

#include "app_main.h"
void MeasureDensitySpread(void);
uint32_t SinglePointMeasurement();
uint32_t SinglePointMonitoring();
uint32_t SinglePoint_ReadSensor(volatile DensityMeasurement *result);
#endif /* INC_MEASURE_DENSITY_H_ */
