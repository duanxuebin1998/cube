/*
 * wartsila_density_measurement.h
 *
 *  Created on: 2025Äê12ÔÂ1ÈÕ
 *      Author: Duan Xuebin
 */

#ifndef INC_WARTSILA_DENSITY_MEASUREMENT_H_
#define INC_WARTSILA_DENSITY_MEASUREMENT_H_
#include "motor_ctrl.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "system_parameter.h"
#include "measure_oilLevel.h"
#include "measure.h"

uint32_t Wartsila_Density_SpreadMeasurement(DensityDistribution *dist);
#endif /* INC_WARTSILA_DENSITY_MEASUREMENT_H_ */
