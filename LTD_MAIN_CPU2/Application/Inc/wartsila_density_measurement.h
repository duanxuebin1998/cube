/*
 * wartsila_density_measurement.h
 *
 *  Created on: 2025年12月1日
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

/**
 * @brief 执行测量流程中的 Wartsila_Density_SpreadMeasurement 逻辑。
 *
 * @param dist 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Wartsila_Density_SpreadMeasurement(DensityDistribution *dist);
#endif /* INC_WARTSILA_DENSITY_MEASUREMENT_H_ */
