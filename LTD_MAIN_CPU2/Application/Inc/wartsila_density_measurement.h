/*
 * wartsila_density_measurement.h
 *
 *  Created on: 2025年12月1日
 *      Author: Duan Xuebin
 */

#ifndef INC_WARTSILA_DENSITY_MEASUREMENT_H_
/* INC_WARTSILA_DENSITY_MEASUREMENT_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
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
 * @brief  Wartsila 密度分布测量（点间只按位置移动，到点后用密度/频率判定空气）
 *
 * 点间移动和起始点定位不做运动中液位检测；到点后使用密度模式读取实际密度值和浮点频率。
 * 空气点不保存，切液位模式慢速下行，首次识别到 OIL 时把当前位置作为液位。
 * 最终只保留位置严格小于“液位 - 距离限制”的液体点。
 *
 * @param  dist  输出测量结果的结构体指针（一般为 &g_measurement.density_distribution）
 * @return 错误码，NO_ERROR 表示成功
 */
uint32_t Wartsila_Density_SpreadMeasurement(DensityDistribution *dist);
#endif /* INC_WARTSILA_DENSITY_MEASUREMENT_H_ */
