/*
 * weight.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 */

#ifndef INC_WEIGHT_H_
#define INC_WEIGHT_H_

#include "as5145.h"

#include "motor_ctrl.h"
#define ZERO_WEIGHT_THRESHOLD 70
#define BOTTOM_WEIGHT_THRESHOLD 70
#define IMPACT_WEIGHT_THRESHOLD 20
#define MAX_WEIGHT
typedef enum
{
	NORMAL, IMPACT, ZERO, BOTTOM
} Weight_StateTypeDef;

typedef struct
{
	int stable_weight; /* 稳态基准重量（单位：0.1克） */
	int current_weight; /* 实时采样重量（不带滤波处理） */
	int zero_weight; /* 零点标定基准值（带载状态） */
	int bottom_weight; /* 罐底称重值 */
	int bottom_threshold; /* 罐底判断动态阈值 */
	int zero_threshold; /* 零点阈值*/
} Weight_ParamentTypeDef;

extern Weight_ParamentTypeDef weight_parament;

void get_stable_weight(void);
Weight_StateTypeDef determine_weight_status(void);

#endif /* INC_WEIGHT_H_ */
