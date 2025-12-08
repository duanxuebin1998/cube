/*
 * weight.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 */

#ifndef INC_WEIGHT_H_
#define INC_WEIGHT_H_

#include "as5145.h"
#include "system_parameter.h"
//#include "motor_ctrl.h"

#define ZERO_WEIGHT_THRESHOLD 70
#define BOTTOM_WEIGHT_THRESHOLD 2000
#define IMPACT_WEIGHT_THRESHOLD 4000.0
typedef enum {
	NORMAL, IMPACT, ZERO, BOTTOM
} Weight_StateTypeDef;

typedef struct {
	int stable_weight; /* 稳态基准重量（单位：0.1克） */
	int current_weight; /* 实时采样重量（不带滤波处理） */
	int empty_weight; /* 空载称重 */
	int full_weight; /* 满载称重 */
} Weight_ParamentTypeDef;

extern Weight_ParamentTypeDef weight_parament;

uint32_t weight_init() ;
//uint32_t get_stable_weight(void);
uint32_t get_empty_weight(void);
uint32_t get_full_weight(void);
//Weight_StateTypeDef determine_weight_status(void);
Weight_StateTypeDef check_zero_point_status(void);
Weight_StateTypeDef check_bottom_status(void);
uint32_t CheckWeightCollision(void);
void Weight_Update(int32_t currWeight);
#endif /* INC_WEIGHT_H_ */
