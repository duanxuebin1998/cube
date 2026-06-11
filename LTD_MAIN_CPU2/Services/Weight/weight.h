/*
 * weight.h
 *
 *  Created on: Jan 18, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_WEIGHT_H_
#define INC_WEIGHT_H_

#include "AS5145.h"
#include "system_parameter.h"
/* #include "motor_ctrl.h" */

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

/**
 * @brief 初始化称重数据中的 weight_init 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t weight_init() ;
/* uint32_t get_stable_weight(void); */
uint32_t get_empty_weight(void);
uint32_t get_full_weight(void);
/* Weight_StateTypeDef determine_weight_status(void); */
Weight_StateTypeDef check_zero_point_status(void);
uint32_t check_bottom_status(Weight_StateTypeDef *status); /* 检测罐底状态，返回错误码并通过参数输出 NORMAL/BOTTOM */
uint32_t CheckWeightCollision(void);
/**
 * @brief 更新称重模块当前重量并维护稳定/空载/满载状态。
 * @param currWeight 当前重量采样值。
 */
void Weight_Update(int32_t currWeight);
/**
 * @brief 接收称重数据中的 Weight_MarkFrameReceived 逻辑。
 */
void Weight_MarkFrameReceived(void);
/**
 * @brief 检查称重数据中的 Weight_CheckCommunicationTimeout 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Weight_CheckCommunicationTimeout(void);
/**
 * @brief 检查称重数据中的 Weight_CheckOwnCommunicationTimeout 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Weight_CheckOwnCommunicationTimeout(void);
#endif /* INC_WEIGHT_H_ */
