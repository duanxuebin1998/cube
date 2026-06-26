/*
 * measureTankHeight.h
 *
 *  Created on: Feb 27, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_TANK_HEIGHT_H_
#define INC_MEASURE_TANK_HEIGHT_H_
#include "system_parameter.h"

#define BOTTOM_WEIGHT_THRESHOLD 0 /* 罐底测量参数：罐底 扭力 阈值。 */


typedef enum {
    BOTTOM_DET_BY_WEIGHT = 0,
    BOTTOM_DET_BY_GYRO   = 1,
} BottomDetectMode;


/* 记录零点角度基准（回零后或开始测罐底前保存） */
typedef struct {
    float x0_deg;
    float y0_deg;
    uint8_t valid;
} GyroZeroRef;

extern int32_t bottom_value;
/**
 * @brief 执行罐高测量中的 SearchBottom 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t SearchBottom(void);
/**
 * @brief 保存罐高测量中的 Bottom_SaveGyroZeroRef 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Bottom_SaveGyroZeroRef(void); /* 保存陀螺仪零点基准 */
#endif /* INC_MEASURE_TANK_HEIGHT_H_ */
