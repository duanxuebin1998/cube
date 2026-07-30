/*
 * measureTankHeight.h
 *
 *  Created on: Feb 27, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_TANK_HEIGHT_H_
/* INC_MEASURE_TANK_HEIGHT_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_MEASURE_TANK_HEIGHT_H_
#include "system_parameter.h"

#define BOTTOM_WEIGHT_THRESHOLD 0 /* 罐底测量参数：罐底 扭力 阈值。 */


typedef enum {
    /* 罐底到达判定所采用的传感依据。 */
    BOTTOM_DET_BY_WEIGHT = 0, /* 使用扭力/重量变化判定探头到达罐底。 */
    BOTTOM_DET_BY_GYRO   = 1, /* 使用陀螺仪角度变化判定探头到达罐底。 */
} BottomDetectMode;


/* 记录零点角度基准（回零后或开始测罐底前保存） */
typedef struct {
    /* 探底角度判定的 X/Y 零位参考及其有效标志。 */
    float x0_deg; /* 建立探底基准时的陀螺仪 X 轴角度，单位为度。 */
    float y0_deg; /* 建立探底基准时的陀螺仪 Y 轴角度，单位为度。 */
    uint8_t valid; /* X/Y 零位参考已经成功采集的标志；为假时禁止执行角度探底判定。 */
} GyroZeroRef;

extern int32_t bottom_value;
/**
 * @brief 罐底测量函数 - 执行完整的罐底搜索流程
 *        包含：粗找罐底（3次重试） + 两次精找（各3次重试）
 *        状态切换时不重试
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
uint32_t SearchBottom(void);
/**
 * @brief 保存探底流程使用的陀螺仪零位参考角。
 * @return NO_ERROR 表示多次陀螺仪采样稳定且平均零位已保存；采样离散度超限返回 MEASUREMENT_ZERO_REPEAT_FAIL，传感器读取失败时原样返回其错误码。
 */
uint32_t Bottom_SaveGyroZeroRef(void); /* 保存陀螺仪零点基准 */
#endif /* INC_MEASURE_TANK_HEIGHT_H_ */
