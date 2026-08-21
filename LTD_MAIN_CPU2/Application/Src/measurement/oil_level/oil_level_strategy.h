/**
 * @file oil_level_strategy.h
 * @brief 将设备参数中的液位测量方式映射为无副作用的算法族选择结果。
 *
 * 本头文件只描述方式编号和选择结果。传感器切换、电机运动、结果发布及故障处理
 * 均由对应算法实现负责。
 */

#ifndef INC_OIL_LEVEL_STRATEGY_H_
#define INC_OIL_LEVEL_STRATEGY_H_

#include <stdint.h>

/** 设备参数 liquidLevelMeasurementMethod 使用的液位测量方式编号。 */
typedef enum {
    OIL_LEVEL_METHOD_RELATIVE_FREQ = 0U,            /* 传统相对频率步进找液位。 */
    OIL_LEVEL_METHOD_FIXED_FREQ = 1U,               /* 传统固定频率步进找液位。 */
    OIL_LEVEL_METHOD_DENSITY = 2U,                  /* 密度闭环找液位。 */
    OIL_LEVEL_METHOD_ULTRASONIC_RESERVED = 3U,      /* 超声波保留方式，当前不支持。 */
    OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ = 4U, /* 相对频率连续速度闭环。 */
    OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ = 5U     /* 固定频率连续速度闭环。 */
} OilLevelMeasurementMethod;

/** 液位测量方式归并后的算法族。 */
typedef enum {
    OIL_LEVEL_ALGORITHM_LEGACY_STEP = 0,          /* 方式 0 和方式 1 的传统步进流程。 */
    OIL_LEVEL_ALGORITHM_DENSITY_CLOSED_LOOP = 1,  /* 方式 2 的密度闭环。 */
    OIL_LEVEL_ALGORITHM_FREQUENCY_CLOSED_LOOP = 2,/* 方式 4 和方式 5 的频率闭环。 */
    OIL_LEVEL_ALGORITHM_UNSUPPORTED = 3           /* 保留方式或无效参数。 */
} OilLevelAlgorithmFamily;

/** 策略选择器返回给搜索和跟随调度层的无副作用结果。 */
typedef struct {
    uint32_t raw_method;              /* 参数中保存的原始方式编号。 */
    OilLevelAlgorithmFamily family;   /* 由原始方式选择出的算法族。 */
    uint8_t direct_search;            /* 非零表示支持一次找液位命令。 */
    uint8_t continuous_follow;        /* 非零表示支持连续跟随命令。 */
    uint8_t fixed_frequency_target;   /* 非零表示使用固定频率目标。 */
} OilLevelStrategySelection;

/**
 * @brief 在不修改设备状态和测量状态的情况下选择算法族。
 * @param raw_method 参数中配置的液位测量方式编号。
 * @param selection 输出选择结果；传入 NULL 时不执行任何操作。
 */
void OilLevelStrategy_Select(uint32_t raw_method,
                             OilLevelStrategySelection *selection);

#endif /* INC_OIL_LEVEL_STRATEGY_H_ */
