/*
 * measure_density.h
 *
 *  Created on: Nov 8, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_DENSITY_H_
#define INC_MEASURE_DENSITY_H_

#include "app_main.h"
/* ===================== 配置宏 ===================== */

/* 单次测量最多点数 */
#ifndef MAX_MEASUREMENT_POINTS
#define MAX_MEASUREMENT_POINTS  100
#endif

/* 国标分段阈值（单位：0.1mm） */
#ifndef Spread_Gb_Onepiont_Posion
#define Spread_Gb_Onepiont_Posion   (30000u)  /* 3m */
#endif

#ifndef Spread_Gb_Twopiont_Posion
#define Spread_Gb_Twopiont_Posion   (45000u)  /* 4.5m */
#endif

/* 每米测步距（单位：0.1mm） */
#ifndef METER_STEP_01MM
#define METER_STEP_01MM             (10000u)  /* 1m */
#endif



typedef enum {
    DENS_MODE_SPREAD  = 1, /* 普通分布测 */
    DENS_MODE_GB      = 3, /* 国标测 */
    DENS_MODE_METER   = 4, /* 每米测 */
    DENS_MODE_INTERVAL= 5, /* 区间测 */
} DensitySpreadModeId;

/* ===================== 对外接口：四种模式入口 ===================== */
/* 普通分布测 */
void CMD_MeasureDensitySpread_Spread(void);

/* 国标测 */
void CMD_MeasureDensitySpread_GB(void);

/* 每米测 */
void CMD_MeasureDensitySpread_Meter(void);

/* 区间测 */
void CMD_MeasureDensitySpread_Interval(void);

/* ===================== 公共工具接口（本文件 .c 内实现，可能被其他模块复用） ===================== */
/* 打印分布测量结果（含点表与平均值） */
void Print_DensitySpreadResult(const DensityDistribution *dist);

/* 单点读数：内部完成稳定判定并写入 result */
uint32_t SinglePoint_ReadSensor(volatile DensityMeasurement *result);

uint32_t Density_MeasureByMode_Exact(DensitySpreadModeId mode, DensityDistribution *out_dist);
void CMD_SinglePointMeasurement();
void CMD_SinglePointMonitoring();

#endif /* INC_MEASURE_DENSITY_H_ */
