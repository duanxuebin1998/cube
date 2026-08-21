/**
 * @file measure_density_internal.h
 * @brief density 目录内部共用的坐标、稳定采集、国标过滤和点阵打印接口。
 *
 * 本头文件通过 measure_density.h 取得公开测量类型，因此可独立包含，不依赖调用方
 * 预先包含 system_parameter.h 或其它头文件。
 */

#ifndef APPLICATION_MEASUREMENT_DENSITY_INTERNAL_H
#define APPLICATION_MEASUREMENT_DENSITY_INTERNAL_H

#include "measure_density.h"

#include <stdint.h>

/** 把有符号位置钳位为可发布的无符号 0.1 mm 坐标。 */
uint32_t DensityInternal_ValueToU01mmClamped(int32_t value_01mm, const char *tag);
/** 读取当前传感器位置并钳位为无符号 0.1 mm 坐标。 */
uint32_t DensityInternal_CurrentPositionToU01mmClamped(void);
/** 按 0.1 mm 单位打印测点数组，供普通分布与 SI Profile 共用。 */
void DensityInternal_PrintPoints01mm(const char *tag, const int32_t *p01, uint32_t n);
/** 在稳定窗口内读取密度、温度和频率；命令切换和底层错误原样返回。 */
uint32_t DensityInternal_ReadSensorWithStableWindow(volatile DensityMeasurement *result,
                                                    uint32_t stable_win_ms,
                                                    uint8_t *stable_out);
/** 按国标密度差规则过滤点阵并重算平均值，仅供普通分布内核调用。 */
void DensityInternal_FilterGbPointsByDensity20(DensityDistribution *dist,
                                               int32_t oil_level_01mm,
                                               uint32_t order,
                                               int32_t oil_standard_th);

#endif /* APPLICATION_MEASUREMENT_DENSITY_INTERNAL_H */
