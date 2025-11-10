/*
 * dsm_v2.h
 *
 *  Created on: Nov 10, 2025
 *      Author: admin
 */
#ifndef DSM_V2_H
#define DSM_V2_H

#include <stdint.h>
#include "main.h"
#include "sensor.h"
// 模式枚举
typedef enum {
    DSM_V2_MODE_LEVEL   = 'L',   // 液位模式
    DSM_V2_MODE_DENSITY = 'D',   // 密度模式
} dsm_v2_mode_t;

// === 对外 API ===

// 模式切换（param 固定 0x00）
int DSM_V2_SwitchMode(dsm_v2_mode_t mode);
int DSM_V2_SwitchToLevelMode(void);
int DSM_V2_SwitchToDensityMode(void);

// 通用读取
int DSM_V2_Read_FloatParam(uint8_t param, float *out_value);
int DSM_V2_Read_IntParam  (uint8_t param, int32_t *out_value);

// 参数读取
int DSM_V2_Read_SoftwareVersion(float *v);        // R 00
int DSM_V2_Read_LevelFrequency(uint32_t *freq_hz);   // R 04
int DSM_V2_Read_Temperature    (float *t);        // R 06
int DSM_V2_Read_Density        (float *rho);      // R 07
int DSM_V2_Read_DynamicViscosity(float *mu);      // R 08 动力粘度
int DSM_V2_Read_KinematicViscosity(float *nu);    // R 09 运动粘度
int DSM_V2_Read_MeanSquare45   (float *msq45);    // R 17 (0x11)
int DSM_V2_Read_MeanSquare22p5 (float *msq22p5);  // R 18 (0x12)
int DSM_V2_Read_SensorID       (uint32_t *sensor_id); // R 22 (0x16) 整型


#endif // DSM_V2_H
