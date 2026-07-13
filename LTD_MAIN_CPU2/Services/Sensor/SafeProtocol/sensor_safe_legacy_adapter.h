#ifndef SENSOR_SAFE_LEGACY_ADAPTER_H_
#define SENSOR_SAFE_LEGACY_ADAPTER_H_

#include <stdint.h>

#include "sensor_safe_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数用途：探测并激活安全传感器，同时保持旧 sensor.c 返回码口径。
 * 调用场景：蓝牙透传链路检查成功后、LTD 和 DSM 探测之前。
 * 关键约束：失败时会撤销安全传输状态，调用方必须继续旧协议回退探测。
 */
uint32_t SensorSafeAdapter_Probe(uint32_t *sensor_id);

/* 撤销安全协议活动状态并释放 UART6 DMA，供旧协议回退使用。 */
void SensorSafeAdapter_Deactivate(void);

/* 仅停止安全传输 DMA，不撤销已建立的安全会话。 */
void SensorSafeAdapter_AbortTransport(void);

uint8_t SensorSafeAdapter_IsActive(void);
uint8_t SensorSafeAdapter_SupportsWaterCap(void);
uint8_t SensorSafeAdapter_SupportsGyro(void);
uint8_t SensorSafeAdapter_SupportsParamTable(void);

uint32_t SensorSafeAdapter_EnableDensityMode(void);
uint32_t SensorSafeAdapter_EnableLevelMode(void);
uint32_t SensorSafeAdapter_ReadDensity(float *frequency_hz,
                                       float *density_kg_m3,
                                       float *temperature_c);
uint32_t SensorSafeAdapter_ReadLevelFrequency(uint32_t *frequency_hz);
uint32_t SensorSafeAdapter_ReadWaterCapacitance(float *capacitance_pf);
uint32_t SensorSafeAdapter_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg);

/*
 * 函数用途：对 CPU2 上层提供一次调用读取固定 115 项 LTD 参数表。
 * 调用场景：上电参数同步或维护导出，不得放入实时测量循环。
 * 关键约束：底层自动完成 39 页；容量不足时 value_count_out 返回 115。
 */
uint32_t SensorSafeAdapter_ReadAllParams(SensorSafeParameterValue *values,
                                         uint16_t value_capacity,
                                         uint16_t *value_count_out);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_LEGACY_ADAPTER_H_ */
