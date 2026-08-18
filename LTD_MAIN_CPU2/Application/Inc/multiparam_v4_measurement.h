/*
 * multiparam_v4_measurement.h
 * 多参数V4主动与交互测量数据到CPU2运行态的独立适配层。
 */
#ifndef MULTIPARAM_V4_MEASUREMENT_H
#define MULTIPARAM_V4_MEASUREMENT_H

#include <stdint.h>

/*
 * 函数用途：初始化主动发布代次和交互附加刷新节流状态。
 * 调用场景：确认多参数V4设备并建立通信方式后。
 */
void MULTIPARAM_V4_MeasurementInit(void);

/*
 * 函数用途：把PendSV刚发布的新快照直接更新到CPU2运行数据。
 * 调用场景：统一PendSV_Handler确认新快照后调用。
 * 关键约束：不打印、不等待，每个快照代次只发布一次。
 */
uint32_t MULTIPARAM_V4_MeasurementProcessDeferred(void);

/*
 * 函数用途：在交互通信方式下读取并刷新三项多参数V4附加调试量。
 * 调用场景：密度、液位、水位或姿态交互读取成功后调用。
 * 关键约束：内部按500ms节流，各字段独立成功更新，使用单次短事务且不在中断中调用。
 */
uint32_t MULTIPARAM_V4_MeasurementRefreshInteractiveDebugData(void);

uint32_t MULTIPARAM_V4_MeasurementReadDensity(float *frequency_hz,
                                               float *density_kg_m3,
                                               float *temperature_c);
uint32_t MULTIPARAM_V4_MeasurementReadLevelFrequency(uint32_t *frequency_hz);
uint32_t MULTIPARAM_V4_MeasurementReadWaterCapacitance(float *capacitance_pf);
uint32_t MULTIPARAM_V4_MeasurementReadMagneticZeroVoltage(float *voltage_v);
uint32_t MULTIPARAM_V4_MeasurementReadGyro(float *angle_x_deg, float *angle_y_deg);

#endif /* MULTIPARAM_V4_MEASUREMENT_H */
