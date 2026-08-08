/*
 * multiparam_v4_measurement.h
 * 多参数V4主动上报数据到CPU2运行态的独立测量适配层。
 */
#ifndef MULTIPARAM_V4_MEASUREMENT_H
#define MULTIPARAM_V4_MEASUREMENT_H

#include <stdint.h>

/*
 * 函数用途：初始化主动测量发布代次。
 * 调用场景：确认多参数V4主动模式并启动常驻接收后。
 */
void MULTIPARAM_V4_MeasurementInit(void);

/*
 * 函数用途：把PendSV刚发布的新快照直接更新到CPU2运行数据。
 * 调用场景：统一PendSV_Handler确认新快照后调用。
 * 关键约束：不打印、不等待，每个快照代次只发布一次。
 */
uint32_t MULTIPARAM_V4_MeasurementProcessDeferred(void);

uint32_t MULTIPARAM_V4_MeasurementReadDensity(float *frequency_hz,
                                               float *density_kg_m3,
                                               float *temperature_c);
uint32_t MULTIPARAM_V4_MeasurementReadLevelFrequency(uint32_t *frequency_hz);
uint32_t MULTIPARAM_V4_MeasurementReadWaterCapacitance(float *capacitance_pf);
uint32_t MULTIPARAM_V4_MeasurementReadGyro(float *angle_x_deg, float *angle_y_deg);

#endif /* MULTIPARAM_V4_MEASUREMENT_H */
