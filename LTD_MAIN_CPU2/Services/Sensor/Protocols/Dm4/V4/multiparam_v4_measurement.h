/*
 * 模块职责：把DM4-V4主动快照或交互参数转换为CPU2统一测量结果。
 * 调用边界：本层不组帧、不直接操作UART6，也不决定当前物理传感器类型。
 * 数值约束：无效值保持数据未就绪语义，不伪造0值，也不把协议允许的未初始化值直接升级为故障。
 */
#ifndef MULTIPARAM_V4_MEASUREMENT_H
/* 本头文件的包含保护标记。 */
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

/*
 * 函数用途：读取DM4-V4频率、密度和温度三项核心测量量。
 * 调用场景：V4驱动表的read_density回调。
 * 关键约束：三个输出指针都必须有效；主动和交互方式使用相同业务有效性标准。
 */
uint32_t MULTIPARAM_V4_MeasurementReadDensity(float *frequency_hz,
                                               float *density_kg_m3,
                                               float *temperature_c);

/*
 * 函数用途：读取DM4-V4液位频率。
 * 调用场景：V4驱动表的read_level_frequency回调。
 * 关键约束：只读取，不执行模式恢复和电机动作。
 */
uint32_t MULTIPARAM_V4_MeasurementReadLevelFrequency(uint32_t *frequency_hz);

/*
 * 函数用途：读取DM4-V4水位电容。
 * 调用场景：服务层完成模式及功能使能后调用。
 * 关键约束：无效浮点值返回协议格式异常，不把无效值写成正常零值。
 */
uint32_t MULTIPARAM_V4_MeasurementReadWaterCapacitance(float *capacitance_pf);

/*
 * 函数用途：读取DM4-V4零点霍尔电压。
 * 调用场景：服务层完成磁零点功能使能后调用。
 * 关键约束：只负责读数，不处理测水与磁零点互斥状态。
 */
uint32_t MULTIPARAM_V4_MeasurementReadMagneticZeroVoltage(float *voltage_v);

/*
 * 函数用途：读取DM4-V4双轴姿态角。
 * 调用场景：V4驱动表的read_gyro回调。
 * 关键约束：两个输出作为同一组数据校验和返回。
 */
uint32_t MULTIPARAM_V4_MeasurementReadGyro(float *angle_x_deg, float *angle_y_deg);

#endif /* MULTIPARAM_V4_MEASUREMENT_H */
