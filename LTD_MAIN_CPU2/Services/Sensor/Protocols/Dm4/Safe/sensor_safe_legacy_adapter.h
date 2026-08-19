/*
 * 模块职责：声明DM4进入Safe会话后供统一传感器驱动调用的兼容接口。
 * 探测边界：Safe不是独立物理传感器类型，本头文件的Probe接口不得加入普通上电识别链。
 * 预留约束：未冻结的Safe进入和退出流程保持显式接口，不用持久化类型值替代会话状态。
 */
#ifndef SENSOR_SAFE_LEGACY_ADAPTER_H_
/* SENSOR_SAFE_LEGACY_ADAPTER_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_LEGACY_ADAPTER_H_

#include <stdint.h>

#include "sensor_safe_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 探测并激活安全传感器，同时保持旧 sensor.c 返回码口径。
 *
 * @details 调用场景：蓝牙透传链路检查成功后、LTD 和 DSM 探测之前。
 * @note 关键约束：失败时会撤销安全传输状态，调用方必须继续旧协议回退探测。
 */
uint32_t SensorSafeAdapter_Probe(uint32_t *sensor_id);

/**
 * @brief 撤销安全会话并释放 UART6 DMA；不清除身份模块持久化启动计数。
 *
 * 撤销安全协议活动状态并释放 UART6 DMA，供旧协议回退使用。
 */
void SensorSafeAdapter_Deactivate(void);

/**
 * @brief 仅中止当前 UART6 DMA 事务，用于命令切换快速退出，不修改会话对象。
 *
 * 仅停止安全传输 DMA，不撤销已建立的安全会话。
 */
void SensorSafeAdapter_AbortTransport(void);

/**
 * @brief 查询安全服务是否已完成 HELLO 且可接收业务命令。
 *
 * @return 1 表示安全服务已完成探测并处于 active 状态，可接收业务命令；0 表示服务上下文尚未激活。
 */
uint8_t SensorSafeAdapter_IsActive(void);
/**
 * @brief 根据已验收 HELLO 能力位判断水位电容命令是否可用。
 *
 * @return 1 表示安全服务已激活且 HELLO 能力位声明支持水位电容；否则返回 0。
 */
uint8_t SensorSafeAdapter_SupportsWaterCap(void);
/**
 * @brief 根据已验收 HELLO 能力位判断陀螺仪命令是否可用。
 *
 * @return 1 表示安全服务已激活且 HELLO 能力位声明支持陀螺仪；否则返回 0。
 */
uint8_t SensorSafeAdapter_SupportsGyro(void);
/**
 * @brief 查询当前安全传感器是否声明参数维护和完整参数表读取能力。
 *
 * @return 1 表示安全服务处于活动态，且身份能力位包含 SENSOR_SAFE_CAP_PARAM_RW；0 表示服务未激活，或传感器未声明参数读写能力。
 */
uint8_t SensorSafeAdapter_SupportsParamTable(void);

/**
 * @brief 把旧业务的密度模式入口映射到安全协议模式命令。
 *
 * @return NO_ERROR 表示安全传感器已确认密度模式且稳定等待完成；命令切换、传输、协议、远端、能力或模式错误返回映射后的整机错误码。
 */
uint32_t SensorSafeAdapter_EnableDensityMode(void);
/**
 * @brief 把旧业务的液位模式入口映射到安全协议模式命令。
 *
 * @return 返回安全液位模式切换映射后的整机错误码；NO_ERROR 表示模式确认和稳定等待均完成。
 */
uint32_t SensorSafeAdapter_EnableLevelMode(void);
/**
 * @brief 读取安全协议密度组合值并保持旧业务层的浮点单位。
 *
 * @details 调用场景：现有密度测量流程识别为安全传感器后调用。
 * @note 关键约束：恢复日志只由适配层记录，最终故障仍交给原测量流程统一出口。
 *
 * @param frequency_hz 传感器频率，单位 Hz。
 * @param density_kg_m3 用于返回实时密度的输出参数，单位 kg/m3。
 * @param temperature_c 温度值，单位 ℃。
 * @return NO_ERROR 表示频率、密度和温度三个输出均已更新；数据无效映射为 DENSITY_INVALID，其他服务、传输、协议或远端失败返回对应整机错误码。
 */
uint32_t SensorSafeAdapter_ReadDensity(float *frequency_hz,
                                       float *density_kg_m3,
                                       float *temperature_c);
/**
 * @brief 读取并映射液位频率，失败时保留原业务的 SONIC_FREQ_ABNORMAL 口径。
 *
 * @param frequency_hz 传感器频率，单位 Hz。
 * @return NO_ERROR 表示液位频率输出已更新；数据无效映射为 SONIC_FREQ_ABNORMAL，其他服务、传输、协议或远端失败返回对应整机错误码。
 */
uint32_t SensorSafeAdapter_ReadLevelFrequency(uint32_t *frequency_hz);
/**
 * @brief 读取水位电容并把安全服务结果映射为现有整机错误码。
 *
 * @param capacitance_pf 用于返回水位通道电容值的输出参数，单位 pF。
 * @return 返回安全水位电容读取映射后的整机错误码；NO_ERROR 表示输出值有效，不支持、远端或通信失败返回对应错误。
 */
uint32_t SensorSafeAdapter_ReadWaterCapacitance(float *capacitance_pf);
/**
 * @brief 读取双轴姿态角并把安全服务结果映射为现有整机错误码。
 *
 * @param angle_x_deg 用于返回陀螺仪 X 轴角度的输出参数，单位度。
 * @param angle_y_deg 用于返回陀螺仪 Y 轴角度的输出参数，单位度。
 * @return NO_ERROR 表示 X、Y 双轴角度均已更新；数据无效映射为 SENSOR_GYRO_ANGLE_ERROR，其他服务、传输、协议或远端失败返回对应整机错误码。
 */
uint32_t SensorSafeAdapter_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg);

/**
 * @brief 对 CPU2 上层提供一次调用读取固定 115 项 LTD 参数表。
 *
 * @details 调用场景：上电参数同步或维护导出，不得放入实时测量循环。
 * @note 关键约束：底层自动完成 39 页；容量不足时 value_count_out 返回 115。
 */
uint32_t SensorSafeAdapter_ReadAllParams(SensorSafeParameterValue *values,
                                         uint16_t value_capacity,
                                         uint16_t *value_count_out);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_LEGACY_ADAPTER_H_ */
