/*
 * fault_manager.h
 *
 *  Created on: Mar 13, 2025
 *      Author: 1
 */

#ifndef INC_FAULT_MANAGER_H_
#define INC_FAULT_MANAGER_H_

#include <stdint.h>   // 处理 uint8_t, uint32_t 等类型
#include <string.h>   // 处理 memset、memcpy 等函数

#define	NO_ERROR 0
/* 故障大类枚举 */
typedef enum
{
	FAULT_MOTOR = 11,          // 电机类故障
	FAULT_ENCODER,            // 编码器类故障
	FAULT_SENSOR,             // 传感器类故障
	FAULT_WEIGHT,             // 称重故障
	FAULT_MEASUREMENT,        // 测量过程类错误
	FAULT_WIRELESS_SLIPRING,  // 无线滑环类故障
	FAULT_PARAM_STORAGE,      // 参数/存储错误
	FAULT_OTHER               // 其他类型错误
} FaultCategory;

/* 故障等级（决定处理优先级和恢复策略） */
typedef enum
{
	FAULT_SEVERITY_NONE = 0,      // 无故障
	FAULT_SEVERITY_WARNING,   	  // 警告：不影响核心功能，需要打印故障信息
	FAULT_SEVERITY_ERROR,         // 错误：功能降级/有限次重试
	FAULT_SEVERITY_CRITICAL,      // 严重错误：安全保护，立即停机等待人工干预
	FAULT_SEVERITY_FATAL          // 致命错误：强制系统重启
} FaultSeverity;

/* 故障信息结构体 */
typedef struct
{
	FaultCategory category;    // 故障大类
	uint8_t error_code;        // 自定义错误码
	uint8_t location;          // 故障位置
	FaultSeverity severity;    // 故障等级
	uint8_t retry_count;       // 重试次数
} FaultInfo;

/* 电机类故障枚举 */
typedef enum
{
	MOTOR_FAIL_SETTING = 1,    // 电机设置失败
	MOTOR_UNKNOWN_FEEDBACK,    // 未知反馈
	MOTOR_RESET_FAIL,          // 复位失败
	MOTOR_DISABLED,            // 电机被禁止
	MOTOR_ALARM_TRIGGERED,     // 电机报警
	MOTOR_STEP_ERROR,          // 步进数错误
	MOTOR_OVERCURRENT,         // 过电流故障
	MOTOR_OVERTEMPERATURE      // 过温故障
} MotorFault;

/* 编码器类故障枚举 */
typedef enum
{
	ENCODER_TIMEOUT = 1,       // 通信超时
	ENCODER_PARITY_ERROR,      // 偶校验失败
	ENCODER_DATA_LOST,         // 通信失败
	ENCODER_POWERON_FAIL,      // 上电初始化失败
	ENCODER_INVALID_DATA,      // 持续无效数据
	ENCODER_OCF_INCOMPLETE,    // OCF 未完成，数据无效
	ENCODER_CORDIC_OVERFLOW,   // CORDIC 溢出，数据无效
	ENCODER_LINEARITY_WARNING  // 线性度报警，数据可能无效
} EncoderFault;

/* 传感器类故障枚举 */
typedef enum
{
	SENSOR_BCC_ERROR = 1,       // 数据校验失败
	SENSOR_COMM_TIMEOUT,        // 通信超时
	SONIC_FREQ_ABNORMAL,        // 震动管频率异常
	DENSITY_INVALID,            // 密度值异常
	SENSOR_TEMPERATURE_ERROR,   // 传感器温度异常
	SENSOR_VOLTAGE_ERROR        // 传感器电压异常
} SensorFault;

/* 称重类故障枚举 */
typedef enum
{
	WEIGHT_OUT_OF_RANGE = 1,        // 称重超范围(超过上限阈值)
	WEIGHT_UNDER_RANGE,             // 称重超范围(小于下限阈值)
	WEIGHT_COLLISION_DETECTED,      // 检测到磕碰/障碍
	WEIGHT_DRIFT_ERROR,             // 称重漂移异常
	WEIGHT_SENSOR_SATURATION        // 称重传感器饱和
} WeightFault;

/* 测量过程故障枚举 */
typedef enum
{
	MEASUREMENT_POSITION_ERROR = 1, // 位置测量错误
	MEASUREMENT_TIMEOUT,            // 测量超时
	MEASUREMENT_ZERO_CALIB_TIMEOUT, // 标定零点超限次
	MEASUREMENT_OVERSPEED,          // 液位变化超速
	MEASUREMENT_ZERO_REPEAT_FAIL,   // 零点重复性差
	MEASUREMENT_HEIGHT_DEVIATION,   // 实高偏差过大
	MEASUREMENT_OILLEVEL_HIGH,      // 液位值过高
	MEASUREMENT_OILLEVEL_LOW,       // 向下未找到液位值
	MEASUREMENT_OILLEVEL_NOTFOUND,  // 向上未找到液位值
	MEASUREMENT_WEIGHT_DOWN_FAIL,   // 下行寻重失败
	MEASUREMENT_WEIGHT_UP_FAIL     // 上行寻重失败
} MeasurementFault;

/* 无线滑环类故障枚举 */
typedef enum
{
	SLIPRING_COMM_FAIL = 1,   // 与从机通讯故障
	SLIPRING_BCC_ERROR,       // 通讯过程BCC错误
	SLIPRING_PACKET_LOSS,     // 数据包丢失率超限
	SLIPRING_SIGNAL_WEAK      // 无线信号强度不足
} WirelessSlipringFault;

/* 参数/存储类故障枚举 */
typedef enum
{
	PARAM_EEPROM_FAIL = 1,        // EEPROM写入失败
	PARAM_UNINITIALIZED,          // 参数未初始化
	PARAM_RANGE_ERROR,            // 参数范围错误
	PARAM_ADDRESS_OVERFLOW,       // 参数地址越界
	PARAM_CRC_ERROR               // 参数读取CRC校验错误
} ParamStorageFault;

/* 其他故障枚举 */
typedef enum
{
	OTHER_UNKNOWN_ERROR = 1,      // 未知故障
	OTHER_ADDRESS_READ_ERROR,      // 地址读取错误
	OTHER_POWER_FLUCTUATION,       // 电源波动异常
} OtherFault;

// 故障恢复动作定义
typedef enum
{
	FAULT_ACTION_NONE,          // 无操作（仅记录日志）
	FAULT_ACTION_RETRY,         // 重试操作（如重新初始化外设）
	FAULT_ACTION_RESET_MODULE,  // 复位模块（如重启通信芯片）
	FAULT_ACTION_SYSTEM_REBOOT  // 系统级复位
} FaultRecoveryAction;

// 故障恢复策略配置
typedef struct
{
	FaultSeverity severity;      // 触发等级
	uint8_t max_retries;        // 最大重试次数
	FaultRecoveryAction action;  // 恢复动作
} FaultRecoveryPolicy;

void fault_info_init(void);
uint32_t update_and_encode_fault(FaultCategory category, uint8_t error_code,
		uint8_t location, FaultSeverity severity);

#endif /* INC_FAULT_MANAGER_H_ */
