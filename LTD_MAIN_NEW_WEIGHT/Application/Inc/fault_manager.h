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

//#define ERROR_PRINT(msg)  printf("ERROR: %s | FILE: %s | LINE: %d\r\n", msg, __FILE__, __LINE__)

/* 故障大类枚举 */
typedef enum {
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
typedef enum {
	FAULT_SEVERITY_NONE = 0,      // 无故障
	FAULT_SEVERITY_WARNING,   	  // 警告：不影响核心功能，需要打印故障信息
	FAULT_SEVERITY_ERROR,         // 错误：功能降级/有限次重试
	FAULT_SEVERITY_CRITICAL,      // 严重错误：安全保护，立即停机等待人工干预
	FAULT_SEVERITY_FATAL          // 致命错误：强制系统重启
} FaultSeverity;

/* 故障信息结构体 */
typedef struct {
	uint8_t error_code;        // 自定义错误码
	uint8_t location;          // 故障位置
	FaultSeverity severity;    // 故障等级
	uint8_t retry_count;       // 重试次数
} FaultInfo;
/**
 * @brief 系统错误码定义（2025-10修正版）
 * @note  错误码格式：0x00TT000N
 */
typedef enum {
	/* ==================== 正常状态 ==================== */
	NO_ERROR = 0,                   // 正常状态
	STATE_SWITCH = 1,               // 状态切换（非故障）

	/* ==================== 11电机类故障 (0x000B0000 - 0x000BFFFF) ==================== */
	MOTOR_FAIL_SETTING = 0x000B0001, // 电机设置失败
	MOTOR_UNKNOWN_FEEDBACK = 0x000B0002, // 未知反馈
	MOTOR_RESET_FAIL = 0x000B0003, // 复位失败
	MOTOR_DISABLED = 0x000B0004, // 电机被禁止
	MOTOR_ALARM_TRIGGERED = 0x000B0005, // 电机报警
	MOTOR_STEP_ERROR = 0x000B0006, // 步进数错误
	MOTOR_CHARGE_PUMP_UNDER_VOLTAGE = 0x000B0007, // 电荷泵欠压
	MOTOR_OVERTEMPERATURE = 0x000B0008, // 过温故障
	MOTOR_RUN_TIMEOUT = 0x000B0009, // 电机运行超时

	/* ==================== 12编码器类故障 (0x000C0000 - 0x000CFFFF) ==================== */
	ENCODER_TIMEOUT = 0x000C0001, // 通信超时
	ENCODER_PARITY_ERROR = 0x000C0002, // 偶校验失败
	ENCODER_LOST_STEP = 0x000C0003, // 通信失败
	ENCODER_POWERON_FAIL = 0x000C0004, // 上电初始化失败
	ENCODER_INVALID_DATA = 0x000C0005, // 持续无效数据
	ENCODER_OCF_INCOMPLETE = 0x000C0006, // OCF未完成，数据无效
	ENCODER_CORDIC_OVERFLOW = 0x000C0007, // CORDIC溢出，数据无效
	ENCODER_LINEARITY_WARNING = 0x000C0008, // 线性度报警，数据可能无效
	ENCODER_DIFF_EXCESS = 0x000C0009, // 相邻编码差值变化超限

	/* ==================== 13传感器类故障 (0x000D0000 - 0x000DFFFF) ==================== */
	SENSOR_BCC_ERROR = 0x000D0001, // 数据校验失败
	SENSOR_COMM_TIMEOUT = 0x000D0002, // 通信超时
	SONIC_FREQ_ABNORMAL = 0x000D0003, // 震动管频率异常
	DENSITY_INVALID = 0x000D0004, // 密度值异常
	SENSOR_TEMPERATURE_ERROR = 0x000D0005, // 传感器温度异常
	SENSOR_VOLTAGE_ERROR = 0x000D0006, // 传感器电压异常
	SLIPRING_COMM_FAIL = 0x000D0007, // 无线滑环通信故障
	SLIPRING_BCC_ERROR = 0x000D0008, // 无线滑环BCC校验错误
	SLIPRING_PACKET_LOSS = 0x000D0009, // 无线滑环数据包丢失
	SLIPRING_SIGNAL_WEAK = 0x000D000A, // 无线信号强度不足

	/* ==================== 测量过程类错误 (0x000F0000 - 0x000FFFFF) ==================== */
	MEASUREMENT_POSITION_ERROR = 0x000F0001, // 位置测量错误
	MEASUREMENT_TIMEOUT = 0x000F0002, // 测量超时
	MEASUREMENT_ZERO_OUT_OF_RANGE = 0x000F0003, // 标定零点超限
	MEASUREMENT_OVERSPEED = 0x000F0004, // 液位变化超速
	MEASUREMENT_ZERO_REPEAT_FAIL = 0x000F0005, // 零点重复性差
	MEASUREMENT_HEIGHT_DEVIATION = 0x000F0006, // 实高偏差过大
	MEASUREMENT_OILLEVEL_HIGH = 0x000F0007, // 液位值过高
	MEASUREMENT_OILLEVEL_LOW = 0x000F0008, // 向下未找到液位值
	MEASUREMENT_OILLEVEL_NOTFOUND = 0x000F0009, // 向上未找到液位值
	MEASUREMENT_WEIGHT_DOWN_FAIL = 0x000F000A, // 下行寻重失败
	MEASUREMENT_WEIGHT_UP_FAIL = 0x000F000B, // 上行寻重失败

	/* ==================== 参数/存储类错误 (0x00110000 - 0x0011FFFF) ==================== */
	PARAM_EEPROM_FAIL = 0x00110001, // EEPROM写入失败
	PARAM_UNINITIALIZED = 0x00110002, // 参数未初始化
	PARAM_RANGE_ERROR = 0x00110003, // 参数范围错误
	PARAM_ADDRESS_OVERFLOW = 0x00110004, // 参数地址越界
	PARAM_CRC_ERROR = 0x00110005, // 参数读取CRC校验错误

	/* ==================== 称重类故障 (0x00120000 - 0x0012FFFF) ==================== */
	WEIGHT_OUT_OF_RANGE = 0x00120001, // 称重超范围(上限)
	WEIGHT_UNDER_RANGE = 0x00120002, // 称重超范围(下限)
	WEIGHT_COLLISION_DETECTED = 0x00120003, // 检测到磕碰/障碍
	WEIGHT_DRIFT_ERROR = 0x00120004, // 称重漂移异常
	WEIGHT_SENSOR_SATURATION = 0x00120005, // 称重传感器饱和

	/* ==================== 其他类型错误 (0x00130000 - 0x0013FFFF) ==================== */
	OTHER_UNKNOWN_ERROR = 0x00130001, // 未知故障
	OTHER_ADDRESS_READ_ERROR = 0x00130002, // 地址读取错误
	OTHER_POWER_FLUCTUATION = 0x00130003  // 电源波动异常

} ErrorCode;

//typedef enum {
//    /* 正常状态枚举 */
//    NO_ERROR = 0,               // 正常状态
//    STATE_SWITCH = 1,           // 状态切换
//
//    /* 电机类故障枚举 (0x00110000 - 0x0011FFFF) */
//    MOTOR_FAIL_SETTING = 0X000B0001,    // 电机设置失败
//    MOTOR_UNKNOWN_FEEDBACK = 0X000B0002,// 未知反馈
//    MOTOR_RESET_FAIL = 0X000B0003,      // 复位失败
//    MOTOR_DISABLED = 0X000B0004,        // 电机被禁止
//    MOTOR_ALARM_TRIGGERED = 0X000B0005, // 电机报警
//    MOTOR_STEP_ERROR = 0X000B0006,      // 步进数错误
//	MOTOR_CHARGE_PUMP_UNDER_VOLTAGE = 0X000B0007,     // 欠压故障
//    MOTOR_OVERTEMPERATURE = 0X000B0008, // 过温故障
//
//    /* 编码器类故障枚举 (0x00120000 - 0x0012FFFF) */
//    ENCODER_TIMEOUT = 0X00120001,       // 通信超时
//    ENCODER_PARITY_ERROR = 0X00120002,  // 偶校验失败
//    ENCODER_DATA_LOST = 0X00120003,     // 通信失败
//    ENCODER_POWERON_FAIL = 0X00120004,  // 上电初始化失败
//    ENCODER_INVALID_DATA = 0X00120005,  // 持续无效数据
//    ENCODER_OCF_INCOMPLETE = 0X00120006,// OCF 未完成
//    ENCODER_CORDIC_OVERFLOW = 0X00120007,// CORDIC 溢出
//    ENCODER_LINEARITY_WARNING = 0X00120008,// 线性度报警
//
//    /* 传感器类故障枚举 (0x00130000 - 0x0013FFFF) */
//    SENSOR_BCC_ERROR = 0X00130001,     // 数据校验失败
//    SENSOR_COMM_TIMEOUT = 0X00130002,  // 通信超时
//    SONIC_FREQ_ABNORMAL = 0X00130003,  // 震动管频率异常
//    DENSITY_INVALID = 0X00130004,      // 密度值异常
//    SENSOR_TEMPERATURE_ERROR = 0X00130005,// 传感器温度异常
//    SENSOR_VOLTAGE_ERROR = 0X00130006, // 传感器电压异常
//
//    /* 称重类故障枚举 (0x00140000 - 0x0014FFFF) */
//    WEIGHT_OUT_OF_RANGE = 0X00140001,  // 称重超范围(上限)
//    WEIGHT_UNDER_RANGE = 0X00140002,   // 称重超范围(下限)
//    WEIGHT_COLLISION_DETECTED = 0X00140003,// 检测到磕碰/障碍
//    WEIGHT_DRIFT_ERROR = 0X00140004,   // 称重漂移异常
//    WEIGHT_SENSOR_SATURATION = 0X00140005,// 传感器饱和
//
//    /* 测量过程故障枚举 (0x00150000 - 0x0015FFFF) */
//    MEASUREMENT_POSITION_ERROR = 0X00150001,// 位置测量错误
//    MEASUREMENT_TIMEOUT = 0X00150002,     // 测量超时
//    MEASUREMENT_ZERO_OUT_OF_RANGE = 0X00150003,// 标定零点超限
//    MEASUREMENT_OVERSPEED = 0X00150004,   // 液位变化超速
//    MEASUREMENT_ZERO_REPEAT_FAIL = 0X00150005,// 零点重复性差
//    MEASUREMENT_HEIGHT_DEVIATION = 0X00150006,// 实高偏差过大
//    MEASUREMENT_OILLEVEL_HIGH = 0X00150007,// 液位值过高
//    MEASUREMENT_OILLEVEL_LOW = 0X00150008, // 向下未找到液位
//    MEASUREMENT_OILLEVEL_NOTFOUND = 0X00150009,// 向上未找到液位
//    MEASUREMENT_WEIGHT_DOWN_FAIL = 0X0015000A,// 下行寻重失败
//    MEASUREMENT_WEIGHT_UP_FAIL = 0X0015000B,  // 上行寻重失败
//
//    /* 无线滑环类故障枚举 (0x00160000 - 0x0016FFFF) */
//    SLIPRING_COMM_FAIL = 0X00160001,   // 与从机通讯故障
//    SLIPRING_BCC_ERROR = 0X00160002,   // 通讯过程BCC错误
//    SLIPRING_PACKET_LOSS = 0X00160003, // 数据包丢失率超限
//    SLIPRING_SIGNAL_WEAK = 0X00160004, // 无线信号强度不足
//
//    /* 参数/存储类故障枚举 (0x00170000 - 0x0017FFFF) */
//    PARAM_EEPROM_FAIL = 0X00170001,    // EEPROM写入失败
//    PARAM_UNINITIALIZED = 0X00170002,  // 参数未初始化
//    PARAM_RANGE_ERROR = 0X00170003,    // 参数范围错误
//    PARAM_ADDRESS_OVERFLOW = 0X00170004,// 参数地址越界
//    PARAM_CRC_ERROR = 0X00170005,      // 参数读取CRC错误
//
//    /* 其他故障枚举 (0x00180000 - 0x0018FFFF) */
//    OTHER_UNKNOWN_ERROR = 0X00180001,  // 未知故障
//    OTHER_ADDRESS_READ_ERROR = 0X00180002,// 地址读取错误
//    OTHER_POWER_FLUCTUATION = 0X00180003 // 电源波动异常
//} ErrorCode;
//
// 故障恢复动作定义
typedef enum {
	FAULT_ACTION_NONE,          // 无操作（仅记录日志）
	FAULT_ACTION_RETRY,         // 重试操作（如重新初始化外设）
	FAULT_ACTION_RESET_MODULE,  // 复位模块（如重启通信芯片）
	FAULT_ACTION_SYSTEM_REBOOT  // 系统级复位
} FaultRecoveryAction;

// 故障恢复策略配置
typedef struct {
	FaultSeverity severity;      // 触发等级
	uint8_t max_retries;        // 最大重试次数
	FaultRecoveryAction action;  // 恢复动作
} FaultRecoveryPolicy;
// 错误信息结构体
typedef struct {
	const char *file;
	uint32_t line;
	const char *func;
	uint32_t error_code; // 错误码;
} ErrorInfo;
extern ErrorInfo err; // 全局错误信息变量
//错误信息打印
#define CHECK_ERROR(errorcode)                                                   \
    do {                                                                         \
        /* 填充错误上下文 */                                                    \
        err.file = GetShortFilename(__FILE__);                                   \
        err.line = __LINE__;                                                     \
        err.func = __func__;                                                     \
                                                                                 \
        /* Step 1: 优先检查全局设备错误状态 */                                  \
        if (g_measurement.device_status.error_code != NO_ERROR) {          \
            err.error_code = g_measurement.device_status.error_code;             \
            LogError(&err);                                                      \
            return err.error_code;                                               \
        }                                                                        \
                                                                                 \
        /* Step 2: 检查函数返回错误码 */                                         \
        if ((errorcode) != NO_ERROR) {                                           \
            err.error_code = (errorcode);                                        \
            LogError(&err);                                                      \
            return err.error_code;                                               \
        }                                                                        \
                                                                                 \
        /* Step 3: 检查是否有命令切换 */                                         \
        if (g_deviceParams.command != CMD_NONE) {                                \
            err.error_code = STATE_SWITCH;                                       \
            return err.error_code;                                               \
        }                                                                        \
    } while (0)

#define CHECK_COMMAND_SWITCH(ret)                  \
    do {                                           \
        if(g_deviceParams.command != CMD_NONE){    \
        	printf("检测到命令切换请求，停止当前操作\r\n");\
            return STATE_SWITCH;;                  \
        }                                          \
        if(ret == STATE_SWITCH){                   \
            return STATE_SWITCH;;                  \
        }                                          \
    } while (0)

//#define CHECK_ERROR(errorcode) \
//	do \
//	{\
//		err.file = GetShortFilename(__FILE__);\
//		err.line = __LINE__;\
//		err.func = __func__; \
//		if((g_measurement.device_status.error_code != NO_ERROR) \
//		&&(g_measurement.device_status.error_code != STATE_SWITCH))\
//		{	\
//			err.error_code = (g_measurement.device_status.error_code);\
//			LogError(&err);\
//			return err.error_code;\
//		}\
//		else if ((errorcode) != NO_ERROR) \
//		{\
//			err.error_code = (errorcode); \
//			LogError(&err); \
//			return err.error_code;\
//		}\
//		if(g_deviceParams.command != CMD_NONE) \
//		{	\
//			err.error_code =  STATE_SWITCH;\
//			return err.error_code;\
//		}\
//	} while(0)
////设备报错或命令打断检测

void fault_info_init(void);
uint32_t update_errorcode(uint8_t error_code);
void LogError(const ErrorInfo *err);
const char* GetShortFilename(const char *fullpath);
#endif /* INC_FAULT_MANAGER_H_ */
