/*
 * fault_manager.h
 *
 *  Created on: Mar 13, 2025
 *      Author: Duan Xuebin
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

extern volatile FaultInfo g_faultInfo;           // 错误信息全局变量

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

#define CHECK_ERROR(errorcode)                                                   \
    do {                                                                         \
        /* Step 1: 优先检查函数返回错误码 */                                      \
        if ((errorcode) != NO_ERROR) {                                           \
            err.error_code = (errorcode);                                        \
            HandleError();                                                       \
            return err.error_code;                                               \
        }                                                                        \
                                                                                 \
        /* Step 2: 检查全局设备错误状态 */                                        \
        if (g_measurement.device_status.error_code != NO_ERROR) {                \
            err.error_code = g_measurement.device_status.error_code;             \
            HandleError();                                                       \
            return err.error_code;                                               \
        }                                                                        \
                                                                                 \
        /* Step 3: 检查是否有命令切换 */                                          \
        if (g_deviceParams.command != CMD_NONE) {                                \
            err.error_code = STATE_SWITCH;                                       \
            HandleError();                                                       \
            return err.error_code;                                               \
        }                                                                        \
    } while (0)


#define RETURN_ERROR(errorcode)                                                  \
    do {                                                                         \
        if ((errorcode) != NO_ERROR) {                                           \
            err.file       = GetShortFilename(__FILE__);                         \
            err.line       = __LINE__;                                           \
            err.func       = __func__;                                           \
            err.error_code = (errorcode);                                        \
                                                                                 \
            HandleError();        /* 停机 / 报警 / 记录到全局状态等 */            \
            printError(&err);     /* 串口/日志输出可读信息 */                     \
                                                                                 \
            return err.error_code;                                               \
        }                                                                        \
    } while (0)


#define SET_ERROR(errorcode)                                                     \
    do {                                                                         \
        if (((errorcode) != STATE_SWITCH) && ((errorcode) != NO_ERROR)) {        \
            HandleError();                                                       \
            g_measurement.device_status.device_state       = STATE_ERROR;        \
            g_measurement.device_status.error_code         = errorcode;          \
            g_measurement.device_status.zero_point_status  = 1;                  \
            return;                                                              \
        }                                                                        \
    } while (0)


#define CHECK_COMMAND_SWITCH(ret)                                                \
    do {                                                                         \
        if (g_deviceParams.command != CMD_NONE) {                                \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            return STATE_SWITCH;                                                 \
        }                                                                        \
        if ((ret) == STATE_SWITCH) {                                             \
            return STATE_SWITCH;                                                 \
        }                                                                        \
    } while (0)

#define CHECK_COMMAND_SWITCH_NO_RETURN()                                          \
    do {                                                                         \
        if (g_deviceParams.command != CMD_NONE) {                                \
            printf("检测到命令切换请求，停止当前操作\r\n");                   		    \
            return ;                                                 \
        }                                                                        \
    } while (0)

void fault_info_init(void);
uint32_t update_errorcode(uint8_t error_code);
void HandleError(void);
void printError(const ErrorInfo* err);
const char* GetShortFilename(const char *fullpath);
#endif /* INC_FAULT_MANAGER_H_ */
