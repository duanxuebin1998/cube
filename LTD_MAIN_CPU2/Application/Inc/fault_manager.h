/*
 * fault_manager.h
 *
 *  Created on: Mar 13, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_FAULT_MANAGER_H_
#define INC_FAULT_MANAGER_H_

#include <stdint.h>   /* 处理 uint8_t, uint32_t 等类型 */
#include <string.h>   /* 处理 memset、memcpy 等函数 */
#include "error_log.h"

/* #define ERROR_PRINT(msg) printf("ERROR: %s | FILE: %s | LINE: %d\r\n", msg, __FILE__, __LINE__) */

/* 故障大类枚举 */
typedef enum {
	FAULT_MOTOR = 11,          /* 电机类故障 */
	FAULT_ENCODER,            /* 编码器类故障 */
	FAULT_SENSOR,             /* 传感器类故障 */
	FAULT_WEIGHT,             /* 扭力故障 */
	FAULT_MEASUREMENT,        /* 测量过程类错误 */
	FAULT_WIRELESS_SLIPRING,  /* 无线滑环类故障 */
	FAULT_PARAM_STORAGE,      /* 参数/存储错误 */
	FAULT_OTHER               /* 其他类型错误 */
} FaultCategory;

/* 故障等级（决定处理优先级和恢复策略） */
typedef enum {
	FAULT_SEVERITY_NONE = 0,      /* 无故障 */
	FAULT_SEVERITY_WARNING,   	  /* 警告：不影响核心功能，需要打印故障信息 */
	FAULT_SEVERITY_ERROR,         /* 错误：功能降级/有限次重试 */
	FAULT_SEVERITY_CRITICAL,      /* 严重错误：安全保护，立即停机等待人工干预 */
	FAULT_SEVERITY_FATAL          /* 致命错误：强制系统重启 */
} FaultSeverity;



/* */
/* 故障恢复动作定义 */
typedef enum {
	FAULT_ACTION_NONE,          /* 无操作（仅记录日志） */
	FAULT_ACTION_RETRY,         /* 重试操作（如重新初始化外设） */
	FAULT_ACTION_RESET_MODULE,  /* 复位模块（如重启通信芯片） */
	FAULT_ACTION_SYSTEM_REBOOT  /* 系统级复位 */
} FaultRecoveryAction;

/* 故障恢复策略配置 */
typedef struct {
	FaultSeverity severity;      /* 触发等级 */
	uint8_t max_retries;        /* 最大重试次数 */
	FaultRecoveryAction action;  /* 恢复动作 */
} FaultRecoveryPolicy;
/* 错误信息结构体 */
typedef struct {
	const char *file;
	uint32_t line;
	const char *func;
	uint32_t error_code; /* 错误码; */
} ErrorInfo;
extern ErrorInfo err; /* 全局错误信息变量 */

/**
 * @brief 执行故障处理中的 FaultManager_ReportErrorExit 逻辑。
 *
 * @param error_code 故障或错误码。
 */
void FaultManager_ReportErrorExit(uint32_t error_code);
/**
 * @brief 处理故障处理中的 FaultManager_HandleCheckError 逻辑。
 *
 * @param error_code 故障或错误码。
 * @param file 业务参数。
 * @param line 业务参数。
 * @param func 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t FaultManager_HandleCheckError(uint32_t error_code,
                                       const char *file,
                                       uint32_t line,
                                       const char *func);
/**
 * @brief 处理 CHECK_ERROR 宏捕获到的全局错误状态。
 *
 * @param error_code 全局设备错误码。
 * @param file 捕获全局错误的检查点文件名。
 * @param line 捕获全局错误的检查点行号。
 * @param func 捕获全局错误的检查点函数名。
 * @return 原全局错误码。
 */
uint32_t FaultManager_HandleGlobalError(uint32_t error_code,
                                        const char *file,
                                        uint32_t line,
                                        const char *func);
/**
 * @brief 执行故障处理中的 FaultManager_SetErrorState 逻辑。
 *
 * @param error_code 故障或错误码。
 * @param file 业务参数。
 * @param line 业务参数。
 * @param func 业务参数。
 */
void FaultManager_SetErrorState(uint32_t error_code,
                                const char *file,
                                uint32_t line,
                                const char *func);
/**
 * @brief 处理空闲兜底捕获到的全局错误状态。
 *
 * @param error_code 全局设备错误码。
 * @param file 捕获全局错误的检查点文件名。
 * @param line 捕获全局错误的检查点行号。
 * @param func 捕获全局错误的检查点函数名。
 */
void FaultManager_SetGlobalErrorState(uint32_t error_code,
                                      const char *file,
                                      uint32_t line,
                                      const char *func);

/* 统一错误检查宏，发现错误后进入故障处理出口。 */
#define CHECK_ERROR(errorcode)                                                   \
    do {                                                                         \
        /* Step 1: 优先检查函数返回错误码 */                                      \
        uint32_t check_error_code = (uint32_t)(errorcode);                       \
        if (check_error_code != NO_ERROR) {                                      \
            return FaultManager_HandleCheckError(check_error_code,               \
                                                 GetShortFilename(__FILE__),     \
                                                 __LINE__,                       \
                                                 __func__);                      \
        }                                                                        \
                                                                                 \
        /* Step 2: 检查全局设备错误状态 */                                        \
        if (g_measurement.device_status.error_code != NO_ERROR) {                \
            return FaultManager_HandleGlobalError(                               \
                g_measurement.device_status.error_code,                          \
                GetShortFilename(__FILE__),                                      \
                __LINE__,                                                        \
                __func__);                                                       \
        }                                                                        \
                                                                                 \
        /* Step 3: 检查是否有命令切换 */                                          \
        if (HasEffectiveCommandSwitchRequest()) {                                \
            err.error_code = STATE_SWITCH;                                       \
            HandleError();                                                       \
            return err.error_code;                                               \
        }                                                                        \
    } while (0)


/* 统一错误返回宏，用于带返回值流程的故障退出。 */
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


/* 统一置故障状态宏，记录错误码并触发错误状态。 */
#define SET_ERROR(errorcode)                                                     \
    do {                                                                         \
        uint32_t set_error_code = (uint32_t)(errorcode);                         \
        if ((set_error_code != STATE_SWITCH) && (set_error_code != NO_ERROR)) {  \
            FaultManager_SetErrorState(set_error_code,                           \
                                       GetShortFilename(__FILE__),               \
                                       __LINE__,                                 \
                                       __func__);                                \
            return;                                                              \
        }                                                                        \
    } while (0)


/* 检查命令切换请求，命中时按指定返回值退出当前流程。 */
#define CHECK_COMMAND_SWITCH(ret)                                                \
    do {                                                                         \
        if (HasEffectiveCommandSwitchRequest()) {                                \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            HandleError();                                                       \
            return STATE_SWITCH;                                                 \
        }                                                                        \
        if ((ret) == STATE_SWITCH) {                                             \
            HandleError();                                                       \
            return STATE_SWITCH;                                                 \
        }                                                                        \
    } while (0)

/* 检查命令切换请求，命中时直接退出 void 流程。 */
#define CHECK_COMMAND_SWITCH_NO_RETURN()                                          \
    do {                                                                         \
        if (HasEffectiveCommandSwitchRequest()) {                                \
            printf("检测到命令切换请求，停止当前操作\r\n");                   		    \
            return ;                                                 \
        }                                                                        \
    } while (0)

/**
 * @brief 初始化故障处理中的 fault_info_init 逻辑。
 */
void fault_info_init(void);
/**
 * @brief 处理故障处理中的 HandleError 逻辑。
 */
void HandleError(void);
/**
 * @brief 显示或打印故障处理中的 printError 逻辑。
 *
 * @param err 业务参数。
 */
void printError(const ErrorInfo* err);
/**
 * @brief 读取故障处理中的 GetShortFilename 逻辑。
 *
 * @param fullpath 业务参数。
 * @return 返回业务对象或缓冲区指针，NULL 表示无有效对象。
 */
const char* GetShortFilename(const char *fullpath);
#endif /* INC_FAULT_MANAGER_H_ */
