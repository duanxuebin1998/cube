/*
 * fault_manager.c
 *
 *  Created on: Mar 13, 2025
 *      Author: Duan Xuebin
 */
#include "fault_manager.h"
#include "system_parameter.h"
#include <stdio.h>
#include <string.h>
#include "motor_ctrl.h"
#include "error_log.h"

ErrorInfo err; // 全局错误信息变量

/**
 * @brief 错误打印函数
 * @param err 错误信息结构体指针
 * @note STATE_SWITCH 只停机不记录
 */
void printError(const ErrorInfo* err)
{
    if (err == NULL) return; // 空指针保护

    FaultManager_ReportErrorExit(err->error_code);
}

/**
 * @brief 统一最终报错出口。
 * @note 只负责打印最终报错日志，STATE_SWITCH 不记录，重复错误码会按短时间窗口去重。
 */
void FaultManager_ReportErrorExit(uint32_t error_code)
{
    char detail[96];

    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    if (ErrorLog_TakeRecentReport(error_code) != 0U) {
        return;
    }

    snprintf(detail, sizeof(detail),
             "文件：%s,行号：%lu,函数：%s",
             (err.file != NULL) ? err.file : "未知",
             (unsigned long)err.line,
             (err.func != NULL) ? err.func : "未知");

    // 错误	阶段：最终报错	模块：ErrorLog_GetModuleByCode(error_code)	操作：错误出口	原因：ErrorLog_GetReasonByCode(error_code)	错误码：error_code	错误名：ErrorLog_GetCodeName(error_code)	处理：停止测量	详情：detail
    ErrorLog_ReportDetail(ErrorLog_GetModuleByCode(error_code),
                          ERROR_LOG_OP_ERROR_EXIT,
                          ErrorLog_GetReasonByCode(error_code),
                          error_code,
                          ERROR_LOG_ACTION_STOP_MEASURE,
                          detail);
}

/**
 * @brief CHECK_ERROR 宏的统一处理入口。
 * @note 记录文件、行号和函数名后输出最终报错，并执行停机处理。
 */
uint32_t FaultManager_HandleCheckError(uint32_t error_code,
                                       const char *file,
                                       uint32_t line,
                                       const char *func)
{
    err.file = file;
    err.line = line;
    err.func = func;
    err.error_code = error_code;

    FaultManager_ReportErrorExit(err.error_code);
    HandleError();
    return err.error_code;
}

/**
 * @brief SET_ERROR 宏的统一处理入口。
 * @note 输出最终报错后把设备状态切换为错误态，并标记后续需要回零。
 */
void FaultManager_SetErrorState(uint32_t error_code,
                                const char *file,
                                uint32_t line,
                                const char *func)
{
    err.file = file;
    err.line = line;
    err.func = func;
    err.error_code = error_code;

    FaultManager_ReportErrorExit(err.error_code);
    HandleError();
    g_measurement.device_status.device_state = STATE_ERROR;
    g_measurement.device_status.error_code = error_code;
    g_measurement.device_status.zero_point_status = 1;
}
/**
 * @brief 错误处理函数
 * @note 任何错误都必须立即停机
 */
void HandleError(void)
{
    uint32_t ret;

    ret = MotorCtrl_SlowStop();
    if (ret != NO_ERROR) {
        // 错误	阶段：错误报警	模块：电机	操作：停止电机	原因：ErrorLog_GetReasonByCode(ret)	处理：保持原故障
        ErrorLog_Warn(ERROR_LOG_MODULE_MOTOR,
                      "停止电机",
                      ErrorLog_GetReasonByCode(ret),
                      "保持原故障");
    }
}

/* 旧的错误记录函数示例（保留注释备查）
void LogError(const ErrorInfo* err) {
    MotorCtrl_SlowStop(); // 慢速停止电机
    if (g_measurement.device_status.error_code == NO_ERROR &&
        g_measurement.device_status.error_code != STATE_SWITCH &&
        err->error_code != STATE_SWITCH) {

        g_measurement.device_status.error_code = err->error_code; // 更新错误码
        g_measurement.device_status.zero_point_status = 1;        // 设置零点状态为需要回零点

        printf("故障 代码: 0x%X | 文件: %s | 行号: %lu | 函数: %s \r\n",
               err->error_code, err->file, err->line, err->func);
    }
}
*/

/**
 * @brief 提取短文件名 (从路径中提取)
 * @param fullpath 完整路径
 * @return 指向短文件名的指针
 */
const char* GetShortFilename(const char* fullpath) {
    const char* p = fullpath + strlen(fullpath);
    while (p > fullpath) {
        if (*(p - 1) == '/' || *(p - 1) == '\\') break;
        p--;
    }
    return p;
}

// 全局故障信息结构体
volatile FaultInfo g_faultInfo = { .severity = FAULT_SEVERITY_NONE };

/**
 * @brief 故障信息初始化函数（系统启动时调用）
 */
void fault_info_init(void) {
    MotorCtrl_SlowStop(); // 初始化时确保电机停止
    g_measurement.device_status.error_code = NO_ERROR; // 清除设备状态错误码
}

/**
 * @brief 更新错误码并返回用于寄存器或日志的编码值
 * @param error_code 错误码
 * @return 32位故障寄存器值（位置8位 + 错误码8位）
 */
uint32_t update_errorcode(uint8_t error_code) {
    // 更新全局故障信息
    g_faultInfo.error_code = error_code;

    // 实时寄存器编码：高8位为 location，低8位为 error_code
    uint32_t register_value =
        ((uint32_t)(g_faultInfo.location & 0xFF) << 8) |
        (uint32_t)(g_faultInfo.error_code & 0xFF);

    return register_value;
}

/**
 * @brief 带阈值的故障状态检测函数
 * @param threshold 故障严重等级阈值
 * @return 1 表示存在大于等于阈值的故障；0 表示无
 *
 * 故障等级映射规则：
 * 1. 当系统存在更高或等于阈值的故障时返回 1
 * 2. 故障等级按严重程度递增排列（4 > 3 > 2 > 1 > 0）
 * 3. 支持动态阈值配置（1-4级）
 */
uint8_t check_fault_with_threshold(FaultSeverity threshold) {
    // 原子操作读取当前严重级别（防止中断干扰）
    FaultSeverity current_severity = g_faultInfo.severity;

    return (current_severity >= threshold) ? 1 : 0;
}
