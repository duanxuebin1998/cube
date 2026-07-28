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
#include "TMC5130.h"
#include "error_log.h"
#include "encoder.h"
#include "power_monitor.h"

ErrorInfo err; /* 全局错误信息变量 */
static uint8_t s_handle_error_skip_logged = 0U; /* 故障处理故障记录，供恢复、显示或日志链路使用。 */

/*
 * 函数用途：在短临界区发布一致的故障快照。
 * 调用场景：同步 SET_ERROR、空闲兜底和编码器异步故障锁存。
 * 关键约束：先发布状态、实际错误码和回零标志，退出临界区后才允许停机或打印。
 */
static void FaultManager_PublishErrorSnapshot(uint32_t error_code)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_measurement.device_status.device_state = STATE_ERROR;
    g_measurement.device_status.error_code = error_code;
    g_measurement.device_status.zero_point_status = 1U;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

void FaultManager_LatchAsyncError(uint32_t error_code)
{
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }
    FaultManager_PublishErrorSnapshot(error_code);
    /*
     * 异步入口不能执行慢停，但必须在快照发布后立即拉高ENN，
     * 避免测量流程下一次CHECK_ERROR之前电机继续运动。
     */
    stpr_disableDriver(&stepper);
}

/* 返回 TMC5130 故障快照中的阶段中文名称。 */
static const char *FaultManager_GetTmcStageText(uint8_t stage)
{
    switch (stage) {
    case TMC5130_DIAG_STAGE_PARAMETER:
        return "参数无效";
    case TMC5130_DIAG_STAGE_ACCESS_BUSY:
        return "SPI访问冲突";
    case TMC5130_DIAG_STAGE_SPI_READ_TRIGGER:
        return "寄存器读触发帧";
    case TMC5130_DIAG_STAGE_SPI_READ_DATA:
        return "寄存器读数据帧";
    case TMC5130_DIAG_STAGE_SPI_WRITE:
        return "寄存器写入";
    case TMC5130_DIAG_STAGE_XACTUAL_UNSTABLE:
        return "位置连续读数不稳定";
    case TMC5130_DIAG_STAGE_CONFIGURATION_LOST:
        return "驱动配置丢失";
    case TMC5130_DIAG_STAGE_CHIP_RESET:
        return "驱动芯片复位";
    default:
        return "未记录";
    }
}

/* 返回 TMC5130 故障快照中的访问方向中文名称。 */
static const char *FaultManager_GetTmcDirectionText(uint8_t direction)
{
    if (direction == TMC5130_DIAG_DIRECTION_WRITE) {
        return "写";
    }
    if (direction == TMC5130_DIAG_DIRECTION_READ) {
        return "读";
    }
    return "无";
}

/*
 * 函数用途：把最后一次 TMC5130 故障现场附加到最终错误详情。
 * 调用场景：TMC5130 通信异常或配置丢失进入普通或全局最终错误出口时调用。
 * 关键约束：只读取已保存快照，不访问 SPI，不覆盖原文件和函数定位信息。
 */
static void FaultManager_AppendTmcDiagnostic(uint32_t error_code,
                                             char *detail,
                                             size_t detail_size)
{
    TMC5130DiagnosticSnapshot snapshot;
    size_t used;

    if (((error_code != MOTOR_TMC_COMM_ERROR) &&
         (error_code != MOTOR_TMC_CONFIG_LOST)) ||
        (detail == NULL) ||
        (detail_size == 0U)) {
        return;
    }

    TMC5130_GetDiagnosticSnapshot(&snapshot);
    if (snapshot.sequence == 0U) {
        return;
    }

    used = strlen(detail);
    if (used >= detail_size) {
        return;
    }

    (void)snprintf(detail + used,
                   detail_size - used,
                   ",TMC阶段：%s,方向：%s,寄存器：0x%02X,底层状态：%lu,响应状态：0x%02X,快照错误码：%lu,期望值：0x%08lX,实际值：0x%08lX,连续读数：%ld/%ld/%ld",
                   FaultManager_GetTmcStageText(snapshot.stage),
                   FaultManager_GetTmcDirectionText(snapshot.direction),
                   (unsigned int)snapshot.address,
                   (unsigned long)snapshot.hal_status,
                   (unsigned int)snapshot.response_status,
                   (unsigned long)snapshot.error_code,
                   (unsigned long)snapshot.expected_value,
                   (unsigned long)snapshot.actual_value,
                   (long)snapshot.sample_first,
                   (long)snapshot.sample_second,
                   (long)snapshot.sample_third);
}

/**
 * @brief 错误打印函数
 * @param err 错误信息结构体指针
 * @note STATE_SWITCH 只停机不记录
 */
void printError(const ErrorInfo* err)
{
    if (err == NULL) return; /* 空指针保护 */

    FaultManager_ReportErrorExit(err->error_code);
}

/**
 * @brief 统一最终报错出口。
 * @note 只负责打印最终报错日志，STATE_SWITCH 不记录，重复错误码会按短时间窗口去重。
 */
void FaultManager_ReportErrorExit(uint32_t error_code)
{
    char detail[320];

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (ErrorLog_TakeRecentReport(error_code) != 0U) {
        return;
    }

    snprintf(detail, sizeof(detail),
             "文件：%s,行号：%lu,函数：%s",
             (err.file != NULL) ? err.file : "未知",
             (unsigned long)err.line,
             (err.func != NULL) ? err.func : "未知");
    FaultManager_AppendTmcDiagnostic(error_code, detail, sizeof(detail));

    /* 错误 阶段：最终报错 模块：ErrorLog_GetModuleByCode(error_code) 操作：错误出口 原因：ErrorLog_GetReasonByCode(error_code) 错误码：error_code 错误名：ErrorLog_GetCodeName(error_code) 处理：停止测量 详情：detail */
    ErrorLog_ReportDetail(ErrorLog_GetModuleByCode(error_code),
                          ERROR_LOG_OP_ERROR_EXIT,
                          ErrorLog_GetReasonByCode(error_code),
                          error_code,
                          ERROR_LOG_ACTION_STOP_MEASURE,
                          detail);
}

/*
 * 函数用途：统一打印全局错误状态的最终报错日志。
 * 调用场景：CHECK_ERROR 或主循环空闲兜底捕获到已有全局错误码时调用。
 * 关键约束：只说明当前检查点捕获了全局错误，不把错误归因到当前测量返回值。
 */
static void FaultManager_ReportGlobalErrorExit(uint32_t error_code,
                                               const char *file,
                                               uint32_t line,
                                               const char *func)
{
    char detail[384];

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (ErrorLog_TakeRecentReport(error_code) != 0U) {
        return;
    }

    snprintf(detail,
             sizeof(detail),
             "来源：全局错误状态,触发检查点：%s,行号：%lu,函数：%s,当前命令：%lu,当前状态：0x%04lX",
             (file != NULL) ? file : "未知",
             (unsigned long)line,
             (func != NULL) ? func : "未知",
             (unsigned long)g_measurement.device_status.current_command,
             (unsigned long)g_measurement.device_status.device_state);
    FaultManager_AppendTmcDiagnostic(error_code, detail, sizeof(detail));

    /* 错误 阶段：最终报错 模块：ErrorLog_GetModuleByCode(error_code) 操作：检查全局错误状态 原因：ErrorLog_GetReasonByCode(error_code) 错误码：error_code 错误名：ErrorLog_GetCodeName(error_code) 处理：停止测量 详情：detail */
    ErrorLog_ReportDetail(ErrorLog_GetModuleByCode(error_code),
                          "检查全局错误状态",
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

/*
 * 函数用途：处理 CHECK_ERROR 宏捕获到的全局错误状态。
 * 调用场景：函数返回值正常，但 g_measurement.device_status.error_code 已有错误码。
 * 关键约束：保持原停机行为，只把日志归因标记为全局错误状态检查点。
 */
uint32_t FaultManager_HandleGlobalError(uint32_t error_code,
                                        const char *file,
                                        uint32_t line,
                                        const char *func)
{
    err.file = file;
    err.line = line;
    err.func = func;
    err.error_code = error_code;

    FaultManager_ReportGlobalErrorExit(err.error_code, file, line, func);
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

    FaultManager_PublishErrorSnapshot(error_code);
    FaultManager_ReportErrorExit(err.error_code);
    HandleError();
}

/*
 * 函数用途：处理主循环空闲兜底捕获到的全局错误状态。
 * 调用场景：没有待执行命令且全局 error_code 已经非零时调用。
 * 关键约束：保持原错误态和停机动作，只把最终报错详情标记为全局错误状态来源。
 */
void FaultManager_SetGlobalErrorState(uint32_t error_code,
                                      const char *file,
                                      uint32_t line,
                                      const char *func)
{
    err.file = file;
    err.line = line;
    err.func = func;
    err.error_code = error_code;

    FaultManager_PublishErrorSnapshot(error_code);
    FaultManager_ReportGlobalErrorExit(err.error_code, file, line, func);
    HandleError();
}
/**
 * @brief 错误处理函数
 * @note 任何错误都必须立即停机
 */
void HandleError(void)
{
    uint32_t ret;

    if (!MotorCtrl_IsDriverInitValid()) {
        /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
        if (s_handle_error_skip_logged == 0U) {
            printf("错误停机跳过 | 电机驱动未初始化，等待自动恢复重新初始化\r\n");
            s_handle_error_skip_logged = 1U;
        }
        return;
    }
    s_handle_error_skip_logged = 0U;

    ret = MotorCtrl_SlowStop();
    /* 先处理异常边界，避免故障处理状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 错误 阶段：错误报警 模块：电机 操作：停止电机 原因：ErrorLog_GetReasonByCode(ret) 处理：保持原故障 */
        ErrorLog_Warn(ERROR_LOG_MODULE_MOTOR,
                      "停止电机",
                      ErrorLog_GetReasonByCode(ret),
                      "保持原故障");
    }
}

/* 旧的错误记录函数示例（保留注释备查）
void LogError(const ErrorInfo* err) {
    MotorCtrl_SlowStop(); / / 慢速停止电机
    if (g_measurement.device_status.error_code == NO_ERROR &&
        g_measurement.device_status.error_code != STATE_SWITCH &&
        err->error_code != STATE_SWITCH) {

        g_measurement.device_status.error_code = err->error_code; / / 更新错误码
        g_measurement.device_status.zero_point_status = 1;        / / 设置零点状态为需要回零点

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

/* 全局故障信息结构体 */

/**
 * @brief 故障信息初始化函数（系统启动时调用）
 */
void fault_info_init(void)
{
    MotorCtrl_SlowStop(); /* 初始化时确保电机停止 */
    /*
     * 编码器异步故障只能由新的顶层正式过程先解除锁存。
     * 粗找、精找和回零内部重试即使调用本函数，也不能越权清除。
     */
    if ((!Encoder_HasLatchedFault()) &&
        (!PowerMonitor_HasLatchedFault())) {
        g_measurement.device_status.error_code = NO_ERROR;
    }
}
