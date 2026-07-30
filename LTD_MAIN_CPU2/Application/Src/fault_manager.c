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
static uint8_t s_handle_error_skip_logged = 0U; /* 驱动未初始化时 HandleError 的“跳过慢停”提示锁存；同一连续故障只打印一次，驱动恢复有效后清零。 */

/**
 * @brief 在短临界区发布一致的故障快照。
 *
 * @details 调用场景：同步 SET_ERROR、空闲兜底和编码器异步故障锁存。
 * @note 关键约束：先发布状态、实际错误码和回零标志，退出临界区后才允许停机或打印。
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
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

/**
 * @brief 锁存异步故障，供主循环统一停止设备并发布错误状态。
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
 */
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

/**
 * @brief 返回 TMC5130 故障快照中的阶段中文名称。
 *
 * @param stage 外设故障现场的诊断阶段编号；用于区分参数检查、总线访问、寄存器读写、回读核对和设备状态检查等失败位置。
 * @return 返回 TMC5130 故障快照中的阶段中文名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
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

/**
 * @brief 返回 TMC5130 故障快照中的访问方向中文名称。
 *
 * @param direction 外设故障现场中的访问方向枚举；写方向、读方向和未指定方向分别使用对应模块的诊断常量编码。
 * @return 返回 TMC5130 故障快照中的访问方向中文名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
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

/**
 * @brief 把最后一次 TMC5130 故障现场附加到最终错误详情。
 *
 * @details 调用场景：TMC5130 通信异常或配置丢失进入普通或全局最终错误出口时调用。
 * @note 关键约束：只读取已保存快照，不访问 SPI，不覆盖原文件和函数定位信息。
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
 * @param detail 错误或诊断记录使用的详细信息。
 * @param detail_size 详情。
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
 * @brief 从错误信息对象取得错误码并转交统一最终报错出口。
 *
 * @param err 待上报错误信息对象；函数读取其中的错误码，传入 NULL 时直接返回。
 * @note 传入 NULL 时直接返回；统一报错出口会过滤 NO_ERROR 和 STATE_SWITCH，并对短时间内的重复错误码去重，本函数自身不执行停机。
 */
void printError(const ErrorInfo* err)
{
    if (err == NULL) return; /* 空指针保护 */

    FaultManager_ReportErrorExit(err->error_code);
}

/**
 * @brief 统一最终报错出口。
 * @note 只负责打印最终报错日志，STATE_SWITCH 不记录，重复错误码会按短时间窗口去重。
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
 */
void FaultManager_ReportErrorExit(uint32_t error_code)
{
    char detail[320];

    /* NO_ERROR 和命令切换都不是故障，不进入最终报错及停机日志链路。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    /* 同一故障码刚由下层输出过最终报错时跳过重复出口，避免一次故障在相邻调用层连续打印。 */
    if (ErrorLog_TakeRecentReport(error_code) != 0U) {
        return;
    }

    snprintf(detail, sizeof(detail),
             "文件：%s,行号：%lu,函数：%s",
             (err.file != NULL) ? err.file : "未知",
             (unsigned long)err.line,
             (err.func != NULL) ? err.func : "未知");
    FaultManager_AppendTmcDiagnostic(error_code, detail, sizeof(detail));

    ErrorLog_ReportDetail(ErrorLog_GetModuleByCode(error_code),
                          ERROR_LOG_OP_ERROR_EXIT,
                          ErrorLog_GetReasonByCode(error_code),
                          error_code,
                          ERROR_LOG_ACTION_STOP_MEASURE,
                          detail);
}

/**
 * @brief 统一打印全局错误状态的最终报错日志。
 *
 * @details 调用场景：CHECK_ERROR 或主循环空闲兜底捕获到已有全局错误码时调用。
 * @note 关键约束：只说明当前检查点捕获了全局错误，不把错误归因到当前测量返回值。
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
 * @param file 诊断输出关联的源文件名称。
 * @param line 触发当前错误出口或运动保护检查的源代码行号；与 file 和 func 一起保存，用于最终故障日志定位。
 * @param func 触发全局错误处理的源函数名字符串，用于最终故障日志定位。
 */
static void FaultManager_ReportGlobalErrorExit(uint32_t error_code,
                                               const char *file,
                                               uint32_t line,
                                               const char *func)
{
    char detail[384];

    /* 全局错误检查同样忽略正常返回和命令切换，只为真实故障生成现场详情。 */
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    /* 消费短时间内已经输出的同码报错，防止局部出口和全局兜底重复上报。 */
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
 *
 * @param error_code CHECK_ERROR 捕获的原始整机错误码；函数原样记录、上报并返回。
 * @param file 触发 CHECK_ERROR 的源文件名字符串，保存到全局故障上下文供诊断输出使用。
 * @param line 触发 CHECK_ERROR 的源代码行号。
 * @param func 触发 CHECK_ERROR 的函数名字符串，保存到全局故障上下文供诊断输出使用。
 * @return 返回传入并已写入全局故障上下文的原始 error_code；函数不改写错误码，但会先完成最终上报和停机处理。
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
 * @brief 处理 CHECK_ERROR 宏捕获到的全局错误状态。
 *
 * @details 调用场景：函数返回值正常，但 g_measurement.device_status.error_code 已有错误码。
 * @note 关键约束：保持原停机行为，只把日志归因标记为全局错误状态检查点。
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
 * @param file 诊断输出关联的源文件名称。
 * @param line 触发当前错误出口或运动保护检查的源代码行号；与 file 和 func 一起保存，用于最终故障日志定位。
 * @param func 触发 CHECK_ERROR 的源函数名字符串，保存到全局故障上下文并用于日志定位。
 * @return 返回故障管理处理后仍需向调用流程传播的整机错误码；原错误已被消费时返回 NO_ERROR。
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
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
 * @param file 诊断输出关联的源文件名称。
 * @param line 触发当前错误出口或运动保护检查的源代码行号；与 file 和 func 一起保存，用于最终故障日志定位。
 * @param func 触发 SET_ERROR 的源函数名字符串，保存到全局故障上下文并用于日志定位。
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

/**
 * @brief 处理主循环空闲兜底捕获到的全局错误状态。
 *
 * @details 调用场景：没有待执行命令且全局 error_code 已经非零时调用。
 * @note 关键约束：保持原错误态和停机动作，只把最终报错详情标记为全局错误状态来源。
 *
 * @param error_code 待记录、转换或判断的错误码。该值使用整机分模块错误码编码，并由故障出口发布、锁存或附加 TMC5130 现场。
 * @param file 诊断输出关联的源文件名称。
 * @param line 触发当前错误出口或运动保护检查的源代码行号；与 file 和 func 一起保存，用于最终故障日志定位。
 * @param func 触发主循环全局错误兜底的源函数名字符串，用于故障上下文和日志定位。
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
 * @brief 在驱动已经完成初始化时慢速停止电机，并保留原始故障状态。
 *
 * @note 驱动未初始化时不访问 TMC5130，只输出一次跳过提示；慢停失败仅记录警告，不覆盖触发本次处理的原始故障码。
 */
void HandleError(void)
{
    uint32_t ret;

    if (!MotorCtrl_IsDriverInitValid()) {
        /* 驱动尚未初始化时无法执行慢停；等待自动恢复期间只提示一次，避免主循环反复刷同一条跳过日志。 */
        if (s_handle_error_skip_logged == 0U) {
            printf("错误停机跳过 | 电机驱动未初始化，等待自动恢复重新初始化\r\n");
            s_handle_error_skip_logged = 1U;
        }
        return;
    }
    s_handle_error_skip_logged = 0U;

    ret = MotorCtrl_SlowStop();
    /* 错误停机中的慢停失败只补充诊断，不能用新的停机错误覆盖触发 HandleError 的原始故障。 */
    if (ret != NO_ERROR) {
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
