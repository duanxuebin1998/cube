/*
 * 模块职责：维护V4探测包、最近交互事务、通信质量和最近异常主动帧的诊断状态。
 * 上下文约束：记录函数只更新内存；格式化打印只能从命令或主循环线程态调用，
 * UART中断、定时器中断和PendSV路径均不得直接printf。
 * 报警边界：连续三个异常包只生成待处理错误，由主循环统一取走并进入全局错误流程。
 */
#include "multiparam_v4_internal.h"

#include "error_log.h"
#include "main.h"
#include "my_crc.h"
#include "sensor_transport_config.h"
#include "sensor_uart6_owner.h"
#include "system_parameter.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t s_identification_probe_mode = 0U;

void MULTIPARAM_V4_SetIdentificationProbeMode(uint8_t enabled)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_identification_probe_mode = (enabled != 0U) ? 1U : 0U;
    s_quality_alarm_latched = 0U;
    s_pending_quality_error = NO_ERROR;
    s_diagnostics.consecutive_abnormal_frames = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：输出探测阶段单个V4原始包的十六进制内容。
 * 调用场景：传感器识别任务开启跟踪后，8字节交互事务实际发送或退出时。
 * 关键约束：只能在任务上下文调用，禁止从UART中断、定时器中断或PendSV解析路径打印。
 */
void MULTIPARAM_V4_PrintProbePacket(const uint8_t request[8],
                                           const char *direction,
                                           const uint8_t *bytes,
                                           uint16_t length,
                                           uint32_t result,
                                           const char *stage)
{
    if ((s_probe_trace_enabled == 0U) || (request == NULL) ||
        (direction == NULL) || (bytes == NULL)) {
        return;
    }

    printf("探测包\t协议=V4\t操作=%c%02u\t序号=%lu\t方向=%s\t长度=%u\tHEX=",
           (char)request[1],
           (unsigned int)request[6],
           (unsigned long)s_probe_trace_transaction,
           direction,
           (unsigned int)length);
    for (uint16_t index = 0U; index < length; index++) {
        printf((index == 0U) ? "%02X" : " %02X", bytes[index]);
    }
    if (direction[0] == 'R') {
        printf("\t结果=0x%08lX\t阶段=%s",
               (unsigned long)result,
               (stage != NULL) ? stage : "未确定");
    }
    printf("\r\n");
}

/*
 * 函数用途：控制V4交互探测原始包跟踪。
 * 调用场景：自动识别进入和离开R01、R67及其通信方式切换窗口时。
 * 关键约束：关闭时不影响任何正常测量事务；每次开启重新从序号1计数。
 */
void MULTIPARAM_V4_SetProbeTraceEnabled(uint8_t enabled)
{
    s_probe_trace_enabled = (enabled != 0U) ? 1U : 0U;
    if (s_probe_trace_enabled != 0U) {
        s_probe_trace_transaction = 0U;
    }
}

/*
 * 函数用途：对V4累计诊断计数执行无回卷的饱和加法。
 * 调用场景：统计丢包、异常帧和通信事务计数时。
 * 关键约束：溢出时固定为UINT32_MAX，不能回卷为较小值误导通信质量判断。
 */
uint32_t MULTIPARAM_V4_SaturatingAdd(uint32_t value, uint32_t increment)
{
    return (value > (UINT32_MAX - increment)) ? UINT32_MAX : (value + increment);
}

/*
 * 函数用途：记录主动流异常、保存最近异常原包并产生连续3包故障请求。
 * 调用场景：PendSV帧解析、停流窗口主动帧抢占和主动流超时确认。
 * 关键约束：只更新内存状态，不打印；达到门限后挂起PendSV，由异步故障入口立即锁存。
 */
void MULTIPARAM_V4_RecordQualityAnomaly(uint32_t error_code,
                                               uint32_t count,
                                               const uint8_t *data,
                                               uint16_t length)
{
    uint32_t previous;
    uint16_t capture_length = length;

    if ((error_code == NO_ERROR) || (count == 0U)) {
        return;
    }
    if (capture_length > MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE) {
        capture_length = MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE;
    }

    memset(&s_last_abnormal_frame, 0, sizeof(s_last_abnormal_frame));
    s_last_abnormal_frame.valid = 1U;
    s_last_abnormal_frame.length = capture_length;
    s_last_abnormal_frame.error_code = error_code;
    s_last_abnormal_frame.received_tick = HAL_GetTick();
    if ((data != NULL) && (capture_length != 0U)) {
        memcpy(s_last_abnormal_frame.data, data, capture_length);
    }

    previous = s_diagnostics.consecutive_abnormal_frames;
    s_diagnostics.abnormal_frames =
        MULTIPARAM_V4_SaturatingAdd(s_diagnostics.abnormal_frames, count);
    s_diagnostics.consecutive_abnormal_frames =
        MULTIPARAM_V4_SaturatingAdd(previous, count);
    if (s_diagnostics.consecutive_abnormal_frames >
        s_diagnostics.max_consecutive_abnormal_frames) {
        s_diagnostics.max_consecutive_abnormal_frames =
            s_diagnostics.consecutive_abnormal_frames;
    }
    if ((s_identification_probe_mode == 0U) &&
        (s_quality_alarm_latched == 0U) &&
        (previous < MULTIPARAM_V4_CONSECUTIVE_ERROR_LIMIT) &&
        (s_diagnostics.consecutive_abnormal_frames >=
         MULTIPARAM_V4_CONSECUTIVE_ERROR_LIMIT)) {
        s_quality_alarm_latched = 1U;
        s_diagnostics.quality_alarm_events = MULTIPARAM_V4_SaturatingAdd(
            s_diagnostics.quality_alarm_events, 1U);
        if (s_pending_quality_error == NO_ERROR) {
            s_pending_quality_error = error_code;
        }
        /* 质量门限可能在线程态事务中达到，显式挂起PendSV保证故障锁存不依赖主循环。 */
        MULTIPARAM_V4_RequestDeferredFromISR();
    }
}

/* 有效且序号连续的新帧结束当前连续异常段，但不撤销已经待上报的故障。 */
/*
 * 函数用途：记录一次成功主动帧并清零连续异常计数。
 * 调用场景：主动帧通过完整校验且快照成功发布后。
 * 关键约束：只更新诊断内存，不清累计异常、不打印，也不撤销已经挂起的质量错误。
 */
void MULTIPARAM_V4_RecordQualitySuccess(void)
{
    s_diagnostics.consecutive_abnormal_frames = 0U;
    s_quality_alarm_latched = 0U;
}

/*
 * 函数用途：在任务上下文输出最近一次有效V4主动帧或本次监听的空接收结果。
 * 调用场景：自动识别的两段主动帧监听窗口退出时。
 * 关键约束：原帧在关中断区内复制，printf仅在恢复中断后执行。
 */
void MULTIPARAM_V4_PrintActiveProbePacket(const char *operation, uint32_t result)
{
    uint8_t frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE];
    uint16_t length = 0U;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((result == NO_ERROR) && (s_latest_active_frame_valid != 0U)) {
        memcpy(frame, s_latest_active_frame, sizeof(frame));
        length = MULTIPARAM_V4_ACTIVE_FRAME_SIZE;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    printf("探测包\t协议=V4\t操作=%s\t方向=RX\t长度=%u\tHEX=",
           (operation != NULL) ? operation : "主动帧",
           (unsigned int)length);
    for (uint16_t index = 0U; index < length; index++) {
        printf((index == 0U) ? "%02X" : " %02X", frame[index]);
    }
    printf("\t结果=0x%08lX\r\n", (unsigned long)result);
}

/*
 * 函数用途：原子复制当前V4累计通信质量计数。
 * 调用场景：串口V4质量查询和维护诊断输出。
 * 关键约束：仅短临界区复制，不清零计数，不在中断中格式化打印。
 */
void MULTIPARAM_V4_GetDiagnostics(multiparam_v4_diagnostics_t *diagnostics)
{
    uint32_t primask;

    if (diagnostics == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(diagnostics, &s_diagnostics, sizeof(*diagnostics));
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：原子复制最近一次主动流异常码及原始包。
 * 调用场景：串口打印“上一个异常包”时。
 * 关键约束：无记录时返回valid为0；读取不消费记录，便于重复诊断。
 */
void MULTIPARAM_V4_GetLastAbnormalFrame(multiparam_v4_abnormal_frame_t *frame)
{
    uint32_t primask;

    if (frame == NULL) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(frame, &s_last_abnormal_frame, sizeof(*frame));
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：在线程态取走连续三个异常包产生的待处理错误码。
 * 调用场景：PendSV完成主动帧解析或超时确认之后。
 * 关键约束：读取后原子清零，确保同一质量事件只进入一次全局错误流程。
 */
uint32_t MULTIPARAM_V4_TakeQualityError(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t error_code;

    __disable_irq();
    error_code = s_pending_quality_error;
    s_pending_quality_error = NO_ERROR;
    if (primask == 0U) {
        __enable_irq();
    }
    return error_code;
}

/*
 * 函数用途：清零V4累计通信诊断，建立新的现场观测窗口。
 * 调用场景：串口V4C命令执行后，再用V4G比较后续主动帧和交互事务质量。
 * 关键约束：保留通信方式、DMA接收状态、最近快照、序号基线和当前超时锁存。
 */
void MULTIPARAM_V4_ClearDiagnostics(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    memset(&s_diagnostics, 0, sizeof(s_diagnostics));
    memset(&s_last_abnormal_frame, 0, sizeof(s_last_abnormal_frame));
    s_quality_alarm_latched = 0U;
    s_pending_quality_error = NO_ERROR;
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：开始记录一次8字节交互事务的诊断上下文。
 * 调用场景：普通交互事务完成 UART 清理、准备启动 DMA 时。
 * 关键约束：只在主循环事务上下文写入，不在 UART 中断中打印。
 */
void MULTIPARAM_V4_BeginTransactionDiagnostic(const uint8_t request[8])
{
    memset(&s_last_transaction_diagnostic, 0, sizeof(s_last_transaction_diagnostic));
    if (request != NULL) {
        memcpy(s_last_transaction_diagnostic.request, request, 8U);
        s_last_transaction_diagnostic.valid = 1U;
    }
}

/*
 * 函数用途：保存一次8字节交互事务退出时的原始响应和硬件状态。
 * 调用场景：事务成功、短帧、UART错误、超时或命令切换的统一退出点。
 * 关键约束：必须在停止 DMA 前调用，避免 HAL 收尾覆盖故障现场。
 */
void MULTIPARAM_V4_RecordTransactionDiagnostic(const uint8_t reply[8],
                                                       uint16_t received_length,
                                                       uint32_t start_tick,
                                                       const char *stage)
{
    uint16_t copy_length = received_length;

    if (copy_length > MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE) {
        copy_length = MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE;
    }
    memset(s_last_transaction_diagnostic.reply, 0,
           sizeof(s_last_transaction_diagnostic.reply));
    if ((reply != NULL) && (copy_length > 0U)) {
        memcpy(s_last_transaction_diagnostic.reply, reply, copy_length);
    }
    s_last_transaction_diagnostic.received_length = copy_length;
    s_last_transaction_diagnostic.elapsed_ms = HAL_GetTick() - start_tick;
    s_last_transaction_diagnostic.uart_error = huart6.ErrorCode;
    s_last_transaction_diagnostic.uart_state = (uint32_t)huart6.gState;
    s_last_transaction_diagnostic.stage = (stage != NULL) ? stage : "未确定";
}

/*
 * 函数用途：把实际收到的交互响应转换为定长十六进制文字。
 * 调用场景：R01 诊断日志需要保留原始总线内容时。
 * 关键约束：最多输出固定8字节，不把未接收到的缓冲区零值伪装成响应。
 */
static void MULTIPARAM_V4_FormatReplyBytes(char *text, size_t text_size)
{
    size_t used = 0U;

    if ((text == NULL) || (text_size == 0U)) {
        return;
    }
    text[0] = '\0';
    for (uint16_t index = 0U;
         index < s_last_transaction_diagnostic.received_length;
         index++) {
        int written = snprintf(text + used, text_size - used,
                               (index == 0U) ? "%02X" : " %02X",
                               s_last_transaction_diagnostic.reply[index]);
        if ((written < 0) || ((size_t)written >= (text_size - used))) {
            text[text_size - 1U] = '\0';
            break;
        }
        used += (size_t)written;
    }
}

/*
 * 函数用途：按指定场景打印最近一次V4交互事务的请求包和实际接收包。
 * 调用场景：部件参数或测量异常需要保留现场原始收发内容时。
 * 关键约束：只在任务上下文调用；帧校验通过后发生的值域错误要明确标记为业务校验失败。
 */
static void MULTIPARAM_V4_PrintLastTransactionPacketsForScene(
    const char *scene,
    const char *operation,
    uint32_t result,
    uint32_t attempt,
    uint32_t max_attempts)
{
    const char *stage = s_last_transaction_diagnostic.stage;

    if (s_last_transaction_diagnostic.valid == 0U) {
        return;
    }
    if (scene == NULL) {
        scene = "未指定";
    }
    if (operation == NULL) {
        operation = "未指定";
    }
    if (stage == NULL) {
        stage = "未确定";
    } else if ((result != NO_ERROR) && (strcmp(stage, "校验通过") == 0)) {
        stage = "帧校验通过，业务校验失败";
    }

    printf("通信包\t协议=V4\t场景=%s\t操作=%s\t指令=%c%02u",
           scene,
           operation,
           (char)s_last_transaction_diagnostic.request[1],
           (unsigned int)s_last_transaction_diagnostic.request[6]);
    if (max_attempts != 0U) {
        printf("\t尝试=%lu/%lu",
               (unsigned long)attempt,
               (unsigned long)max_attempts);
    }
    printf("\t方向=TX\t长度=8\tHEX=");
    for (uint16_t index = 0U; index < MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE; index++) {
        printf((index == 0U) ? "%02X" : " %02X",
               s_last_transaction_diagnostic.request[index]);
    }
    printf("\r\n");

    printf("通信包\t协议=V4\t场景=%s\t操作=%s\t指令=%c%02u",
           scene,
           operation,
           (char)s_last_transaction_diagnostic.request[1],
           (unsigned int)s_last_transaction_diagnostic.request[6]);
    if (max_attempts != 0U) {
        printf("\t尝试=%lu/%lu",
               (unsigned long)attempt,
               (unsigned long)max_attempts);
    }
    printf("\t方向=RX\t长度=%u\tHEX=",
           (unsigned int)s_last_transaction_diagnostic.received_length);
    for (uint16_t index = 0U;
         index < s_last_transaction_diagnostic.received_length;
         index++) {
        printf((index == 0U) ? "%02X" : " %02X",
               s_last_transaction_diagnostic.reply[index]);
    }
    printf("\t结果=0x%08lX\t阶段=%s\t耗时=%lu ms"
           "\tUART状态=0x%08lX\tUART错误=0x%08lX\r\n",
           (unsigned long)result,
           stage,
           (unsigned long)s_last_transaction_diagnostic.elapsed_ms,
           (unsigned long)s_last_transaction_diagnostic.uart_state,
           (unsigned long)s_last_transaction_diagnostic.uart_error);
    s_last_transaction_diagnostic.failure_packets_printed = 1U;
}

/*
 * 函数用途：打印V4幂等事务某一次失败的原始收发包。
 * 调用场景：协议事务确认失败可重试后、开始下一次发送之前。
 * 关键约束：只在线程态打印；操作名由实际请求功能码和参数号生成。
 */
void MULTIPARAM_V4_PrintTransactionFailureAttempt(uint32_t result,
                                                  uint32_t attempt,
                                                  uint32_t max_attempts)
{
    char operation[8];

    if ((result == NO_ERROR) || (result == STATE_SWITCH)) {
        return;
    }
    (void)snprintf(operation,
                   sizeof(operation),
                   "%c%02u",
                   (char)s_last_transaction_diagnostic.request[1],
                   (unsigned int)s_last_transaction_diagnostic.request[6]);
    MULTIPARAM_V4_PrintLastTransactionPacketsForScene(
        "协议事务重试", operation, result, attempt, max_attempts);
}

/*
 * 函数用途：打印最近一次V4交互事务的请求包和实际接收包。
 * 调用场景：部件参数读取已经失败，需要保留现场原始收发内容时。
 * 关键约束：只在任务上下文调用，保持既有“读取部件参数”场景名称。
 */
void MULTIPARAM_V4_PrintLastTransactionPackets(const char *operation,
                                               uint32_t result)
{
    if (s_last_transaction_diagnostic.failure_packets_printed != 0U) {
        return;
    }
    MULTIPARAM_V4_PrintLastTransactionPacketsForScene(
        "读取部件参数", operation, result, 0U, 0U);
}

/*
 * 函数用途：在业务异常后按当前通信方式打印最近一次V4原始通信包。
 * 调用场景：液位频率读取失败或频率值被业务层拒绝之后。
 * 关键约束：只在线程态打印，不发送额外请求；主动模式复制原包后再恢复中断。
 */
void MULTIPARAM_V4_PrintFailurePackets(const char *operation,
                                       uint32_t result)
{
    uint8_t frame[MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE];
    const char *source = "无";
    const char *communication_mode =
        (s_communication_mode == MULTIPARAM_V4_COMMUNICATION_ACTIVE) ?
        "主动" : "未知";
    uint16_t length = 0U;
    uint32_t received_tick = 0U;
    uint32_t primask;

    if ((result == NO_ERROR) || (result == STATE_SWITCH)) {
        return;
    }
    if (s_communication_mode == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        if (s_last_transaction_diagnostic.failure_packets_printed == 0U) {
            MULTIPARAM_V4_PrintLastTransactionPacketsForScene(
                "异常后诊断", operation, result, 0U, 0U);
        }
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((s_last_abnormal_frame.valid != 0U) &&
        (s_last_abnormal_frame.error_code == result) &&
        ((int32_t)(s_last_abnormal_frame.received_tick -
                   s_latest_snapshot.received_tick) >= 0)) {
        length = s_last_abnormal_frame.length;
        if (length > MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE) {
            length = MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE;
        }
        if (length != 0U) {
            memcpy(frame, s_last_abnormal_frame.data, length);
        }
        received_tick = s_last_abnormal_frame.received_tick;
        source = "最近异常包";
    } else if (s_latest_active_frame_valid != 0U) {
        length = MULTIPARAM_V4_ACTIVE_FRAME_SIZE;
        memcpy(frame, s_latest_active_frame, length);
        received_tick = s_latest_snapshot.received_tick;
        source = "最近有效主动包";
    }
    if (primask == 0U) {
        __enable_irq();
    }

    printf("通信包\t协议=V4\t场景=异常后诊断\t操作=%s"
           "\t通信方式=%s\t来源=%s\t方向=RX\t长度=%u\tHEX=",
           (operation != NULL) ? operation : "未指定",
           communication_mode,
           source,
           (unsigned int)length);
    if (length == 0U) {
        printf("<无原包>");
    } else {
        for (uint16_t index = 0U; index < length; index++) {
            printf((index == 0U) ? "%02X" : " %02X", frame[index]);
        }
    }
    printf("\t结果=0x%08lX\t阶段=业务读取失败",
           (unsigned long)result);
    if (received_tick != 0U) {
        printf("\t帧年龄=%lu ms",
               (unsigned long)(HAL_GetTick() - received_tick));
    }
    printf("\r\n");
}

/*
 * 函数用途：输出一次收到非零异常内容的R01协议版本探测现场。
 * 调用场景：R01短帧、坏帧、UART错误或完整帧字段校验失败后。
 * 关键约束：零字节超时不打印，避免不支持V4协议的候选设备反复刷屏。
 */
void MULTIPARAM_V4_LogProtocolProbeFailure(uint32_t error_code)
{
    char reply_text[3U * MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE];
    char detail[384];

    if (s_last_transaction_diagnostic.received_length == 0U) {
        return;
    }
    MULTIPARAM_V4_FormatReplyBytes(reply_text, sizeof(reply_text));
    (void)snprintf(
        detail,
        sizeof(detail),
        "TX=%02X %02X %02X %02X %02X %02X %02X %02X,"
        "RX长度=%u,RX=%s,响应耗时=%lu ms,校验阶段=%s,"
        "UART状态=0x%08lX,UART错误=0x%08lX",
        s_last_transaction_diagnostic.request[0],
        s_last_transaction_diagnostic.request[1],
        s_last_transaction_diagnostic.request[2],
        s_last_transaction_diagnostic.request[3],
        s_last_transaction_diagnostic.request[4],
        s_last_transaction_diagnostic.request[5],
        s_last_transaction_diagnostic.request[6],
        s_last_transaction_diagnostic.request[7],
        (unsigned int)s_last_transaction_diagnostic.received_length,
        reply_text,
        (unsigned long)s_last_transaction_diagnostic.elapsed_ms,
        (s_last_transaction_diagnostic.stage != NULL)
            ? s_last_transaction_diagnostic.stage
            : "未确定",
        (unsigned long)s_last_transaction_diagnostic.uart_state,
        (unsigned long)s_last_transaction_diagnostic.uart_error);
    /* 错误 阶段：错误报警 模块：传感器 操作：读取R01协议版本 原因：本次错误码对应原因 处理：继续候选协议识别 详情：原始收发、耗时和校验阶段。 */
    ErrorLog_WarnDetail(ERROR_LOG_MODULE_SENSOR,
                        "读取R01协议版本",
                        ErrorLog_GetReasonByCode(error_code),
                        "继续候选协议识别",
                        detail);
}
