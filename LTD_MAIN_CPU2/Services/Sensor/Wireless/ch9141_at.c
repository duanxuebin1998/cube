/*
 * 模块职责：封装CH9141K软件AT模式进入、命令应答、异步文本等待和透明链路恢复。
 * 所有权边界：AT事务独占UART6-CH9141所有权，结束后必须确认退出AT并恢复连续空闲。
 * 清理约束：RSSI等命令可能已执行但ACK不完整，发送前即登记清理责任，失败时执行有界恢复。
 */

#include "ch9141_at.h"
#include "Protocols/Dm4/Safe/sensor_safe_transport_uart6.h"
#include "sensor_uart6_owner.h"

#include "sensor_transport_config.h"
#include "error_log.h"
#include "system_parameter.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define CH9141_AT_COMMAND_TX_TIMEOUT_MS 200U /* CH9141K AT 指令参数：命令 发送 超时 毫秒。 */
#define CH9141_AT_BYTE_RX_TIMEOUT_MS    20U /* CH9141K AT 指令参数：字节 RX 超时 毫秒。 */
#define CH9141_AT_PRE_COMMAND_IDLE_MS 30U /* CH9141K AT 指令参数：前置 命令 IDLE 毫秒。 */
#define CH9141_AT_PRE_COMMAND_DRAIN_TIMEOUT_MS 200U /* CH9141K AT 指令参数：命令前清空总超时毫秒。 */
#define CH9141_AT_SOFTWARE_IDLE_MS      500U /* CH9141K AT 指令参数：SOFTWARE IDLE 毫秒。 */
#define CH9141_AT_RECOVERY_IDLE_MS      500U /* CH9141K AT 恢复透明传输后的最终连续空闲时间。 */
#define CH9141_AT_PREPARE_DRAIN_TIMEOUT_MS 1000U /* CH9141K AT 指令参数：准备串口清空总超时毫秒。 */
#define CH9141_AT_ENTER_TIMEOUT_MS      1000U /* CH9141K AT 指令参数：ENTER 超时 毫秒。 */

/* 默认只打印业务摘要和失败 AT 详情；设为 1 可恢复逐条 AT 收发日志。 */
#ifndef CH9141_AT_VERBOSE_LOG
#define CH9141_AT_VERBOSE_LOG           0U /* CH9141K AT 指令参数：详细日志 日志。 */
#endif

/* 上电阶段已经收到孤立百分号、等待继续清理 RSSI 文本的标志。 */
static uint8_t ch9141_boot_percent_pending = 0U;
static volatile CH9141Uart6LinkState s_ch9141_uart6_link_state = CH9141_UART6_TRANSPARENT_READY; /* UART6透明传输与AT交接状态。 */
static uint32_t CH9141_AT_PrepareUart6Internal(uint32_t idle_ms, uint8_t check_command_switch);

/**
 * @brief 返回 UART6 当前透明传输与 AT 操作交接状态。
 *
 * 调用场景：DSM 事务发送前检查；该函数只读取状态，不执行串口操作。
 * 关键约束：只有 CH9141_UART6_TRANSPARENT_READY 允许 DSM 启动新事务。
 *
 * @return UART6 当前链路交接状态。
 */
CH9141Uart6LinkState CH9141_AT_GetUart6LinkState(void)
{
    return s_ch9141_uart6_link_state;
}

/**
 * @brief 返回等待模式名称，便于现场日志确认当前 AT 命令在等什么结束条件。
 *
 * @param wait_mode CH9141K AT 响应完成判定模式；决定等待 OK、提示符、退出结果或仅收集异步文本。
 * @return 返回等待模式名称，便于现场日志确认当前 AT 命令在等什么结束条件对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *CH9141_AT_WaitModeName(CH9141AtWaitMode wait_mode)
{
    switch (wait_mode) {
    case CH9141_AT_WAIT_SCAN_END:
        return "SCAN_END";
    case CH9141_AT_WAIT_LINK:
        return "LINK";
    case CH9141_AT_WAIT_RSSI:
        return "RSSI";
    case CH9141_AT_WAIT_ACK:
    default:
        return "ACK";
    }
}

/**
 * @brief 生成用于打印的 AT 指令文本。
 *
 * LINK/CONN/CONADD 后面的逗号字段是密码，现场日志只保留动作和目标，不直接打印密码。
 *
 * @param cmd 命令值。该值是准备发送或记录的 NUL 结尾 CH9141 AT 命令文字，日志会按既定规则隐藏或压缩敏感载荷。
 * @return 返回存放用于打印的 AT 指令文本的模块静态缓冲区首地址；后续调用可能覆盖其内容，调用方不得释放。
 */
static const char *CH9141_AT_LogCommand(const char *cmd)
{
    static char log_cmd[96];
    const char *comma;

    if (cmd == NULL) {
        return "<NULL>";
    }

    comma = strchr(cmd, ',');
    if ((comma != NULL) &&
        ((strncmp(cmd, "AT+LINK=", 8U) == 0) ||
         (strncmp(cmd, "AT+CONN=", 8U) == 0) ||
         (strncmp(cmd, "AT+CONADD=", 10U) == 0))) {
        snprintf(log_cmd, sizeof(log_cmd), "%.*s,******", (int)(comma - cmd), cmd);
        return log_cmd;
    }

    snprintf(log_cmd, sizeof(log_cmd), "%s", cmd);
    return log_cmd;
}

/**
 * @brief 打印 AT 原始响应。
 *
 * 只在失败路径打印原文，避免扫描成功时把候选内容重复刷屏；业务层会打印解析后的候选。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 */
static void CH9141_AT_PrintRawResponse(const CH9141AtResponse *response)
{
    if ((response == NULL) || (response->len == 0U)) {
        printf("CH9141K AT\t响应原文=<空>\r\n");
        return;
    }

    printf("CH9141K AT\t响应原文开始\r\n%s\r\nCH9141K AT\t响应原文结束\r\n", response->text);
}

/**
 * @brief 打印 AT 指令结果摘要。
 *
 * 现场排查时先看返回码和标志位，失败时再看原始响应，能快速区分无响应、PAIR ERR 和格式异常。
 *
 * @param cmd 命令值。该值是准备发送或记录的 NUL 结尾 CH9141 AT 命令文字，日志会按既定规则隐藏或压缩敏感载荷。
 * @param wait_mode CH9141K AT 响应完成判定模式；决定等待 OK、提示符、退出结果或仅收集异步文本。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param ret 上一层调用返回的结果码。该值与 CH9141 原始响应一并打印，用于区分超时、协议拒绝和成功 ACK。
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 */
static void CH9141_AT_PrintResult(const char *cmd,
                                  CH9141AtWaitMode wait_mode,
                                  uint32_t timeout_ms,
                                  uint32_t ret,
                                  const CH9141AtResponse *response)
{
    /* 关闭详细日志时省略成功事务，失败和异常响应仍必须打印，避免静默丢失链路诊断。 */
    if ((CH9141_AT_VERBOSE_LOG == 0U) && (ret == NO_ERROR)) {
        return;
    }

    uint16_t len = (response == NULL) ? 0U : response->len;
    uint8_t has_ok = (response == NULL) ? 0U : response->has_ok;
    uint8_t has_err = (response == NULL) ? 0U : response->has_err;
    uint8_t has_link_ok = (response == NULL) ? 0U : response->has_link_ok;
    uint8_t has_pair_err = (response == NULL) ? 0U : response->has_pair_err;
    uint8_t has_scan_end = (response == NULL) ? 0U : response->has_scan_end;
    uint8_t has_rssi = (response == NULL) ? 0U : response->has_rssi;

    printf("CH9141K AT\t结果\t指令=%s\t等待=%s\t超时=%lu ms\t返回=0x%08lX\t长度=%u\tOK=%u\tERR=%u\tLINK_OK=%u\tPAIR_ERR=%u\tSCAN_END=%u\tRSSI=%u\r\n",
           CH9141_AT_LogCommand(cmd),
           CH9141_AT_WaitModeName(wait_mode),
           (unsigned long)timeout_ms,
           (unsigned long)ret,
           (unsigned)len,
           (unsigned)has_ok,
           (unsigned)has_err,
           (unsigned)has_link_ok,
           (unsigned)has_pair_err,
           (unsigned)has_scan_end,
           (unsigned)has_rssi);

    if (ret != NO_ERROR) {
        CH9141_AT_PrintRawResponse(response);
    }
}

/**
 * @brief 清除 UART6 硬件错误标志，避免历史 ORE/ErrorCode 影响下一条 AT 命令。
 */
static void CH9141_AT_ClearUartError(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
}

/**
 * @brief 丢弃 UART6 历史残留，直到连续 idle_ms 没有新字节或达到固定总时限。
 *
 * 匹配会暂时把 UART6 从透传业务切到 AT 配置，进入前必须清掉旧半包。
 * 持续收包时返回通信超时，避免上电检测永久占住主循环。
 *
 * @param idle_ms 进入 AT 操作前要求 UART6 连续无数据的空闲时间，单位 ms。
 * @param total_timeout_ms 本次 UART6 接收排空允许占用的总时限，单位 ms。
 * @param check_command_switch 非零表示排空期间响应命令切换，恢复收尾传 0 以保证 AT 退出动作不被半途跳过。
 * @return NO_ERROR 表示 UART6 已连续空闲达到 idle_ms；等待期间检测到命令切换返回 STATE_SWITCH，总时限内始终有残留字节返回 SENSOR_DEVICE_COMM_TIMEOUT。
 */
static uint32_t CH9141_AT_DrainRxUntilIdle(uint32_t idle_ms,
                                          uint32_t total_timeout_ms,
                                          uint8_t check_command_switch)
{
    uint8_t dump;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_rx_tick = start_tick;

    for (;;) {
        uint32_t now_tick;

        if (HAL_UART_Receive(&huart6, &dump, 1U, 1U) == HAL_OK) {
            now_tick = HAL_GetTick();
            last_rx_tick = now_tick;
        } else {
            now_tick = HAL_GetTick();
            if ((now_tick - last_rx_tick) >= idle_ms) {
                return NO_ERROR;
            }
        }

        if ((check_command_switch != 0U) && HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        /* 持续透传数据会不断刷新空闲计时，必须用固定总时限保证主循环能够退出。 */
        if ((now_tick - start_tick) >= total_timeout_ms) {
            return SENSOR_DEVICE_COMM_TIMEOUT;
        }
    }
}

/**
 * @brief 清空 CH9141K AT 响应接收状态和缓存。
 *
 * @param response 可写 CH9141 AT 响应累积对象；函数按职责清空、追加接收字节、更新 ACK/ERROR与提示符标志，或输出完整等待结果。
 */
void CH9141_AT_ResetResponse(CH9141AtResponse *response)
{
    if (response == NULL) {
        return;
    }

    memset(response->text, 0, sizeof(response->text));
    response->len = 0U;
    response->has_ok = 0U;
    response->has_err = 0U;
    response->has_link_ok = 0U;
    response->has_pair_err = 0U;
    response->has_scan_end = 0U;
    response->has_rssi = 0U;
}
/**
 * @brief 标记传感器重新上电后的首次百分号透传过滤机会。
 *
 * 设备启动或明确重新给传感器供电后，由主循环任务调用。
 *
 * @note 只允许首次进入 AT 失败且响应仅为百分号时重试一次。
 */
void CH9141_AT_NotifySensorPowerOn(void)
{
    ch9141_boot_percent_pending = 1U;
}

/**
 * @brief 判断 AT 响应是否只有一个百分号和可选 CR/LF。
 *
 * 其他任何字节都不能被忽略，避免掩盖真实 AT 异常或串口串扰。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @return 1 表示响应非空，除可选 CR 或 LF 外只包含且恰好包含一个百分号；0 表示响应为空、没有百分号、出现第二个百分号，或包含其它字符。
 */
static uint8_t CH9141_AT_ResponseIsBootPercentOnly(const CH9141AtResponse *response)
{
    uint16_t index;
    uint8_t percent_seen = 0U;

    if (response == NULL) {
        return 0U;
    }

    for (index = 0U; index < response->len; index++) {
        char value = response->text[index];

        if ((value == '\r') || (value == '\n')) {
            continue;
        }
        if ((value == '%') && (percent_seen == 0U)) {
            percent_seen = 1U;
            continue;
        }
        return 0U;
    }

    return percent_seen;
}

/**
 * @brief 判断文本中是否已经出现完整数字，跳过扫描候选的 "1." 序号格式。
 *
 * @param text 待检查的 NUL 结尾 CH9141 原始响应文字；函数只做数字、RSSI 或控制行识别，不修改内容。
 * @param end 待扫描文本区间的尾后指针；函数只在 text 至 end 之间查找数字字符。
 * @return 1 表示文本中已经出现完整数字，跳过扫描候选的 "1." 序号格式；0 表示文本中尚未出现完整数字，跳过扫描候选的 "1." 序号格式。
 */
static uint8_t CH9141_AT_TextHasNumber(const char *text, const char *end)
{
    while ((text != NULL) && (end != NULL) && (text < end)) {
        const char *number = text;
        uint8_t has_digit = 0U;

        if (((*number == '-') || (*number == '+')) && ((number + 1) < end)) {
            number++;
        }
        while ((number < end) && (*number >= '0') && (*number <= '9')) {
            has_digit = 1U;
            number++;
        }

        if (has_digit != 0U) {
            if ((number >= end) || (*number != '.')) {
                return 1U;
            }
            text = number + 1;
        } else {
            text++;
        }
    }
    return 0U;
}

/**
 * @brief 判断响应文本里是否已经出现完整 RSSI 上报内容。
 *
 * CH9141K RSSI 命令先返回 OK，随后异步输出 RSSI；仅收到不完整的 "-2" 不能提前结束。
 *
 * @param text 待检查的 NUL 结尾 CH9141 原始响应文字；函数只做数字、RSSI 或控制行识别，不修改内容。
 * @return true 表示响应中含有可识别的 RSSI 异步文本前缀；否则返回 false。
 */
static uint8_t CH9141_AT_ResponseHasRssiText(const char *text)
{
    const char *cursor;

    if (text == NULL) {
        return 0U;
    }

    cursor = text;
    while (*cursor != '\0') {
        const char *line_start = cursor;
        const char *raw_end;
        const char *line_end;
        uint8_t has_rssi_token = 0U;
        uint8_t has_signed_line = 0U;

        while ((*line_start == '\r') || (*line_start == '\n') ||
               (*line_start == ' ') || (*line_start == '\t')) {
            line_start++;
        }

        raw_end = line_start;
        while ((*raw_end != '\0') && (*raw_end != '\r') && (*raw_end != '\n')) {
            raw_end++;
        }
        if (*raw_end == '\0') {
            return 0U;
        }

        line_end = raw_end;
        while ((line_end > line_start) && ((line_end[-1] == ' ') || (line_end[-1] == '\t'))) {
            line_end--;
        }

        if (line_end > line_start) {
            const char *scan;
            uint16_t line_len = (uint16_t)(line_end - line_start);

            if (!(((line_len == 2U) && (strncmp(line_start, "OK", 2U) == 0)) ||
                  ((line_len >= 2U) && (line_start[0] == 'A') && (line_start[1] == 'T')))) {
                for (scan = line_start; scan < line_end; scan++) {
                    if (((uint16_t)(line_end - scan) >= 4U) &&
                        ((scan[0] == 'R') || (scan[0] == 'r')) &&
                        ((scan[1] == 'S') || (scan[1] == 's')) &&
                        ((scan[2] == 'S') || (scan[2] == 's')) &&
                        ((scan[3] == 'I') || (scan[3] == 'i'))) {
                        has_rssi_token = 1U;
                        break;
                    }
                }

                if (((line_start[0] == '-') || (line_start[0] == '+')) &&
                    ((line_start + 1) < line_end) &&
                    (line_start[1] >= '0') && (line_start[1] <= '9')) {
                    has_signed_line = 1U;
                }

                if (((has_rssi_token != 0U) || (has_signed_line != 0U)) &&
                    (CH9141_AT_TextHasNumber(line_start, line_end) != 0U)) {
                    return 1U;
                }
            }
        }

        cursor = raw_end;
        while ((*cursor == '\r') || (*cursor == '\n')) {
            cursor++;
        }
    }

    return 0U;
}
/**
 * @brief 按独立响应行识别 AT 控制 token，避免扫描数据里的 NAME/文本误触发 OK/ERR。
 *
 * @param text 待检查的 NUL 结尾 CH9141 原始响应文字；函数只做数字、RSSI 或控制行识别，不修改内容。
 * @param token 用于串行化访问共享资源的控制标记。
 * @param prefix_match 前缀。
 * @return 1 表示输入文本包含独立 AT 控制行；仅在候选数据字段中出现相似子串时返回 0。
 */
static uint8_t CH9141_AT_TextHasControlLine(const char *text, const char *token, uint8_t prefix_match)
{
    uint16_t token_len;

    if ((text == NULL) || (token == NULL)) {
        return 0U;
    }

    token_len = (uint16_t)strlen(token);
    while (*text != '\0') {
        const char *start = text;
        const char *end;
        uint16_t line_len;

        while ((*start == '\r') || (*start == '\n') || (*start == ' ') || (*start == '\t')) {
            start++;
        }
        end = start;
        while ((*end != '\0') && (*end != '\r') && (*end != '\n')) {
            end++;
        }
        while ((end > start) && ((end[-1] == ' ') || (end[-1] == '\t'))) {
            end--;
        }

        line_len = (uint16_t)(end - start);
        if ((line_len >= token_len) && (strncmp(start, token, token_len) == 0)) {
            if (line_len == token_len) {
                return 1U;
            }
            if (prefix_match != 0U) {
                char next = start[token_len];

                /* 只允许 ERR: / ERR= / ERR<空格> 等控制格式，避免 ERROR... 数据行被误判。 */
                if ((next == ' ') || (next == '\t') || (next == ':') || (next == '=') || (next == ',')) {
                    return 1U;
                }
            }
        }

        text = end;
        while ((*text == '\r') || (*text == '\n')) {
            text++;
        }
    }

    return 0U;
}


/**
 * @brief 根据已收集文本刷新 AT 响应标志位。
 *
 * @param response 可写 CH9141 AT 响应累积对象；函数按职责清空、追加接收字节、更新 ACK/ERROR与提示符标志，或输出完整等待结果。
 * @param wait_mode CH9141K AT 响应完成判定模式；决定等待 OK、提示符、退出结果或仅收集异步文本。
 */
static void CH9141_AT_UpdateFlags(CH9141AtResponse *response, CH9141AtWaitMode wait_mode)
{
    if (response == NULL) {
        return;
    }

    if (CH9141_AT_TextHasControlLine(response->text, "LINK OK", 0U) != 0U) {
        response->has_link_ok = 1U;
    }
    if (CH9141_AT_TextHasControlLine(response->text, "PAIR ERR", 0U) != 0U) {
        response->has_pair_err = 1U;
        response->has_err = 1U;
    }
    if (CH9141_AT_TextHasControlLine(response->text, "SCAN END", 0U) != 0U) {
        response->has_scan_end = 1U;
    }
    if (CH9141_AT_TextHasControlLine(response->text, "OK", 0U) != 0U) {
        response->has_ok = 1U;
    }
    if (CH9141_AT_TextHasControlLine(response->text, "ERR", 1U) != 0U) {
        response->has_err = 1U;
    }
    if ((wait_mode == CH9141_AT_WAIT_RSSI) &&
        (CH9141_AT_ResponseHasRssiText(response->text) != 0U)) {
        response->has_rssi = 1U;
    }
}

/**
 * @brief 向响应缓存追加一个字节，并保持字符串以 NUL 结尾。
 *
 * @param response 可写 CH9141 AT 响应累积对象；函数按职责清空、追加接收字节、更新 ACK/ERROR与提示符标志，或输出完整等待结果。
 * @param value 字节使用的输入数值。
 * @param wait_mode CH9141K AT 响应完成判定模式；决定等待 OK、提示符、退出结果或仅收集异步文本。
 */
static void CH9141_AT_AppendByte(CH9141AtResponse *response, uint8_t value, CH9141AtWaitMode wait_mode)
{
    if (response == NULL) {
        return;
    }

    if (response->len < (CH9141_AT_RESPONSE_TEXT_SIZE - 1U)) {
        response->text[response->len] = (char)value;
        response->len++;
        response->text[response->len] = '\0';
    }
    CH9141_AT_UpdateFlags(response, wait_mode);
}

/**
 * @brief 判断当前响应是否满足调用方要求的结束条件。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @param wait_mode CH9141K AT 响应完成判定模式；决定等待 OK、提示符、退出结果或仅收集异步文本。
 * @return 1 表示响应已经达到当前等待模式的终止条件：扫描结束或错误、连接成功或配对错误、RSSI 到达或错误、普通 ACK 的 OK 或错误；0 表示响应为空，或当前等待模式所需的结束标志尚未出现。
 */
static uint8_t CH9141_AT_IsComplete(const CH9141AtResponse *response,
                                    CH9141AtWaitMode wait_mode)
{
    if (response == NULL) {
        return 0U;
    }

    switch (wait_mode) {
    case CH9141_AT_WAIT_SCAN_END:
        return (response->has_scan_end != 0U) || (response->has_err != 0U);
    case CH9141_AT_WAIT_LINK:
        return (response->has_link_ok != 0U) ||
               (response->has_pair_err != 0U) ||
               ((response->has_err != 0U) && (response->has_link_ok == 0U));
    case CH9141_AT_WAIT_RSSI:
        return (response->has_rssi != 0U) || (response->has_err != 0U);
    case CH9141_AT_WAIT_ACK:
    default:
        return (response->has_ok != 0U) || (response->has_err != 0U);
    }
}

/**
 * @brief 阻塞收集 AT 响应，直到指定结束条件、命令切换或超时。
 *
 * @param wait_mode CH9141K AT 响应完成判定模式；决定等待 OK、提示符、退出结果或仅收集异步文本。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param response 可写 CH9141 AT 响应累积对象；函数按职责清空、追加接收字节、更新 ACK/ERROR与提示符标志，或输出完整等待结果。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t CH9141_AT_CollectResponse(CH9141AtWaitMode wait_mode,
                                          uint32_t timeout_ms,
                                          CH9141AtResponse *response)
{
    uint8_t byte_value;
    uint32_t start_tick = HAL_GetTick();

    if (response == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        HAL_StatusTypeDef status;

        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }

        status = HAL_UART_Receive(&huart6, &byte_value, 1U, CH9141_AT_BYTE_RX_TIMEOUT_MS);
        if (status == HAL_OK) {
            CH9141_AT_AppendByte(response, byte_value, wait_mode);
            if (CH9141_AT_IsComplete(response, wait_mode) != 0U) {
                if (response->has_err != 0U) {
                    return SLIPRING_COMM_FAIL;
                }
                return NO_ERROR;
            }
        } else if (status == HAL_ERROR) {
            CH9141_AT_ClearUartError();
            return COMM_UART_TRANSFER_ERROR;
        } else {
            /* 读 1 字节超时是轮询过程的正常空窗，清掉可能残留的硬件错误后继续等。 */
            CH9141_AT_ClearUartError();
        }
    }

    return SENSOR_DEVICE_COMM_TIMEOUT;
}

/**
 * @brief AT 入口失败或被命令切换打断后，把 CH9141K 和 UART6 尽量恢复到透传可用状态。
 *
 * @details 调用场景：CH9141_AT_EnterSoftwareMode() 的失败出口；只在主循环任务上下文调用。
 * @note 关键约束：仅在 AT... 已经发出且可能进入 AT 模式时发送 AT+EXIT，避免无谓污染透传传感器链路。
 *
 * @param send_exit true 表示恢复透明传输前先发送退出 AT 模式命令，false 表示只恢复串口状态。
 * @return NO_ERROR 表示恢复动作和最终空闲确认均成功；其他值保留首次发送、排空或 UART 恢复错误。
 */
static uint32_t CH9141_AT_RecoverTransparentMode(uint8_t send_exit)
{
    static const uint8_t exit_cmd[] = "AT+EXIT\r\n";
    uint32_t first_error = NO_ERROR;
    uint32_t step_ret;

    s_ch9141_uart6_link_state = CH9141_UART6_RECOVERING;

    (void)HAL_UART_DMAStop(&huart6);
    CH9141_AT_ClearUartError();
    if (send_exit != 0U) {
        if (HAL_UART_Transmit(&huart6,
                              (uint8_t *)exit_cmd,
                              (uint16_t)(sizeof(exit_cmd) - 1U),
                              CH9141_AT_COMMAND_TX_TIMEOUT_MS) != HAL_OK) {
            first_error = COMM_UART_TRANSFER_ERROR;
        }
        step_ret = CH9141_AT_DrainRxUntilIdle(CH9141_AT_PRE_COMMAND_IDLE_MS,
                                              CH9141_AT_PRE_COMMAND_DRAIN_TIMEOUT_MS,
                                              0U);
        if ((first_error == NO_ERROR) && (step_ret != NO_ERROR)) {
            first_error = step_ret;
        }
    }
    if ((HAL_UART_Abort(&huart6) != HAL_OK) && (first_error == NO_ERROR)) {
        first_error = COMM_UART_TRANSFER_ERROR;
    }
    CH9141_AT_ClearUartError();
    step_ret = CH9141_AT_DrainRxUntilIdle(CH9141_AT_PRE_COMMAND_IDLE_MS,
                                          CH9141_AT_PRE_COMMAND_DRAIN_TIMEOUT_MS,
                                          0U);
    if ((first_error == NO_ERROR) && (step_ret != NO_ERROR)) {
        first_error = step_ret;
    }
    CH9141_AT_ClearUartError();
    s_ch9141_uart6_link_state = (first_error == NO_ERROR)
                                ? CH9141_UART6_TRANSPARENT_READY
                                : CH9141_UART6_NOT_READY;
    return first_error;
}

/**
 * @brief 在 RSSI 查询正常清理失败后，强制关闭异步上报并退出 AT 模式。
 *
 * 无线连接状态查询已进入 AT 模式，但 RSSI OFF 或 AT EXIT 未得到完整应答。
 *
 * @note 忽略命令切换以完成 UART6 清场；只允许在主循环任务上下文调用。
 */
static uint32_t CH9141_AT_RecoverRssiQueryOwned(void)
{
    static const uint8_t rssi_off_cmd[] = "AT+RSSI=OFF\r\n";
    static const uint8_t exit_cmd[] = "AT+EXIT\r\n";
    uint32_t first_error = NO_ERROR;
    uint32_t step_ret;

    s_ch9141_uart6_link_state = CH9141_UART6_RECOVERING;

    (void)HAL_UART_DMAStop(&huart6);
    CH9141_AT_ClearUartError();
    printf("CH9141K AT\tRSSI查询清理\t动作=强制关闭RSSI并退出AT\r\n");

    if (HAL_UART_Transmit(&huart6,
                          (uint8_t *)rssi_off_cmd,
                          (uint16_t)(sizeof(rssi_off_cmd) - 1U),
                          CH9141_AT_COMMAND_TX_TIMEOUT_MS) != HAL_OK) {
        first_error = COMM_UART_TRANSFER_ERROR;
    }
    step_ret = CH9141_AT_DrainRxUntilIdle(CH9141_AT_PRE_COMMAND_IDLE_MS,
                                          CH9141_AT_PRE_COMMAND_DRAIN_TIMEOUT_MS,
                                          0U);
    if ((first_error == NO_ERROR) && (step_ret != NO_ERROR)) {
        first_error = step_ret;
    }
    CH9141_AT_ClearUartError();

    if ((HAL_UART_Transmit(&huart6,
                           (uint8_t *)exit_cmd,
                           (uint16_t)(sizeof(exit_cmd) - 1U),
                           CH9141_AT_COMMAND_TX_TIMEOUT_MS) != HAL_OK) &&
        (first_error == NO_ERROR)) {
        first_error = COMM_UART_TRANSFER_ERROR;
    }
    step_ret = CH9141_AT_DrainRxUntilIdle(CH9141_AT_PRE_COMMAND_IDLE_MS,
                                          CH9141_AT_PRE_COMMAND_DRAIN_TIMEOUT_MS,
                                          0U);
    if ((first_error == NO_ERROR) && (step_ret != NO_ERROR)) {
        first_error = step_ret;
    }

    if ((HAL_UART_Abort(&huart6) != HAL_OK) && (first_error == NO_ERROR)) {
        first_error = COMM_UART_TRANSFER_ERROR;
    }
    CH9141_AT_ClearUartError();
    step_ret = CH9141_AT_PrepareUart6Internal(CH9141_AT_RECOVERY_IDLE_MS, 0U);
    if ((first_error == NO_ERROR) && (step_ret != NO_ERROR)) {
        first_error = step_ret;
    }
    if (first_error != NO_ERROR) {
        s_ch9141_uart6_link_state = CH9141_UART6_NOT_READY;
    }
    return first_error;
}

/**
 * @brief 中止 UART6 上的安全协议和 DMA 收发，清场到连续空闲后再进入 AT 操作。
 *
 * @param idle_ms 清场结束前 UART6 必须连续保持无数据的时间，单位 ms。
 * @return 返回清场结果码；NO_ERROR 表示 UART6 已达到连续空闲，其他值表示超时或通信错误。
 */
static uint32_t CH9141_AT_PrepareUart6Internal(uint32_t idle_ms, uint8_t check_command_switch)
{
    /* 切换到 CH9141 AT 前先清除安全协议同步传输状态，避免共享 UART6 保留忙标志。 */
    SensorSafeTransportUart6_Abort();
    uint32_t ret;

    s_ch9141_uart6_link_state = CH9141_UART6_RECOVERING;

    if (CH9141_AT_VERBOSE_LOG != 0U) {
        printf("CH9141K AT\t准备UART6\t停止DMA并等待空闲=%lu ms\r\n", (unsigned long)idle_ms);
    }

    (void)HAL_UART_DMAStop(&huart6);
    (void)HAL_UART_Abort(&huart6);
    CH9141_AT_ClearUartError();
    ret = CH9141_AT_DrainRxUntilIdle(idle_ms,
                                     CH9141_AT_PREPARE_DRAIN_TIMEOUT_MS,
                                     check_command_switch);
    CH9141_AT_ClearUartError();
    s_ch9141_uart6_link_state = (ret == NO_ERROR)
                                ? CH9141_UART6_TRANSPARENT_READY
                                : CH9141_UART6_NOT_READY;
    return ret;
}

/**
 * @brief 按可中断模式准备 UART6，并发布透明传输交接结果。
 *
 * 调用场景：进入 CH9141K AT 前或正常 AT+EXIT 后由主循环调用。
 * 关键约束：命令切换会终止普通准备；强制恢复流程改用内部不可中断入口。
 *
 * @param idle_ms 清场结束前 UART6 必须连续保持无数据的时间，单位 ms。
 * @return NO_ERROR 表示透明传输已经就绪，其他值表示命令切换、超时或 UART 错误。
 */
uint32_t CH9141_AT_PrepareUart6(uint32_t idle_ms)
{
    uint32_t result;

    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_CH9141_AT) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = CH9141_AT_PrepareUart6Internal(idle_ms, 1U);
    (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_CH9141_AT);
    return result;
}

/**
 * @brief 在 AT 退出或模块复位后完成不可中断的 UART6 透明传输交接。
 *
 * 调用场景：AT+EXIT 或 AT+RESET 已经发出后的收尾阶段；只能在主循环任务上下文调用。
 * 关键约束：新命令可以打断业务等待，但不能把 UART6 留在 RECOVERING/NOT_READY 后直接启动下一次 DSM 事务。
 *
 * @param idle_ms 交接完成前 UART6 必须连续保持无数据的时间，单位 ms。
 * @return NO_ERROR 表示透明传输已经就绪，其他值表示总时限内未达到连续空闲或 UART 错误。
 */
uint32_t CH9141_AT_CompleteTransparentHandoff(uint32_t idle_ms)
{
    uint32_t result;

    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_CH9141_AT) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = CH9141_AT_PrepareUart6Internal(idle_ms, 0U);
    (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_CH9141_AT);
    return result;
}


/**
 * @brief 准备 UART6 后进入 CH9141K 软件 AT 模式，失败时重试并恢复透传。
 *
 * @param response 可写 CH9141 AT 响应累积对象；函数按职责清空、追加接收字节、更新 ACK/ERROR与提示符标志，或输出完整等待结果。
 * @return NO_ERROR 表示 UART6 清理、软件 AT 进入和提示符确认成功；其他值为排空、发送、异步等待、命令切换或恢复透传阶段保留下来的具体错误码。
 */
static uint32_t CH9141_AT_EnterSoftwareModeOwned(CH9141AtResponse *response)
{
    uint32_t ret;
    uint8_t boot_percent_retry_allowed = ch9141_boot_percent_pending;

    if (CH9141_AT_VERBOSE_LOG != 0U) {
        printf("CH9141K AT\t进入软件AT模式\r\n");
    }
    ret = CH9141_AT_PrepareUart6(CH9141_AT_SOFTWARE_IDLE_MS);
    if (ret != NO_ERROR) {
        ch9141_boot_percent_pending = 0U;
        return ret;
    }

    s_ch9141_uart6_link_state = CH9141_UART6_ENTERING_AT;
    /* CH9141K 软件 AT 入口命令为 AT...，协议不使用裸 AT 作为入口。 */
    ret = CH9141_AT_SendCommand("AT...", CH9141_AT_WAIT_ACK, CH9141_AT_ENTER_TIMEOUT_MS, response);
    if ((ret != NO_ERROR) &&
        (boot_percent_retry_allowed != 0U) &&
        (CH9141_AT_ResponseIsBootPercentOnly(response) != 0U)) {
        ch9141_boot_percent_pending = 0U;
        printf("CH9141K AT\t上电百分号透传已忽略\t动作=重试进入AT\r\n");
        ret = CH9141_AT_SendCommand("AT...", CH9141_AT_WAIT_ACK, CH9141_AT_ENTER_TIMEOUT_MS, response);
    } else if ((ret == SENSOR_DEVICE_COMM_TIMEOUT) &&
               (boot_percent_retry_allowed != 0U) &&
               (response != NULL) &&
               (response->len != 0U) &&
               (response->has_ok == 0U) &&
               (response->has_err == 0U)) {
        ch9141_boot_percent_pending = 0U;
        ErrorLog_Retry(ERROR_LOG_MODULE_SLIPRING_COMM,
                       "进入蓝牙AT模式",
                       ErrorLog_GetReasonByCode(ret),
                       1U,
                       1U,
                       ret);
        /* 首次上电收到非完整响应时先恢复透明模式和 UART6，再从空闲等待开始完整重试一次。 */
        CH9141_AT_RecoverTransparentMode(1U);
        CH9141_AT_ResetResponse(response);
        ret = CH9141_AT_PrepareUart6(CH9141_AT_SOFTWARE_IDLE_MS);
        if (ret == NO_ERROR) {
            s_ch9141_uart6_link_state = CH9141_UART6_ENTERING_AT;
            ret = CH9141_AT_SendCommand("AT...",
                                        CH9141_AT_WAIT_ACK,
                                        CH9141_AT_ENTER_TIMEOUT_MS,
                                        response);
        }
        if (ret == NO_ERROR) {
            ErrorLog_Recover(ERROR_LOG_MODULE_SLIPRING_COMM,
                             "进入蓝牙AT模式",
                             "上电AT响应恢复",
                             1U,
                             1U);
        }
    } else {
        ch9141_boot_percent_pending = 0U;
    }
    /* 入口失败时必须恢复透传和 UART6，避免后续传感器命令接在半截 AT 状态后面。 */
    if (ret != NO_ERROR) {
        uint8_t send_exit = 0U;

        if ((ret == SENSOR_DEVICE_COMM_TIMEOUT) &&
            (response != NULL) &&
            (response->len == 0U)) {
            printf("CH9141K AT\tAT...进入无响应\t请检查UART6链路、模块供电、AT入口时序或硬件AT引脚\r\n");
        }
        if ((ret == STATE_SWITCH) ||
            ((response != NULL) && (response->len != 0U))) {
            send_exit = 1U;
        }
        (void)CH9141_AT_RecoverTransparentMode(send_exit);
    } else {
        s_ch9141_uart6_link_state = CH9141_UART6_AT_ACTIVE;
    }

    return ret;
}


/**
 * @brief 等待并解析 CH9141K 后续异步上报，不发送新命令。
 *
 * @param wait_mode 工作模式。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param response 可写 CH9141 AT 响应累积对象；函数按职责清空、追加接收字节、更新 ACK/ERROR与提示符标志，或输出完整等待结果。
 * @return NO_ERROR 表示在 timeout_ms 内收到 wait_mode 要求的完整异步结果；response 为空或模式无效返回 SYSTEM_CALL_CONDITION_ERROR，其他值为UART6 接收、超时、ERROR 响应或命令切换错误。
 */
static uint32_t CH9141_AT_WaitAsyncOwned(CH9141AtWaitMode wait_mode,
                             uint32_t timeout_ms,
                             CH9141AtResponse *response)
{
    uint32_t ret;

    if (response == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    CH9141_AT_ResetResponse(response);
    CH9141_AT_ClearUartError();
    if (CH9141_AT_VERBOSE_LOG != 0U) {
        printf("CH9141K AT\t等待异步上报\t等待=%s\t超时=%lu ms\r\n",
               CH9141_AT_WaitModeName(wait_mode),
               (unsigned long)timeout_ms);
    }

    ret = CH9141_AT_CollectResponse(wait_mode, timeout_ms, response);
    CH9141_AT_PrintResult("<异步上报>", wait_mode, timeout_ms, ret, response);
    return ret;
}



/**
 * @brief 清空残留接收数据，发送一条 AT 命令并按等待模式收集响应。
 *
 * @param cmd 命令值。该值是准备发送或记录的 NUL 结尾 CH9141 AT 命令文字，日志会按既定规则隐藏或压缩敏感载荷。
 * @param wait_mode 工作模式。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param response 可写 CH9141 AT 响应累积对象；函数按职责清空、追加接收字节、更新 ACK/ERROR与提示符标志，或输出完整等待结果。
 * @return NO_ERROR 表示残留数据已清理、命令已发送并收到 wait_mode 要求的响应；命令或响应对象无效返回 SYSTEM_CALL_CONDITION_ERROR，其他值为排空、UART6 发送、响应等待、超时、ERROR 或命令切换错误。
 */
static uint32_t CH9141_AT_SendCommandOwned(const char *cmd,
                               CH9141AtWaitMode wait_mode,
                               uint32_t timeout_ms,
                               CH9141AtResponse *response)
{
    static const uint8_t line_end[] = "\r\n";
    uint16_t cmd_len;
    uint32_t ret;
    uint8_t leaves_at_mode;

    if ((cmd == NULL) || (response == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    leaves_at_mode = (uint8_t)(((strcmp(cmd, "AT+EXIT") == 0) ||
                                (strcmp(cmd, "AT+RESET") == 0)) ? 1U : 0U);
    if (leaves_at_mode != 0U) {
        /* EXIT/RESET 一旦开始，只有后续 PrepareUart6 成功才能重新声明透明传输就绪。 */
        s_ch9141_uart6_link_state = CH9141_UART6_RECOVERING;
    }

    CH9141_AT_ResetResponse(response);
    (void)HAL_UART_DMAStop(&huart6);
    CH9141_AT_ClearUartError();
    /* 上一条命令可能遗留 RSSI 等异步尾包，发送新 AT 前先短暂清空。 */
    ret = CH9141_AT_DrainRxUntilIdle(CH9141_AT_PRE_COMMAND_IDLE_MS,
                                     CH9141_AT_PRE_COMMAND_DRAIN_TIMEOUT_MS,
                                     1U);
    CH9141_AT_ClearUartError();
    if (ret != NO_ERROR) {
        if (leaves_at_mode != 0U) {
            s_ch9141_uart6_link_state = CH9141_UART6_NOT_READY;
        }
        CH9141_AT_PrintResult(cmd, wait_mode, timeout_ms, ret, response);
        return ret;
    }

    if (CH9141_AT_VERBOSE_LOG != 0U) {
        printf("CH9141K AT\t发送\t指令=%s\t等待=%s\t超时=%lu ms\r\n",
               CH9141_AT_LogCommand(cmd),
               CH9141_AT_WaitModeName(wait_mode),
               (unsigned long)timeout_ms);
    }

    cmd_len = (uint16_t)strlen(cmd);
    /* AT 命令正文发送失败时清理 UART6 硬件错误并立即返回，不得继续发送行结束符或等待不存在的应答。 */
    if (HAL_UART_Transmit(&huart6, (uint8_t *)cmd, cmd_len, CH9141_AT_COMMAND_TX_TIMEOUT_MS) != HAL_OK) {
        CH9141_AT_ClearUartError();
        ret = COMM_UART_TRANSFER_ERROR;
        if (leaves_at_mode != 0U) {
            s_ch9141_uart6_link_state = CH9141_UART6_NOT_READY;
        }
        CH9141_AT_PrintResult(cmd, wait_mode, timeout_ms, ret, response);
        return ret;
    }
    /* 命令正文成功但 CRLF 发送失败仍属于不完整 AT 帧，清理 UART6 后按传输错误返回。 */
    if (HAL_UART_Transmit(&huart6, (uint8_t *)line_end, 2U, CH9141_AT_COMMAND_TX_TIMEOUT_MS) != HAL_OK) {
        CH9141_AT_ClearUartError();
        ret = COMM_UART_TRANSFER_ERROR;
        if (leaves_at_mode != 0U) {
            s_ch9141_uart6_link_state = CH9141_UART6_NOT_READY;
        }
        CH9141_AT_PrintResult(cmd, wait_mode, timeout_ms, ret, response);
        return ret;
    }

    ret = CH9141_AT_CollectResponse(wait_mode, timeout_ms, response);
    if ((leaves_at_mode != 0U) && (ret != NO_ERROR)) {
        s_ch9141_uart6_link_state = CH9141_UART6_NOT_READY;
    }
    CH9141_AT_PrintResult(cmd, wait_mode, timeout_ms, ret, response);
    return ret;
}
uint32_t CH9141_AT_RecoverRssiQuery(void)
{
    uint32_t result;

    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_CH9141_AT) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = CH9141_AT_RecoverRssiQueryOwned();
    (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_CH9141_AT);
    return result;
}

uint32_t CH9141_AT_EnterSoftwareMode(CH9141AtResponse *response)
{
    uint32_t result;

    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_CH9141_AT) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = CH9141_AT_EnterSoftwareModeOwned(response);
    (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_CH9141_AT);
    return result;
}

uint32_t CH9141_AT_WaitAsync(CH9141AtWaitMode wait_mode,
                             uint32_t timeout_ms,
                             CH9141AtResponse *response)
{
    uint32_t result;

    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_CH9141_AT) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = CH9141_AT_WaitAsyncOwned(wait_mode, timeout_ms, response);
    (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_CH9141_AT);
    return result;
}

uint32_t CH9141_AT_SendCommand(const char *cmd,
                               CH9141AtWaitMode wait_mode,
                               uint32_t timeout_ms,
                               CH9141AtResponse *response)
{
    uint32_t result;

    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_CH9141_AT) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = CH9141_AT_SendCommandOwned(cmd, wait_mode, timeout_ms, response);
    (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_CH9141_AT);
    return result;
}
