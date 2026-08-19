/*
 * 模块职责：实现CH9141K候选扫描、RSSI/名称选择、连接保存和只读连接状态诊断。
 * 业务边界：匹配流程只管理无线链路，不探测或修改下游传感器协议。
 * 清理约束：所有AT流程结束前必须关闭异步RSSI、退出AT并恢复透明传输，避免污染后续传感器应答。
 */

#include "wireless_pairing.h"
#include "main.h"

#include "ch9141_at.h"
#include "error_log.h"
#include "sensor_transport_config.h"
#include "system_parameter.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIRELESS_PAIRING_PASSWORD              "000000" /* 无线滑环 AT 配对密码。 */
#define WIRELESS_PAIRING_MAX_CANDIDATES        8U /* 无线滑环扫描候选设备最大数量。 */
#define WIRELESS_PAIRING_MAC_TEXT_SIZE         18U /* 无线滑环 MAC 地址文本缓冲区长度。 */
#define WIRELESS_PAIRING_NAME_TEXT_SIZE        19U /* 无线滑环名称文本缓冲区长度。 */
#define WIRELESS_PAIRING_LINE_TEXT_SIZE        128U /* 无线滑环 AT 响应行缓冲区长度。 */
#define WIRELESS_PAIRING_SCAN_TIMEOUT_MS       12000U /* 无线滑环扫描超时时间，单位 ms。 */
#define WIRELESS_PAIRING_LINK_TIMEOUT_MS       10000U /* 无线滑环连接超时时间，单位 ms。 */
#define WIRELESS_PAIRING_ACK_TIMEOUT_MS        1500U /* 无线滑环 AT 应答超时时间，单位 ms。 */
#define WIRELESS_PAIRING_RSSI_REPORT_PERIOD_MS 1000U /* 无线滑环 RSSI 上报周期，单位 ms。 */
#define WIRELESS_PAIRING_RSSI_ASYNC_TIMEOUT_MS  1500U /* 无线滑环 RSSI 异步上报等待超时，单位 ms。 */
#define WIRELESS_PAIRING_RESET_ACK_TIMEOUT_MS  800U /* 无线滑环复位应答超时时间，单位 ms。 */
#define WIRELESS_PAIRING_RESET_WAIT_MS         2500U /* 无线滑环复位后等待时间，单位 ms。 */
#define WIRELESS_PAIRING_POST_RESET_IDLE_MS    500U /* 无线滑环复位后串口空闲等待时间，单位 ms。 */
#define WIRELESS_PAIRING_HOST_MODE             1U /* 无线滑环主机模式取值。 */
#define WIRELESS_PAIRING_HOST_CONNECTED_STATE  0x03U /* 无线滑环主机已连接状态值。 */
#define WIRELESS_PAIRING_SLAVE_CONNECTED_STATE 0x05U /* 无线滑环从机已连接状态值。 */
#define WIRELESS_PAIRING_RSSI_NEAR_THRESHOLD   (-55) /* 无线滑环近场 RSSI 判定阈值，单位 dBm。 */
#define WIRELESS_PAIRING_RSSI_MIN_GAP_DB       8 /* 无线滑环 RSSI 最小领先差值，单位 dB。 */

typedef struct {
    uint8_t index;                                  /* CH9141K 扫描结果序号，优先用于 AT+LINK。 */
    char mac[WIRELESS_PAIRING_MAC_TEXT_SIZE];       /* 统一转大写后的 MAC 文本。 */
    int16_t rssi;                                  /* 候选设备RSSI，单位dBm。 */
    /* 无线扫描中的单个候选设备，保留 RSSI、可选名称和原始扫描行。 */
    uint8_t has_rssi; /* 响应文本中已经解析出完整 RSSI 字段的标志。 */
    char name[WIRELESS_PAIRING_NAME_TEXT_SIZE]; /* 扫描结果中解析出的可选设备名称文本。 */
    uint8_t has_name; /* 该候选扫描记录中包含有效设备名称的标志。 */
    char line[WIRELESS_PAIRING_LINE_TEXT_SIZE];     /* 保留原始扫描行，便于现场核对解析规则。 */
} WirelessPairingCandidate;

typedef struct {
    /* 一次无线扫描解析出的候选数组、数量和名称字段能力。 */
    WirelessPairingCandidate candidates[WIRELESS_PAIRING_MAX_CANDIDATES]; /* 本次扫描解析出的候选设备数组。 */
    uint8_t count; /* candidates 中实际填充的候选数量，不得超过 WIRELESS_PAIRING_MAX_CANDIDATES。 */
    uint8_t scan_has_name_field; /* 扫描响应是否包含设备名称字段的能力标志。 */
} WirelessPairingScanResult;

/* CH9141K 当前连接查询只能读到 MAC；名称用本次运行期最近一次成功匹配的候选补充。 */
static char s_wireless_pairing_last_peer_mac[WIRELESS_PAIRING_MAC_TEXT_SIZE]; /* 最近一次成功配对或连接的对端 MAC 文本。 */
static char s_wireless_pairing_last_peer_name[WIRELESS_PAIRING_NAME_TEXT_SIZE]; /* 最近一次成功配对或连接的对端名称文本。 */
static uint8_t s_wireless_pairing_last_peer_has_name; /* 最近对端名称文本有效的标志。 */

/**
 * @brief 打印普通结果摘要，不走 ErrorLog_*，避免维护调试失败被记成最终故障链路。
 *
 * @param stage 用于诊断日志或参数保存记录的 NUL 结尾阶段名称；标识本次输出对应的加载、比较、写入、回读或协议处理阶段。
 * @param ret 上一层调用返回的结果码。无线配对流程按调用点将其打印、映射链路错误或作为最终状态发布。
 */
static void WirelessPairing_PrintRet(const char *stage, uint32_t ret)
{
    const char *name = ErrorLog_GetCodeName(ret);
    const char *reason = ErrorLog_GetReasonByCode(ret);

    if (stage == NULL) {
        stage = "无线滑环匹配";
    }

    if (ret == NO_ERROR) {
        printf("%s\t结果=成功\r\n", stage);
    } else {
        printf("%s\t结果=失败\t错误码=0x%08lX\t错误名=%s\t原因=%s\r\n",
               stage,
               (unsigned long)ret,
               name,
               reason);
    }
}

/**
 * @brief ASCII 小写转大写，用于 MAC 和关键字解析。
 *
 * @param c 待转换为大写形式的 ASCII 字符。
 * @return 返回字符的大写 ASCII 形式；非小写英文字母保持原值。
 */
static char WirelessPairing_ToUpper(char c)
{
    if ((c >= 'a') && (c <= 'z')) {
        return (char)(c - 'a' + 'A');
    }
    return c;
}

/**
 * @brief 判断字符是否是 MAC 地址中的十六进制字符。
 *
 * @param c 待判断是否为十六进制数字的字符。
 * @return 1 表示字符属于 0-9、A-F 或 a-f，0 表示不是十六进制字符。
 */
static uint8_t WirelessPairing_IsHex(char c)
{
    return (((c >= '0') && (c <= '9')) ||
            ((c >= 'A') && (c <= 'F')) ||
            ((c >= 'a') && (c <= 'f'))) ? 1U : 0U;
}

/**
 * @brief 判断字符是否为 ASCII 字母或数字，用于避免从 MAC: 字段名中间误截 MAC。
 *
 * @param c 待判断是否为 ASCII 字母或数字的字符。
 * @return 1 表示字符属于 ASCII 数字或英文字母，0 表示不是字母数字字符。
 */
static uint8_t WirelessPairing_IsAsciiAlnum(char c)
{
    return (((c >= '0') && (c <= '9')) ||
            ((c >= 'A') && (c <= 'Z')) ||
            ((c >= 'a') && (c <= 'z'))) ? 1U : 0U;
}

/**
 * @brief 判断名称字段的结束符。
 *
 * @param c 待判断是否为响应字段分隔符的字符。
 * @return 1 表示字符是字符串结束、空白、逗号、分号或换行分隔符；其他字符返回 0。
 */
static uint8_t WirelessPairing_IsDelimiter(char c)
{
    return ((c == '\0') || (c == ' ') || (c == '\t') ||
            (c == ',') || (c == ';') || (c == '\r') || (c == '\n')) ? 1U : 0U;
}

/**
 * @brief 大小写不敏感地判断字符串前缀。
 *
 * @param text 无线扫描解析使用的 NUL 结尾只读文字；函数按 ASCII 不区分大小写执行前缀或子串匹配。
 * @param prefix 待匹配的 ASCII 前缀文本；比较忽略英文字母大小写。
 * @return 1 表示输入文本以前缀开头且忽略 ASCII 大小写后完全匹配；空指针或不匹配时返回 0。
 */
static uint8_t WirelessPairing_StrStartsWithIgnoreCase(const char *text, const char *prefix)
{
    if ((text == NULL) || (prefix == NULL)) {
        return 0U;
    }

    while (*prefix != '\0') {
        if (WirelessPairing_ToUpper(*text) != WirelessPairing_ToUpper(*prefix)) {
            return 0U;
        }
        text++;
        prefix++;
    }
    return 1U;
}

/**
 * @brief 大小写不敏感地在一行文本中查找关键字。
 *
 * @param text 无线扫描解析使用的 NUL 结尾只读文字；函数按 ASCII 不区分大小写执行前缀或子串匹配。
 * @param needle 待在输入文本中查找的 ASCII 关键字。
 * @return 成功时返回指向大小写不敏感地在一行文本中查找关键字的指针；输入非法或未找到匹配项时返回 NULL。
 */
static const char *WirelessPairing_FindIgnoreCase(const char *text, const char *needle)
{
    if ((text == NULL) || (needle == NULL) || (*needle == '\0')) {
        return NULL;
    }

    while (*text != '\0') {
        if (WirelessPairing_StrStartsWithIgnoreCase(text, needle) != 0U) {
            return text;
        }
        text++;
    }
    return NULL;
}

/**
 * @brief 从扫描行中提取冒号分隔 MAC，并统一转大写。
 *
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 * @param out 规范化 MAC 文本输出数组，容量为 WIRELESS_PAIRING_MAC_TEXT_SIZE；成功时写入大写冒号格式并以 NUL 结尾。
 * @return 1 表示已提取并规范化完整 12 位十六进制 MAC；格式不完整或输出参数非法时返回 0。
 */
static uint8_t WirelessPairing_CopyMacFromLine(const char *line, char out[WIRELESS_PAIRING_MAC_TEXT_SIZE])
{
    uint16_t i;

    if ((line == NULL) || (out == NULL)) {
        return 0U;
    }

    for (i = 0U; line[i] != '\0'; i++) {
        uint8_t ok = 1U;

        /* MAC 前后必须是字段边界，避免把 "MAC:D1..." 中的 "AC:D1..." 误当成地址。 */
        if ((i > 0U) && (WirelessPairing_IsAsciiAlnum(line[i - 1U]) != 0U)) {
            continue;
        }

        for (uint8_t j = 0U; j < 17U; j++) {
            if (line[i + j] == '\0') {
                ok = 0U;
                break;
            }
            if (((j + 1U) % 3U) == 0U) {
                if (line[i + j] != ':') {
                    ok = 0U;
                    break;
                }
            } else if (WirelessPairing_IsHex(line[i + j]) == 0U) {
                ok = 0U;
                break;
            }
        }

        if ((ok != 0U) &&
            ((line[i + 17U] == ':') || (WirelessPairing_IsHex(line[i + 17U]) != 0U))) {
            ok = 0U;
        }

        if (ok != 0U) {
            for (uint8_t j = 0U; j < 17U; j++) {
                out[j] = WirelessPairing_ToUpper(line[i + j]);
            }
            out[17] = '\0';
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 从扫描文本解析候选序号，仅接受完整有效整数。
 *
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 * @param index 索引值。
 * @return 1 表示已解析出完整、非负且范围有效的候选序号；格式或范围非法时返回 0。
 */
static uint8_t WirelessPairing_ParseIndexFromLine(const char *line, uint8_t *index)
{
    int value = 0;
    uint8_t has_digit = 0U;

    if ((line == NULL) || (index == NULL)) {
        return 0U;
    }

    while ((*line >= '0') && (*line <= '9')) {
        has_digit = 1U;
        value = (value * 10) + (*line - '0');
        line++;
    }

    if ((has_digit == 0U) || (*line != '.') || (value <= 0) || (value > 255)) {
        return 0U;
    }

    *index = (uint8_t)value;
    return 1U;
}

/**
 * @brief 从扫描行解析 RSSI 数值。
 *
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 * @param rssi 用于返回解析后的接收信号强度，单位 dBm。
 * @return 1 表示已从扫描行解析出范围有效的 RSSI；未找到字段或数值非法时返回 0。
 */
static uint8_t WirelessPairing_ParseRssiFromLine(const char *line, int16_t *rssi)
{
    const char *cursor;
    int value;

    if ((line == NULL) || (rssi == NULL)) {
        return 0U;
    }

    cursor = WirelessPairing_FindIgnoreCase(line, "RSSI");
    if (cursor == NULL) {
        return 0U;
    }

    while ((*cursor != '\0') && (*cursor != '-') && ((*cursor < '0') || (*cursor > '9'))) {
        cursor++;
    }
    if (*cursor == '\0') {
        return 0U;
    }

    value = atoi(cursor);
    if (value < -128) {
        value = -128;
    } else if (value > 127) {
        value = 127;
    }
    *rssi = (int16_t)value;
    return 1U;
}

/**
 * @brief 从扫描行中解析名称字段。
 *
 * 只接受 NAME:xxx、NAME=xxx 等显式字段，避免数字编号误命中 RSSI、电压或 MAC 片段。
 *
 * @param line 待解析的无线扫描响应行。
 * @param out 用于接收去除字段前缀和尾部分隔符后的设备名称。
 * @return 1 表示已从显式 NAME 字段提取出非空名称；0 表示字段不存在、输出参数非法或名称为空。
 */
static uint8_t WirelessPairing_ParseNameFromLine(const char *line,
                                                 char out[WIRELESS_PAIRING_NAME_TEXT_SIZE])
{
    static const char *tokens[] = {
        "NAME:",
        "NAME=",
        "PNAME:",
        "PNAME=",
    };
    const char *start = NULL;

    if ((line == NULL) || (out == NULL)) {
        return 0U;
    }

    for (uint8_t i = 0U; i < (sizeof(tokens) / sizeof(tokens[0])); i++) {
        const char *hit = WirelessPairing_FindIgnoreCase(line, tokens[i]);
        if (hit != NULL) {
            start = hit + strlen(tokens[i]);
            break;
        }
    }

    if ((start == NULL) || (*start == '\0')) {
        return 0U;
    }

    while (*start == ' ') {
        start++;
    }

    for (uint8_t i = 0U; i < (WIRELESS_PAIRING_NAME_TEXT_SIZE - 1U); i++) {
        if (WirelessPairing_IsDelimiter(start[i]) != 0U) {
            out[i] = '\0';
            return (i > 0U) ? 1U : 0U;
        }
        out[i] = start[i];
    }
    out[WIRELESS_PAIRING_NAME_TEXT_SIZE - 1U] = '\0';
    return 1U;
}

/**
 * @brief 缓存最近一次成功连接的候选信息，供 SPC 状态查询补充远端名称。
 *
 * CH9141K 的当前连接查询只返回 MAC，不提供已连接远端蓝牙名称；名称只能来自运行期扫描结果。
 *
 * @param candidate 待校验或比较的候选值。该无线候选记录包含扫描索引、MAC、名称和 RSSI，用于缓存、发布或连接前复核。
 */
static void WirelessPairing_RememberPeer(const WirelessPairingCandidate *candidate)
{
    if (candidate == NULL) {
        return;
    }

    snprintf(s_wireless_pairing_last_peer_mac, sizeof(s_wireless_pairing_last_peer_mac), "%s", candidate->mac);
    if (candidate->has_name != 0U) {
        snprintf(s_wireless_pairing_last_peer_name, sizeof(s_wireless_pairing_last_peer_name), "%s", candidate->name);
        s_wireless_pairing_last_peer_has_name = 1U;
    } else {
        s_wireless_pairing_last_peer_name[0] = '\0';
        s_wireless_pairing_last_peer_has_name = 0U;
    }
}

/**
 * @brief 按 MAC 查询运行期缓存的远端名称。
 *
 * @param mac 待查询的 6 字节蓝牙 MAC 地址，按扫描结果中的字节顺序保存。
 * @return 成功时返回指向按 MAC 查询运行期缓存的远端名称的指针；输入非法或未找到匹配项时返回 NULL。
 */
static const char *WirelessPairing_GetCachedName(const char *mac)
{
    if ((mac == NULL) || (*mac == '\0') || (s_wireless_pairing_last_peer_has_name == 0U)) {
        return NULL;
    }
    if (strcmp(s_wireless_pairing_last_peer_mac, mac) != 0) {
        return NULL;
    }
    return s_wireless_pairing_last_peer_name;
}

/**
 * @brief 去掉 AT 响应单行首尾空白，返回有效内容起点。
 *
 * @param line 可写 CH9141 单行响应缓冲区；函数原地删除首尾空白并保持结果以 NUL 结尾。
 * @return 成功时返回指向去掉 AT 响应单行首尾空白，返回有效内容起点的指针；输入非法或未找到匹配项时返回 NULL。
 */
static char *WirelessPairing_TrimLine(char *line)
{
    char *start;
    char *end;

    if (line == NULL) {
        return NULL;
    }

    start = line;
    while ((*start == ' ') || (*start == '\t')) {
        start++;
    }

    end = start + strlen(start);
    while ((end > start) && ((end[-1] == ' ') || (end[-1] == '\t'))) {
        end--;
        *end = '\0';
    }
    return start;
}
/**
 * @brief 判断响应行是否为 AT 控制关键字，允许后接常见分隔符。
 *
 * 不能只按前缀匹配 ERR/LINK OK，否则名称或数据行以 ERROR、LINK OK... 开头时会被误判为控制行。
 *
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 * @param token 用于串行化访问共享资源的控制标记。
 * @return 1 表示该响应行属于 AT 控制回显、OK、ERROR 或查询状态行，0 表示是候选业务数据行。
 */
static uint8_t WirelessPairing_IsControlLine(const char *line, const char *token)
{
    uint16_t token_len;
    char next;

    if ((line == NULL) || (token == NULL)) {
        return 0U;
    }

    token_len = (uint16_t)strlen(token);
    if (WirelessPairing_StrStartsWithIgnoreCase(line, token) == 0U) {
        return 0U;
    }

    next = line[token_len];
    if (next == '\0') {
        return 1U;
    }
    if ((next == ' ') || (next == '\t') || (next == ':') || (next == '=') || (next == ',')) {
        return 1U;
    }
    return 0U;
}
/**
 * @brief 判断 AT 响应行是否只是回显、OK/ERR 或扫描结束提示。
 *
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 * @return 1 表示行为空指针或空白行、AT 命令回显，或 OK、ERR、LINK OK、PAIR ERR、SCAN END 控制行，扫描结果解析应忽略；0 表示该行可能包含实际设备信息，需继续解析。
 */
static uint8_t WirelessPairing_IsIgnorableAtLine(const char *line)
{
    const char *cursor;

    if (line == NULL) {
        return 1U;
    }

    cursor = line;
    while ((*cursor == ' ') || (*cursor == '\t')) {
        cursor++;
    }
    if (*cursor == '\0') {
        return 1U;
    }
    if ((WirelessPairing_ToUpper(cursor[0]) == 'A') &&
        (WirelessPairing_ToUpper(cursor[1]) == 'T') &&
        ((cursor[2] == '\0') || (cursor[2] == '+'))) {
        return 1U;
    }
    if ((strcmp(cursor, "OK") == 0) ||
        (WirelessPairing_IsControlLine(cursor, "ERR") != 0U) ||
        (WirelessPairing_IsControlLine(cursor, "LINK OK") != 0U) ||
        (WirelessPairing_IsControlLine(cursor, "PAIR ERR") != 0U) ||
        (WirelessPairing_IsControlLine(cursor, "SCAN END") != 0U)) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 返回十六进制字符值，非十六进制返回 -1。
 *
 * @param c 待转换为 0 至 15 数值的十六进制字符。
 * @return 返回十六进制字符对应的 0 至 15；输入不是 0-9、A-F 或 a-f 时返回 -1。
 */
static int8_t WirelessPairing_HexValue(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return (int8_t)(c - '0');
    }
    c = WirelessPairing_ToUpper(c);
    if ((c >= 'A') && (c <= 'F')) {
        return (int8_t)(c - 'A' + 10);
    }
    return -1;
}

/**
 * @brief 将 AA:BB:CC:DD:EE:FF 格式 MAC 拆成 3 个 16 位数值，便于通过输入寄存器发布。
 *
 * @param mac 用于接收由三个 16 位字段拼接得到的 6 字节蓝牙 MAC 地址。
 * @param mac_high 用于返回 48 位 MAC 地址最高 16 位。
 * @param mac_mid 用于返回 48 位 MAC 地址中间 16 位。
 * @param mac_low 用于返回 48 位 MAC 地址最低 16 位。
 * @return 1 表示输入及三个输出指针有效，17 字符 MAC 的 12 个十六进制数字和5 个冒号位置均合法，并已拆成高/中/低 16 位；0 表示指针为空、存在非十六进制字符或分隔符格式错误。
 */
static uint8_t WirelessPairing_ParseMacWords(const char *mac,
                                             uint32_t *mac_high,
                                             uint32_t *mac_mid,
                                             uint32_t *mac_low)
{
    uint8_t bytes[6];

    if ((mac == NULL) || (mac_high == NULL) || (mac_mid == NULL) || (mac_low == NULL)) {
        return 0U;
    }

    for (uint8_t i = 0U; i < 6U; i++) {
        int8_t high = WirelessPairing_HexValue(mac[i * 3U]);
        int8_t low = WirelessPairing_HexValue(mac[i * 3U + 1U]);

        if ((high < 0) || (low < 0)) {
            return 0U;
        }
        if ((i < 5U) && (mac[i * 3U + 2U] != ':')) {
            return 0U;
        }

        bytes[i] = (uint8_t)(((uint8_t)high << 4U) | (uint8_t)low);
    }

    *mac_high = ((uint32_t)bytes[0] << 8U) | (uint32_t)bytes[1];
    *mac_mid = ((uint32_t)bytes[2] << 8U) | (uint32_t)bytes[3];
    *mac_low = ((uint32_t)bytes[4] << 8U) | (uint32_t)bytes[5];
    return 1U;
}

/**
 * @brief 清空无线连接状态快照。
 *
 * @param status 可写无线连接状态对象；包含蓝牙链路、配对状态、MAC 有效性和规范化 MAC，函数按职责复位、填充或输出这些字段。
 */
static void WirelessPairing_ResetConnectionStatus(WirelessConnectionStatus *status)
{
    if (status != NULL) {
        memset(status, 0, sizeof(*status));
    }
}

/**
 * @brief 解析连接 MAC 并按高、中、低 16 位写入状态快照。
 *
 * @param status 可写无线连接状态对象；包含蓝牙链路、配对状态、MAC 有效性和规范化 MAC，函数按职责复位、填充或输出这些字段。
 * @param mac 准备写入当前连接信息的 6 字节蓝牙 MAC 地址。
 * @return 1 表示状态对象有效，连接 MAC 已通过格式校验并写入三段数值及有效标志；0 表示状态对象/文本为空或 MAC 解析失败，状态中不发布该地址。
 */
static uint8_t WirelessPairing_FillConnectionMac(WirelessConnectionStatus *status, const char *mac)
{
    uint32_t mac_high = 0U;
    uint32_t mac_mid = 0U;
    uint32_t mac_low = 0U;

    if ((status == NULL) ||
        (WirelessPairing_ParseMacWords(mac, &mac_high, &mac_mid, &mac_low) == 0U)) {
        return 0U;
    }

    status->mac_valid = 1U;
    status->mac_high = mac_high;
    status->mac_mid = mac_mid;
    status->mac_low = mac_low;
    return 1U;
}

/**
 * @brief 将连接快照逐字段写入共享测量结果，并在全部字段写完后递增更新计数。
 *
 * @param snapshot 无线连接状态只读快照；包含链路、配对结果、MAC 有效性和规范化 MAC，发布时作为一个一致对象写入共享状态。
 */
static void WirelessPairing_PublishConnectionStatus(const WirelessConnectionStatus *snapshot)
{
    volatile WirelessPairingStatus *status = &g_measurement.wireless_pairing_status;

    if (snapshot == NULL) {
        return;
    }

    status->connection_valid = snapshot->connection_valid;
    status->rssi_valid = snapshot->rssi_valid;
    status->rssi = snapshot->rssi;
    status->connection_error_code = snapshot->error_code;

    if ((snapshot->connection_valid != 0U) && (snapshot->mac_valid != 0U)) {
        status->mac_valid = 1U;
        status->mac_high = snapshot->mac_high;
        status->mac_mid = snapshot->mac_mid;
        status->mac_low = snapshot->mac_low;
    } else {
        status->mac_valid = 0U;
        status->mac_high = 0U;
        status->mac_mid = 0U;
        status->mac_low = 0U;
    }

    status->rssi_update_counter++;
    if (status->rssi_update_counter == 0U) {
        status->rssi_update_counter = 1U;
    }
}

/**
 * @brief 把 CH9141 蓝牙主机状态查询错误映射为链路错误码。
 *
 * @param ret 上一层调用返回的结果码。无线配对流程按调用点将其打印、映射链路错误或作为最终状态发布。
 * @return NO_ERROR 表示蓝牙主机状态有效且链路正常；可直接识别的 AT、UART、格式或模式错误保留原码，无法细分的状态查询失败映射为 WIRELESS_HOST_COMM_TIMEOUT。
 */
static uint32_t WirelessPairing_MapBluetoothLinkError(uint32_t ret)
{
    if ((ret == NO_ERROR) || (ret == STATE_SWITCH)) {
        return ret;
    }

    if (ret == SENSOR_DEVICE_COMM_TIMEOUT) {
        return WIRELESS_HOST_COMM_TIMEOUT;
    }

    return ret;
}

/**
 * @brief 查询 CH9141 蓝牙主机，并判断蓝牙从机连接是否有效。
 *
 * 调用场景：上电识别、传感器通信超时归因、配对收尾和串口维护测试；只在任务上下文调用。
 * status_out 返回本次 AT 查询得到的临时状态，用于现场打印 MAC/RSSI，避免使用可能保留配对结果的共享 MAC 字段。
 *
 * @param status_out 用于接收本次查询得到的状态快照。
 * @return NO_ERROR 表示状态查询成功且蓝牙从机连接有效；其它值为查询错误、映射后的 CH9141 或链路错误，或连接无效时的 WIRELESS_SLAVE_COMM_TIMEOUT，并同步写入连接状态。
 */
uint32_t WirelessPairing_CheckBluetoothLinkDetailed(WirelessConnectionStatus *status_out)
{
    WirelessConnectionStatus status;
    uint32_t ret;

    ret = WirelessPairing_ReadConnectionStatus(&status);
    if ((ret != NO_ERROR) && (status.error_code == NO_ERROR)) {
        status.error_code = ret;
    }
    WirelessPairing_PublishConnectionStatus(&status);

    ret = WirelessPairing_MapBluetoothLinkError(ret);
    if (ret != NO_ERROR) {
        g_measurement.wireless_pairing_status.connection_error_code = ret;
        status.error_code = ret;
        if (status_out != NULL) {
            *status_out = status;
        }
        return ret;
    }

    if (status.connection_valid == 0U) {
        ret = WirelessPairing_MapBluetoothLinkError(status.error_code);
        if (ret == NO_ERROR) {
            ret = WIRELESS_SLAVE_COMM_TIMEOUT;
        }
        g_measurement.wireless_pairing_status.connection_error_code = ret;
        status.error_code = ret;
        if (status_out != NULL) {
            *status_out = status;
        }
        return ret;
    }

    if (status_out != NULL) {
        *status_out = status;
    }
    return NO_ERROR;
}

/**
 * @brief 查询 CH9141 当前蓝牙链路并解析连接状态。
 *
 * @return 返回无线链路检查结果码；NO_ERROR 表示链路已连接，其他值区分查询失败、未连接或命令切换。
 */
uint32_t WirelessPairing_CheckBluetoothLink(void)
{
    return WirelessPairing_CheckBluetoothLinkDetailed(NULL);
}

/**
 * @brief 发布无线滑环匹配状态给 CPU3；只发布数值字段，显示端本地格式化 MAC。
 *
 * @param result 本次扫描、匹配或连接的无线配对结果码，按数值字段发布给 CPU3。
 * @param candidate 待校验或比较的候选值。该无线候选记录包含扫描索引、MAC、名称和 RSSI，用于缓存、发布或连接前复核。
 * @param error_code 待记录、转换或判断的错误码。该值是无线扫描、匹配、连接或保存流程的最终失败原因，随状态发布给 CPU3。
 */
static void WirelessPairing_PublishStatus(uint32_t result,
                                          const WirelessPairingCandidate *candidate,
                                          uint32_t error_code)
{
    volatile WirelessPairingStatus *status = &g_measurement.wireless_pairing_status;
    uint32_t publish_result = result;
    uint32_t publish_error = error_code;

    if (result == WIRELESS_PAIRING_RESULT_SUCCESS) {
        uint32_t mac_high = 0U;
        uint32_t mac_mid = 0U;
        uint32_t mac_low = 0U;

        if ((candidate != NULL) &&
            (WirelessPairing_ParseMacWords(candidate->mac, &mac_high, &mac_mid, &mac_low) != 0U)) {
            status->mac_high = mac_high;
            status->mac_mid = mac_mid;
            status->mac_low = mac_low;
            status->mac_valid = 1U;
            publish_error = NO_ERROR;
        } else {
            status->mac_valid = 0U;
            publish_result = WIRELESS_PAIRING_RESULT_FAILED;
            publish_error = WIRELESS_RESP_FORMAT_ERROR;
        }
    } else if (result == WIRELESS_PAIRING_RESULT_RUNNING) {
        publish_error = NO_ERROR;
    }

    if (publish_result == WIRELESS_PAIRING_RESULT_RUNNING) {
        g_measurement.device_status.error_code = NO_ERROR;
        g_measurement.device_status.device_state = STATE_WIRELESS_PAIRING;
    } else if (publish_result == WIRELESS_PAIRING_RESULT_SUCCESS) {
        g_measurement.device_status.error_code = NO_ERROR;
        g_measurement.device_status.device_state = STATE_WIRELESS_PAIRING_OVER;
    } else if (publish_result == WIRELESS_PAIRING_RESULT_FAILED) {
        if (publish_error == NO_ERROR) {
            publish_error = WIRELESS_RESP_FORMAT_ERROR;
        }
        g_measurement.device_status.error_code = publish_error;
        g_measurement.device_status.device_state = STATE_ERROR;
    }

    status->result = publish_result;
    status->error_code = publish_error;
    status->update_counter++;
    if (status->update_counter == 0U) {
        status->update_counter = 1U;
    }
}

/**
 * @brief 从一行 BLEMODE/BLESTA 响应中解析 1 字节十六进制或十进制码。
 *
 * 只接受裸数值或 `KEY=VALUE` / `KEY:VALUE`，避免把残留的 `RSSI -35dB` 当成状态码。
 *
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 * @param value 用于返回从目标响应行解析的 1 字节状态值。
 * @return 1 表示已从目标响应行解析出合法字节值，0 表示标签、数字格式或范围不符合要求。
 */
static uint8_t WirelessPairing_ParseByteFromLine(const char *line, uint8_t *value)
{
    const char *cursor;
    const char *eq;
    const char *colon;
    int8_t high;
    int8_t low;

    if ((line == NULL) || (value == NULL)) {
        return 0U;
    }

    cursor = line;
    eq = strrchr(line, '=');
    colon = strrchr(line, ':');
    if ((eq != NULL) && ((colon == NULL) || (eq > colon))) {
        cursor = eq + 1;
    } else if (colon != NULL) {
        cursor = colon + 1;
    }

    while ((*cursor == ' ') || (*cursor == '\t')) {
        cursor++;
    }
    if (WirelessPairing_HexValue(*cursor) < 0) {
        return 0U;
    }

    high = WirelessPairing_HexValue(*cursor);
    cursor++;
    low = WirelessPairing_HexValue(*cursor);
    if (low >= 0) {
        *value = (uint8_t)(((uint8_t)high << 4) | (uint8_t)low);
    } else {
        *value = (uint8_t)high;
    }
    return 1U;
}

/**
 * @brief 判断 BLEMODE 数值是否为协议定义的合法模式。
 *
 * @param value 有效模式数值使用的输入数值。
 * @return 1 表示 BLEMODE 数值为协议定义的合法模式；0 表示 BLEMODE 数值不是协议定义的合法模式。
 */
static uint8_t WirelessPairing_IsValidModeValue(uint8_t value)
{
    return (value <= 0x02U) ? 1U : 0U;
}

/**
 * @brief 判断 BLESTA 数值是否为协议状态表中的合法状态。
 *
 * @param value 有效状态数值使用的输入数值。
 * @return 1 表示 BLESTA 数值为协议状态表中的合法状态；0 表示 BLESTA 数值不是协议状态表中的合法状态。
 */
static uint8_t WirelessPairing_IsValidStatusValue(uint8_t value)
{
    switch (value) {
    case 0x00U:
    case 0x01U:
    case 0x02U:
    case 0x03U:
    case 0x04U:
    case 0x05U:
    case 0x07U:
        return 1U;
    default:
        return 0U;
    }
}

/**
 * @brief 从 AT 查询响应中解析合法 BLEMODE/BLESTA 数值，可跳过前序残留异步行。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @param value 用于返回跳过允许残留行后解析的 1 字节状态值。
 * @param out_line 用于接收命中的 BLEMODE 或 BLESTA 原始响应行。
 * @param out_size out_line 缓冲区容量，单位字节。
 * @param status_value 状态数值。
 * @return 1 表示已跳过允许的异步残留行并解析出目标字节值，0 表示未找到合法目标响应。
 */
static uint8_t WirelessPairing_ParseByteResponseFiltered(const CH9141AtResponse *response,
                                                         uint8_t *value,
                                                         char *out_line,
                                                         uint16_t out_size,
                                                         uint8_t status_value)
{
    char line[WIRELESS_PAIRING_LINE_TEXT_SIZE];
    uint8_t line_len = 0U;

    if ((response == NULL) || (value == NULL)) {
        return 0U;
    }
    if ((out_line != NULL) && (out_size > 0U)) {
        out_line[0] = '\0';
    }

    for (uint16_t i = 0U; i <= response->len; i++) {
        char c = response->text[i];

        if ((c == '\r') || (c == '\n') || (c == '\0')) {
            if (line_len > 0U) {
                char *trimmed;
                uint8_t parsed;

                line[line_len] = '\0';
                trimmed = WirelessPairing_TrimLine(line);
                if ((trimmed != NULL) && (WirelessPairing_IsIgnorableAtLine(trimmed) == 0U) &&
                    (WirelessPairing_ParseByteFromLine(trimmed, &parsed) != 0U)) {
                    if (((status_value == 0U) && (WirelessPairing_IsValidModeValue(parsed) != 0U)) ||
                        ((status_value != 0U) && (WirelessPairing_IsValidStatusValue(parsed) != 0U))) {
                        *value = parsed;
                        if ((out_line != NULL) && (out_size > 0U)) {
                            snprintf(out_line, out_size, "%s", trimmed);
                        }
                        return 1U;
                    }
                }
                line_len = 0U;
            }
        } else if (line_len < (sizeof(line) - 1U)) {
            line[line_len] = c;
            line_len++;
        }
    }

    return 0U;
}

/**
 * @brief 解析无线模块工作模式查询响应。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @param value 待原地读取或更新的数值对象。解析成功时写入无线模块模式字节，失败时不把不完整或越界文字当作有效模式。
 * @param out_line 用于接收命中的 BLEMODE 或 BLESTA 原始响应行。
 * @param out_size 模式文字输出缓冲区容量，单位字节；成功解析后写入 NUL 结尾结果且不超过该容量。
 * @return 1 表示已解析出合法 BLEMODE 值，0 表示响应中没有可接受的模式结果。
 */
static uint8_t WirelessPairing_ParseModeResponse(const CH9141AtResponse *response,
                                                uint8_t *value,
                                                char *out_line,
                                                uint16_t out_size)
{
    return WirelessPairing_ParseByteResponseFiltered(response, value, out_line, out_size, 0U);
}

/**
 * @brief 解析无线模块配对状态查询响应。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @param value 待原地读取或更新的数值对象。解析成功时写入无线模块配对状态字节，失败时不把不完整或越界文字当作有效状态。
 * @param out_line 用于接收命中的 BLEMODE 或 BLESTA 原始响应行。
 * @param out_size 状态文字输出缓冲区容量，单位字节；成功解析后写入 NUL 结尾结果且不超过该容量。
 * @return 1 表示已解析出合法 BLESTA 值，0 表示响应中没有可接受的状态结果。
 */
static uint8_t WirelessPairing_ParseStatusResponse(const CH9141AtResponse *response,
                                                  uint8_t *value,
                                                  char *out_line,
                                                  uint16_t out_size)
{
    return WirelessPairing_ParseByteResponseFiltered(response, value, out_line, out_size, 1U);
}

/**
 * @brief 从一行文本中解析第一个带符号整数。
 *
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 * @param value 用于返回响应中第一个未溢出的带符号整数。
 * @return 1 表示已解析出范围有效的带符号整数，0 表示响应不含合法整数或发生溢出。
 */
static uint8_t WirelessPairing_ParseSignedNumber(const char *line, int16_t *value)
{
    const char *cursor;
    int parsed;

    if ((line == NULL) || (value == NULL)) {
        return 0U;
    }

    cursor = line;
    while (*cursor != '\0') {
        if (((*cursor == '-') || (*cursor == '+')) &&
            (cursor[1] >= '0') && (cursor[1] <= '9')) {
            break;
        }
        if ((*cursor >= '0') && (*cursor <= '9')) {
            break;
        }
        cursor++;
    }
    if (*cursor == '\0') {
        return 0U;
    }

    parsed = atoi(cursor);
    if (parsed < -32768) {
        parsed = -32768;
    } else if (parsed > 32767) {
        parsed = 32767;
    }
    *value = (int16_t)parsed;
    return 1U;
}

/**
 * @brief 从 CH9141K 异步 RSSI 上报文本中提取 RSSI。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @param rssi 用于返回解析后的接收信号强度，单位 dBm。
 * @return 1 表示已从 CH9141K 异步上报提取出有效 RSSI；文本不匹配或数值非法时返回 0。
 */
static uint8_t WirelessPairing_ParseRssiResponse(const CH9141AtResponse *response, int16_t *rssi)
{
    char line[WIRELESS_PAIRING_LINE_TEXT_SIZE];
    uint8_t line_len = 0U;

    if ((response == NULL) || (rssi == NULL)) {
        return 0U;
    }

    for (uint16_t i = 0U; i <= response->len; i++) {
        char c = response->text[i];

        if ((c == '\r') || (c == '\n') || (c == '\0')) {
            if (line_len > 0U) {
                char *trimmed;

                line[line_len] = '\0';
                trimmed = WirelessPairing_TrimLine(line);
                if ((trimmed != NULL) && (WirelessPairing_IsIgnorableAtLine(trimmed) == 0U)) {
                    if ((WirelessPairing_ParseRssiFromLine(trimmed, rssi) != 0U) ||
                        (WirelessPairing_ParseSignedNumber(trimmed, rssi) != 0U)) {
                        return 1U;
                    }
                }
                line_len = 0U;
            }
        } else if (line_len < (sizeof(line) - 1U)) {
            line[line_len] = c;
            line_len++;
        }
    }
    return 0U;
}

/**
 * @brief 返回 BLEMODE 数值对应的中文说明。
 *
 * @param mode CH9141 BLEMODE 查询得到的模式值，用于映射主机、从机或未知模式文字。
 * @return 返回 BLEMODE 数值对应的中文说明对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *WirelessPairing_ModeName(uint8_t mode)
{
    switch (mode) {
    case 0x00U:
        return "广播模式";
    case WIRELESS_PAIRING_HOST_MODE:
        return "主机模式";
    case 0x02U:
        return "从机模式";
    default:
        return "未知模式";
    }
}

/**
 * @brief 返回 BLESTA 状态码对应的中文说明。
 *
 * CH9141K 的 BLESTA 状态码需要按 BLEMODE 分表解释：主机模式 0x03 表示连接成功，
 * 从机模式 0x03 表示准备广播状态，不能把从机状态表套用到主机查询结果。
 *
 * @param mode CH9141 BLESTA 查询得到的连接状态值；参数名沿用旧接口，实际语义为状态码而非工作模式。
 * @param status 无线配对状态编码；函数将扫描、匹配、连接、成功或失败状态映射为稳定诊断名称。
 * @return 返回 BLESTA 状态码对应的中文说明对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *WirelessPairing_StatusName(uint8_t mode, uint8_t status)
{
    if (mode == WIRELESS_PAIRING_HOST_MODE) {
        switch (status) {
        case 0x00U:
            return "未初始化";
        case 0x01U:
            return "扫描中";
        case 0x02U:
            return "连接中";
        case WIRELESS_PAIRING_HOST_CONNECTED_STATE:
            return "连接成功";
        case 0x04U:
            return "断开连接中";
        case 0x07U:
            return "错误";
        default:
            return "未知状态";
        }
    }

    if (mode == 0x02U) {
        switch (status) {
        case 0x00U:
            return "未初始化";
        case 0x01U:
            return "设备初始化完成";
        case 0x02U:
            return "广播";
        case 0x03U:
            return "准备广播状态";
        case 0x04U:
            return "连接超时";
        case WIRELESS_PAIRING_SLAVE_CONNECTED_STATE:
            return "连接成功";
        case 0x07U:
            return "错误";
        default:
            return "未知状态";
        }
    }

    if (mode == 0x00U) {
        switch (status) {
        case 0x00U:
            return "未初始化";
        case 0x01U:
            return "设备初始化完成";
        case 0x02U:
            return "广播";
        case 0x07U:
            return "错误";
        default:
            return "未知状态";
        }
    }

    return "未知状态";
}

/**
 * @brief 判断候选列表里是否已存在指定 MAC。
 *
 * @param scan 当前扫描过程的状态对象。该结果保存固定容量候选数组、解析统计和扫描错误，函数按只读或可写声明查询、追加、打印或选择候选。
 * @param mac 待在候选列表中查重的 6 字节蓝牙 MAC 地址。
 * @return 1 表示候选列表里已存在指定 MAC；0 表示候选列表里尚未存在指定 MAC。
 */
static uint8_t WirelessPairing_CandidateExists(const WirelessPairingScanResult *scan, const char *mac)
{
    if ((scan == NULL) || (mac == NULL)) {
        return 0U;
    }

    for (uint8_t i = 0U; i < scan->count; i++) {
        if (strcmp(scan->candidates[i].mac, mac) == 0) {
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief 尝试把一行扫描文本加入候选列表。
 *
 * 非候选行通常是命令回显、OK 或 SCAN END，不打印；只有容量超限和重复 MAC 才给现场提示。
 *
 * @param scan 当前扫描过程的状态对象。该结果保存固定容量候选数组、解析统计和扫描错误，函数按只读或可写声明查询、追加、打印或选择候选。
 * @param line 当前待解析的 NUL 结尾 CH9141 扫描或 AT 响应行；函数只处理该行中的索引、MAC、名称、RSSI、状态或控制标记。
 */
static void WirelessPairing_AddCandidate(WirelessPairingScanResult *scan, const char *line)
{
    WirelessPairingCandidate *candidate;
    char mac[WIRELESS_PAIRING_MAC_TEXT_SIZE] = {0};

    if ((scan == NULL) || (line == NULL)) {
        return;
    }
    if (WirelessPairing_CopyMacFromLine(line, mac) == 0U) {
        return;
    }
    if (WirelessPairing_CandidateExists(scan, mac) != 0U) {
        printf("无线滑环匹配\t忽略重复候选\tMAC=%s\r\n", mac);
        return;
    }
    if (scan->count >= WIRELESS_PAIRING_MAX_CANDIDATES) {
        printf("无线滑环匹配\t候选超过缓存上限\t上限=%u\t忽略MAC=%s\r\n",
               (unsigned)WIRELESS_PAIRING_MAX_CANDIDATES,
               mac);
        return;
    }

    candidate = &scan->candidates[scan->count];
    memset(candidate, 0, sizeof(*candidate));
    snprintf(candidate->mac, sizeof(candidate->mac), "%s", mac);
    snprintf(candidate->line, sizeof(candidate->line), "%s", line);
    (void)WirelessPairing_ParseIndexFromLine(line, &candidate->index);
    if (WirelessPairing_ParseRssiFromLine(line, &candidate->rssi) != 0U) {
        candidate->has_rssi = 1U;
    }
    if (WirelessPairing_ParseNameFromLine(line, candidate->name) != 0U) {
        candidate->has_name = 1U;
        scan->scan_has_name_field = 1U;
    }

    scan->count++;
}

/**
 * @brief 逐行解析 AT+SCAN=ON 响应，提取候选从机。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @param scan 当前扫描过程的状态对象。该结果保存固定容量候选数组、解析统计和扫描错误，函数按只读或可写声明查询、追加、打印或选择候选。
 */
static void WirelessPairing_ParseScanResponse(const CH9141AtResponse *response,
                                              WirelessPairingScanResult *scan)
{
    char line[WIRELESS_PAIRING_LINE_TEXT_SIZE];
    uint8_t line_len = 0U;
    uint16_t line_count = 0U;

    if ((response == NULL) || (scan == NULL)) {
        return;
    }

    memset(scan, 0, sizeof(*scan));
    for (uint16_t i = 0U; i <= response->len; i++) {
        char c = response->text[i];

        if ((c == '\r') || (c == '\n') || (c == '\0')) {
            if (line_len > 0U) {
                line[line_len] = '\0';
                line_count++;
                WirelessPairing_AddCandidate(scan, line);
                line_len = 0U;
            }
        } else if (line_len < (sizeof(line) - 1U)) {
            line[line_len] = c;
            line_len++;
        }
    }

    printf("无线滑环匹配\t扫描响应解析\t响应长度=%u\t原始行数=%u\t候选数=%u\r\n",
           (unsigned)response->len,
           (unsigned)line_count,
           (unsigned)scan->count);
}

/**
 * @brief 打印扫描候选明细，包含解析字段和原始行，便于现场反查 CH9141K 固件输出格式。
 *
 * @param scan 当前扫描过程的状态对象。该结果保存固定容量候选数组、解析统计和扫描错误，函数按只读或可写声明查询、追加、打印或选择候选。
 */
static void WirelessPairing_PrintScan(const WirelessPairingScanResult *scan)
{
    uint8_t rssi_count = 0U;
    uint8_t name_count = 0U;
    uint8_t index_count = 0U;

    if (scan == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < scan->count; i++) {
        const WirelessPairingCandidate *candidate = &scan->candidates[i];
        if (candidate->has_rssi != 0U) {
            rssi_count++;
        }
        if (candidate->has_name != 0U) {
            name_count++;
        }
        if (candidate->index != 0U) {
            index_count++;
        }
    }

    printf("无线滑环匹配\t扫描候选数量：%u\t带序号=%u\t带RSSI=%u\t带名称=%u\r\n",
           (unsigned)scan->count,
           (unsigned)index_count,
           (unsigned)rssi_count,
           (unsigned)name_count);
    for (uint8_t i = 0U; i < scan->count; i++) {
        const WirelessPairingCandidate *candidate = &scan->candidates[i];
        printf("无线滑环匹配\t候选%u\t序号=%u\tMAC=%s",
               (unsigned)(i + 1U),
               (unsigned)candidate->index,
               candidate->mac);
        if (candidate->has_rssi != 0U) {
            printf("\tRSSI=%d dB", (int)candidate->rssi);
        } else {
            printf("\tRSSI=未知");
        }
        if (candidate->has_name != 0U) {
            printf("\tNAME=%s", candidate->name);
        }
        printf("\r\n");
        printf("无线滑环匹配\t候选%u原始\t%s\r\n", (unsigned)(i + 1U), candidate->line);
    }

    if ((scan->count > 0U) && (scan->scan_has_name_field == 0U)) {
        printf("无线滑环匹配\t提示：当前扫描结果没有名称字段，SPN名称匹配不可用\r\n");
    }
}

/**
 * @brief 复位 CH9141K，复位期间无 OK 响应按可接受超时处理。
 *
 * @return NO_ERROR 表示复位命令成功，或复位期间未收到 OK 但符合可接受超时语义；其他值为进入 AT、发送、异步等待、命令切换或恢复阶段的具体错误。
 */
static uint32_t WirelessPairing_ResetModule(void)
{
    CH9141AtResponse response;
    uint32_t ret;

    printf("无线滑环匹配\t复位CH9141K模块\t等待复位=%lu ms\r\n",
           (unsigned long)WIRELESS_PAIRING_RESET_WAIT_MS);
    ret = CH9141_AT_SendCommand("AT+RESET",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_RESET_ACK_TIMEOUT_MS,
                                &response);
    if (response.has_err != 0U) {
        WirelessPairing_PrintRet("无线滑环匹配\t复位命令返回错误", ret);
        return ret;
    }
    /* 复位命令除“未等到应答”外的发送或协议错误都立即失败；超时由下一分支按模块已经重启的兼容场景处理。 */
    if ((ret != NO_ERROR) && (ret != SENSOR_DEVICE_COMM_TIMEOUT)) {
        WirelessPairing_PrintRet("无线滑环匹配\t复位命令发送失败", ret);
        return ret;
    }
    /* CH9141K 可能在执行复位后立即断开 AT 应答，因此仅复位命令超时可视为已重启并继续等待上电。 */
    if (ret == SENSOR_DEVICE_COMM_TIMEOUT) {
        printf("无线滑环匹配\t复位命令未等到OK，按模块已重启处理并继续等待\r\n");
    }

    /* 复位命令后固定等待 WIRELESS_PAIRING_RESET_WAIT_MS，让 CH9141K 完成重启并重新进入可接收 AT 命令的状态。 */
    HAL_Delay(WIRELESS_PAIRING_RESET_WAIT_MS);
    WirelessPairing_PrintRet("无线滑环匹配\t复位等待完成", NO_ERROR);
    return NO_ERROR;
}

/**
 * @brief 判断 BLEMODE 查询响应是否表示主机模式。
 *
 * @param response 只读 CH9141 AT 响应快照；包含原始接收文字、有效长度、ACK/ERROR/提示符及异步 RSSI 等解析标志。
 * @return 1 表示 BLEMODE 响应解析成功，且模式值等于 WIRELESS_PAIRING_HOST_MODE；0 表示响应格式无法解析，或当前模式不是主机模式。
 */
static uint8_t WirelessPairing_ResponseHasHostMode(const CH9141AtResponse *response)
{
    uint8_t mode;

    if (WirelessPairing_ParseModeResponse(response, &mode, NULL, 0U) == 0U) {
        return 0U;
    }
    return (mode == WIRELESS_PAIRING_HOST_MODE) ? 1U : 0U;
}

/**
 * @brief 进入 AT 并确保 CH9141K 工作在主机模式。
 *
 * @return NO_ERROR 表示已进入 AT 且主机模式查询或切换确认成功；响应无法解析返回 WIRELESS_RESP_FORMAT_ERROR，最终模式仍非主机返回 WIRELESS_NOT_HOST_MODE，其他值保留具体 AT/UART/超时错误。
 */
static uint32_t WirelessPairing_EnterAtAndHostMode(void)
{
    CH9141AtResponse response;
    uint32_t ret;
    uint8_t mode = 0xFFU;

    printf("无线滑环匹配\t阶段：进入AT并确认主机模式\r\n");
    ret = CH9141_AT_EnterSoftwareMode(&response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t进入AT", ret);
        printf("无线滑环匹配\t进入AT响应=%s\r\n", response.text);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t进入AT", ret);

    printf("无线滑环匹配\t查询CH9141K BLEMODE\r\n");
    ret = CH9141_AT_SendCommand("AT+BLEMODE?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t查询主机模式", ret);
        printf("无线滑环匹配\tBLEMODE响应=%s\r\n", response.text);
        return ret;
    }

    if (WirelessPairing_ParseModeResponse(&response, &mode, NULL, 0U) == 0U) {
        printf("无线滑环匹配\tBLEMODE解析失败\t响应=%s\r\n", response.text);
        return WIRELESS_RESP_FORMAT_ERROR;
    }
    printf("无线滑环匹配\tBLEMODE\t模式=0x%02X(%s)\r\n",
           (unsigned)mode,
           WirelessPairing_ModeName(mode));

    if (WirelessPairing_ResponseHasHostMode(&response) != 0U) {
        printf("无线滑环匹配\t主机模式确认\t结果=已是主机模式\r\n");
        return NO_ERROR;
    }

    printf("无线滑环匹配\t主机模式确认\t结果=需要切换\t目标=0x01(主机模式)\r\n");
    ret = CH9141_AT_SendCommand("AT+BLEMODE=1",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t设置主机模式", ret);
        printf("无线滑环匹配\t设置主机模式响应=%s\r\n", response.text);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t设置主机模式", ret);

    ret = WirelessPairing_ResetModule();
    if (ret != NO_ERROR) {
        return ret;
    }

    printf("无线滑环匹配\t主机模式复位后重新进入AT\r\n");
    ret = CH9141_AT_EnterSoftwareMode(&response);
    WirelessPairing_PrintRet("无线滑环匹配\t重新进入AT", ret);
    if (ret != NO_ERROR) {
        printf("无线滑环匹配\t重新进入AT响应=%s\r\n", response.text);
        return ret;
    }

    ret = CH9141_AT_SendCommand("AT+BLEMODE?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        return ret;
    }
    if ((WirelessPairing_ParseModeResponse(&response, &mode, NULL, 0U) == 0U) ||
        (mode != WIRELESS_PAIRING_HOST_MODE)) {
        printf("无线滑环匹配\t切换后仍不是主机模式\t响应=%s\r\n", response.text);
        return WIRELESS_NOT_HOST_MODE;
    }
    return NO_ERROR;
}

/**
 * @brief 扫描 CH9141K 可见从机，并解析候选列表。
 *
 * @param scan 当前扫描过程的状态对象。该结果保存固定容量候选数组、解析统计和扫描错误，函数按只读或可写声明查询、追加、打印或选择候选。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t WirelessPairing_Scan(WirelessPairingScanResult *scan)
{
    CH9141AtResponse response;
    uint32_t ret;
    uint32_t disconn_ret;

    if (scan == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    printf("无线滑环匹配\t阶段：扫描准备\t超时=%lu ms\r\n",
           (unsigned long)WIRELESS_PAIRING_SCAN_TIMEOUT_MS);
    ret = WirelessPairing_EnterAtAndHostMode();
    if (ret != NO_ERROR) {
        return ret;
    }

    disconn_ret = CH9141_AT_SendCommand("AT+DISCONN",
                                        CH9141_AT_WAIT_ACK,
                                        WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                        &response);
    if (disconn_ret == NO_ERROR) {
        printf("无线滑环匹配\t扫描前断开旧连接\t结果=已发送断开请求\r\n");
    } else {
        printf("无线滑环匹配\t断开旧连接返回非成功，继续扫描\t错误码=0x%08lX\t响应=%s\r\n",
               (unsigned long)disconn_ret,
               response.text);
    }

    printf("无线滑环匹配\t开始扫描\r\n");
    ret = CH9141_AT_SendCommand("AT+SCAN=ON",
                                CH9141_AT_WAIT_SCAN_END,
                                WIRELESS_PAIRING_SCAN_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t扫描", ret);
        return ret;
    }

    WirelessPairing_ParseScanResponse(&response, scan);
    WirelessPairing_PrintScan(scan);
    if (scan->count == 0U) {
        printf("无线滑环匹配\t扫描完成但没有解析到候选从机\r\n");
        return WIRELESS_SCAN_NO_DEVICE;
    }
    return NO_ERROR;
}

/**
 * @brief 按近距离 RSSI 策略选择候选。
 *
 * 单候选只要求 RSSI 高于阈值；多候选还要求最强信号比第二名至少强指定差值。
 *
 * @param scan 当前扫描过程的状态对象。该结果保存固定容量候选数组、解析统计和扫描错误，函数按只读或可写声明查询、追加、打印或选择候选。
 * @param selected 用于接收 RSSI 最大且满足可连接条件的扫描候选记录。
 * @return 1 表示已按 RSSI 门限和唯一性规则选出候选；无候选或存在歧义时返回 0。
 */
static uint8_t WirelessPairing_SelectByRssi(const WirelessPairingScanResult *scan,
                                            const WirelessPairingCandidate **selected)
{
    const WirelessPairingCandidate *best = NULL;
    int16_t second_rssi = -128;
    uint8_t rssi_count = 0U;

    if ((scan == NULL) || (selected == NULL)) {
        return 0U;
    }

    *selected = NULL;
    for (uint8_t i = 0U; i < scan->count; i++) {
        const WirelessPairingCandidate *candidate = &scan->candidates[i];
        if (candidate->has_rssi == 0U) {
            printf("无线滑环匹配\tRSSI选择\t候选缺少RSSI\tMAC=%s\r\n", candidate->mac);
            continue;
        }
        rssi_count++;

        if ((best == NULL) || (candidate->rssi > best->rssi)) {
            if (best != NULL) {
                second_rssi = best->rssi;
            }
            best = candidate;
        } else if (candidate->rssi > second_rssi) {
            second_rssi = candidate->rssi;
        }
    }

    if (best == NULL) {
        printf("无线滑环匹配\tRSSI选择失败\t没有带RSSI的候选\r\n");
        return 0U;
    }
    if (best->rssi < WIRELESS_PAIRING_RSSI_NEAR_THRESHOLD) {
        printf("无线滑环匹配\tRSSI选择失败\t最强信号不足\tMAC=%s\tRSSI=%d dB\t阈值=%d dB\r\n",
               best->mac,
               (int)best->rssi,
               WIRELESS_PAIRING_RSSI_NEAR_THRESHOLD);
        return 0U;
    }
    if ((scan->count > 1U) && (rssi_count < 2U)) {
        printf("无线滑环匹配\tRSSI选择失败\t多候选但可比较RSSI不足\t候选数=%u\t带RSSI=%u\r\n",
               (unsigned)scan->count,
               (unsigned)rssi_count);
        return 0U;
    }
    if ((scan->count > 1U) && ((best->rssi - second_rssi) < WIRELESS_PAIRING_RSSI_MIN_GAP_DB)) {
        printf("无线滑环匹配\tRSSI选择失败\t差值不足\t最强MAC=%s\t最强RSSI=%d dB\t第二RSSI=%d dB\t差值=%d dB\t要求=%d dB\r\n",
               best->mac,
               (int)best->rssi,
               (int)second_rssi,
               (int)(best->rssi - second_rssi),
               WIRELESS_PAIRING_RSSI_MIN_GAP_DB);
        return 0U;
    }

    printf("无线滑环匹配\tRSSI选择通过\tMAC=%s\tRSSI=%d dB\t候选数=%u\t带RSSI=%u\r\n",
           best->mac,
           (int)best->rssi,
           (unsigned)scan->count,
           (unsigned)rssi_count);
    *selected = best;
    return 1U;
}

/**
 * @brief 按显式名称字段选择唯一候选，并区分未找到与名称重复。
 *
 * @param scan 当前扫描过程的状态对象。该结果保存固定容量候选数组、解析统计和扫描错误，函数按只读或可写声明查询、追加、打印或选择候选。
 * @param target_name 需要精确匹配的目标设备名称，比较时遵循扫描名称的大小写规则。
 * @param selected 用于接收名称匹配且 RSSI 最优的扫描候选记录。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t WirelessPairing_SelectByName(const WirelessPairingScanResult *scan,
                                             const char *target_name,
                                             const WirelessPairingCandidate **selected)
{
    const WirelessPairingCandidate *match = NULL;

    if ((scan == NULL) || (target_name == NULL) || (selected == NULL) || (*target_name == '\0')) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    *selected = NULL;
    printf("无线滑环匹配\t名称选择\t目标名称=%s\r\n", target_name);
    for (uint8_t i = 0U; i < scan->count; i++) {
        const WirelessPairingCandidate *candidate = &scan->candidates[i];
        if ((candidate->has_name != 0U) && (strcmp(candidate->name, target_name) == 0)) {
            printf("无线滑环匹配\t名称命中\t候选=%u\tMAC=%s\tNAME=%s\r\n",
                   (unsigned)(i + 1U),
                   candidate->mac,
                   candidate->name);
            if (match != NULL) {
                printf("无线滑环匹配\t名称重复\t目标=%s\t首个MAC=%s\t重复MAC=%s\r\n",
                       target_name,
                       match->mac,
                       candidate->mac);
                return WIRELESS_NAME_NOT_UNIQUE;
            }
            match = candidate;
        }
    }

    if (match == NULL) {
        return WIRELESS_NAME_NOT_FOUND;
    }

    *selected = match;
    return NO_ERROR;
}

/**
 * @brief 连接选中的候选并保存为 CH9141K 默认连接，然后验证透传链路。
 *
 * @param candidate 待校验或比较的候选值。该无线候选记录包含扫描索引、MAC、名称和 RSSI，用于缓存、发布或连接前复核。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t WirelessPairing_ConnectAndSave(const WirelessPairingCandidate *candidate)
{
    CH9141AtResponse response;
    uint32_t ret;
    char cmd[64];

    if (candidate == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    printf("无线滑环匹配\t选择目标\t序号=%u\tMAC=%s",
           (unsigned)candidate->index,
           candidate->mac);
    if (candidate->has_rssi != 0U) {
        printf("\tRSSI=%d dB", (int)candidate->rssi);
    }
    if (candidate->has_name != 0U) {
        printf("\tNAME=%s", candidate->name);
    }
    printf("\r\n");

    if (candidate->index != 0U) {
        snprintf(cmd, sizeof(cmd), "AT+LINK=%u,%s",
                 (unsigned)candidate->index,
                 WIRELESS_PAIRING_PASSWORD);
        printf("无线滑环匹配\t连接方式=按扫描序号LINK\t序号=%u\r\n", (unsigned)candidate->index);
    } else {
        snprintf(cmd, sizeof(cmd), "AT+CONN=%s,%s",
                 candidate->mac,
                 WIRELESS_PAIRING_PASSWORD);
        printf("无线滑环匹配\t连接方式=按MAC连接\tMAC=%s\r\n", candidate->mac);
    }

    ret = CH9141_AT_SendCommand(cmd,
                                CH9141_AT_WAIT_LINK,
                                WIRELESS_PAIRING_LINK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t连接目标", ret);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t连接目标", ret);

    printf("无线滑环匹配\t保存默认连接\tMAC=%s\r\n", candidate->mac);
    snprintf(cmd, sizeof(cmd), "AT+CONADD=%s,%s", candidate->mac, WIRELESS_PAIRING_PASSWORD);
    ret = CH9141_AT_SendCommand(cmd,
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t保存默认连接", ret);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t保存默认连接", ret);

    ret = WirelessPairing_ResetModule();
    if (ret != NO_ERROR) {
        return ret;
    }

    printf("无线滑环匹配\t复位完成，检查蓝牙主机和从机连接状态\r\n");
    ret = WirelessPairing_CheckBluetoothLink();
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t复位后蓝牙链路检查", ret);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t复位后蓝牙链路检查", ret);

    WirelessPairing_RememberPeer(candidate);
    printf("无线滑环匹配\t已缓存当前连接从机\tMAC=%s", candidate->mac);
    if (candidate->has_name != 0U) {
        printf("\tNAME=%s", candidate->name);
    } else {
        printf("\tNAME=未知");
    }
    printf("\r\n");

    printf("无线滑环匹配\t完成，蓝牙链路检查正常\r\n");
    return NO_ERROR;
}

/**
 * @brief 收尾维护调试流程，失败时尽量复位模块恢复透传，最终不设置设备错误态。
 *
 * @param title 用于标识本次无线维护流程的只读标题文字。
 * @param ret 上一层调用返回的结果码。无线配对流程按调用点将其打印、映射链路错误或作为最终状态发布。
 */
static void WirelessPairing_Finish(const char *title, uint32_t ret)
{
    char stage[64];

    if (title == NULL) {
        title = "无线滑环匹配";
    }

    snprintf(stage, sizeof(stage), "%s\t最终结果", title);
    WirelessPairing_PrintRet(stage, ret);
    if (ret != NO_ERROR) {
        uint32_t reset_ret;

        printf("%s\t失败后复位模块，恢复透传\r\n", title);
        reset_ret = WirelessPairing_ResetModule();
        snprintf(stage, sizeof(stage), "%s\t失败恢复复位", title);
        WirelessPairing_PrintRet(stage, reset_ret);
        (void)CH9141_AT_CompleteTransparentHandoff(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
    }

    if (g_measurement.device_status.device_state == STATE_MAINTENANCEMODE) {
        /* 串口调试匹配失败只打印结果，不把设备挂入最终错误态。 */
        g_measurement.device_status.device_state = STATE_STANDBY;
        printf("%s\t设备状态恢复为待机\r\n", title);
    }
    printf("===== %s结束 =====\r\n\r\n", title);
}


/**
 * @brief 读取连接模式、链路状态和对端 MAC。
 *
 * 函数先复位调用方状态对象并进入 CH9141 软件 AT 模式，依次查询 BLEMODE 和 BLESTA；非主机模式或主机未连接属于链路业务状态，写入
 * status->error_code，但不会伪装成 AT 命令执行失败。
 * 确认主机已连接后查询并规范化对端 MAC，再发送带周期参数的 AT+RSSI=ON，等待一条异步 RSSI 上报并解析为 dB；RSSI 读取失败不撤销已经确认的连接和 MAC。
 * AT+RSSI=ON 发送前即置位清理责任，覆盖命令已生效但 ACK 只收到一半的情况；只要尝试过启用 RSSI，finish 路径就必须发送 AT+RSSI=OFF。
 * 关闭 RSSI 失败时调用 CH9141_AT_RecoverRssiQuery 执行有界强制清理并退出 AT；普通关闭成功后再发送 AT+EXIT，最后为共享 UART6 透传留出复位后静默时间。
 * 执行返回码 ret 与链路业务结果 status->error_code 分开维护：调用方必须同时检查 ret、connection_valid、mac_valid、rssi_valid 和
 * error_code，不能仅凭 ret 等于 NO_ERROR 判定无线链路完整可用。
 *
 * @param status 用于接收连接、MAC、RSSI 有效标志、规范化对端 MAC、RSSI dB 值及链路业务错误码的可写状态对象；函数入口先整体复位。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示 status 为空；STATE_SWITCH 表示查询或清理被新命令中断；NO_ERROR
 *         仅表示执行流程已安全结束，链路未连接或 RSSI 无效仍可能记录在 status；其他值为进入 AT、查询或退出 AT 失败。
 * @note 任何可能启用异步 RSSI 的路径都必须先取得清理责任；此顺序用于防止残留 RSSI 文本污染后续 DSM 水位 Cl 请求。
 */
uint32_t WirelessPairing_ReadConnectionStatus(WirelessConnectionStatus *status)
{
    CH9141AtResponse response;
    uint32_t ret = NO_ERROR;
    uint32_t exit_ret = NO_ERROR;
    uint32_t rssi_stop_ret = NO_ERROR;
    uint32_t rssi_ret = NO_ERROR;
    uint32_t recover_ret = NO_ERROR;
    uint32_t prepare_ret = NO_ERROR;
    uint8_t at_entered = 0U;
    uint8_t mode = 0xFFU;
    uint8_t ble_status = 0xFFU;
    uint8_t rssi_enable_attempted = 0U;
    char data_line[WIRELESS_PAIRING_LINE_TEXT_SIZE];
    char mac[WIRELESS_PAIRING_MAC_TEXT_SIZE] = {0};
    char rssi_cmd[32];
    int16_t rssi = 0;

    if (status == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    WirelessPairing_ResetConnectionStatus(status);

    ret = CH9141_AT_EnterSoftwareMode(&response);
    if (ret != NO_ERROR) {
        status->error_code = ret;
        goto finish;
    }
    at_entered = 1U;

    ret = CH9141_AT_SendCommand("AT+BLEMODE?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        status->error_code = ret;
        goto finish;
    }
    if (WirelessPairing_ParseModeResponse(&response, &mode, data_line, sizeof(data_line)) == 0U) {
        ret = WIRELESS_RESP_FORMAT_ERROR;
        status->error_code = ret;
        goto finish;
    }

    ret = CH9141_AT_SendCommand("AT+BLESTA?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        status->error_code = ret;
        goto finish;
    }
    if (WirelessPairing_ParseStatusResponse(&response, &ble_status, data_line, sizeof(data_line)) == 0U) {
        ret = WIRELESS_RESP_FORMAT_ERROR;
        status->error_code = ret;
        goto finish;
    }

    if (mode != WIRELESS_PAIRING_HOST_MODE) {
        ret = NO_ERROR;
        status->error_code = WIRELESS_HOST_COMM_TIMEOUT;
        goto finish;
    }

    if (ble_status != WIRELESS_PAIRING_HOST_CONNECTED_STATE) {
        ret = NO_ERROR;
        status->error_code = WIRELESS_SLAVE_COMM_TIMEOUT;
        goto finish;
    }

    status->connection_valid = 1U;

    ret = CH9141_AT_SendCommand("AT+CCADD?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        status->error_code = ret;
        goto finish;
    }
    if ((WirelessPairing_CopyMacFromLine(response.text, mac) == 0U) ||
        (WirelessPairing_FillConnectionMac(status, mac) == 0U)) {
        ret = WIRELESS_RESP_FORMAT_ERROR;
        status->error_code = ret;
        goto finish;
    }

    snprintf(rssi_cmd, sizeof(rssi_cmd), "AT+RSSI=ON,%lu",
             (unsigned long)WIRELESS_PAIRING_RSSI_REPORT_PERIOD_MS);
    /* 命令可能已被模块执行但 ACK 未收完整，发送前即标记为必须清理。 */
    rssi_enable_attempted = 1U;
    rssi_ret = CH9141_AT_SendCommand(rssi_cmd,
                                      CH9141_AT_WAIT_ACK,
                                      WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                      &response);
    if (rssi_ret == NO_ERROR) {
        rssi_ret = CH9141_AT_WaitAsync(CH9141_AT_WAIT_RSSI,
                                       WIRELESS_PAIRING_RSSI_ASYNC_TIMEOUT_MS,
                                       &response);
        if ((rssi_ret == NO_ERROR) && (WirelessPairing_ParseRssiResponse(&response, &rssi) != 0U)) {
            status->rssi_valid = 1U;
            status->rssi = (int32_t)rssi;
        } else if (rssi_ret == NO_ERROR) {
            rssi_ret = WIRELESS_RESP_FORMAT_ERROR;
        }
    }

    if (rssi_ret != NO_ERROR) {
        status->error_code = rssi_ret;
    }
    ret = (rssi_ret == STATE_SWITCH) ? STATE_SWITCH : NO_ERROR;

finish:
    if (rssi_enable_attempted != 0U) {
        rssi_stop_ret = CH9141_AT_SendCommand("AT+RSSI=OFF",
                                              CH9141_AT_WAIT_ACK,
                                              WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                              &response);
        if ((status->error_code == NO_ERROR) && (rssi_stop_ret != NO_ERROR)) {
            status->error_code = rssi_stop_ret;
        }
        if (rssi_stop_ret == STATE_SWITCH) {
            ret = STATE_SWITCH;
        }
        if (rssi_stop_ret != NO_ERROR) {
            /* 普通清理被半 ACK、UART 故障或命令切换打断时，强制关闭 RSSI 并退出 AT。 */
            recover_ret = CH9141_AT_RecoverRssiQuery();
            at_entered = 0U;
            if (recover_ret != NO_ERROR) {
                ret = recover_ret;
                status->error_code = recover_ret;
            }
        }
    }

    if (at_entered != 0U) {
        exit_ret = CH9141_AT_SendCommand("AT+EXIT",
                                         CH9141_AT_WAIT_ACK,
                                         WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                         &response);
        if (exit_ret != NO_ERROR) {
            /* AT EXIT 可能已执行但 ACK 丢失，也可能仍停留在 AT 模式，统一执行有界强制清理。 */
            recover_ret = CH9141_AT_RecoverRssiQuery();
            if (recover_ret != NO_ERROR) {
                ret = recover_ret;
                status->error_code = recover_ret;
            }
        } else {
            prepare_ret = CH9141_AT_CompleteTransparentHandoff(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
            if (prepare_ret != NO_ERROR) {
                ret = prepare_ret;
                status->error_code = prepare_ret;
            }
        }
        if ((ret == NO_ERROR) && (exit_ret != NO_ERROR)) {
            ret = exit_ret;
        }
        if ((status->error_code == NO_ERROR) && (exit_ret != NO_ERROR)) {
            status->error_code = exit_ret;
        }
    }

    return ret;
}

/**
 * @brief 读取并发布一次 CH9141 无线连接状态快照。
 *
 * 函数在栈上建立完整 WirelessConnectionStatus，调用 WirelessPairing_ReadConnectionStatus
 * 填充；若执行失败但读取函数尚未写入业务错误码，则用执行返回码补齐 error_code。
 * 无论读取成功、链路未连接还是 AT 清理失败，都会通过 WirelessPairing_PublishConnectionStatus 整体发布本次快照，使 CPU3
 * 看见同一代际的有效标志、MAC、RSSI 和错误码。
 *
 * @return 返回 WirelessPairing_ReadConnectionStatus 的执行码；NO_ERROR 不保证链路已连接，最终连接、MAC、RSSI
 *         和业务错误必须读取本次已发布快照。
 * @note 发布快照不等同于连接成功；调用方应依据快照有效标志和 error_code 判断链路状态。
 */
uint32_t WirelessPairing_UpdateConnectionStatusSnapshot(void)
{
    WirelessConnectionStatus status;
    uint32_t ret;

    ret = WirelessPairing_ReadConnectionStatus(&status);
    if ((ret != NO_ERROR) && (status.error_code == NO_ERROR)) {
        status.error_code = ret;
    }
    WirelessPairing_PublishConnectionStatus(&status);
    return ret;
}

/**
 * @brief 查询并打印连接模式、状态、对端 MAC、名称和 RSSI，区分查询失败与链路未连接。
 *
 * 查询开始时保存当前设备状态并临时进入维护模式，随后进入 CH9141K 软件 AT 模式，依次读取 BLEMODE 和 BLESTA；非主机已连接状态属于有效查询结果，打印未连接后返回
 * NO_ERROR。
 * 确认主机已连接后读取 CCADD 对端 MAC；名称不能从当前连接直接查询，只能按 MAC 查找最近一次 SPR 或 SPN 成功匹配时保存的缓存。
 * RSSI 查询先发送 AT+RSSI=ON 并单独等待异步上报。开启命令可能已生效但 ACK 不完整，因此发送前就登记清理责任，查询结束始终尝试 AT+RSSI=OFF。
 * 关闭 RSSI 或退出 AT 未确认时调用 CH9141_AT_RecoverRssiQuery 执行有界强制清理，再等待 UART6 连续空闲，避免异步 RSSI 文本污染后续 DSM 透传响应。
 * 最终结果优先保留查询或解析错误，并在函数仍持有临时维护状态时恢复查询前设备状态；本函数只查询，不扫描、不主动断开，也不保存默认连接。
 *
 * @return 返回无线状态查询结果码；NO_ERROR 表示模式、状态及可用详情已完成解析和打印，其他值定位查询或解析失败。
 * @note RSSI 读取失败不改变已经确认的连接判定，但关闭 RSSI 或退出 AT 失败会作为最终错误返回，因为此时后续共享 UART6 透传链路仍存在污染风险。
 */
uint32_t WirelessPairing_PrintConnectionStatus(void)
{
    CH9141AtResponse response;
    uint32_t ret;
    uint32_t exit_ret = NO_ERROR;
    uint32_t rssi_stop_ret = NO_ERROR;
    uint32_t rssi_ret = NO_ERROR;
    uint32_t recover_ret = NO_ERROR;
    uint32_t prepare_ret = NO_ERROR;
    uint8_t at_entered = 0U;
    uint8_t mode = 0xFFU;
    uint8_t status = 0xFFU;
    char data_line[WIRELESS_PAIRING_LINE_TEXT_SIZE];
    char mac[WIRELESS_PAIRING_MAC_TEXT_SIZE] = {0};
    char rssi_cmd[32];
    int16_t rssi = 0;
    uint8_t has_rssi = 0U;
    uint8_t rssi_enable_attempted = 0U;
    const char *cached_name;
    DeviceState previous_state = g_measurement.device_status.device_state;

    printf("\r\n===== 无线滑环连接状态查询开始 =====\r\n");
    printf("无线滑环连接状态\t命令=SPC\t动作=只读查询，不扫描、不断开、不保存默认连接\r\n");
    printf("无线滑环连接状态\t说明=CH9141K当前连接查询只能直接读取MAC，名称来自最近一次SPR/SPN成功匹配缓存\r\n");
    printf("无线滑环连接状态\t查询前设备状态=0x%04lX，临时进入维护模式\r\n",
           (unsigned long)previous_state);
    g_measurement.device_status.device_state = STATE_MAINTENANCEMODE;

    ret = CH9141_AT_EnterSoftwareMode(&response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t进入AT", ret);
        printf("无线滑环连接状态\t进入AT响应=%s\r\n", response.text);
        goto finish;
    }
    at_entered = 1U;
    WirelessPairing_PrintRet("无线滑环连接状态\t进入AT", ret);

    ret = CH9141_AT_SendCommand("AT+BLEMODE?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t查询BLEMODE", ret);
        goto finish;
    }
    if (WirelessPairing_ParseModeResponse(&response, &mode, data_line, sizeof(data_line)) == 0U) {
        printf("无线滑环连接状态\tBLEMODE解析失败\t响应=%s\r\n", response.text);
        ret = WIRELESS_RESP_FORMAT_ERROR;
        goto finish;
    }
    printf("无线滑环连接状态\tBLEMODE\t原始=%s\t模式=0x%02X(%s)\r\n",
           data_line,
           (unsigned)mode,
           WirelessPairing_ModeName(mode));

    ret = CH9141_AT_SendCommand("AT+BLESTA?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t查询BLESTA", ret);
        goto finish;
    }
    if (WirelessPairing_ParseStatusResponse(&response, &status, data_line, sizeof(data_line)) == 0U) {
        printf("无线滑环连接状态\tBLESTA解析失败\t响应=%s\r\n", response.text);
        ret = WIRELESS_RESP_FORMAT_ERROR;
        goto finish;
    }
    printf("无线滑环连接状态\tBLESTA\t原始=%s\t状态=0x%02X(%s)\r\n",
           data_line,
           (unsigned)status,
           WirelessPairing_StatusName(mode, status));

    if ((mode != WIRELESS_PAIRING_HOST_MODE) || (status != WIRELESS_PAIRING_HOST_CONNECTED_STATE)) {
        printf("无线滑环连接状态\t未连接\t模式=0x%02X(%s)\t状态=0x%02X(%s)\r\n",
               (unsigned)mode,
               WirelessPairing_ModeName(mode),
               (unsigned)status,
               WirelessPairing_StatusName(mode, status));
        ret = NO_ERROR;
        goto finish;
    }

    ret = CH9141_AT_SendCommand("AT+CCADD?",
                                CH9141_AT_WAIT_ACK,
                                WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                &response);
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t查询连接MAC", ret);
        goto finish;
    }
    if (WirelessPairing_CopyMacFromLine(response.text, mac) == 0U) {
        printf("无线滑环连接状态\tCCADD解析失败\tBLESTA已连接但未读到MAC\t响应=%s\r\n", response.text);
        ret = WIRELESS_RESP_FORMAT_ERROR;
        goto finish;
    }
    printf("无线滑环连接状态\tCCADD\tMAC=%s\r\n", mac);

    snprintf(rssi_cmd, sizeof(rssi_cmd), "AT+RSSI=ON,%lu",
             (unsigned long)WIRELESS_PAIRING_RSSI_REPORT_PERIOD_MS);
    printf("无线滑环连接状态\tRSSI读取\t协议=先ACK后异步上报\t上报周期=%lu ms\t异步等待=%lu ms\r\n",
           (unsigned long)WIRELESS_PAIRING_RSSI_REPORT_PERIOD_MS,
           (unsigned long)WIRELESS_PAIRING_RSSI_ASYNC_TIMEOUT_MS);
    /* 命令可能已被模块执行但 ACK 未收完整，发送前即标记为必须清理。 */
    rssi_enable_attempted = 1U;
    rssi_ret = CH9141_AT_SendCommand(rssi_cmd,
                                      CH9141_AT_WAIT_ACK,
                                      WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                      &response);
    if (rssi_ret == NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t打开RSSI上报", rssi_ret);

        /* AT+RSSI=ON,<周期> 只直接返回 OK；RSSI 值随后异步上报，需要单独等待。 */
        rssi_ret = CH9141_AT_WaitAsync(CH9141_AT_WAIT_RSSI,
                                       WIRELESS_PAIRING_RSSI_ASYNC_TIMEOUT_MS,
                                       &response);
        /* 只有异步等待成功且文本确实解析出 RSSI 数值时才发布信号强度，收到无 RSSI 的文本仍属于格式错误。 */
        if ((rssi_ret == NO_ERROR) && (WirelessPairing_ParseRssiResponse(&response, &rssi) != 0U)) {
            has_rssi = 1U;
            printf("无线滑环连接状态\tRSSI读取成功\tRSSI=%d dB\r\n", (int)rssi);
        } else {
            if (rssi_ret == NO_ERROR) {
                rssi_ret = WIRELESS_RESP_FORMAT_ERROR;
                printf("无线滑环连接状态\tRSSI解析失败\t说明=收到异步数据但未解析到RSSI数值\r\n");
            } else if (rssi_ret == SENSOR_DEVICE_COMM_TIMEOUT) {
                printf("无线滑环连接状态\tRSSI异步上报超时\t说明=模块已ACK但等待窗口内未输出RSSI，连接状态仍按已连接处理\r\n");
            }
            WirelessPairing_PrintRet("无线滑环连接状态\t等待RSSI异步上报", rssi_ret);
            printf("无线滑环连接状态\tRSSI异步响应=%s\r\n", response.text);
        }
    } else {
        WirelessPairing_PrintRet("无线滑环连接状态\t打开RSSI上报", rssi_ret);
        printf("无线滑环连接状态\tRSSI打开响应=%s\r\n", response.text);
    }

    if (rssi_enable_attempted != 0U) {
        rssi_stop_ret = CH9141_AT_SendCommand("AT+RSSI=OFF",
                                              CH9141_AT_WAIT_ACK,
                                              WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                              &response);
        WirelessPairing_PrintRet("无线滑环连接状态\t关闭RSSI读取", rssi_stop_ret);
        if (rssi_stop_ret != NO_ERROR) {
            /* 普通清理失败时立即强制关闭 RSSI 并退出 AT，避免污染后续 DSM 透传响应。 */
            recover_ret = CH9141_AT_RecoverRssiQuery();
            at_entered = 0U;
            if (recover_ret != NO_ERROR) {
                ret = recover_ret;
            }
        }
        /* 关闭 RSSI 上报失败时只在主查询尚未失败的情况下提升为最终错误，保留更早的真实根因。 */
        if ((ret == NO_ERROR) && (rssi_stop_ret != NO_ERROR)) {
            ret = rssi_stop_ret;
        }
    } else {
        printf("无线滑环连接状态\t关闭RSSI读取\t跳过=未尝试开启RSSI上报\r\n");
    }

    cached_name = WirelessPairing_GetCachedName(mac);
    printf("无线滑环连接状态\t已连接\t从机名称=%s\tMAC=%s",
           (cached_name != NULL) ? cached_name : "未知(当前连接名称无法从CH9141K直接查询)",
           mac);
    if (has_rssi != 0U) {
        printf("\tRSSI=%d dB", (int)rssi);
    } else {
        printf("\tRSSI=读取失败");
    }
    printf("\r\n");
    printf("无线滑环连接状态\t名称来源=%s\r\n",
           (cached_name != NULL) ? "最近一次成功匹配缓存" : "未命中缓存，CH9141K未提供当前连接名称查询");

finish:
    if (at_entered != 0U) {
        exit_ret = CH9141_AT_SendCommand("AT+EXIT",
                                         CH9141_AT_WAIT_ACK,
                                         WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                         &response);
        WirelessPairing_PrintRet("无线滑环连接状态\t退出AT", exit_ret);
        if (exit_ret != NO_ERROR) {
            /* 退出 AT 未确认时执行有界强制清理，保证下一条传感器命令回到透传链路。 */
            recover_ret = CH9141_AT_RecoverRssiQuery();
            if (recover_ret != NO_ERROR) {
                ret = recover_ret;
            }
        } else {
            prepare_ret = CH9141_AT_CompleteTransparentHandoff(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
            if (prepare_ret != NO_ERROR) {
                ret = prepare_ret;
            }
        }
        /* 退出 AT 模式失败同样只在此前无错误时成为最终结果，清理失败不能覆盖扫描或查询阶段的原始故障。 */
        if ((ret == NO_ERROR) && (exit_ret != NO_ERROR)) {
            ret = exit_ret;
        }
    }

    WirelessPairing_PrintRet("无线滑环连接状态\t最终结果", ret);
    if (g_measurement.device_status.device_state == STATE_MAINTENANCEMODE) {
        /* SPC 只是只读诊断，查询结束后恢复查询前状态，避免覆盖正在执行的测量流程。 */
        g_measurement.device_status.device_state = previous_state;
        printf("无线滑环连接状态\t设备状态恢复为查询前状态=0x%04lX\r\n",
               (unsigned long)previous_state);
    }
    printf("===== 无线滑环连接状态查询结束 =====\r\n\r\n");
    return ret;
}

/**
 * @brief 扫描并打印可配对的 CH9141K 候选设备。
 * @return 返回无线扫描结果码；NO_ERROR 表示扫描完成并已打印候选列表，其他值表示 AT 进入、扫描或收尾失败。
 */
uint32_t WirelessPairing_DebugScan(void)
{
    WirelessPairingScanResult scan;
    uint32_t ret;

    printf("\r\n===== 无线滑环扫描调试开始 =====\r\n");
    printf("无线滑环扫描调试\t命令=SPS\t动作=扫描候选，会临时断开当前连接，不保存默认连接\r\n");
    g_measurement.device_status.device_state = STATE_MAINTENANCEMODE;
    ret = WirelessPairing_Scan(&scan);
    if (ret == NO_ERROR) {
        printf("无线滑环扫描调试\t扫描完成后复位模块，恢复透传\r\n");
        ret = WirelessPairing_ResetModule();
        if (ret == NO_ERROR) {
            (void)CH9141_AT_CompleteTransparentHandoff(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
        }
    }
    WirelessPairing_Finish("无线滑环扫描调试", ret);
    return ret;
}

/**
 * @brief 按 RSSI 规则选择候选设备，执行配对并完成收尾处理。
 * @return 返回无线维护结果码；NO_ERROR 表示 RSSI 候选已成功配对，其他值区分扫描、选择、配对或收尾失败。
 */
static uint32_t WirelessPairing_RunByRssiInternal(
    WirelessPairingPostPairingValidator validator)
{
    WirelessPairingScanResult scan;
    const WirelessPairingCandidate *selected = NULL;
    uint32_t ret;

    printf("\r\n===== 无线滑环RSSI匹配开始 =====\r\n");
    printf("无线滑环RSSI匹配\t命令=SPR\t最强RSSI阈值=%d dB\t多候选差值=%d dB\r\n",
           WIRELESS_PAIRING_RSSI_NEAR_THRESHOLD,
           WIRELESS_PAIRING_RSSI_MIN_GAP_DB);
    WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_RUNNING, NULL, NO_ERROR);
    g_measurement.device_status.device_state = STATE_WIRELESS_PAIRING;
    ret = WirelessPairing_Scan(&scan);
    if (ret == NO_ERROR) {
        if (WirelessPairing_SelectByRssi(&scan, &selected) == 0U) {
            printf("无线滑环匹配\tRSSI条件不满足：要求最强RSSI>=%d dB，多候选差值>=%d dB\r\n",
                   WIRELESS_PAIRING_RSSI_NEAR_THRESHOLD,
                   WIRELESS_PAIRING_RSSI_MIN_GAP_DB);
            ret = SLIPRING_SIGNAL_WEAK;
        } else {
            ret = WirelessPairing_ConnectAndSave(selected);
        }
    }
    if ((ret == NO_ERROR) && (selected != NULL) && (validator != NULL)) {
        uint32_t validator_ret = validator();

        if (validator_ret != NO_ERROR) {
            ret = validator_ret;
        }
    }

    WirelessPairing_Finish("无线滑环RSSI匹配", ret);
    /* RSSI 匹配必须同时满足流程成功和存在已选设备，否则统一发布失败，避免成功状态携带空设备信息。 */
    if ((ret == NO_ERROR) && (selected != NULL)) {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_SUCCESS, selected, NO_ERROR);
    } else {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_FAILED, NULL, ret);
    }
    return ret;
}

uint32_t WirelessPairing_RunByRssi(void)
{
    return WirelessPairing_RunByRssiInternal(NULL);
}

uint32_t WirelessPairing_RunByRssiWithValidator(
    WirelessPairingPostPairingValidator validator)
{
    if (validator == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    return WirelessPairing_RunByRssiInternal(validator);
}

/**
 * @brief 按显式名称唯一匹配候选设备，执行配对并完成收尾处理。
 *
 * @param target_name 本次定向配对必须匹配的完整设备名称。
 * @return 返回无线维护结果码；NO_ERROR 表示唯一名称候选已成功配对，WIRELESS_NAME_INVALID 表示名称非法，其他值区分扫描、匹配或配对失败。
 */
uint32_t WirelessPairing_RunByName(const char *target_name)
{
    WirelessPairingScanResult scan;
    const WirelessPairingCandidate *selected = NULL;
    uint32_t ret;

    if ((target_name == NULL) || (*target_name == '\0') ||
        (strlen(target_name) >= WIRELESS_PAIRING_NAME_TEXT_SIZE)) {
        printf("无线滑环匹配\t名称参数无效，格式：SPN=<name>，长度小于18字节\r\n");
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_FAILED, NULL, WIRELESS_NAME_INVALID);
        return WIRELESS_NAME_INVALID;
    }

    printf("\r\n===== 无线滑环名称匹配开始：%s =====\r\n", target_name);
    printf("无线滑环名称匹配\t命令=SPN\t目标名称=%s\t匹配方式=显式名称字段完全匹配\r\n", target_name);
    WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_RUNNING, NULL, NO_ERROR);
    g_measurement.device_status.device_state = STATE_WIRELESS_PAIRING;
    ret = WirelessPairing_Scan(&scan);
    if (ret == NO_ERROR) {
        if (scan.scan_has_name_field == 0U) {
            printf("无线滑环匹配\t扫描结果未包含名称字段，不能按名称匹配\r\n");
            ret = WIRELESS_RESP_FORMAT_ERROR;
        } else {
            ret = WirelessPairing_SelectByName(&scan, target_name, &selected);
            if (ret == NO_ERROR) {
                ret = WirelessPairing_ConnectAndSave(selected);
            }
        }
    }
    WirelessPairing_Finish("无线滑环名称匹配", ret);
    /* 名称匹配同样要求成功返回且候选设备非空，两项缺一都不得发布成功快照。 */
    if ((ret == NO_ERROR) && (selected != NULL)) {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_SUCCESS, selected, NO_ERROR);
    } else {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_FAILED, NULL, ret);
    }
    return ret;
}
