/*
 * wireless_pairing.c
 *
 * CH9141K 无线滑环匹配辅助流程。
 */

#include "wireless_pairing.h"

#include "ch9141_at.h"
#include "error_log.h"
#include "sensor.h"
#include "system_parameter.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIRELESS_PAIRING_PASSWORD              "000000"
#define WIRELESS_PAIRING_MAX_CANDIDATES        8U
#define WIRELESS_PAIRING_MAC_TEXT_SIZE         18U
#define WIRELESS_PAIRING_NAME_TEXT_SIZE        19U
#define WIRELESS_PAIRING_LINE_TEXT_SIZE        128U
#define WIRELESS_PAIRING_SCAN_TIMEOUT_MS       12000U
#define WIRELESS_PAIRING_LINK_TIMEOUT_MS       10000U
#define WIRELESS_PAIRING_ACK_TIMEOUT_MS        1500U
#define WIRELESS_PAIRING_RSSI_REPORT_PERIOD_MS 1000U
#define WIRELESS_PAIRING_RSSI_ASYNC_TIMEOUT_MS  1500U
#define WIRELESS_PAIRING_RESET_ACK_TIMEOUT_MS  800U
#define WIRELESS_PAIRING_RESET_WAIT_MS         2500U
#define WIRELESS_PAIRING_POST_RESET_IDLE_MS    500U
#define WIRELESS_PAIRING_HOST_MODE             1U
#define WIRELESS_PAIRING_HOST_CONNECTED_STATE  0x03U
#define WIRELESS_PAIRING_SLAVE_CONNECTED_STATE 0x05U
#define WIRELESS_PAIRING_HOST_ADDR             1U
#define WIRELESS_PAIRING_SLAVE_ADDR            2U
#define WIRELESS_PAIRING_RSSI_NEAR_THRESHOLD   (-55)
#define WIRELESS_PAIRING_RSSI_MIN_GAP_DB       8

typedef struct {
    uint8_t index;                                  /* CH9141K 扫描结果序号，优先用于 AT+LINK。 */
    char mac[WIRELESS_PAIRING_MAC_TEXT_SIZE];       /* 统一转大写后的 MAC 文本。 */
    int16_t rssi;
    uint8_t has_rssi;
    char name[WIRELESS_PAIRING_NAME_TEXT_SIZE];
    uint8_t has_name;
    char line[WIRELESS_PAIRING_LINE_TEXT_SIZE];     /* 保留原始扫描行，便于现场核对解析规则。 */
} WirelessPairingCandidate;

typedef struct {
    WirelessPairingCandidate candidates[WIRELESS_PAIRING_MAX_CANDIDATES];
    uint8_t count;
    uint8_t scan_has_name_field;
} WirelessPairingScanResult;

/* CH9141K 当前连接查询只能读到 MAC；名称用本次运行期最近一次成功匹配的候选补充。 */
static char s_wireless_pairing_last_peer_mac[WIRELESS_PAIRING_MAC_TEXT_SIZE]; /* 无线滑环匹配模块级变量，保存跨函数共享的业务状态。 */
static char s_wireless_pairing_last_peer_name[WIRELESS_PAIRING_NAME_TEXT_SIZE]; /* 无线滑环匹配模块级变量，保存跨函数共享的业务状态。 */
static uint8_t s_wireless_pairing_last_peer_has_name; /* 无线滑环匹配模块级变量，保存跨函数共享的业务状态。 */

/**
 * @brief 打印普通结果摘要，不走 ErrorLog_*，避免维护调试失败被记成最终故障链路。
 */
static void WirelessPairing_PrintRet(const char *stage, uint32_t ret)
{
    const char *name = ErrorLog_GetCodeName(ret);
    const char *reason = ErrorLog_GetReasonByCode(ret);

    if (stage == NULL) {
        stage = "无线滑环匹配";
    }

    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
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
 */
static uint8_t WirelessPairing_IsHex(char c)
{
    return (((c >= '0') && (c <= '9')) ||
            ((c >= 'A') && (c <= 'F')) ||
            ((c >= 'a') && (c <= 'f'))) ? 1U : 0U;
}

/**
 * @brief 判断字符是否为 ASCII 字母或数字，用于避免从 MAC: 字段名中间误截 MAC。
 */
static uint8_t WirelessPairing_IsAsciiAlnum(char c)
{
    return (((c >= '0') && (c <= '9')) ||
            ((c >= 'A') && (c <= 'Z')) ||
            ((c >= 'a') && (c <= 'z'))) ? 1U : 0U;
}

/**
 * @brief 判断名称字段的结束符。
 */
static uint8_t WirelessPairing_IsDelimiter(char c)
{
    return ((c == '\0') || (c == ' ') || (c == '\t') ||
            (c == ',') || (c == ';') || (c == '\r') || (c == '\n')) ? 1U : 0U;
}

/**
 * @brief 大小写不敏感地判断字符串前缀。
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
 * @brief 执行无线滑环匹配中的 WirelessPairing_ParseIndexFromLine 逻辑。
 *
 * @param line 业务参数。
 * @param index 索引值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
 * @brief 发布无线滑环匹配状态给 CPU3；只发布数值字段，显示端本地格式化 MAC。
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
            publish_error = SENSOR_RESP_FORMAT_ERROR;
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
        /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
        if (publish_error == NO_ERROR) {
            publish_error = SENSOR_RESP_FORMAT_ERROR;
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
 */
static uint8_t WirelessPairing_IsValidModeValue(uint8_t value)
{
    return (value <= 0x02U) ? 1U : 0U;
}

/**
 * @brief 判断 BLESTA 数值是否为协议状态表中的合法状态。
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
 * @brief 执行无线滑环匹配中的 WirelessPairing_ParseModeResponse 逻辑。
 *
 * @param response 业务参数。
 * @param value 待处理数值。
 * @param out_line 业务参数。
 * @param out_size 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t WirelessPairing_ParseModeResponse(const CH9141AtResponse *response,
                                                uint8_t *value,
                                                char *out_line,
                                                uint16_t out_size)
{
    return WirelessPairing_ParseByteResponseFiltered(response, value, out_line, out_size, 0U);
}

/**
 * @brief 执行无线滑环匹配中的 WirelessPairing_ParseStatusResponse 逻辑。
 *
 * @param response 业务参数。
 * @param value 待处理数值。
 * @param out_line 业务参数。
 * @param out_size 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if ((ret != NO_ERROR) && (ret != SENSOR_DEVICE_COMM_TIMEOUT)) {
        WirelessPairing_PrintRet("无线滑环匹配\t复位命令发送失败", ret);
        return ret;
    }
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret == SENSOR_DEVICE_COMM_TIMEOUT) {
        printf("无线滑环匹配\t复位命令未等到OK，按模块已重启处理并继续等待\r\n");
    }

    /* 无线滑环匹配与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
    HAL_Delay(WIRELESS_PAIRING_RESET_WAIT_MS);
    WirelessPairing_PrintRet("无线滑环匹配\t复位等待完成", NO_ERROR);
    return NO_ERROR;
}

/**
 * @brief 判断 BLEMODE 查询响应是否表示主机模式。
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
 */
static uint32_t WirelessPairing_EnterAtAndHostMode(void)
{
    CH9141AtResponse response;
    uint32_t ret;
    uint8_t mode = 0xFFU;

    printf("无线滑环匹配\t阶段：进入AT并确认主机模式\r\n");
    ret = CH9141_AT_EnterSoftwareMode(&response);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t查询主机模式", ret);
        printf("无线滑环匹配\tBLEMODE响应=%s\r\n", response.text);
        return ret;
    }

    if (WirelessPairing_ParseModeResponse(&response, &mode, NULL, 0U) == 0U) {
        printf("无线滑环匹配\tBLEMODE解析失败\t响应=%s\r\n", response.text);
        return SENSOR_RESP_FORMAT_ERROR;
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t设置主机模式", ret);
        printf("无线滑环匹配\t设置主机模式响应=%s\r\n", response.text);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t设置主机模式", ret);

    ret = WirelessPairing_ResetModule();
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    printf("无线滑环匹配\t主机模式复位后重新进入AT\r\n");
    ret = CH9141_AT_EnterSoftwareMode(&response);
    WirelessPairing_PrintRet("无线滑环匹配\t重新进入AT", ret);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("无线滑环匹配\t重新进入AT响应=%s\r\n", response.text);
    }
    return ret;
}

/**
 * @brief 扫描 CH9141K 可见从机，并解析候选列表。
 */
static uint32_t WirelessPairing_Scan(WirelessPairingScanResult *scan)
{
    CH9141AtResponse response;
    uint32_t ret;
    uint32_t disconn_ret;

    if (scan == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    printf("无线滑环匹配\t阶段：扫描准备\t超时=%lu ms\r\n",
           (unsigned long)WIRELESS_PAIRING_SCAN_TIMEOUT_MS);
    ret = WirelessPairing_EnterAtAndHostMode();
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    disconn_ret = CH9141_AT_SendCommand("AT+DISCONN",
                                        CH9141_AT_WAIT_ACK,
                                        WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                        &response);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t扫描", ret);
        return ret;
    }

    WirelessPairing_ParseScanResponse(&response, scan);
    WirelessPairing_PrintScan(scan);
    if (scan->count == 0U) {
        printf("无线滑环匹配\t扫描完成但没有解析到候选从机\r\n");
        return WIRELESS_SLAVE_COMM_TIMEOUT;
    }
    return NO_ERROR;
}

/**
 * @brief 按近距离 RSSI 策略选择候选。
 *
 * 单候选只要求 RSSI 高于阈值；多候选还要求最强信号比第二名至少强指定差值。
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
 * @brief 按显式名称字段选择唯一候选。
 */
static uint8_t WirelessPairing_SelectByName(const WirelessPairingScanResult *scan,
                                            const char *target_name,
                                            const WirelessPairingCandidate **selected)
{
    const WirelessPairingCandidate *match = NULL;

    if ((scan == NULL) || (target_name == NULL) || (selected == NULL) || (*target_name == '\0')) {
        return 0U;
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
                return 0U;
            }
            match = candidate;
        }
    }

    if (match == NULL) {
        return 0U;
    }

    *selected = match;
    return 1U;
}

/**
 * @brief 连接选中的候选并保存为 CH9141K 默认连接，然后验证透传链路。
 */
static uint32_t WirelessPairing_ConnectAndSave(const WirelessPairingCandidate *candidate)
{
    CH9141AtResponse response;
    uint32_t ret;
    char cmd[64];

    if (candidate == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t保存默认连接", ret);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t保存默认连接", ret);

    ret = WirelessPairing_ResetModule();
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    printf("无线滑环匹配\t复位完成，恢复UART6透传并探测节点\r\n");
    (void)CH9141_AT_PrepareUart6(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
    ret = WIRELESS_ProbeNode(WIRELESS_PAIRING_HOST_ADDR);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t复位后主机探测", ret);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t复位后主机探测", ret);

    ret = WIRELESS_ProbeNode(WIRELESS_PAIRING_SLAVE_ADDR);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环匹配\t复位后从机探测", ret);
        return ret;
    }
    WirelessPairing_PrintRet("无线滑环匹配\t复位后从机探测", ret);

    WirelessPairing_RememberPeer(candidate);
    printf("无线滑环匹配\t已缓存当前连接从机\tMAC=%s", candidate->mac);
    if (candidate->has_name != 0U) {
        printf("\tNAME=%s", candidate->name);
    } else {
        printf("\tNAME=未知");
    }
    printf("\r\n");

    printf("无线滑环匹配\t完成，透传链路探测正常\r\n");
    return NO_ERROR;
}

/**
 * @brief 收尾维护调试流程，失败时尽量复位模块恢复透传，最终不设置设备错误态。
 */
static void WirelessPairing_Finish(const char *title, uint32_t ret)
{
    char stage[64];

    if (title == NULL) {
        title = "无线滑环匹配";
    }

    snprintf(stage, sizeof(stage), "%s\t最终结果", title);
    WirelessPairing_PrintRet(stage, ret);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        uint32_t reset_ret;

        printf("%s\t失败后复位模块，恢复透传\r\n", title);
        reset_ret = WirelessPairing_ResetModule();
        snprintf(stage, sizeof(stage), "%s\t失败恢复复位", title);
        WirelessPairing_PrintRet(stage, reset_ret);
        (void)CH9141_AT_PrepareUart6(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
    }

    if (g_measurement.device_status.device_state == STATE_MAINTENANCEMODE) {
        /* 串口调试匹配失败只打印结果，不把设备挂入最终错误态。 */
        g_measurement.device_status.device_state = STATE_STANDBY;
        printf("%s\t设备状态恢复为待机\r\n", title);
    }
    printf("===== %s结束 =====\r\n\r\n", title);
}


/**
 * @brief 显示或打印无线滑环匹配中的 WirelessPairing_PrintConnectionStatus 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t WirelessPairing_PrintConnectionStatus(void)
{
    CH9141AtResponse response;
    uint32_t ret;
    uint32_t exit_ret = NO_ERROR;
    uint32_t rssi_stop_ret = NO_ERROR;
    uint32_t rssi_ret = NO_ERROR;
    uint8_t at_entered = 0U;
    uint8_t mode = 0xFFU;
    uint8_t status = 0xFFU;
    char data_line[WIRELESS_PAIRING_LINE_TEXT_SIZE];
    char mac[WIRELESS_PAIRING_MAC_TEXT_SIZE] = {0};
    char rssi_cmd[32];
    int16_t rssi = 0;
    uint8_t has_rssi = 0U;
    uint8_t rssi_started = 0U;
    const char *cached_name;
    DeviceState previous_state = g_measurement.device_status.device_state;

    printf("\r\n===== 无线滑环连接状态查询开始 =====\r\n");
    printf("无线滑环连接状态\t命令=SPC\t动作=只读查询，不扫描、不断开、不保存默认连接\r\n");
    printf("无线滑环连接状态\t说明=CH9141K当前连接查询只能直接读取MAC，名称来自最近一次SPR/SPN成功匹配缓存\r\n");
    printf("无线滑环连接状态\t查询前设备状态=0x%04lX，临时进入维护模式\r\n",
           (unsigned long)previous_state);
    g_measurement.device_status.device_state = STATE_MAINTENANCEMODE;

    ret = CH9141_AT_EnterSoftwareMode(&response);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t查询BLEMODE", ret);
        goto finish;
    }
    if (WirelessPairing_ParseModeResponse(&response, &mode, data_line, sizeof(data_line)) == 0U) {
        printf("无线滑环连接状态\tBLEMODE解析失败\t响应=%s\r\n", response.text);
        ret = SENSOR_RESP_FORMAT_ERROR;
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t查询BLESTA", ret);
        goto finish;
    }
    if (WirelessPairing_ParseStatusResponse(&response, &status, data_line, sizeof(data_line)) == 0U) {
        printf("无线滑环连接状态\tBLESTA解析失败\t响应=%s\r\n", response.text);
        ret = SENSOR_RESP_FORMAT_ERROR;
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        WirelessPairing_PrintRet("无线滑环连接状态\t查询连接MAC", ret);
        goto finish;
    }
    if (WirelessPairing_CopyMacFromLine(response.text, mac) == 0U) {
        printf("无线滑环连接状态\tCCADD解析失败\tBLESTA已连接但未读到MAC\t响应=%s\r\n", response.text);
        ret = SENSOR_RESP_FORMAT_ERROR;
        goto finish;
    }
    printf("无线滑环连接状态\tCCADD\tMAC=%s\r\n", mac);

    snprintf(rssi_cmd, sizeof(rssi_cmd), "AT+RSSI=ON,%lu",
             (unsigned long)WIRELESS_PAIRING_RSSI_REPORT_PERIOD_MS);
    printf("无线滑环连接状态\tRSSI读取\t协议=先ACK后异步上报\t上报周期=%lu ms\t异步等待=%lu ms\r\n",
           (unsigned long)WIRELESS_PAIRING_RSSI_REPORT_PERIOD_MS,
           (unsigned long)WIRELESS_PAIRING_RSSI_ASYNC_TIMEOUT_MS);
    rssi_ret = CH9141_AT_SendCommand(rssi_cmd,
                                      CH9141_AT_WAIT_ACK,
                                      WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                      &response);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (rssi_ret == NO_ERROR) {
        rssi_started = 1U;
        WirelessPairing_PrintRet("无线滑环连接状态\t打开RSSI上报", rssi_ret);

        /* AT+RSSI=ON,<周期> 只直接返回 OK；RSSI 值随后异步上报，需要单独等待。 */
        rssi_ret = CH9141_AT_WaitAsync(CH9141_AT_WAIT_RSSI,
                                       WIRELESS_PAIRING_RSSI_ASYNC_TIMEOUT_MS,
                                       &response);
        /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
        if ((rssi_ret == NO_ERROR) && (WirelessPairing_ParseRssiResponse(&response, &rssi) != 0U)) {
            has_rssi = 1U;
            printf("无线滑环连接状态\tRSSI读取成功\tRSSI=%d dB\r\n", (int)rssi);
        } else {
            /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
            if (rssi_ret == NO_ERROR) {
                rssi_ret = SENSOR_RESP_FORMAT_ERROR;
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

    if (rssi_started != 0U) {
        rssi_stop_ret = CH9141_AT_SendCommand("AT+RSSI=OFF",
                                              CH9141_AT_WAIT_ACK,
                                              WIRELESS_PAIRING_ACK_TIMEOUT_MS,
                                              &response);
        WirelessPairing_PrintRet("无线滑环连接状态\t关闭RSSI读取", rssi_stop_ret);
        /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
        if ((ret == NO_ERROR) && (rssi_stop_ret != NO_ERROR)) {
            ret = rssi_stop_ret;
        }
    } else {
        printf("无线滑环连接状态\t关闭RSSI读取\t跳过=RSSI上报未成功开启\r\n");
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
        (void)CH9141_AT_PrepareUart6(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
        /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
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
 * @brief 执行无线滑环匹配中的 WirelessPairing_DebugScan 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t WirelessPairing_DebugScan(void)
{
    WirelessPairingScanResult scan;
    uint32_t ret;

    printf("\r\n===== 无线滑环扫描调试开始 =====\r\n");
    printf("无线滑环扫描调试\t命令=SPS\t动作=扫描候选，会临时断开当前连接，不保存默认连接\r\n");
    g_measurement.device_status.device_state = STATE_MAINTENANCEMODE;
    ret = WirelessPairing_Scan(&scan);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret == NO_ERROR) {
        printf("无线滑环扫描调试\t扫描完成后复位模块，恢复透传\r\n");
        ret = WirelessPairing_ResetModule();
        /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            (void)CH9141_AT_PrepareUart6(WIRELESS_PAIRING_POST_RESET_IDLE_MS);
        }
    }
    WirelessPairing_Finish("无线滑环扫描调试", ret);
    return ret;
}

/**
 * @brief 执行无线滑环匹配中的 WirelessPairing_RunByRssi 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t WirelessPairing_RunByRssi(void)
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
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
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
    WirelessPairing_Finish("无线滑环RSSI匹配", ret);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if ((ret == NO_ERROR) && (selected != NULL)) {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_SUCCESS, selected, NO_ERROR);
    } else {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_FAILED, NULL, ret);
    }
    return ret;
}

/**
 * @brief 执行无线滑环匹配中的 WirelessPairing_RunByName 逻辑。
 *
 * @param target_name 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t WirelessPairing_RunByName(const char *target_name)
{
    WirelessPairingScanResult scan;
    const WirelessPairingCandidate *selected = NULL;
    uint32_t ret;

    if ((target_name == NULL) || (*target_name == '\0') ||
        (strlen(target_name) >= WIRELESS_PAIRING_NAME_TEXT_SIZE)) {
        printf("无线滑环匹配\t名称参数无效，格式：SPN=<name>，长度小于18字节\r\n");
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_FAILED, NULL, PARAM_RANGE_ERROR);
        return PARAM_RANGE_ERROR;
    }

    printf("\r\n===== 无线滑环名称匹配开始：%s =====\r\n", target_name);
    printf("无线滑环名称匹配\t命令=SPN\t目标名称=%s\t匹配方式=显式名称字段完全匹配\r\n", target_name);
    WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_RUNNING, NULL, NO_ERROR);
    g_measurement.device_status.device_state = STATE_WIRELESS_PAIRING;
    ret = WirelessPairing_Scan(&scan);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if (ret == NO_ERROR) {
        if (scan.scan_has_name_field == 0U) {
            printf("无线滑环匹配\t扫描结果未包含名称字段，不能按名称匹配\r\n");
            ret = SENSOR_RESP_FORMAT_ERROR;
        } else if (WirelessPairing_SelectByName(&scan, target_name, &selected) == 0U) {
            printf("无线滑环匹配\t未找到唯一名称匹配项：%s\r\n", target_name);
            ret = WIRELESS_SLAVE_COMM_TIMEOUT;
        } else {
            ret = WirelessPairing_ConnectAndSave(selected);
        }
    }
    WirelessPairing_Finish("无线滑环名称匹配", ret);
    /* 先处理异常边界，避免无线滑环匹配状态机带故障继续运行。 */
    if ((ret == NO_ERROR) && (selected != NULL)) {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_SUCCESS, selected, NO_ERROR);
    } else {
        WirelessPairing_PublishStatus(WIRELESS_PAIRING_RESULT_FAILED, NULL, ret);
    }
    return ret;
}
