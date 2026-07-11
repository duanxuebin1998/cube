#include "serial_command_parser.h"

#include <math.h>
#include <string.h>

#define SERIAL_COMMAND_SPEED_MIN_M_MIN 0.10
#define SERIAL_COMMAND_SPEED_MAX_M_MIN 6.00
#define SERIAL_COMMAND_MULTIPLIER_MAX  20UL
#define SERIAL_COMMAND_AO_MIN_X100     320UL
#define SERIAL_COMMAND_AO_MAX_X100     2400UL
#define SERIAL_COMMAND_PAIR_NAME_MAX   17U

/* 判断命令是否与固定文本完全一致。 */
static uint8_t SerialCommandParser_IsExact(const uint8_t *command, const char *expected)
{
    return (uint8_t)(strcmp((const char *)command, expected) == 0);
}

/* 判断首字符是否属于正式单字母业务命令集合。 */
static uint8_t SerialCommandParser_IsFormalPrefix(uint8_t value)
{
    return (uint8_t)((value == 'I') || (value == 'G') || (value == 'K') ||
                     (value == 'R') || (value == 'W') || (value == 'O') ||
                     (value == 'P') || (value == 'Q'));
}

/* 解析无符号十进制整数，并同时执行溢出和范围检查。 */
static uint8_t SerialCommandParser_ParseUnsigned(const char **cursor,
                                                 unsigned long minimum,
                                                 unsigned long maximum,
                                                 unsigned long *value)
{
    const char *p;
    unsigned long parsed = 0UL;
    uint8_t digit_seen = 0U;

    if ((cursor == NULL) || (*cursor == NULL)) {
        return 0U;
    }

    p = *cursor;
    while ((*p >= '0') && (*p <= '9')) {
        unsigned long digit = (unsigned long)(*p - '0');
        if (parsed > ((maximum - digit) / 10UL)) {
            return 0U;
        }
        parsed = (parsed * 10UL) + digit;
        digit_seen = 1U;
        p++;
    }

    if ((digit_seen == 0U) || (parsed < minimum) || (parsed > maximum)) {
        return 0U;
    }

    *cursor = p;
    if (value != NULL) {
        *value = parsed;
    }
    return 1U;
}

/* 解析不带符号和指数的十进制数，并执行有限值和范围检查。 */
static uint8_t SerialCommandParser_ParseDecimal(const char **cursor,
                                                double minimum,
                                                double maximum,
                                                uint8_t allow_zero,
                                                double *value)
{
    const char *p;
    double parsed = 0.0;
    double scale = 0.1;
    uint8_t digit_seen = 0U;

    if ((cursor == NULL) || (*cursor == NULL)) {
        return 0U;
    }

    p = *cursor;
    while ((*p >= '0') && (*p <= '9')) {
        parsed = (parsed * 10.0) + (double)(*p - '0');
        digit_seen = 1U;
        p++;
    }
    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            parsed += (double)(*p - '0') * scale;
            scale *= 0.1;
            digit_seen = 1U;
            p++;
        }
    }

    if ((digit_seen == 0U) || (!isfinite(parsed)) ||
        ((allow_zero == 0U) && (!(parsed > 0.0))) ||
        (parsed < minimum) || (parsed > maximum)) {
        return 0U;
    }

    *cursor = p;
    if (value != NULL) {
        *value = parsed;
    }
    return 1U;
}

/* 解析 BJ/BJP 可选速度，速度存在时必须完整落在允许范围内。 */
static uint8_t SerialCommandParser_ParseOptionalSpeed(const char **cursor)
{
    double speed;

    if ((cursor == NULL) || (*cursor == NULL) || (**cursor == '\0')) {
        return 1U;
    }
    if (**cursor != ',') {
        return 0U;
    }
    (*cursor)++;
    if (SerialCommandParser_ParseDecimal(cursor,
                                         SERIAL_COMMAND_SPEED_MIN_M_MIN,
                                         SERIAL_COMMAND_SPEED_MAX_M_MIN,
                                         0U,
                                         &speed) == 0U) {
        return 0U;
    }
    return (uint8_t)(**cursor == '\0');
}

/* 校验 A0、A+mm 和 A-mm 手动运动命令。 */
static uint8_t SerialCommandParser_ParseA(const uint8_t *command)
{
    const char *cursor;
    unsigned long distance;

    if (SerialCommandParser_IsExact(command, "A0") != 0U) {
        return 1U;
    }
    if ((command[1] != '+') && (command[1] != '-')) {
        return 0U;
    }

    cursor = (const char *)&command[2];
    return (uint8_t)((SerialCommandParser_ParseUnsigned(&cursor, 1UL, 4294967295UL, &distance) != 0U) &&
                     (*cursor == '\0'));
}

/* 校验 AO 定点电流或扫描命令。 */
static uint8_t SerialCommandParser_ParseAo(const uint8_t *command)
{
    const char *cursor;
    unsigned long current_x100;

    if (SerialCommandParser_IsExact(command, "AOS") != 0U) {
        return 1U;
    }

    cursor = (const char *)&command[2];
    return (uint8_t)((SerialCommandParser_ParseUnsigned(&cursor,
                                                        SERIAL_COMMAND_AO_MIN_X100,
                                                        SERIAL_COMMAND_AO_MAX_X100,
                                                        &current_x100) != 0U) &&
                     (*cursor == '\0'));
}

/* 校验 BJ 相对点动和 BJP 绝对定位命令。 */
static uint8_t SerialCommandParser_ParseBj(const uint8_t *command)
{
    const char *cursor;
    double distance;

    if (command[2] == 'P') {
        cursor = (const char *)&command[3];
        if (SerialCommandParser_ParseDecimal(&cursor, 0.0, 4294967295.0, 1U, &distance) == 0U) {
            return 0U;
        }
        return SerialCommandParser_ParseOptionalSpeed(&cursor);
    }
    if ((command[2] != '+') && (command[2] != '-')) {
        return 0U;
    }

    cursor = (const char *)&command[3];
    if (SerialCommandParser_ParseDecimal(&cursor, 0.0, 4294967295.0, 0U, &distance) == 0U) {
        return 0U;
    }
    return SerialCommandParser_ParseOptionalSpeed(&cursor);
}

/* 校验 BE 距离、速度、倍率和传感器通信可选参数。 */
static uint8_t SerialCommandParser_ParseBe(const uint8_t *command)
{
    const char *cursor = (const char *)&command[2];
    unsigned long distance;
    unsigned long multiplier;
    double speed;

    if (SerialCommandParser_ParseUnsigned(&cursor, 1UL, 4294967295UL, &distance) == 0U) {
        return 0U;
    }
    if (*cursor == '\0') {
        return 1U;
    }
    if ((cursor[0] == 'S') && (cursor[1] == '\0')) {
        return 1U;
    }
    if (*cursor != ',') {
        return 0U;
    }
    cursor++;
    if ((cursor[0] == '1') && (cursor[1] == '\0')) {
        return 1U;
    }
    if (SerialCommandParser_ParseDecimal(&cursor,
                                         SERIAL_COMMAND_SPEED_MIN_M_MIN,
                                         SERIAL_COMMAND_SPEED_MAX_M_MIN,
                                         0U,
                                         &speed) == 0U) {
        return 0U;
    }
    if (*cursor == '\0') {
        return 1U;
    }
    if (*cursor != ',') {
        return 0U;
    }
    cursor++;
    if (SerialCommandParser_ParseUnsigned(&cursor, 1UL, SERIAL_COMMAND_MULTIPLIER_MAX, &multiplier) == 0U) {
        return 0U;
    }
    if (*cursor == '\0') {
        return 1U;
    }
    if (*cursor != ',') {
        return 0U;
    }
    cursor++;
    return (uint8_t)(((cursor[0] == 'S') || (cursor[0] == '1')) && (cursor[1] == '\0'));
}

/* 校验 B、BE 和 BJ 命令族并保留既有兼容格式。 */
static uint8_t SerialCommandParser_ParseB(const uint8_t *command)
{
    const char *cursor;
    unsigned long distance;

    if (command[1] == 'J') {
        return SerialCommandParser_ParseBj(command);
    }
    if (command[1] == 'E') {
        return SerialCommandParser_ParseBe(command);
    }

    cursor = (const char *)&command[1];
    if (SerialCommandParser_ParseUnsigned(&cursor, 1UL, 4294967295UL, &distance) == 0U) {
        return 0U;
    }
    return (uint8_t)((*cursor == '\0') ||
                     ((cursor[0] == 'S') && (cursor[1] == '\0')) ||
                     ((cursor[0] == ',') && (cursor[1] == '1') && (cursor[2] == '\0')));
}

/* 校验无线扫描、RSSI 匹配、状态查询和名称匹配命令。 */
static uint8_t SerialCommandParser_ParseSp(const uint8_t *command)
{
    size_t name_length;

    if ((SerialCommandParser_IsExact(command, "SPS") != 0U) ||
        (SerialCommandParser_IsExact(command, "SPR") != 0U) ||
        (SerialCommandParser_IsExact(command, "SPC") != 0U)) {
        return 1U;
    }
    if ((command[2] != 'N') || (command[3] != '=')) {
        return 0U;
    }

    name_length = strlen((const char *)&command[4]);
    return (uint8_t)((name_length > 0U) && (name_length <= SERIAL_COMMAND_PAIR_NAME_MAX));
}

/* 对完整命令执行分类和严格语法校验，拒绝前缀误匹配。 */
SerialCommandParseResult SerialCommandParser_Parse(const uint8_t *command)
{
    static const char *const exact_test_commands[] = {
        "SC", "C", "D", "E", "F", "H", "J", "L", "M", "N", "X",
        "T0", "T1", "T2", "TA", "TS", "TR", "TV", "TP", "TU",
        "YM", "YE", "YS", "YC", "YT"
    };
    SerialCommandParseResult result = { SERIAL_COMMAND_KIND_UNSUPPORTED, 0U };
    size_t i;

    if ((command == NULL) || (command[0] == '\0')) {
        result.kind = SERIAL_COMMAND_KIND_INVALID;
        return result;
    }

    if (SerialCommandParser_IsExact(command, "STOP") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_STOP;
        return result;
    }
    if (SerialCommandParser_IsExact(command, "HELP") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_HELP;
        return result;
    }
    if (SerialCommandParser_IsExact(command, "VER?") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_VERSION;
        return result;
    }
    if (SerialCommandParser_IsExact(command, "STAT?") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_STATUS;
        return result;
    }
    if (SerialCommandParser_IsExact(command, "ERR?") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_ERROR;
        return result;
    }

    if (SerialCommandParser_IsFormalPrefix(command[0]) != 0U) {
        if (command[1] == '\0') {
            result.kind = SERIAL_COMMAND_KIND_FORMAL;
            result.formal_command = command[0];
        } else {
            result.kind = SERIAL_COMMAND_KIND_INVALID;
        }
        return result;
    }

    if ((command[0] == 'A') && (command[1] == 'O')) {
        result.kind = (SerialCommandParser_ParseAo(command) != 0U) ?
                      SERIAL_COMMAND_KIND_TEST : SERIAL_COMMAND_KIND_INVALID;
        return result;
    }
    if (command[0] == 'A') {
        result.kind = (SerialCommandParser_ParseA(command) != 0U) ?
                      SERIAL_COMMAND_KIND_TEST : SERIAL_COMMAND_KIND_INVALID;
        return result;
    }
    if (command[0] == 'B') {
        result.kind = (SerialCommandParser_ParseB(command) != 0U) ?
                      SERIAL_COMMAND_KIND_TEST : SERIAL_COMMAND_KIND_INVALID;
        return result;
    }
    if ((command[0] == 'S') && (command[1] == 'P')) {
        result.kind = (SerialCommandParser_ParseSp(command) != 0U) ?
                      SERIAL_COMMAND_KIND_TEST : SERIAL_COMMAND_KIND_INVALID;
        return result;
    }

    for (i = 0U; i < (sizeof(exact_test_commands) / sizeof(exact_test_commands[0])); i++) {
        if (SerialCommandParser_IsExact(command, exact_test_commands[i]) != 0U) {
            result.kind = SERIAL_COMMAND_KIND_TEST;
            return result;
        }
    }

    if ((command[0] == 'C') || (command[0] == 'D') || (command[0] == 'E') ||
        (command[0] == 'F') || (command[0] == 'H') || (command[0] == 'J') ||
        (command[0] == 'L') || (command[0] == 'M') || (command[0] == 'N') ||
        (command[0] == 'T') || (command[0] == 'Y') || (command[0] == 'X') ||
        (command[0] == 'S')) {
        result.kind = SERIAL_COMMAND_KIND_INVALID;
    }
    return result;
}

/* 复位定长收帧状态，不访问硬件。 */
void SerialCommandRx_Init(SerialCommandRxState *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

/* 逐字节收帧；超长后丢弃整帧，直到 LF 到达再上报。 */
SerialCommandRxEvent SerialCommandRx_PushByte(SerialCommandRxState *state, uint8_t byte)
{
    if (state == NULL) {
        return SERIAL_COMMAND_RX_NONE;
    }

    if (byte == '\r') {
        return SERIAL_COMMAND_RX_NONE;
    }
    if (byte == '\n') {
        if (state->discarding != 0U) {
            SerialCommandRx_Init(state);
            return SERIAL_COMMAND_RX_TOO_LONG;
        }
        if (state->length == 0U) {
            return SERIAL_COMMAND_RX_NONE;
        }
        state->buffer[state->length] = '\0';
        state->length = 0U;
        return SERIAL_COMMAND_RX_READY;
    }
    if (state->discarding != 0U) {
        return SERIAL_COMMAND_RX_NONE;
    }
    if (state->length >= (SERIAL_COMMAND_RX_CAPACITY - 1U)) {
        state->discarding = 1U;
        return SERIAL_COMMAND_RX_NONE;
    }

    state->buffer[state->length] = byte;
    state->length++;
    return SERIAL_COMMAND_RX_NONE;
}
