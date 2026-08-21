/*
 * serial_command_parser.c
 *
 * 文件职责：完成串口字节收帧和完整命令的无副作用分类解析。
 */

#include "serial_command_parser.h"

#include <math.h>
#include <string.h>

/* 串口移动命令允许的最小速度 0.10 m/min；低于该值的参数必须拒绝。 */
#define SERIAL_COMMAND_SPEED_MIN_M_MIN 0.10
/* 串口移动命令允许的最大速度 6.00 m/min；该上限是命令接口边界，实际驱动仍受设备参数限制。 */
#define SERIAL_COMMAND_SPEED_MAX_M_MIN 6.00
/* 串口命令数值倍率允许上限 20；用于限制用户输入的缩放倍数，防止换算溢出。 */
#define SERIAL_COMMAND_MULTIPLIER_MAX  20UL
/* 串口 AO 电流命令允许下限 3.20 mA，线值单位为 0.01 mA；最终输出还会经过 AO 配置和硬件限幅。 */
#define SERIAL_COMMAND_AO_MIN_X100     320UL
/* 串口 AO 电流命令允许上限 24.00 mA，线值单位为 0.01 mA。 */
#define SERIAL_COMMAND_AO_MAX_X100     2400UL
/* 无线配对名称允许的最大字符数 17；解析时还必须为字符串终止符预留缓冲空间。 */
#define SERIAL_COMMAND_PAIR_NAME_MAX   17U
/* 固定频率维护命令允许的最大目标频率，单位 Hz；这里只约束串口语法边界。 */
#define SERIAL_COMMAND_FIXED_FREQUENCY_MAX_HZ 6500UL

/**
 * @brief 判断命令是否与固定文本完全一致。
 *
 * @param command 以 NUL 结尾的待比较命令文本；必须与 expected 在内容和长度上完全一致。
 * @param expected 待与输入命令逐字节比较的完整目标文本，比较区分字符且要求长度完全一致。
 * @return 1 表示 command 与 expected 指向的两个 NUL 结尾字符串逐字符完全一致；0 表示长度或任一字符不同。
 */
static uint8_t SerialCommandParser_IsExact(const uint8_t *command, const char *expected)
{
    return (uint8_t)(strcmp((const char *)command, expected) == 0);
}

/**
 * @brief 判断首字符是否属于正式单字母业务命令集合。
 *
 * @param value 命令前缀使用的输入数值。
 * @return 1 表示首字符属于正式单字母业务命令集合；0 表示首字符不属于正式单字母业务命令集合。
 */
static uint8_t SerialCommandParser_IsFormalPrefix(uint8_t value)
{
    return (uint8_t)((value == 'I') || (value == 'G') || (value == 'K') ||
                     (value == 'R') || (value == 'W') || (value == 'O') ||
                     (value == 'P') || (value == 'Q'));
}

/**
 * @brief 解析无符号十进制整数，并同时执行溢出和范围检查。
 *
 * @param cursor 命令文本输入输出游标；解析成功后推进到首个未消费字符。
 * @param minimum 允许的最小无符号整数；解析值必须与 maximum 共同构成的闭区间内。
 * @param maximum 无符号十进制字段允许的最大值，解析结果必须位于 minimum 至 maximum 的闭区间。
 * @param value 用于返回通过溢出和范围校验的无符号整数。
 * @return 1 表示无符号十进制文本完整合法且结果已写入输出参数，0 表示格式、溢出或范围校验失败。
 */
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

/**
 * @brief 校验 LF、LF? 和 LF=<目标频率>[,<死区>] 固定频率找液位调试命令。
 *
 * @param command 以 NUL 结尾的 LF、LF? 或 LF=<频率>[,<死区>] 命令候选文本。
 * @return 1 表示 LF 命令语法和数值范围合法且参数已保存；不匹配或校验失败时返回 0。
 */
static uint8_t SerialCommandParser_ParseLf(const uint8_t *command)
{
    const char *cursor;

    if ((SerialCommandParser_IsExact(command, "LF") != 0U) ||
        (SerialCommandParser_IsExact(command, "LF?") != 0U)) {
        return 1U;
    }
    if (command[2] != '=') {
        return 0U;
    }

    cursor = (const char *)&command[3];
    if (SerialCommandParser_ParseUnsigned(
            &cursor, 1UL, SERIAL_COMMAND_FIXED_FREQUENCY_MAX_HZ, NULL) == 0U) {
        return 0U;
    }
    if (*cursor == '\0') {
        return 1U;
    }
    if (*cursor++ != ',') {
        return 0U;
    }
    return (uint8_t)((SerialCommandParser_ParseUnsigned(
                              &cursor,
                              1UL,
                              SERIAL_COMMAND_FIXED_FREQUENCY_MAX_HZ,
                              NULL) != 0U) &&
                     (*cursor == '\0'));
}
/**
 * @brief 解析不带符号和指数的十进制数，并执行有限值和范围检查。
 *
 * @param cursor 命令文本输入输出游标；解析成功后推进到首个未消费字符。
 * @param minimum 允许的最小有限浮点值；解析值必须位于 minimum 至 maximum 的闭区间内。
 * @param maximum 浮点十进制字段允许的最大值，解析结果必须位于 minimum 至 maximum 的闭区间。
 * @param allow_zero 非零时允许解析结果等于 0；为 0 时即使 0 位于数值范围内也拒绝，用于要求严格正值的命令字段。
 * @param value 用于返回通过格式、有限值和范围校验的十进制数。
 * @return 1 表示十进制文本完整合法且有限值已写入输出参数，0 表示格式或范围校验失败。
 */
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

/**
 * @brief 解析 BJ/BJP 可选速度，速度存在时必须完整落在允许范围内。
 *
 * @param cursor 命令尾部输入输出游标；可为空或指向字符串结尾，也可指向逗号开头的 0.10 至 6.00 m/min 速度字段。
 * @return 返回 1 表示速度字段省略，或逗号后的 0.10 至 6.00 m/min 数值完整合法且已消费到字符串结尾；指针、分隔符、数值或尾随字符不合法时返回 0。
 */
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

/**
 * @brief 校验 A0、A+mm 和 A-mm 手动运动命令。
 *
 * @param command 以 NUL 结尾的 A0、A+mm 或 A-mm 手动运动命令候选文本。
 * @return 1 表示 A0、A+mm 或 A-mm 命令完整合法；不匹配、存在多余字符或距离越界时返回 0。
 */
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

/**
 * @brief 校验 AO 定点电流或扫描命令。
 *
 * @param command 以 NUL 结尾的 AO 定点电流或 AO 扫描命令候选文本。
 * @return 1 表示 AO 电流或扫描命令完整合法；格式、范围或尾随字符不合法时返回 0。
 */
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

/**
 * @brief 校验 BJ 相对点动和 BJP 绝对定位命令。
 *
 * @param command 以 NUL 结尾的 BJ 相对点动或 BJP 绝对定位命令候选文本。
 * @return 1 表示 BJ/BJP 命令及可选速度参数合法；格式或范围不合法时返回 0。
 */
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

/**
 * @brief 校验 BE 距离、速度、倍率和传感器通信可选参数。
 *
 * @param command 以 NUL 结尾的 BE 电机测试命令候选文本，可包含距离、速度、倍率和传感器通信选项。
 * @return 1 表示 BE 距离、速度、倍率及可选通信标志全部合法；否则返回 0。
 */
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

/**
 * @brief 校验 B、BE 和 BJ 命令族并保留既有兼容格式。
 *
 * @param command 以 NUL 结尾的 B、BE 或 BJ 命令族候选文本。
 * @return 1 表示 B、BE 或 BJ 命令族中的一个兼容格式匹配成功；不匹配或参数非法时返回 0。
 */
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

/**
 * @brief 校验无线扫描、RSSI 匹配、状态查询和名称匹配命令。
 *
 * @param command 以 NUL 结尾的 SPC、SPR、SPC?、SPR?、SPN 或 SPN? 无线命令候选文本。
 * @return 1 表示无线扫描、配对、查询或名称参数命令合法；名称为空、过长或格式不符时返回 0。
 */
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

/**
 * @brief 校验多参数 V4 传感器串口调试命令族。
 *
 * @param command 以 NUL 结尾的 V4 调试命令候选文本。
 * @return 1 表示命令完整合法，0 表示操作码、参数范围或尾随字符不合法。
 */
static uint8_t SerialCommandParser_ParseV4(const uint8_t *command)
{
    const char *cursor;
    unsigned long parameter;

    unsigned long raw_value;

    if ((SerialCommandParser_IsExact(command, "V4?") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4P") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4I") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4A") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4D") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4S") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4O") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4F") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4G") != 0U) ||
        (SerialCommandParser_IsExact(command, "V4C") != 0U)) {
        return 1U;
    }
    if (((command[2] == 'L') || (command[2] == 'M')) &&
        (command[3] == '=') &&
        ((command[4] == '0') || (command[4] == '1')) &&
        (command[5] == '\0')) {
        return 1U;
    }
    if ((command[2] == 'W') && (command[3] == '=')) {
        cursor = (const char *)&command[4];
        if ((SerialCommandParser_ParseUnsigned(&cursor, 66UL, 159UL, &parameter) == 0U) ||
            (*cursor != ',')) {
            return 0U;
        }
        cursor++;
        if ((SerialCommandParser_ParseUnsigned(&cursor, 0UL, 4294967295UL, &raw_value) == 0U) ||
            (*cursor != '\0')) {
            return 0U;
        }
        return 1U;
    }
    if ((command[2] != 'R') || (command[3] != '=')) {
        return 0U;
    }

    cursor = (const char *)&command[4];
    if ((SerialCommandParser_ParseUnsigned(&cursor, 0UL, 159UL, &parameter) == 0U) ||
        (*cursor != '\0')) {
        return 0U;
    }
    return (uint8_t)(((parameter <= 25UL) || (parameter >= 65UL)) ? 1U : 0U);
}

/**
 * @brief 严格校验 V3 调试命令及其参数范围。
 * @param command 以 NUL 结尾的 V3 调试命令。
 * @return 1 表示命令格式和参数范围有效，0 表示必须拒绝。
 */
static uint8_t SerialCommandParser_ParseV3(const uint8_t *command)
{
    const char *cursor;
    unsigned long parameter;

    if ((SerialCommandParser_IsExact(command, "V3?") != 0U) ||
        (SerialCommandParser_IsExact(command, "V3P") != 0U) ||
        (SerialCommandParser_IsExact(command, "V3D") != 0U) ||
        (SerialCommandParser_IsExact(command, "V3L") != 0U) ||
        (SerialCommandParser_IsExact(command, "V3A") != 0U)) {
        return 1U;
    }
    if ((command[2] == 'W') && (command[3] == '=')) {
        uint8_t digit_seen = 0U;
        uint8_t fraction_seen = 0U;

        cursor = (const char *)&command[4];
        if ((SerialCommandParser_ParseUnsigned(&cursor, 20UL, 114UL, &parameter) == 0U) ||
            (*cursor++ != ',')) {
            return 0U;
        }
        if (*cursor == '-') {
            cursor++;
        }
        while ((*cursor >= '0') && (*cursor <= '9')) {
            digit_seen = 1U;
            cursor++;
        }
        if (*cursor == '.') {
            fraction_seen = 1U;
            cursor++;
            while ((*cursor >= '0') && (*cursor <= '9')) {
                digit_seen = 1U;
                cursor++;
            }
        }
        return (uint8_t)((digit_seen != 0U) &&
                         ((fraction_seen == 0U) || (digit_seen != 0U)) &&
                         (*cursor == '\0'));
    }
    if ((command[2] != 'R') || (command[3] != '=')) {
        return 0U;
    }

    cursor = (const char *)&command[4];
    if ((SerialCommandParser_ParseUnsigned(&cursor, 0UL, 255UL, &parameter) == 0U) ||
        (*cursor != '\0')) {
        return 0U;
    }
    return (uint8_t)((parameter == 0UL) ||
                     (parameter == 4UL) ||
                     (parameter == 6UL) ||
                     (parameter == 7UL) ||
                     (parameter == 8UL) ||
                     (parameter == 9UL) ||
                     (parameter == 17UL) ||
                     (parameter == 18UL) ||
                     (parameter == 22UL));
}

/**
 * @brief 对完整命令执行分类和严格语法校验，拒绝前缀误匹配。
 *
 * 函数先识别 STOP、HELP、版本、状态、错误、电源和编码器查询等必须完整匹配的控制命令，再接受单字节正式业务命令，禁止以合法首字母开头的多余尾随字符被误当成正式命令。
 * LF、AO、A、B 和 SP 命令族交给各自严格语法解析器校验数值、分隔符、范围和字符串结尾；固定测试命令则在只读精确匹配表中逐项识别。
 * 已知命令族但格式不完整、参数越界或带有尾随字符时返回 SERIAL_COMMAND_KIND_INVALID；完全未知的命令保持
 * SERIAL_COMMAND_KIND_UNSUPPORTED，便于上层区分语法错误和未实现命令。
 * 本函数只分类并返回正式命令字节，不执行硬件动作、不修改接收缓冲区，也不忽略前后空白或改变字母大小写。
 *
 * @param command 已去除 CR/LF、以 NUL 结尾的完整串口命令；空指针或空字符串按 SERIAL_COMMAND_KIND_INVALID 分类。
 * @return 返回命令分类结果；kind 区分正式命令、测试、停止、查询、语法非法和未支持命令，只有 kind 为 SERIAL_COMMAND_KIND_FORMAL 时
 *         formal_command 才保存已确认的单字节正式命令。
 * @note 命令匹配区分大小写且要求以 NUL 结尾；调用前应由收帧层去除 CR/LF，解析器不会接受前后空格。
 */
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
    /* 电源/编码器查询与PWRTEST必须全字匹配，禁止被正式命令前缀规则误接收。 */
    if (SerialCommandParser_IsExact(command, "PWR?") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_POWER_STATUS;
        return result;
    }
    if (SerialCommandParser_IsExact(command, "ENC?") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_ENCODER_STATUS;
        return result;
    }
    if (SerialCommandParser_IsExact(command, "PWRTEST") != 0U) {
        result.kind = SERIAL_COMMAND_KIND_POWER_TEST;
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

    if ((command[0] == 'L') && (command[1] == 'F')) {
        result.kind = (SerialCommandParser_ParseLf(command) != 0U) ?
                      SERIAL_COMMAND_KIND_TEST : SERIAL_COMMAND_KIND_INVALID;
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
    if ((command[0] == 'V') && (command[1] == '4')) {
        result.kind = (SerialCommandParser_ParseV4(command) != 0U) ?
                      SERIAL_COMMAND_KIND_TEST : SERIAL_COMMAND_KIND_INVALID;
        return result;
    }
    if ((command[0] == 'V') && (command[1] == '3')) {
        result.kind = (SerialCommandParser_ParseV3(command) != 0U) ?
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

/**
 * @brief 复位定长收帧状态，不访问硬件。
 *
 * @param state 待复位的串口命令接收状态对象；函数清空长度、丢弃标志和固定缓冲区。
 */
void SerialCommandRx_Init(SerialCommandRxState *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

/**
 * @brief 逐字节收帧；超长后丢弃整帧，直到 LF 到达再上报。
 *
 * @param state 串口命令逐字节接收状态，保存缓冲区长度和超长丢弃标志；传入 NULL 时忽略输入。
 * @param byte 本次收到的一个原始字节；CR 被忽略，LF 触发完整帧或超长帧事件。
 * @return 返回 SERIAL_COMMAND_RX_READY 表示非空 LF 结尾命令已写入缓冲区，返回 SERIAL_COMMAND_RX_TOO_LONG 表示超长帧已在 LF
 *         处丢弃；其他输入返回 SERIAL_COMMAND_RX_NONE。
 */
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
