/**
 * @file service_debug_sensor_protocol.c
 * @brief CPU2 串口维护命令的 V3/V4 传感器协议读取、写入和诊断实现。
 */

#include "service_debug_internal.h"
#include "Protocols/MultiparamV3/multiparam_v3_communication.h"
#include "Protocols/Dm4/V4/multiparam_v4_communication.h"
#include "sensor_service.h"
#include "sensor_runtime.h"
#include "system_parameter.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 按原始位模式解释IEEE754单精度值，避免调试输出改变NaN/Inf语义。 */
/* 返回V4通信方式的现场可读名称。 */
static const char *TestCommand_V4CommunicationModeName(
    multiparam_v4_communication_mode_t mode)
{
    switch (mode) {
    case MULTIPARAM_V4_COMMUNICATION_ACTIVE:
        return "主动";
    case MULTIPARAM_V4_COMMUNICATION_INTERACTIVE:
        return "交互";
    default:
        return "未知";
    }
}

/* 返回V4测量方式的现场可读名称。 */
static const char *TestCommand_V4MeasurementModeName(
    multiparam_v4_measurement_mode_t mode)
{
    switch (mode) {
    case MULTIPARAM_V4_MEASUREMENT_DENSITY:
        return "密度";
    case MULTIPARAM_V4_MEASUREMENT_LEVEL:
        return "液位";
    default:
        return "无效";
    }
}


/**
 * @brief 将 V4 参数的 32 位原始字按单精度浮点格式还原。
 * @param raw 传感器参数帧中的原始 32 位字。
 * @return 返回按原始位模式解释得到的浮点值，不执行量程或有效性修正。
 */
static float TestCommand_V4RawToFloat(uint32_t raw)
{
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

/* 输出最近主动帧原包、头字段和P0至P14的三种原始解释。 */
static void TestCommand_PrintV4ActiveSnapshot(void)
{
    static const char *const parameter_names[MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT] = {
        "软件版本", "协议版本", "状态字", "磁零点电压", "测量频率",
        "水电容", "温度", "密度", "动力粘度", "运动粘度",
        "供电电压", "X角", "Y角", "45度扫频周期平方均值",
        "22.5度扫频周期平方均值"
    };
    multiparam_v4_snapshot_t snapshot;
    uint32_t result = MULTIPARAM_V4_CopyLatestSnapshot(&snapshot);

    MULTIPARAM_V4_PrintActiveProbePacket("主动帧调试", result);
    if (result != NO_ERROR) {
        printf("V4主动帧\t解包失败\t错误码=0x%08lX\r\n",
               (unsigned long)result);
        return;
    }

    printf("V4主动帧\t解包成功\t地址=%u\t序号=%u\t代次=%lu\t年龄=%lu ms"
           "\t模式=%s\t测水=%u\t磁零点=%u\t快扫完成=%u\t状态=0x%08lX\r\n",
           (unsigned int)snapshot.address,
           (unsigned int)snapshot.sequence,
           (unsigned long)snapshot.generation,
           (unsigned long)MULTIPARAM_V4_GetSnapshotAgeMs(),
           TestCommand_V4MeasurementModeName(snapshot.measurement_mode),
           (unsigned int)snapshot.water_enabled,
           (unsigned int)snapshot.magnetic_zero_enabled,
           (unsigned int)snapshot.fast_sweep_completed,
           (unsigned long)snapshot.status_word);
    for (uint32_t index = 0U; index < MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT; index++) {
        uint32_t raw = snapshot.raw_parameter[index];
        printf("V4主动帧\tP%02lu=%s\tRAW=0x%08lX\tU32=%lu\tI32=%ld\tFLOAT=%.9g\r\n",
               (unsigned long)index,
               parameter_names[index],
               (unsigned long)raw,
               (unsigned long)raw,
               (long)((int32_t)raw),
               (double)TestCommand_V4RawToFloat(raw));
    }
}

/* 输出V4主动接收、帧校验、序号和交互事务累计诊断。 */
static void TestCommand_PrintV4Diagnostics(void)
{
    multiparam_v4_diagnostics_t diagnostics;
    multiparam_v4_abnormal_frame_t abnormal_frame;
    uint32_t age = MULTIPARAM_V4_GetSnapshotAgeMs();
    uint32_t expected_frames;
    uint32_t loss_basis_points;

    MULTIPARAM_V4_GetDiagnostics(&diagnostics);
    MULTIPARAM_V4_GetLastAbnormalFrame(&abnormal_frame);
    expected_frames = (diagnostics.valid_frames > (UINT32_MAX - diagnostics.lost_frames))
                          ? UINT32_MAX
                          : (diagnostics.valid_frames + diagnostics.lost_frames);
    loss_basis_points = (expected_frames == 0U)
                            ? 0U
                            : (uint32_t)(((uint64_t)diagnostics.lost_frames * 10000U) /
                                         expected_frames);
    printf("V4诊断\t通信=%s\t主动接收=%u\t快照年龄=%s",
           TestCommand_V4CommunicationModeName(MULTIPARAM_V4_GetCommunicationMode()),
           (unsigned int)MULTIPARAM_V4_IsActiveReceiverRunning(),
           (age == UINT32_MAX) ? "无" : "有效");
    if (age != UINT32_MAX) {
        printf("(%lu ms)", (unsigned long)age);
    }
    printf("\r\n");
    printf("V4诊断\t接收字节=%lu\t接收事件=%lu\t候选帧=%lu\t有效帧=%lu"
           "\t无线补零帧=%lu\tCRC错=%lu\t固定字段错=%lu\t地址错=%lu"
           "\t数值错=%lu\r\n",
           (unsigned long)diagnostics.received_bytes,
           (unsigned long)diagnostics.receive_events,
           (unsigned long)diagnostics.frames_seen,
           (unsigned long)diagnostics.valid_frames,
           (unsigned long)diagnostics.wireless_zero_padding_frames,
           (unsigned long)diagnostics.crc_errors,
           (unsigned long)diagnostics.fixed_field_errors,
           (unsigned long)diagnostics.address_errors,
           (unsigned long)diagnostics.value_errors);
    printf("V4诊断\t短接收=%lu\t重同步=%lu\t重复帧=%lu\t乱序帧=%lu"
           "\t丢帧=%lu\t序号复位=%lu\t流溢出=%lu\r\n",
           (unsigned long)diagnostics.short_receive_events,
           (unsigned long)diagnostics.resync_events,
           (unsigned long)diagnostics.duplicate_frames,
           (unsigned long)diagnostics.out_of_order_frames,
           (unsigned long)diagnostics.lost_frames,
           (unsigned long)diagnostics.sequence_resets,
           (unsigned long)diagnostics.stream_overflows);
    printf("V4诊断\tUART错=%lu\t接收重启错=%lu\t主动超时=%lu"
           "\t交互事务=%lu\t交互错误=%lu\r\n",
           (unsigned long)diagnostics.uart_errors,
           (unsigned long)diagnostics.receive_restart_errors,
           (unsigned long)diagnostics.timeout_events,
           (unsigned long)diagnostics.interactive_transactions,
           (unsigned long)diagnostics.interactive_errors);
    printf("V4质量\t有效包=%lu\t预计包=%lu\t丢包=%lu\t丢包率=%lu.%02lu%%"
           "\t异常包/缺口=%lu\t连续异常=%lu\t最大连续=%lu\t告警次数=%lu\r\n",
           (unsigned long)diagnostics.valid_frames,
           (unsigned long)expected_frames,
           (unsigned long)diagnostics.lost_frames,
           (unsigned long)(loss_basis_points / 100U),
           (unsigned long)(loss_basis_points % 100U),
           (unsigned long)diagnostics.abnormal_frames,
           (unsigned long)diagnostics.consecutive_abnormal_frames,
           (unsigned long)diagnostics.max_consecutive_abnormal_frames,
           (unsigned long)diagnostics.quality_alarm_events);
    if (abnormal_frame.valid == 0U) {
        printf("V4异常包\t无\r\n");
        return;
    }
    printf("V4异常包\t错误码=0x%08lX\t错误名=%s\t长度=%u\t年龄=%lu ms\tHEX=",
           (unsigned long)abnormal_frame.error_code,
           ErrorLog_GetCodeName(abnormal_frame.error_code),
           (unsigned int)abnormal_frame.length,
           (unsigned long)(HAL_GetTick() - abnormal_frame.received_tick));
    if (abnormal_frame.length == 0U) {
        printf("<无原包>");
    } else {
        for (uint16_t index = 0U; index < abnormal_frame.length; index++) {
            printf("%02X%s", abnormal_frame.data[index],
                   (index + 1U < abnormal_frame.length) ? " " : "");
        }
    }
    printf("\r\n");
}

/* 输出V4调试命令现场帮助。 */
static void TestCommand_PrintV4Help(void)
{
    printf("V4调试\tV4P=广播探测R01，V4I/V4A=交互/主动通信，V4D/V4S=密度/液位模式\r\n");
    printf("V4调试\tV4L=0|1=测水关闭/开启，V4M=0|1=磁零点关闭/开启，V4O=解码R02\r\n");
    printf("V4调试\tV4R=<0..25|65..159>=原始参数读取，V4W=<66..159>,<raw32> V4F=主动帧原包与P0-P14解包\r\n");
    printf("V4调试\tV4G=通信质量与最近异常包，V4C=清零质量诊断；交互操作会打印每次TX/RX原包\r\n");
}

/* Check the runtime identity before executing protocol-specific diagnostics. */
static uint8_t TestCommand_RequireSensorProtocol(const uint8_t *command,
                                                  uint8_t require_v4)
{
    if (SensorRuntime_IsDetectionValid() == 0U) {
        printf("ERR cmd=%s reason=DETECTION_INVALID\r\n",
               (const char *)command);
        return 0U;
    }

    if (require_v4 == 0U) {
        if (g_deviceParams.sensorType != LTD_SENSOR) {
            printf("ERR cmd=%s reason=NOT_V3_SENSOR type=%lu\r\n",
                   (const char *)command,
                   (unsigned long)g_deviceParams.sensorType);
            return 0U;
        }
        return 1U;
    }

    /* DM4 uses one persisted type for V4 and Safe; the session selects the protocol. */
    if (g_deviceParams.sensorType != DM4_SENSOR) {
        printf("ERR cmd=%s reason=NOT_V4_SENSOR type=%lu\r\n",
               (const char *)command,
               (unsigned long)g_deviceParams.sensorType);
        return 0U;
    }
    if (SensorRuntime_GetDm4ProtocolMode() != SENSOR_DM4_PROTOCOL_V4) {
        printf("ERR cmd=%s reason=SAFE_SESSION_ACTIVE\r\n",
               (const char *)command);
        return 0U;
    }
    return 1U;
}
/**
 * @brief 按 V3 参数号执行只读诊断并打印原始结果。
 * @param parameter V3 参数号；仅支持维护命令白名单中的参数。
 * @return 传感器读取结果；不支持的参数返回 SYSTEM_CALL_CONDITION_ERROR。
 */
static int TestCommand_ReadV3Parameter(unsigned long parameter)
{
    int result = SYSTEM_CALL_CONDITION_ERROR;
    float float_value = 0.0f;
    int32_t int_value = 0;
    uint32_t frequency = 0U;

    switch (parameter) {
    case 0UL:
        result = MULTIPARAM_V3_Read_SoftwareVersion(&float_value);
        break;
    case 4UL:
        result = MULTIPARAM_V3_Read_LevelFrequency(&frequency);
        break;
    case 6UL:
        result = MULTIPARAM_V3_Read_Temperature(&float_value);
        break;
    case 7UL:
        result = MULTIPARAM_V3_Read_Density(&float_value);
        break;
    case 8UL:
        result = MULTIPARAM_V3_Read_DynamicViscosity(&float_value);
        break;
    case 9UL:
        result = MULTIPARAM_V3_Read_KinematicViscosity(&float_value);
        break;
    case 17UL:
        result = MULTIPARAM_V3_Read_MeanSquare45(&float_value);
        break;
    case 18UL:
        result = MULTIPARAM_V3_Read_MeanSquare22p5(&float_value);
        break;
    case 22UL:
        /* Keep zero as a valid debug value; this is a raw compatibility read. */
        result = MULTIPARAM_V3_Read_IntParam(0x16U, &int_value);
        break;
    default:
        break;
    }

    printf("V3 result R%02lu=0x%08lX",
           parameter,
           (unsigned long)result);
    if (result == NO_ERROR) {
        if (parameter == 4UL) {
            printf(" VALUE=%lu", (unsigned long)frequency);
        } else if (parameter == 22UL) {
            printf(" VALUE=%ld", (long)int_value);
        } else {
            printf(" VALUE=%.9g", (double)float_value);
        }
    }
    printf("\r\n");
    return result;
}

/**
 * @brief 判断 V3 参数在线路上是否按 32 位整数解释。
 * @param parameter V3 参数号。
 * @return 整数参数返回 1，浮点参数返回 0。
 */
static uint8_t TestCommand_V3ParamIsInteger(unsigned long parameter)
{
    return (uint8_t)(((parameter >= 20UL) && (parameter <= 24UL)) ||
                     (parameter == 107UL) ||
                     (parameter == 108UL));
}

/**
 * @brief 回读刚写入的 V3 参数并打印类型、数值和原始位模式。
 * @param parameter 已完成写入的 V3 参数号。
 * @return 参数回读结果。
 */
static int TestCommand_VerifyV3Write(unsigned long parameter)
{
    int result;
    uint32_t raw_value = 0U;

    if (TestCommand_V3ParamIsInteger(parameter) != 0U) {
        int32_t value = 0;
        result = MULTIPARAM_V3_Read_IntParam((uint8_t)parameter, &value);
        raw_value = (uint32_t)value;
        printf("V3 verify param=%lu type=int value=%ld raw=0x%08lX result=0x%08lX\r\n",
               parameter,
               (long)value,
               (unsigned long)raw_value,
               (unsigned long)result);
    } else {
        float value = 0.0f;
        result = MULTIPARAM_V3_Read_FloatParam((uint8_t)parameter, &value);
        memcpy(&raw_value, &value, sizeof(raw_value));
        printf("V3 verify param=%lu type=float value=%.9g raw=0x%08lX result=0x%08lX\r\n",
               parameter,
               (double)value,
               (unsigned long)raw_value,
               (unsigned long)result);
    }
    return result;
}

/**
 * @brief 解析并执行 V3W 直接写参数维护命令，成功写入后立即回读复核。
 * @param command 已通过串口命令语法校验、以 NUL 结尾的命令文本。
 * @return 命令已消费返回 1；格式、范围或传感器错误通过日志报告。
 * @note 本函数会真实改写传感器参数，仅允许在受控维护场景调用。
 */
static uint8_t TestCommand_HandleV3Write(const uint8_t *command)
{
    const char *cursor;
    char *end;
    unsigned long parameter;
    double parsed_value;
    uint32_t raw_value;
    int result;

    cursor = (const char *)&command[4];
    parameter = strtoul(cursor, &end, 10);
    if ((end == cursor) || (*end != ',')) {
        printf("ERR cmd=%s reason=WRITE_FORMAT\r\n", (const char *)command);
        return 1U;
    }
    cursor = end + 1;
    parsed_value = strtod(cursor, &end);
    if ((end == cursor) || (*end != '\0') || !isfinite(parsed_value)) {
        printf("ERR cmd=%s reason=WRITE_VALUE_INVALID\r\n", (const char *)command);
        return 1U;
    }

    if (TestCommand_V3ParamIsInteger(parameter) != 0U) {
        if ((parsed_value < -2147483648.0) ||
            (parsed_value > 4294967295.0) ||
            (floor(parsed_value) != parsed_value)) {
            printf("ERR cmd=%s reason=WRITE_VALUE_RANGE\r\n", (const char *)command);
            return 1U;
        }
        raw_value = (uint32_t)(int64_t)parsed_value;
    } else {
        float float_value = (float)parsed_value;
        if (!isfinite(float_value)) {
            printf("ERR cmd=%s reason=WRITE_VALUE_RANGE\r\n", (const char *)command);
            return 1U;
        }
        memcpy(&raw_value, &float_value, sizeof(raw_value));
    }

    printf("V3 write direct param=%lu raw=0x%08lX\r\n",
           parameter,
           (unsigned long)raw_value);
    result = MULTIPARAM_V3_WriteRawParam((uint8_t)parameter, raw_value);
    printf("V3 write result=0x%08lX\r\n", (unsigned long)result);
    if (result == NO_ERROR) {
        result = TestCommand_VerifyV3Write(parameter);
        if (result != NO_ERROR) {
            printf("V3 write verify failed result=0x%08lX\r\n", (unsigned long)result);
        }
    }
    return 1U;
}
/**
 * @brief 打印 V3 维护命令的紧凑帮助文本。
 */
static void TestCommand_PrintV3Help(void)
{
    printf("V3 debug: V3P=probe V3D=density V3L=level V3A=all reads V3R=... V3W=20..114,value\r\n");
}

/**
 * @brief 分派 V3 探测、模式切换、参数读取和参数写入维护命令。
 * @param command 已完成基础收帧和语法分类的命令文本。
 * @return V3 命令已消费返回 1，非 V3 命令返回 0。
 */
uint8_t TestCommand_HandleV3(const uint8_t *command)
{
    unsigned long parameter;
    const char *cursor;
    static const unsigned long all_parameters[] = { 0UL, 4UL, 6UL, 7UL, 8UL, 9UL, 17UL, 18UL, 22UL };

    if ((command == NULL) || (command[0] != 'V') || (command[1] != '3')) {
        return 0U;
    }
    if (command[2] == '?') {
        TestCommand_PrintV3Help();
        return 1U;
    }
    if (TestCommand_RequireSensorProtocol(command, 0U) == 0U) {
        return 1U;
    }

    if (command[2] == 'W') {
        (void)TestCommand_HandleV3Write(command);
        return 1U;
    }
    if (command[2] == 'P') {
        int32_t sensor_id = 0;
        int result = MULTIPARAM_V3_Read_IntParam(0x16U, &sensor_id);
        printf("V3 probe result=0x%08lX sensor_id=%ld\r\n",
               (unsigned long)result,
               (long)sensor_id);
        return 1U;
    }
    if (command[2] == 'D') {
        int result = MULTIPARAM_V3_SwitchToDensityMode();
        printf("V3 mode=density result=0x%08lX\r\n", (unsigned long)result);
        return 1U;
    }
    if (command[2] == 'L') {
        int result = MULTIPARAM_V3_SwitchToLevelMode();
        printf("V3 mode=level result=0x%08lX\r\n", (unsigned long)result);
        return 1U;
    }
    if (command[2] == 'A') {
        for (uint32_t index = 0U;
             index < (sizeof(all_parameters) / sizeof(all_parameters[0]));
             index++) {
            (void)TestCommand_ReadV3Parameter(all_parameters[index]);
        }
        return 1U;
    }
    if (command[2] == 'R') {
        cursor = (const char *)&command[4];
        parameter = strtoul(cursor, NULL, 10);
        (void)TestCommand_ReadV3Parameter(parameter);
        return 1U;
    }
    return 1U;
}

/**
 * @brief 按 32 位线路原值写入一个 V4 参数，并立即回读验证。
 * @param command 已通过串口命令语法校验、以 NUL 结尾的 V4W 命令文本。
 * @return 命令已消费返回 1；写入、回读或比对失败通过日志报告。
 * @note 写入前要求 V4 处于交互通信模式，本函数会真实改写传感器参数。
 */
static uint8_t TestCommand_HandleV4Write(const uint8_t *command)
{
    const char *cursor;
    char *end;
    unsigned long parameter;
    unsigned long parsed_raw;
    uint32_t raw_value;
    uint32_t readback = 0U;
    uint32_t result;
    uint32_t verify_result;

    if (MULTIPARAM_V4_GetCommunicationMode() !=
        MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        printf("ERR cmd=%s reason=NOT_INTERACTIVE\r\n",
               (const char *)command);
        return 1U;
    }

    cursor = (const char *)&command[4];
    parameter = strtoul(cursor, &end, 10);
    if ((end == cursor) || (*end != ',')) {
        printf("ERR cmd=%s reason=WRITE_FORMAT\r\n", (const char *)command);
        return 1U;
    }
    if ((parameter < 66UL) || (parameter > 159UL)) {
        printf("ERR cmd=%s reason=WRITE_PARAMETER_RANGE\r\n", (const char *)command);
        return 1U;
    }

    cursor = end + 1;
    parsed_raw = strtoul(cursor, &end, 10);
    if ((end == cursor) || (*end != 0) || (parsed_raw > 4294967295UL)) {
        printf("ERR cmd=%s reason=WRITE_VALUE_RANGE\r\n", (const char *)command);
        return 1U;
    }
    raw_value = (uint32_t)parsed_raw;

    printf("V4 write direct param=%lu raw=0x%08lX\r\n",
           parameter,
           (unsigned long)raw_value);
    MULTIPARAM_V4_SetProbeTraceEnabled(1U);
    result = MULTIPARAM_V4_WriteParamRaw((uint8_t)parameter, raw_value);
    verify_result = result;
    if (result == NO_ERROR) {
        verify_result = MULTIPARAM_V4_ReadParamRaw((uint8_t)parameter, &readback);
        if (verify_result == NO_ERROR) {
            printf("V4 write verify param=%lu raw=0x%08lX result=0x%08lX\r\n",
                   parameter,
                   (unsigned long)readback,
                   (unsigned long)((readback == raw_value) ? NO_ERROR : SENSOR_RESP_FORMAT_ERROR));
            if (readback != raw_value) {
                verify_result = SENSOR_RESP_FORMAT_ERROR;
            }
        } else {
            printf("V4 write verify param=%lu result=0x%08lX\r\n",
                   parameter,
                   (unsigned long)verify_result);
        }
    }
    MULTIPARAM_V4_SetProbeTraceEnabled(0U);
    printf("V4 write result=0x%08lX\r\n", (unsigned long)verify_result);
    return 1U;
}

/**
 * @brief 分派 V4 探测、模式控制、功能控制、参数读写和状态读取维护命令。
 * @param command 已完成基础收帧和语法分类的命令文本。
 * @return V4 命令已消费返回 1，非 V4 命令返回 0。
 */
uint8_t TestCommand_HandleV4(const uint8_t *command)
{
    uint32_t result = NO_ERROR;
    uint32_t raw = 0U;
    uint32_t status_word = 0U;
    unsigned long parameter = 0UL;
    float protocol_version = 0.0f;
    multiparam_v4_operating_state_t operating_state = {0};
    const char *operation = "未指定";
    uint8_t print_active_after = 0U;

    if ((command == NULL) || (command[0] != 'V') || (command[1] != '4')) {
        return 0U;
    }
    if (command[2] == '?') {
        TestCommand_PrintV4Help();
        return 1U;
    }
    if (TestCommand_RequireSensorProtocol(command, 1U) == 0U) {
        return 1U;
    }
    if (command[2] == 'W') {
        (void)TestCommand_HandleV4Write(command);
        return 1U;
    }
    if (command[2] == 'F') {
        TestCommand_PrintV4ActiveSnapshot();
        return 1U;
    }
    if (command[2] == 'G') {
        TestCommand_PrintV4Diagnostics();
        return 1U;
    }
    if (command[2] == 'C') {
        MULTIPARAM_V4_ClearDiagnostics();
        printf("V4诊断\t累计计数、连续异常和异常包已清零，通信方式、主动接收和最近快照保持不变\r\n");
        TestCommand_PrintV4Diagnostics();
        return 1U;
    }

    if (command[2] == 'P') {
        operation = "广播探测R01";
        MULTIPARAM_V4_Deinit();
        MULTIPARAM_V4_Init(MULTIPARAM_V4_ANY_ADDRESS);
    }
    MULTIPARAM_V4_SetProbeTraceEnabled(1U);
    switch (command[2]) {
    case 'P':
        result = MULTIPARAM_V4_ProbeProtocolVersion(&protocol_version);
        break;
    case 'I':
        operation = "切换交互通信";
        result = MULTIPARAM_V4_EnterInteractive();
        break;
    case 'A':
        operation = "切换主动通信";
        result = MULTIPARAM_V4_EnterActive();
        print_active_after = 1U;
        break;
    case 'D':
        operation = "切换密度模式";
        result = MULTIPARAM_V4_SelectDensityMode();
        break;
    case 'S':
        operation = "切换液位模式";
        result = MULTIPARAM_V4_SelectLevelMode();
        break;
    case 'L':
        operation = (command[4] == '1') ? "开启测水" : "关闭测水";
        result = MULTIPARAM_V4_EnsureWaterEnabled((uint8_t)(command[4] - '0'));
        break;
    case 'M':
        operation = (command[4] == '1') ? "开启磁零点" : "关闭磁零点";
        result = MULTIPARAM_V4_EnsureMagneticZeroEnabled((uint8_t)(command[4] - '0'));
        break;
    case 'O':
        operation = "读取并解码R02";
        result = MULTIPARAM_V4_ReadOperatingState(&status_word, &operating_state);
        break;
    case 'R':
        parameter = strtoul((const char *)&command[4], NULL, 10);
        operation = "读取原始参数";
        result = MULTIPARAM_V4_ReadParamRaw((uint8_t)parameter, &raw);
        break;
    default:
        result = SYSTEM_CALL_CONDITION_ERROR;
        break;
    }
    MULTIPARAM_V4_SetProbeTraceEnabled(0U);

    printf("V4调试\t操作=%s\t结果=0x%08lX\t通信=%s\t主动接收=%u\r\n",
           operation,
           (unsigned long)result,
           TestCommand_V4CommunicationModeName(MULTIPARAM_V4_GetCommunicationMode()),
           (unsigned int)MULTIPARAM_V4_IsActiveReceiverRunning());
    if ((result == SENSOR_STREAM_STATE_ERROR) &&
        (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_ACTIVE)) {
        printf("V4调试\t当前处于主动通信，请先发送V4I取得交互窗口\r\n");
    }
    if ((command[2] == 'P') && (result == NO_ERROR)) {
        printf("V4调试\tR01协议版本=%.3f\r\n", (double)protocol_version);
    } else if ((command[2] == 'O') && (result == NO_ERROR)) {
        printf("V4状态\tR02=0x%08lX\t模式=%s\t测水=%u\t磁零点=%u"
               "\t扫频阶段=%u\t驱动阶段=%u\t温度新值=%u\t密度新值=%u\t粘度新值=%u"
               "\t陀螺异常=%u\t测水异常=%u\t电压正常=%u\r\n",
               (unsigned long)status_word,
               TestCommand_V4MeasurementModeName(operating_state.measurement_mode),
               (unsigned int)operating_state.water_enabled,
               (unsigned int)operating_state.magnetic_zero_enabled,
               (unsigned int)operating_state.sweep_stage,
               (unsigned int)operating_state.drive_stage,
               (unsigned int)((status_word >> 8U) & 1U),
               (unsigned int)((status_word >> 9U) & 1U),
               (unsigned int)((status_word >> 10U) & 1U),
               (unsigned int)((status_word >> 16U) & 1U),
               (unsigned int)((status_word >> 17U) & 1U),
               (unsigned int)((status_word >> 18U) & 1U));
    } else if ((command[2] == 'R') && (result == NO_ERROR)) {
        printf("V4参数\tR%02lu\tRAW=0x%08lX\tU32=%lu\tI32=%ld\tFLOAT=%.9g\r\n",
               parameter,
               (unsigned long)raw,
               (unsigned long)raw,
               (long)((int32_t)raw),
               (double)TestCommand_V4RawToFloat(raw));
    }
    if ((print_active_after != 0U) && (result == NO_ERROR)) {
        TestCommand_PrintV4ActiveSnapshot();
    }
    return 1U;
}
