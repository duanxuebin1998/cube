/*
 * @FilePath     : \CUBE\LTD_MAIN_CPU2\Services\Sensor\sensor.c
 * @Description  : 传感器通信相关函数
 * @Author       : Aubon
 * @Date         : 2024-02-23 10:20:27
 * @LastEditors  : Duan Xuebin
 * @LastEditTime : 2026-05-12 11:06:48
 * Copyright 2024 Aubon, All Rights Reserved.
 * 2024-02-23 10:20:27
 */

#include "sensor.h"
#include "error_log.h"
#include "AS5145.h"
#include "measure_tank_height.h"
#include "measure.h"
#include "motor_ctrl.h"
#include "abortable_delay.h"
#include "wireless_pairing.h"
#include "sensor_safe_legacy_adapter.h"

#define SENSOR_LEVEL_FREQ_RECOVERY_LIFT_MM 1.0f /* 传感器数据处理参数：传感器 液位 频率 恢复 抬升 MM。 */
#define SENSOR_DENSITY_MODE_SETTLE_MS 3000U /* 传感器数据处理参数：传感器 密度 模式 稳定 毫秒。 */
#define SENSOR_V4_ACTIVE_FAST_PROBE_MS 750U /* 建链前快速窗口覆盖一个500ms标称周期，并把完整等待留给蓝牙恢复后的第二窗口。 */
#define READ_PART_PARAMS_REFRESH_INTERVAL_MS 1000U /* 部件参数读取刷新间隔，单位 ms。 */
#define READ_PART_PARAMS_RSSI_REFRESH_INTERVAL_MS 5000U /* 部件参数 RSSI 刷新间隔，单位 ms。 */

static uint32_t Sensor_UpdateWirelessRssiForPartParams(uint8_t force_update);
static void Sensor_PrintBluetoothLinkSnapshot(const WirelessConnectionStatus *status);
static uint32_t s_sensor_detection_result = SENSOR_DEVICE_COMM_TIMEOUT;


/*
 * 函数用途：提交本次已确认的传感器身份，并按实际变化决定是否持久化。
 * 调用场景：DetectSensorType 的任一候选协议识别成功后调用。
 * 关键约束：识别失败时不得覆盖 FRAM 中的上次有效身份；相同身份不重复触发参数保存和板间参数更新。
 */
static void Sensor_CommitDetectedIdentity(uint32_t sensor_type, uint32_t sensor_id)
{
    uint8_t identity_changed =
            ((g_deviceParams.sensorType != sensor_type) ||
             (g_deviceParams.sensorID != sensor_id)) ? 1U : 0U;

    g_deviceParams.sensorType = sensor_type;
    g_deviceParams.sensorID = sensor_id;
    s_sensor_detection_result = NO_ERROR;

    if (identity_changed != 0U) {
        save_device_params();
    }
}


/**
 * @brief 判断本次上电或最近一次重新探测是否已经确认传感器身份。
 * @return 1 表示当前运行态身份有效，0 表示探测尚未完成、失败或被命令切换打断。
 */
uint8_t Sensor_IsDetectionValid(void)
{
    return (s_sensor_detection_result == NO_ERROR) ? 1U : 0U;
}


/**
 * @brief 获取最近一次传感器自动识别结果。
 * @return NO_ERROR 表示身份有效；其他值为最近一次探测的通信、协议或命令切换结果。
 */
uint32_t Sensor_GetDetectionResult(void)
{
    return s_sensor_detection_result;
}



/**
 * @brief 在各候选协议探测结果中选择最终识别错误。
 *
 * @param active_v4_before_ret 蓝牙查询前的 V4 主动帧监听结果。
 * @param active_v4_after_ret 蓝牙恢复透传后的 V4 主动帧监听结果。
 * @param version_ret 公共 R01 协议版本读取结果。
 * @param dsm_ret DSM 一代的 CN 传感器编号读取结果。
 * @return 优先保留具体协议错误；全部无响应时返回传感器通信超时。
 */
static uint32_t Sensor_SelectProbeError(uint32_t active_v4_before_ret,
                                        uint32_t active_v4_after_ret,
                                        uint32_t version_ret,
                                        uint32_t dsm_ret)
{
    const uint32_t results[] = {active_v4_before_ret,
                                active_v4_after_ret,
                                version_ret,
                                dsm_ret};

    for (uint32_t index = 0U; index < (sizeof(results) / sizeof(results[0])); index++) {
        if ((results[index] != NO_ERROR) &&
            (results[index] != SENSOR_DEVICE_COMM_TIMEOUT)) {
            return results[index];
        }
    }
    for (uint32_t index = 0U; index < (sizeof(results) / sizeof(results[0])); index++) {
        if (results[index] == SENSOR_DEVICE_COMM_TIMEOUT) {
            return SENSOR_DEVICE_COMM_TIMEOUT;
        }
    }
    return SENSOR_CAPABILITY_UNSUPPORTED;
}


/**
 * @brief 记录传感器识别阶段的最终错误。
 *
 * NO_ERROR 和 STATE_SWITCH 都不是故障，不写入全局 error_code，避免命令切换被误报。
 *
 * @param err 待发布到测量状态中的整机错误码。
 */
static void Sensor_SetCommDetectError(uint32_t err)
{
    /* 传感器探测只锁存真实通信故障；正常结果和用户命令切换不写入全局错误码。 */
    if ((err != NO_ERROR) && (err != STATE_SWITCH)) {
        g_measurement.device_status.error_code = err;
    }
}


/*
 * 函数用途：统一收口传感器自动识别的最终失败状态和现场警告。
 * 调用场景：协议版本或传感器类型已经确定，但编号、通信方式恢复或全部候选探测最终失败时调用。
 * 关键约束：STATE_SWITCH 不作为故障记录；调用方通过 detail 保留失败阶段和多错误上下文。
 */
static uint32_t Sensor_RecordDetectionFailure(uint32_t err, const char *detail)
{
    s_sensor_detection_result = err;
    Sensor_SetCommDetectError(err);
    if ((err != NO_ERROR) && (err != STATE_SWITCH)) {
        /* 错误报警：传感器识别最终失败，继续保留维护和恢复类命令。 */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_SENSOR,
                            "传感器识别",
                            ErrorLog_GetReasonByCode(err),
                            ERROR_LOG_ACTION_CONTINUE,
                            detail);
    }
    return err;
}


/**
 * @brief 在传感器无响应后执行蓝牙链路诊断。
 *
 * 该函数只在普通任务上下文调用，允许打印错误报警；诊断过程中若收到命令切换，
 * 立即返回 STATE_SWITCH，不把切换动作当作通信故障。
 *
 * @param ret 上一层调用返回的结果码。仅当该值为传感器通信超时时继续执行蓝牙链路和传感器诊断，否则原样返回。
 * @param context 标识触发通信超时的传感器操作或测量阶段的 NUL 结尾只读文字，用于后续链路诊断日志。
 * @return 原结果不是通信超时时直接透传；超时时返回蓝牙链路诊断结果、STATE_SWITCH 或 SENSOR_DEVICE_COMM_TIMEOUT。
 */
static uint32_t Sensor_DiagnoseCommTimeout(uint32_t ret, const char *context)
{
    uint32_t link_ret;
    uint32_t diag_ret;
    const char *op_context = (context != NULL) ? context : "未知操作";
    char detail[96];

    if (ret != SENSOR_DEVICE_COMM_TIMEOUT) {
        return ret;
    }

    snprintf(detail, sizeof(detail), "原操作：%s", op_context);
    ErrorLog_WarnDetail(ERROR_LOG_MODULE_SENSOR,
                        ERROR_LOG_OP_COMM_DIAG,
                        ErrorLog_GetReasonByCode(ret),
                        ERROR_LOG_ACTION_CONTINUE,
                        detail);

    link_ret = WirelessPairing_CheckBluetoothLink();
    if (link_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }

    diag_ret = link_ret;
    if (diag_ret != NO_ERROR) {
        snprintf(detail, sizeof(detail), "原操作：%s,链路：蓝牙", op_context);
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_SLIPRING_COMM,
                            "蓝牙链路诊断",
                            ErrorLog_GetReasonByCode(diag_ret),
                            ERROR_LOG_ACTION_CONTINUE,
                            detail);
        return diag_ret;
    }

    return SENSOR_DEVICE_COMM_TIMEOUT;
}

/**
 * @brief 识别传感器前检查蓝牙主机和蓝牙从机连接状态。
 *
 * 通过 CH9141 状态查询确认蓝牙连接是否可用，失败时映射为主机或从机通信超时，便于现场定位。
 *
 * @param status 可写无线连接状态对象；包含蓝牙链路、配对状态、MAC 有效性和规范化 MAC，函数按职责复位、填充或输出这些字段。
 * @return 返回无线链路检查结果码；NO_ERROR 表示主从蓝牙链路可用于传感器探测，其他值区分未连接、查询失败或命令切换。
 */
static uint32_t Sensor_ProbeWirelessLink(WirelessConnectionStatus *status)
{
    return WirelessPairing_CheckBluetoothLinkDetailed(status);
}

/**
 * @brief 打印最近一次蓝牙链路检查得到的从机 MAC 和 RSSI。
 *
 * 调用场景：DetectSensorType() 完成蓝牙链路检查后调用；不再次访问 UART6，避免延长传感器识别时序。
 * 关键约束：RSSI 获取失败不代表链路断开，按本次快照有效位分别打印。
 *
 * @param status 只读无线连接状态快照；包含蓝牙链路、配对状态、MAC 有效性和规范化 MAC，用于生成诊断输出。
 */
static void Sensor_PrintBluetoothLinkSnapshot(const WirelessConnectionStatus *status)
{
    if (status == NULL) {
        printf("蓝牙链路正常 | 从机MAC=未读取 | RSSI=未读取\r\n");
        return;
    }

    printf("蓝牙链路正常 | 从机MAC=");
    if (status->mac_valid != 0U) {
        printf("%02lX:%02lX:%02lX:%02lX:%02lX:%02lX",
               (unsigned long)((status->mac_high >> 8U) & 0xFFU),
               (unsigned long)(status->mac_high & 0xFFU),
               (unsigned long)((status->mac_mid >> 8U) & 0xFFU),
               (unsigned long)(status->mac_mid & 0xFFU),
               (unsigned long)((status->mac_low >> 8U) & 0xFFU),
               (unsigned long)(status->mac_low & 0xFFU));
    } else {
        printf("未读取");
    }

    printf(" | RSSI=");
    if (status->rssi_valid != 0U) {
        printf("%ld dB", (long)status->rssi);
    } else {
        printf("未读取");
    }
    printf("\r\n");
}

/**
 * @brief 解析 DSM 一代字符串编号中的数字部分。
 *
 * CN 指令返回通常形如 N2009924H，系统参数只能保存 uint32_t，因此这里只提取连续数字并保存为传感器编号。
 *
 * @param id_text 包含 DSM 文本标识字段的十六进制字符串，必须完整表示固定长度标识。
 * @param sensor_id_out 用于返回探测并完成格式校验的传感器编号。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t Sensor_ParseDsmTextId(const char *id_text, uint32_t *sensor_id_out)
{
    uint32_t value = 0U;
    uint8_t has_digit = 0U;

    if ((id_text == NULL) || (sensor_id_out == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    while (*id_text != '\0') {
        if ((*id_text >= '0') && (*id_text <= '9')) {
            uint32_t digit = (uint32_t)(*id_text - '0');

            /* 超长编号不得回绕成伪造的 uint32_t 传感器编号。 */
            if (value > ((UINT32_MAX - digit) / 10U)) {
                return SENSOR_RESP_FORMAT_ERROR;
            }
            value = (value * 10U) + digit;
            has_digit = 1U;
        }
        id_text++;
    }

    if ((has_digit == 0U) || (value == 0U)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    *sensor_id_out = value;
    return NO_ERROR;
}

/**
 * @brief 用静默传感器编号读数探测 多参数传感器通信协议 V3.0（当前LTD使用） 传感器。
 *
 * 识别阶段的候选协议未命中不是最终故障，因此这里不打印错误重试日志。
 *
 * @param sensor_id_out 用于返回探测并完成格式校验的传感器编号。
 * @return 返回 多参数传感器通信协议 V3.0（当前LTD使用） 静默传感器编号读取结果；NO_ERROR 表示探测成功，其他值表示通信或响应校验失败。
 */
static uint32_t Sensor_ProbeLtdSensor(uint32_t *sensor_id_out)
{
    uint32_t sensor_id = 0U;
    uint32_t ret = MULTIPARAM_V3_Probe_SensorID(&sensor_id);

    /* 只有 R22 探测成功且调用方提供输出地址时才发布设备编号，失败时保留调用方原值。 */
    if ((ret == NO_ERROR) && (sensor_id_out != NULL)) {
        *sensor_id_out = sensor_id;
    }
    return ret;
}

/**
 * @brief 返回 DSM RAM 会话版本分类的诊断名称。
 * @param profile DSM CPU1 版本所属维护线。
 * @return 只读静态文字，用于探测阶段日志，不参与协议或持久化判断。
 */
static const char *Sensor_DsmVersionProfileName(DsmVersionProfile profile)
{
    switch (profile) {
    case DSM_VERSION_PROFILE_UNREAD:
        return "unread";
    case DSM_VERSION_PROFILE_CV07_AMBIGUOUS:
        return "CV07-ambiguous";
    case DSM_VERSION_PROFILE_V3:
        return "V3";
    case DSM_VERSION_PROFILE_V4:
        return "V4";
    case DSM_VERSION_PROFILE_V5_0_AMBIGUOUS:
        return "V5.0-ambiguous";
    case DSM_VERSION_PROFILE_V5:
        return "V5";
    case DSM_VERSION_PROFILE_V6:
        return "V6";
    default:
        return "unknown";
    }
}

/**
 * @brief 用振动管编号探测 DSM 一代传感器。
 *
 * CN 编号读取用于识别协议并保存编号；识别成功后仍切到密度模式，保持初始化后的运行模式语义不变。
 *
 * @param sensor_id_out 用于返回探测并完成格式校验的传感器编号。
 * @return NO_ERROR 表示编号命令收发成功且文本格式有效；其他值为 DSM 通信、超时、校验、响应格式或编号内容校验错误。
 */
static uint32_t Sensor_ProbeDsmSensor(uint32_t *sensor_id_out)
{
    char id_text[RCVBUFFLEN] = {0};
    const DsmSessionContext *version_context;
    uint32_t sensor_id = 0U;
    uint32_t version_ret;
    uint32_t ret;

    version_ret = DSM_ReadVersionContext();
    if (version_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }

    version_context = DSM_GetSessionContext();
    if ((version_context != NULL) &&
        ((version_context->cpu1_valid != 0U) || (version_context->cpu0_valid != 0U))) {
        if (version_context->cpu1_valid != 0U) {
            printf("DSM版本 | CPU1=%s | profile=%s | 低电压=%lu",
                   version_context->cpu1_version,
                   Sensor_DsmVersionProfileName(version_context->profile),
                   (unsigned long)version_context->low_voltage);
        } else {
            printf("DSM版本 | CPU1=%s | profile=%s | 低电压=%lu",
                   (version_context->profile == DSM_VERSION_PROFILE_CV07_AMBIGUOUS)
                       ? "Cv未响应"
                       : "未读取",
                   Sensor_DsmVersionProfileName(version_context->profile),
                   (unsigned long)version_context->low_voltage);
        }
        if (version_context->cpu0_valid != 0U) {
            printf(" | CPU0=%lu.%02lu | 组合前缀=%s\r\n",
                   (unsigned long)(version_context->cpu0_version_x100 / 100U),
                   (unsigned long)(version_context->cpu0_version_x100 % 100U),
                   version_context->combined_prefix);
        } else {
            printf(" | CPU0=未读取\r\n");
        }
    }
    if (version_ret != NO_ERROR) {
        /* 老版本可能不支持 Cv；版本读取失败不能替代 CN 探测结果。 */
        printf("DSM版本读取未完成，继续按CN兼容探测。错误码=0x%08lX\r\n",
               (unsigned long)version_ret);
    }

    ret = Read_VibrationTube_ID(id_text, sizeof(id_text));
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = Sensor_ParseDsmTextId(id_text, &sensor_id);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = DSM_EnableDensityMode();
    if (ret != NO_ERROR) {
        return ret;
    }

    if (sensor_id_out != NULL) {
        *sensor_id_out = sensor_id;
    }
    return NO_ERROR;
}

/*
 * 函数用途：在有界窗口内被动识别多参数V4主动上报帧。
 * 调用场景：蓝牙链路建立后、任何同步协议探测之前。
 * 关键约束：识别成功后保留V4常驻接收；未命中或命令切换时释放UART6。
 */
static uint32_t Sensor_ProbeV4Active(const char *operation,
                                     uint8_t *address_out,
                                     uint32_t timeout_ms)
{
    multiparam_v4_snapshot_t snapshot;
    uint32_t start_tick;
    uint32_t result;

    MULTIPARAM_V4_Init(MULTIPARAM_V4_ANY_ADDRESS);
    MULTIPARAM_V4_MeasurementInit();
    result = MULTIPARAM_V4_StartActiveReceive();
    if (result != NO_ERROR) {
        MULTIPARAM_V4_PrintActiveProbePacket(operation, result);
        MULTIPARAM_V4_Deinit();
        return result;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (HasEffectiveCommandSwitchRequest()) {
            MULTIPARAM_V4_PrintActiveProbePacket(operation, STATE_SWITCH);
            MULTIPARAM_V4_Deinit();
            return STATE_SWITCH;
        }
        MULTIPARAM_V4_Service();
        result = MULTIPARAM_V4_CopyLatestSnapshot(&snapshot);
        if ((result == NO_ERROR) &&
            (MULTIPARAM_V4_IsSnapshotFresh(timeout_ms) != 0U)) {
            if (address_out != NULL) {
                *address_out = snapshot.address;
            }
            MULTIPARAM_V4_PrintActiveProbePacket(operation, NO_ERROR);
            return NO_ERROR;
        }
        HAL_Delay(1U);
    }

    MULTIPARAM_V4_PrintActiveProbePacket(operation, SENSOR_DEVICE_COMM_TIMEOUT);
    MULTIPARAM_V4_Deinit();
    return SENSOR_DEVICE_COMM_TIMEOUT;
}

/*
 * 函数用途：通过公共 R01 读取多参数传感器协议版本。
 * 调用场景：未收到 V4 主动帧时，用同一条读取指令区分当前 V3.0 和 V4.0 协议。
 * 关键约束：R01 只负责版本分流；识别版本后必须再按对应地址读取传感器编号。
 */
static uint32_t Sensor_ProbeMultiparamVersion(float *protocol_version_out)
{
    float protocol_version = NAN;
    uint32_t result;

    MULTIPARAM_V4_Init(0U);
    MULTIPARAM_V4_MeasurementInit();
    MULTIPARAM_V4_SetProbeTraceEnabled(1U);
    result = MULTIPARAM_V4_ProbeProtocolVersion(&protocol_version);
    MULTIPARAM_V4_SetProbeTraceEnabled(0U);
    if ((protocol_version_out != NULL) && isfinite(protocol_version)) {
        *protocol_version_out = protocol_version;
    }
    if (result != NO_ERROR) {
        MULTIPARAM_V4_Deinit();
    }
    return result;
}


/*
 * 函数用途：读取 V4 传感器编号并保持识别前的主动或交互通信方式。
 * 调用场景：V4 主动帧或 R01=4.0 已确认后、提交传感器身份前调用。
 * 关键约束：主动上报时先按既有 20 ms 窗口切到交互模式读取 R67，读取结束后必须恢复主动上报。
 */
static uint32_t Sensor_ReadV4SensorIdPreservingMode(uint32_t *sensor_id_out,
                                                    char *detail,
                                                    size_t detail_size)
{
    multiparam_v4_communication_mode_t original_mode;
    uint32_t result;
    uint32_t restore_result = NO_ERROR;

    if ((sensor_id_out == NULL) || (detail == NULL) || (detail_size == 0U)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    detail[0] = '\0';

    original_mode = MULTIPARAM_V4_GetCommunicationMode();
    if (original_mode == MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        result = MULTIPARAM_V4_EnterInteractive();
        if (result != NO_ERROR) {
            (void)snprintf(detail, detail_size,
                           "阶段=进入交互模式,结果=0x%08lX",
                           (unsigned long)result);
            return result;
        }
    } else if (original_mode != MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        (void)snprintf(detail, detail_size, "阶段=检查V4通信方式,状态=%u",
                       (unsigned int)original_mode);
        return SENSOR_STREAM_STATE_ERROR;
    }

    result = MULTIPARAM_V4_ReadSensorID(sensor_id_out);
    if (original_mode == MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        restore_result = MULTIPARAM_V4_EnterActive();
        (void)snprintf(detail, detail_size,
                       "R67读取=0x%08lX,主动恢复=0x%08lX",
                       (unsigned long)result,
                       (unsigned long)restore_result);
        /* 恢复失败意味着传感器持久化通信方式未闭环，其优先级高于本次编号读取错误。 */
        if (restore_result != NO_ERROR) {
            return restore_result;
        }
    } else {
        (void)snprintf(detail, detail_size, "R67读取=0x%08lX",
                       (unsigned long)result);
    }
    return result;
}


/*
 * 函数用途：完成 V4 协议识别后的编号读取和身份提交。
 * 调用场景：主动帧或公共 R01 已经确认 V4.0 后调用。
 * 关键约束：R67 编号读取失败时不得用编号 0 提交伪身份，并释放 V4 对 UART6 的占用。
 */
static uint32_t Sensor_CompleteV4Detection(const char *communication,
                                           uint8_t sensor_address)
{
    char detail[96] = {0};
    uint32_t sensor_id = 0U;
    uint32_t result;

    MULTIPARAM_V4_SetProbeTraceEnabled(1U);
    result = Sensor_ReadV4SensorIdPreservingMode(&sensor_id,
                                                 detail,
                                                 sizeof(detail));
    MULTIPARAM_V4_SetProbeTraceEnabled(0U);

    if (result != NO_ERROR) {
        MULTIPARAM_V4_Deinit();
        return Sensor_RecordDetectionFailure(result, detail);
    }

    Sensor_CommitDetectedIdentity(MULTIPARAM_V4_SENSOR, sensor_id);
    printf("识别成功：多参数V4传感器 | 通信=%s | 地址=%u | 编号=%lu\r\n",
           (communication != NULL) ? communication : "未知",
           (unsigned int)sensor_address,
           (unsigned long)sensor_id);
    printf("====================================\r\n");
    return NO_ERROR;
}

/*
 * 函数用途：在多参数V4主动通信下集中取得交互窗口并执行一个模式命令。
 * 调用场景：业务明确切换密度或液位模式时。
 * 关键约束：无论模式命令成功与否都尝试恢复原主动通信，不在普通读取中写参数65。
 */
static uint32_t Sensor_RunV4ModeCommand(uint32_t (*command)(void))
{
    multiparam_v4_communication_mode_t original_mode;
    uint32_t command_result;
    uint32_t restore_result;

    if (command == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    original_mode = MULTIPARAM_V4_GetCommunicationMode();
    if (original_mode == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        return command();
    }
    if (original_mode != MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    command_result = MULTIPARAM_V4_EnterInteractive();
    if (command_result != NO_ERROR) {
        return command_result;
    }
    command_result = command();
    restore_result = MULTIPARAM_V4_EnterActive();
    return (command_result != NO_ERROR) ? command_result : restore_result;
}

/*
 * 函数用途：用新鲜主动快照判断目标V4功能状态是否已经满足。
 * 调用场景：主动通信下的高频水电容或零点霍尔读取前。
 * 关键约束：快照过期时不得短路，必须取得交互窗口读取R02恢复事实。
 */
static uint8_t Sensor_V4ActiveFeatureMatches(multiparam_v4_feature_state_t target_feature,
                                             uint8_t enabled)
{
    multiparam_v4_snapshot_t snapshot;

    if (MULTIPARAM_V4_GetCommunicationMode() != MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        return 0U;
    }
    if ((MULTIPARAM_V4_CopyLatestSnapshot(&snapshot) != NO_ERROR) ||
        ((HAL_GetTick() - snapshot.received_tick) > MULTIPARAM_V4_ACTIVE_TIMEOUT_MS)) {
        return 0U;
    }
    if (enabled != 0U) {
        return (uint8_t)(((snapshot.measurement_mode == MULTIPARAM_V4_MEASUREMENT_DENSITY) &&
                          (snapshot.feature_state == target_feature)) ? 1U : 0U);
    }
    return (uint8_t)((snapshot.feature_state != target_feature) ? 1U : 0U);
}

/*
 * 函数用途：在多参数V4主动通信下集中取得交互窗口并执行一个目标状态命令。
 * 调用场景：业务明确使能或关闭测水、零点霍尔功能时。
 * 关键约束：L/M翻转的应答丢失恢复由协议层先读状态完成，业务层不得盲目重发。
 */
static uint32_t Sensor_RunV4FeatureCommand(uint32_t (*command)(uint8_t), uint8_t enabled)
{
    multiparam_v4_communication_mode_t original_mode;
    uint32_t command_result;
    uint32_t restore_result;

    if (command == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    original_mode = MULTIPARAM_V4_GetCommunicationMode();
    if (original_mode == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        return command(enabled);
    }
    if (original_mode != MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    command_result = MULTIPARAM_V4_EnterInteractive();
    if (command_result != NO_ERROR) {
        return command_result;
    }
    command_result = command(enabled);
    restore_result = MULTIPARAM_V4_EnterActive();
    return (command_result != NO_ERROR) ? command_result : restore_result;
}

/**
 * @brief 判断当前传感器是否提供水位电容通道。
 *
 * @details 调用场景：水位电容读取、部件参数和回零流程。
 * @note 关键约束：安全传感器只检查水位电容能力，不依赖姿态能力。
 *
 * @return true 表示当前传感器类型声明支持水位电容通道；否则返回 false。
 */
int Sensor_SupportsWaterCapChannel(void)
{
    /* 探测无效时 FRAM 中的历史类型不能授权回零附加通信。 */
    if (Sensor_IsDetectionValid() == 0U) {
        return 0;
    }
    return (int)(((g_deviceParams.sensorType == DSM_SENSOR) ||
                  (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) ||
                  ((g_deviceParams.sensorType == SAFE_SENSOR) &&
                   (SensorSafeAdapter_IsActive() != 0U) &&
                   (SensorSafeAdapter_SupportsWaterCap() != 0U))) ? 1 : 0);
}

/**
 * @brief 判断当前传感器是否提供姿态角通道。
 *
 * @details 调用场景：姿态读取、部件参数和回零流程。
 * @note 关键约束：安全传感器只检查姿态能力，不依赖水位电容能力。
 *
 * @return true 表示当前传感器类型声明支持陀螺仪通道；否则返回 false。
 */
int Sensor_SupportsGyroChannel(void)
{
    /* 探测无效时 FRAM 中的历史类型不能授权回零附加通信。 */
    if (Sensor_IsDetectionValid() == 0U) {
        return 0;
    }
    return (int)(((g_deviceParams.sensorType == DSM_SENSOR) ||
                  (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) ||
                  ((g_deviceParams.sensorType == SAFE_SENSOR) &&
                   (SensorSafeAdapter_IsActive() != 0U) &&
                   (SensorSafeAdapter_SupportsGyro() != 0U))) ? 1 : 0);
}

/**
 * @brief 自动识别传感器类型，覆盖多参数V4主动/交互、多参数V3.0和DSM一代。
 *
 * @return uint32_t 识别成功返回 NO_ERROR，失败返回具体通信、协议、编号或状态切换错误。
 */
uint32_t DetectSensorType(void) {
    uint32_t ret;
    uint32_t active_v4_before_link_ret;
    uint32_t active_v4_after_link_ret;
    uint32_t version_ret;
    uint32_t v3_ret;
    uint32_t dsm_ret;
    uint32_t sensor_id = 0U;
    uint8_t sensor_address = 0U;
    float protocol_version = 0.0f;
    WirelessConnectionStatus bluetooth_status;

    /* 新探测开始前，FRAM 中的上次结果只作为历史信息，不能授权新的测量流程。 */
    s_sensor_detection_result = SENSOR_DEVICE_COMM_TIMEOUT;

    printf("========== 传感器识别开始 ==========\r\n");
    printf("[1/6] 蓝牙查询前监听多参数V4主动帧\r\n");
    active_v4_before_link_ret = Sensor_ProbeV4Active("主动帧-蓝牙查询前",
                                                     &sensor_address,
                                                     SENSOR_V4_ACTIVE_FAST_PROBE_MS);
    if (active_v4_before_link_ret == NO_ERROR) {
        return Sensor_CompleteV4Detection("主动上报", sensor_address);
    }
    if (active_v4_before_link_ret == STATE_SWITCH) {
        s_sensor_detection_result = STATE_SWITCH;
        return STATE_SWITCH;
    }

    printf("[2/6] 检查蓝牙链路\r\n");
    ret = Sensor_ProbeWirelessLink(&bluetooth_status);
    if (ret != NO_ERROR) {
        return Sensor_RecordDetectionFailure(ret, "阶段=蓝牙链路检查");
    }
    Sensor_PrintBluetoothLinkSnapshot(&bluetooth_status);

    /* AT 查询并恢复透传后再次监听，吸收链路恢复期间尚未处理完的主动帧。 */
    printf("[3/6] 蓝牙恢复透传后再次监听多参数V4主动帧\r\n");
    active_v4_after_link_ret = Sensor_ProbeV4Active("主动帧-透传恢复后",
                                                    &sensor_address,
                                                    MULTIPARAM_V4_ACTIVE_TIMEOUT_MS);
    if (active_v4_after_link_ret == NO_ERROR) {
        return Sensor_CompleteV4Detection("主动上报", sensor_address);
    }
    if (active_v4_after_link_ret == STATE_SWITCH) {
        s_sensor_detection_result = STATE_SWITCH;
        return STATE_SWITCH;
    }

    printf("[4/6] 读取R01区分多参数协议V3.0和V4.0\r\n");
    version_ret = Sensor_ProbeMultiparamVersion(&protocol_version);
    if (version_ret == NO_ERROR) {
        return Sensor_CompleteV4Detection("交互", 0U);
    }
    if (version_ret == STATE_SWITCH) {
        s_sensor_detection_result = STATE_SWITCH;
        return STATE_SWITCH;
    }

    if ((version_ret == SENSOR_PROTOCOL_VERSION_INCOMPATIBLE) &&
        isfinite(protocol_version) &&
        (fabsf(protocol_version - 3.0f) <= 0.01f)) {
        /* R01 已完成 V3.0 版本分流，R22 只用于读取该协议自己的传感器编号。 */
        printf("[5/6] R01版本=%.2f，读取V3.0的R22传感器编号\r\n",
               protocol_version);
        version_ret = NO_ERROR;
        v3_ret = Sensor_ProbeLtdSensor(&sensor_id);
        if (v3_ret == NO_ERROR) {
            Sensor_CommitDetectedIdentity(LTD_SENSOR, sensor_id);
            printf("识别成功：LTD传感器 | 协议=多参数V3.0 | 编号=%lu\r\n",
                   (unsigned long)sensor_id);
            printf("====================================\r\n");
            return NO_ERROR;
        }
        return Sensor_RecordDetectionFailure(v3_ret, "阶段=读取V3.0的R22传感器编号");
    } else if ((version_ret == SENSOR_PROTOCOL_VERSION_INCOMPATIBLE) &&
               isfinite(protocol_version)) {
        printf("R01版本=%.2f，不属于当前支持的V3.0或V4.0\r\n",
               protocol_version);
        return Sensor_RecordDetectionFailure(version_ret, "阶段=检查R01协议版本");
    } else {
        printf("R01未取得有效协议版本，不使用R22猜测协议类型\r\n");
    }

    printf("[6/6] 尝试DSM一代协议并读取CN传感器编号\r\n");
    dsm_ret = Sensor_ProbeDsmSensor(&sensor_id);
    if (dsm_ret == NO_ERROR) {
        Sensor_CommitDetectedIdentity(DSM_SENSOR, sensor_id);
        printf("识别成功：DSM传感器 | 编号=%lu | 密度模式握手成功\r\n",
               (unsigned long)sensor_id);
        printf("====================================\r\n");
        return NO_ERROR;
    }
    if (dsm_ret == STATE_SWITCH) {
        s_sensor_detection_result = STATE_SWITCH;
        return STATE_SWITCH;
    }

    ret = Sensor_SelectProbeError(active_v4_before_link_ret,
                                  active_v4_after_link_ret,
                                  version_ret,
                                  dsm_ret);
    printf("识别失败：未匹配支持的传感器 | V4主动前=%s | V4主动后=%s | R01=%s | DSM=%s\r\n",
           ErrorLog_GetReasonByCode(active_v4_before_link_ret),
           ErrorLog_GetReasonByCode(active_v4_after_link_ret),
           ErrorLog_GetReasonByCode(version_ret),
           ErrorLog_GetReasonByCode(dsm_ret));
    return Sensor_RecordDetectionFailure(ret, "阶段=候选协议全部失败");
}

/**
 * @brief 切换传感器到密度测量模式并等待模式生效。
 * @return 返回整机错误码；NO_ERROR 表示传感器已进入密度模式并完成稳定等待，其他值由模式切换或链路诊断返回。
 */
uint32_t EnableDensityMode(void) {
    uint32_t ret;

    if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = DSM_EnableDensityMode();
    } else if (g_deviceParams.sensorType == SAFE_SENSOR) {
        ret = SensorSafeAdapter_EnableDensityMode();
    } else if (g_deviceParams.sensorType == LTD_SENSOR) {
        ret = MULTIPARAM_V3_SwitchToDensityMode();
    } else if (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) {
        ret = Sensor_RunV4ModeCommand(MULTIPARAM_V4_SelectDensityMode);
    } else {
        ret = SENSOR_CAPABILITY_UNSUPPORTED;
    }
    return Sensor_DiagnoseCommTimeout(ret, "切换密度模式");
}

/**
 * @brief 读取 LTD 部件参数前切换并确认传感器密度模式。
 * @return NO_ERROR 表示 LTD 密度模式切换成功且稳定等待完成；切换失败返回原通信或协议错误，等待期间出现命令切换返回 STATE_SWITCH。
 */
static uint32_t Sensor_PrepareLtdDensityModeForPartParams(void)
{
	uint32_t ret = EnableDensityMode();
	if (ret != NO_ERROR) {
		return ret;
	}
	printf("读取部件参数\tLTD已切换密度模式，等待%lu ms稳定\r\n",
	       (unsigned long)SENSOR_DENSITY_MODE_SETTLE_MS);
	return AbortableDelay_CommandSwitch(SENSOR_DENSITY_MODE_SETTLE_MS, 50U);
}

/**
 * @brief 按当前传感器类型切换液位模式，并执行对应的稳定等待。
 * @return 返回整机错误码；NO_ERROR 表示当前传感器已进入液位模式并完成稳定等待，其他值透传模式切换失败。
 */
uint32_t EnableLevelMode(void) {
    uint32_t ret;

    ret = MotorCtrl_SlowStop();
    if (ret != NO_ERROR) {
        return ret;
    }

    if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = DSM_EnableLevelMode();
    } else if (g_deviceParams.sensorType == SAFE_SENSOR) {
        ret = SensorSafeAdapter_EnableLevelMode();
    } else if (g_deviceParams.sensorType == LTD_SENSOR) {
        ret = MULTIPARAM_V3_SwitchToLevelMode();
    } else if (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) {
        ret = Sensor_RunV4ModeCommand(MULTIPARAM_V4_SelectLevelMode);
    } else {
        ret = SENSOR_CAPABILITY_UNSUPPORTED;
    }
    ret = Sensor_DiagnoseCommTimeout(ret, "切换液位模式");
    if (ret == NO_ERROR) {
        printf("切换液位模式成功，等待%lu ms稳定\r\n", (unsigned long)SENSOR_LEVEL_MODE_SETTLE_MS);
        ret = AbortableDelay_CommandSwitch(SENSOR_LEVEL_MODE_SETTLE_MS, 100U);
    }
    return ret;
}

/*
 * 函数用途：按当前传感器类型读取一次液位模式频率，不执行运动或模式恢复。
 * 调用场景：液位静态读取、运动中状态判断和主动快照消费。
 * 关键约束：多参数V4由统一测量适配层按主动或交互方式读取，并在解释R04前核对R02模式。
 */
uint32_t Sensor_ReadLevelFrequency(uint32_t *frequency_out)
{
    if (frequency_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (g_deviceParams.sensorType == DSM_SENSOR) {
        return Read_Level_Frequency(frequency_out);
    }
    if (g_deviceParams.sensorType == SAFE_SENSOR) {
        return SensorSafeAdapter_ReadLevelFrequency(frequency_out);
    }
    if (g_deviceParams.sensorType == LTD_SENSOR) {
        return MULTIPARAM_V3_Read_LevelFrequency(frequency_out);
    }
    if (g_deviceParams.sensorType != MULTIPARAM_V4_SENSOR) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    return MULTIPARAM_V4_MeasurementReadLevelFrequency(frequency_out);
}

/* 读取一次并以整数 Hz 返回。 */
/* 这里的循环是业务层“等待有效频率”，不是底层串口通信重试； */
/* 真正的通信重试统一收敛在各协议层，所有 UART6 传感器/无线协议统一使用 UART6_COMM_MAX_RETRY。 */
/* 如果频率连续 3 次为 0 或大于 6500Hz，且电机静止，则上行 1mm 后切密度/液位模式恢复； */
/* 若多轮恢复后仍无有效频率，则返回 SONIC_FREQ_ABNORMAL。 */

/**
 * @brief 判断电机是否已经停止，供液位频率恢复动作使用。
 *
 * 先看上层显示状态，再读取驱动运动状态；该函数会访问 TMC5130，不允许在中断中调用。
 *
 * @return 1 表示电机已经停止，供液位频率恢复动作使用；0 表示电机尚未停止，供液位频率恢复动作使用。
 */
static uint8_t Sensor_IsMotorStopped(void)
{
    uint32_t motor_state = MotorCtrl_GetDisplayState();
    bool is_moving = true;
    uint32_t ret;

    if ((motor_state == 1U) || (motor_state == 2U)) {
        return 0U;
    }

    ret = MotorCtrl_IsDriverMoving(&stepper, &is_moving);
    if (ret != NO_ERROR) {
        return 0U;
    }
    return is_moving ? 0U : 1U;
}

/**
 * @brief 电机停止时尝试恢复液位频率有效读数。
 *
 * 仅在频率连续异常且电机静止时执行：微动上行后切密度/液位模式重新稳定；
 * 等待过程使用可打断延时，命令切换时直接返回 STATE_SWITCH。
 *
 * @return NO_ERROR 表示无需恢复或已经重新取得有效液位频率；恢复模式、稳定等待或频率读取失败时返回对应具体错误。
 */
static uint32_t Sensor_RecoverLevelFrequencyWhenStopped(void)
{
	uint32_t ret;

	if (!Sensor_IsMotorStopped()) {
		ret = EnableLevelMode();
		if (ret != NO_ERROR) {
			return ret;
		}
		return NO_ERROR;
	}

	ret = MotorCtrl_MoveAndWait(SENSOR_LEVEL_FREQ_RECOVERY_LIFT_MM,
	                            MOTOR_DIRECTION_UP,
	                            MotorCtrl_GetDefaultSpeedX100());
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = EnableDensityMode();
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = AbortableDelay_CommandSwitch(SENSOR_DENSITY_MODE_SETTLE_MS, 100U);
	if (ret != NO_ERROR) {
		return ret;
	}

	ret = EnableLevelMode();
	if (ret != NO_ERROR) {
		return ret;
	}

	return NO_ERROR;
}

/**
 * @brief 按协议层重试策略读取整数 Hz 液位频率；连续三次为 0 或超过 6500 Hz 时执行受电机状态约束的模式恢复，多轮恢复仍无效则返回 SONIC_FREQ_ABNORMAL。
 *
 * @param frequency_out 用于返回读取或平均后的液位通道频率。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t DSM_Get_LevelMode_Frequence(volatile uint32_t *frequency_out) {
	if (frequency_out == NULL) {
		return SYSTEM_CALL_CONDITION_ERROR;   /* 比设备通信错误更合理 */
	}

	uint32_t ret;
	uint32_t hz = 0;
	const int MAX_INVALID_FREQ_RETRY = 3;
	const int MAX_MODE_SWITCH_RECOVERY = 3;
	int mode_switch_recovery_count = 0;

	while (1) {
		for (int attempt = 0; attempt < MAX_INVALID_FREQ_RETRY; attempt++) {
			ret = Sensor_ReadLevelFrequency(&hz);

			if (ret != NO_ERROR) {
				return Sensor_DiagnoseCommTimeout(ret, "读取液位频率");  /* 读取失败直接返回错误码 */
			}

			if (hz != 0 && hz <= 6500) {
				*frequency_out = hz;
				printf("液位频率: %lu Hz\r\n", (unsigned long)*frequency_out);
				return NO_ERROR;
			}

            ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                           ERROR_LOG_OP_READ_LEVEL_FREQ,
                           ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                           (uint32_t)(attempt + 1),
                           MAX_INVALID_FREQ_RETRY,
                           SONIC_FREQ_ABNORMAL);
			ret = AbortableDelay_CommandSwitch(1000U, 100U);
			if (ret != NO_ERROR) {
				return ret;
			}
		}

		if (mode_switch_recovery_count >= MAX_MODE_SWITCH_RECOVERY) {
			return SONIC_FREQ_ABNORMAL;
		}

		mode_switch_recovery_count++;
        ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                       ERROR_LOG_OP_SWITCH_MODE,
                       ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                       (uint32_t)mode_switch_recovery_count,
                       MAX_MODE_SWITCH_RECOVERY,
                       SONIC_FREQ_ABNORMAL);
		ret = Sensor_RecoverLevelFrequencyWhenStopped();
		if (ret != NO_ERROR) {
			return ret;
		}
        ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                         ERROR_LOG_OP_SWITCH_MODE,
                         ErrorLog_GetCodeName(SONIC_FREQ_ABNORMAL),
                         (uint32_t)mode_switch_recovery_count,
                         MAX_MODE_SWITCH_RECOVERY);
	}
}

/**
 * @brief 获取液位跟随频率的平均值
 *        （10 次采样，2s 间隔，去 2 大 2 小，取中间 6 次均值）
 *
 * @param frequency_out 用于返回读取或平均后的液位通道频率。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t DSM_Get_LevelMode_Frequence_Avg(volatile uint32_t *frequency_out) {
	if (frequency_out == NULL) {
		return SYSTEM_CALL_CONDITION_ERROR;
	}

	uint32_t values[10];
	uint32_t ret;

	for (int i = 0; i < 10; i++) {
		ret = DSM_Get_LevelMode_Frequence(&values[i]);
		if (ret != NO_ERROR) {
			return ret;
		}
		printf("第 %d 次液位频率: %lu Hz\r\n", i + 1, (unsigned long) values[i]);
		ret = AbortableDelay_CommandSwitch(2000U, 100U); /* 2 秒间隔，可被命令切换打断 */
		if (ret != NO_ERROR) {
			return ret;
		}
	}

	/* 冒泡排序（升序） */
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9 - i; j++) {
			if (values[j] > values[j + 1]) {
				uint32_t tmp = values[j];
				values[j] = values[j + 1];
				values[j + 1] = tmp;
			}
		}
	}

	/* 去掉两个最大与两个最小 */
	float sum = 0.0f;
	for (int i = 2; i < 8; i++) {
		sum += (float) values[i];
	}

	float avg = sum / 6.0f;
	uint32_t avg_u32 = (avg >= 0.0f) ? (uint32_t) (avg + 0.5f) : 0u;
	*frequency_out = avg_u32;

	printf("液位频率平均值(去极值): %lu Hz\r\n", (unsigned long) *frequency_out);
	return NO_ERROR;
}

/**
 * @brief 读取并打印传感器密度诊断文本。
 *
 * @param frequency 传感器频率输出指针，读取成功时写入 Hz 浮点值并用于诊断打印。
 * @param density 传感器密度输出指针，读取成功时写入 kg/m3 浮点值并用于诊断打印。
 * @param temp 用于返回传感器温度的输出参数，单位 ℃。
 * @return NO_ERROR 表示三个输出指针有效且诊断读取已完成；任一输出指针为空返回 SYSTEM_CALL_CONDITION_ERROR。
 */
uint32_t Read_Density_text(float *frequency, float *density, float *temp) {
	if (frequency == NULL || temp == NULL || density == NULL) {
		return SYSTEM_CALL_CONDITION_ERROR;
	}
	*frequency = 5500.123f;
	*density = 800.5f;
	*temp = -180.52f;
	printf("密度: %.2f  频率: %.3f  温度: %.3f ℃\r\n", *density, *frequency, *temp);

	return 0;
}

/**
 * @brief 按固定修正参数校正密度和温度结果。
 *
 * @param density 待原地修正的密度值指针，输入和输出单位均为 kg/m3。
 * @param temp 待原地修正的温度值，输入和输出单位均为 ℃。
 */
static void Apply_Fixed_DensityTemp_Correction(float *density, float *temp)
{
    if ((density)&&(*density>200.0)) {
        *density = (*density) + ((float)g_deviceParams.densityCorrection - (float)DENSITY_CORRECTION_BASE_RAW) / 100.0f;
    }

    if (temp) {
        *temp = (*temp) + ((float)g_deviceParams.temperatureCorrection-1000.0f)/ 10.0f;
    }
}

/**
 * @brief 按传感器类型读取频率、密度和温度，应用固定修正并更新调试快照。
 *
 * @param frequency 实时传感器频率输出指针，成功时写入 Hz 浮点值。
 * @param density 实时传感器密度输出指针，成功时写入经过固定修正的 kg/m3 浮点值。
 * @param temp 用于返回传感器温度的输出参数，单位 ℃。
 * @return NO_ERROR 表示频率、密度和温度均已按当前传感器类型读取并更新输出；输出指针为空返回 SYSTEM_CALL_CONDITION_ERROR，其他值为模式准备、通信诊断或单项读取保留的具体错误。
 */
uint32_t Read_Density(float *frequency, float *density, float *temp) {
    float hz_45 = 0.0f;
    float hz_225 = 0.0f;
    uint32_t ret = NO_ERROR;

    if ((frequency == NULL) || (temp == NULL) || (density == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = DSM_Read_Frequency_Density_Temp(frequency, density, temp);
    } else if (g_deviceParams.sensorType == SAFE_SENSOR) {
        ret = SensorSafeAdapter_ReadDensity(frequency, density, temp);
    } else if (g_deviceParams.sensorType == LTD_SENSOR) {
        ret = MULTIPARAM_V3_Read_Temperature(temp);
        if (ret != NO_ERROR) {
            return Sensor_DiagnoseCommTimeout(ret, "读取LTD温度");
        }
        ret = MULTIPARAM_V3_Read_Density(density);
        if (ret != NO_ERROR) {
            return Sensor_DiagnoseCommTimeout(ret, "读取LTD密度");
        }
        ret = MULTIPARAM_V3_Read_DensityFrequency(frequency, &hz_45, &hz_225);
        if (ret != NO_ERROR) {
            return Sensor_DiagnoseCommTimeout(ret, "读取LTD频率");
        }
    } else if (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) {
        ret = MULTIPARAM_V4_MeasurementReadDensity(frequency, density, temp);
        if ((ret == NO_ERROR) &&
            ((*frequency < 0.0f) || (*frequency > 6600.0f) ||
             (*density < 0.0f) || (*density > 3000.0f) ||
             (*temp < -200.0f) || (*temp > 300.0f))) {
            ret = SENSOR_RESP_FORMAT_ERROR;
        }
    } else {
        ret = SENSOR_CAPABILITY_UNSUPPORTED;
    }

    ret = Sensor_DiagnoseCommTimeout(ret, "读取密度");
    if (ret == NO_ERROR) {
        float density_before = *density;
        float temp_before = *temp;

        Apply_Fixed_DensityTemp_Correction(density, temp);
        printf("原始密度: %.3f  修正后密度: %.3f\r\n", density_before, *density);
        printf("原始温度: %.3f ℃  修正后温度: %.3f ℃\r\n", temp_before, *temp);
        printf("频率: %.3f Hz\r\n", *frequency);
        g_measurement.debug_data.temperature = TEMP_TO_RAW(*temp);
        g_measurement.debug_data.frequency = *frequency;
    }
    return ret;
}


/**
 * @brief 读取水位传感器电容值。
 *
 * @param cap_out 用于返回水位通道电容值，单位 pF。
 * @return 返回整机错误码；NO_ERROR 表示电容值有效，PARAM_FEATURE_UNSUPPORTED 表示当前传感器不支持该通道，其他值表示通信或响应异常。
 */
uint32_t Sensor_ReadWaterCapacitance(float *cap_out)
{
    uint32_t ret = NO_ERROR;

    if (cap_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (!Sensor_SupportsWaterCapChannel()) {
        printf("当前传感器类型不支持读取水位电容\r\n");
        return PARAM_FEATURE_UNSUPPORTED;
    }

    if (g_deviceParams.sensorType == SAFE_SENSOR) {
        ret = SensorSafeAdapter_ReadWaterCapacitance(cap_out);
    } else if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = Read_Water_Capacitance(cap_out);
    } else if (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) {
        if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
            ret = Sensor_SetWaterEnabled(1U);
        }
        if (ret == NO_ERROR) {
            uint32_t start_tick = HAL_GetTick();

            do {
                ret = MULTIPARAM_V4_MeasurementReadWaterCapacitance(cap_out);
                if (ret != SENSOR_DATA_STALE) {
                    break;
                }
                if (HasEffectiveCommandSwitchRequest()) {
                    ret = STATE_SWITCH;
                    break;
                }
                if ((HAL_GetTick() - start_tick) >= MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) {
                    break;
                }
                HAL_Delay(MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS);
            } while (1);
        }
    } else {
        ret = SENSOR_STREAM_STATE_ERROR;
    }
    return Sensor_DiagnoseCommTimeout(ret, "读取水位电容");
}

/**
 * @brief 读取传感器陀螺仪姿态角。
 *
 * @param angle_x_deg 用于返回陀螺仪 X 轴角度的输出参数，单位度。
 * @param angle_y_deg 用于返回陀螺仪 Y 轴角度的输出参数，单位度。
 * @return 当前传感器不支持姿态通道时返回 PARAM_FEATURE_UNSUPPORTED；否则 NO_ERROR 表示双轴角度有效，通信超时会经蓝牙链路诊断映射，其他底层错误或
 *         STATE_SWITCH 原样返回。
 */
uint32_t Sensor_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg)
{
    uint32_t ret;

    if ((angle_x_deg == NULL) || (angle_y_deg == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (!Sensor_SupportsGyroChannel()) {
        printf("当前传感器类型不支持读取姿态角\r\n");
        return PARAM_FEATURE_UNSUPPORTED;
    }

    if (g_deviceParams.sensorType == SAFE_SENSOR) {
        ret = SensorSafeAdapter_ReadGyroAngle(angle_x_deg, angle_y_deg);
    } else if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = Read_Gyro_Angle(angle_x_deg, angle_y_deg);
    } else if (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) {
        ret = MULTIPARAM_V4_MeasurementReadGyro(angle_x_deg, angle_y_deg);
    } else {
        ret = SENSOR_STREAM_STATE_ERROR;
    }
    return Sensor_DiagnoseCommTimeout(ret, "读取陀螺仪");
}

/*
 * 函数用途：按目标状态控制多参数V4测水功能。
 * 调用场景：测水流程需要显式启停功能时。
 */
uint32_t Sensor_SetWaterEnabled(uint8_t enabled)
{
    uint8_t target_enabled = (enabled != 0U) ? 1U : 0U;

    if (g_deviceParams.sensorType != MULTIPARAM_V4_SENSOR) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    if (Sensor_V4ActiveFeatureMatches(MULTIPARAM_V4_FEATURE_WATER,
                                      target_enabled) != 0U) {
        return NO_ERROR;
    }
    return Sensor_RunV4FeatureCommand(MULTIPARAM_V4_EnsureWaterEnabled,
                                      target_enabled);
}

/*
 * 函数用途：按目标状态控制多参数V4磁零点功能。
 * 调用场景：零点流程需要显式启停磁零点功能时。
 */
uint32_t Sensor_SetMagneticZeroEnabled(uint8_t enabled)
{
    uint8_t target_enabled = (enabled != 0U) ? 1U : 0U;

    if (g_deviceParams.sensorType != MULTIPARAM_V4_SENSOR) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    if (Sensor_V4ActiveFeatureMatches(MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO,
                                      target_enabled) != 0U) {
        return NO_ERROR;
    }
    return Sensor_RunV4FeatureCommand(MULTIPARAM_V4_EnsureMagneticZeroEnabled,
                                      target_enabled);
}

/*
 * 函数用途：读取V4零点霍尔电压并等待功能开启后的首个有效值。
 * 调用场景：零点流程明确需要R03磁零点电压时。
 * 关键约束：自动进入密度模式并切换到M功能，读取结束后保持M开启。
 */
uint32_t Sensor_ReadMagneticZeroVoltage(float *voltage_v)
{
    uint32_t start_tick;
    uint32_t ret = NO_ERROR;

    if (voltage_v == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (g_deviceParams.sensorType != MULTIPARAM_V4_SENSOR) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    if (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_ACTIVE) {
        ret = Sensor_SetMagneticZeroEnabled(1U);
    }
    if (ret != NO_ERROR) {
        return Sensor_DiagnoseCommTimeout(ret, "开启零点霍尔");
    }

    start_tick = HAL_GetTick();
    do {
        ret = MULTIPARAM_V4_MeasurementReadMagneticZeroVoltage(voltage_v);
        if (ret != SENSOR_DATA_STALE) {
            break;
        }
        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        if ((HAL_GetTick() - start_tick) >= MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) {
            break;
        }
        HAL_Delay(MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS);
    } while (1);
    return Sensor_DiagnoseCommTimeout(ret, "读取零点霍尔");
}

/**
 * @brief 执行传感器通信与关键数据读取测试。
 * @return 当前实现完成传感器诊断打印后固定返回 NO_ERROR；通信与数据异常通过各子项日志报告，不通过本返回值上报。
 */
uint32_t Sensor_Test1(void) {
	float frequency = 5500.123f;
	float density = 800.5f;
	float temp = -180.52f;

	printf("密度: %.2f  频率: %.3f  温度: %.3f ℃\r\n", density, frequency, temp);


	return NO_ERROR;
}

/* ================== 读取部件参数：适配层（你按工程实际替换实现） ================== */

/**
 * @brief 通过统一适配入口读取当前编码器计数值。
 *
 * @return 返回按当前适配口径取反后的编码器累计计数。
 */
static int32_t Read_CurrentEncoderValue_Adapter(void)
{
    /* 你现在很多地方用 g_encoder_count；若你的“实际编码器”不同，替换这里 */
    return -(int32_t)g_encoder_count;
}

/**
 * @brief 读取测量调试快照中的当前传感器位置。
 *
 * 该适配层直接返回 g_measurement.debug_data.sensor_position，不重复执行编码器或尺带位置换算。
 *
 * @return 返回当前传感器位置快照，单位 0.1 mm。
 * @note 返回值沿用测量调试快照的位置口径，单位为 0.1 mm。
 */
static int32_t Calc_SensorPosition_Adapter(void)
{
    /* TODO: 替换为你工程中的“绝对位置mm/0.1mm”计算逻辑 */
    return g_measurement.debug_data.sensor_position;
}

/**
 * @brief 从调试数据读取当前尺带长度快照。
 * @return 尺带长度，单位 0.1mm。
 */
static int32_t Calc_CableLength_Adapter(void)
{
    /* TODO: 替换为你工程中的“尺带长度”计算逻辑（允许负值则这里不要 abs） */
    return g_measurement.debug_data.cable_length;
}

/**
 * @brief 通过统一适配入口读取电机步进、距离、速度和状态。
 *
 * @return 返回测量调试快照中的当前电机步进值。
 */
static int32_t Read_MotorStep_Adapter(void)
{
    /* TODO: 若你有 stpr_getPosition() / XACTUAL 等寄存器，替换这里 */
    return g_measurement.debug_data.motor_step;
}

/**
 * @brief 读取当前电机运动距离快照。
 * @return 电机运动距离，单位 0.1mm。
 */
static int32_t Read_MotorDistance_0p1mm_Adapter(void)
{
    /* TODO: 若你有当前运动距离累计，替换这里 */
    return g_measurement.debug_data.motor_distance;
}

/**
 * @brief 读取当前电机速度快照。
 * @return 电机速度，沿用调试数据中的单位口径。
 */
static uint32_t Read_MotorSpeed_Adapter(void)
{
    /* TODO: 若你有实际速度读取接口，替换这里 */
    return g_measurement.debug_data.motor_speed;
}

/**
 * @brief 读取当前电机运行状态快照。
 * @return 0 停止，1 上行，2 下行，其他值表示上层尚未归一化。
 */
static uint32_t Read_MotorState_Adapter(void)
{
    /* TODO: 统一为 0停 1上 2下；如果你已有 g_measurement.debug_data.motor_state 维护，就直接返回 */
    return g_measurement.debug_data.motor_state;
}

/**
 * @brief 读取扭力模块当前实时重量快照。
 *
 * @return 返回扭力模块当前实时重量原始值。
 * @note 该适配层直接读取 weight_parament.current_weight，不触发新的扭力通信事务。
 */
static uint32_t Read_CurrentWeight_Adapter(void)
{
    return (uint32_t)weight_parament.current_weight;
}

/**
 * @brief 按周期刷新蓝牙 RSSI 快照。
 *
 * @details 调用场景：读取部件参数循环末尾调用，用于刷新显示缓存。
 * @note 关键约束：该操作会短时占用 UART6 进入 CH9141 AT 模式；命令切换时必须把 STATE_SWITCH 传给上层，不能继续抢占传感器透传链路。
 *
 * @param force_update 更新。
 * @return NO_ERROR 表示本轮无需刷新、刷新周期未到、RSSI 快照已成功更新或仅 RSSI 数值无效但 UART6 已恢复透明传输；命令切换返回 STATE_SWITCH，其他值表示 AT/UART6 交接未安全完成。
 */
static uint32_t Sensor_UpdateWirelessRssiForPartParams(uint8_t force_update)
{
    static uint32_t last_update_tick = 0U;
    uint32_t now_tick = HAL_GetTick();

    if ((g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) &&
        (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_ACTIVE)) {
        /* 主动上报期间不进入CH9141 AT模式，避免与64字节常驻接收争用UART6。 */
        return NO_ERROR;
    }
    if ((force_update == 0U) &&
        ((now_tick - last_update_tick) < READ_PART_PARAMS_RSSI_REFRESH_INTERVAL_MS)) {
        return NO_ERROR;
    }

    if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    last_update_tick = now_tick;
    return WirelessPairing_UpdateConnectionStatusSnapshot();
}
/*
 * 函数用途：在V4交互部件参数读取失败时打印最后一组原始收发包。
 * 调用场景：姿态角、密度组合量或水位电容读取返回错误后、错误码上抛前。
 * 关键约束：主动上报路径没有对应交互事务，禁止打印可能属于旧事务的缓存包。
 */
static uint32_t Sensor_ReportV4PartParamsFailure(const char *operation,
                                                 uint32_t result)
{
    if ((result != NO_ERROR) &&
        (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) &&
        (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE)) {
        MULTIPARAM_V4_PrintLastTransactionPackets(operation, result);
    }
    return result;
}

/* ================== CMD：读取部件参数 ================== */
/**
 * @brief 读取部件参数的共用实现。
 *
 * 命令入口调用时会进入读取状态；自动恢复检查调用时不改命令状态，
 * 但仍会先检查电机驱动健康，避免 24V 断电后继续报告无效位置。
 *
 * @param update_command_state 更新命令状态。
 * @return NO_ERROR 表示部件参数循环正常结束；命令切换返回 STATE_SWITCH，传感器模式准备、编号、频率、温度、密度或 RSSI 更新失败时返回对应具体错误。
 */
static uint32_t Sensor_ReadPartParamsInternal(uint8_t update_command_state)
{
    uint32_t ret = NO_ERROR;
    uint32_t voltage_ret = NO_ERROR;
    uint32_t density_analysis_ret = NO_ERROR;
    uint8_t is_ltd_sensor = (g_deviceParams.sensorType == LTD_SENSOR) ? 1U : 0U;

    float ax = 0.0f, ay = 0.0f;
    float freq = 0.0f, dens = 0.0f, temp = 0.0f;
    float cap = 0.0f;
    float magnetic_zero_voltage = 0.0f;
    float supply_voltage = 0.0f;
    float period_225 = 0.0f, period_45 = 0.0f;
    float dynamic_viscosity = 0.0f;

    if (update_command_state) {
        ret = (uint32_t)MeasureStart();
        if (ret != NO_ERROR) {
            return ret;
        }
        g_measurement.device_status.device_state = STATE_READPARAMETERING;
    } else if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    /* ---------- 1) 位置类：编码器/位置/尺带长度/步进/距离 ---------- */
    ret = MotorCtrl_CheckDriverGstat();
    if (ret != NO_ERROR) {
        return ret;
    }

    MotorCtrl_RefreshDebugDrumState();
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (!MotorCtrl_IsPositionSourceMotor()) {
        /* 业务停机只消费已经连续确认的锁存错误；单帧瞬态仅保留给ENC?诊断。 */
        ret = AS5145_GetLatchedError();
        if (ret != NO_ERROR) {
            return ret;
        }
    }

    g_measurement.debug_data.current_encoder_value = Read_CurrentEncoderValue_Adapter();
    g_measurement.debug_data.sensor_position       = Calc_SensorPosition_Adapter();
    g_measurement.debug_data.cable_length          = Calc_CableLength_Adapter();
    g_measurement.debug_data.motor_step            = Read_MotorStep_Adapter();
    g_measurement.debug_data.motor_distance        = Read_MotorDistance_0p1mm_Adapter();

    /* ---------- 2) 电机状态类：速度/状态 ---------- */
    g_measurement.debug_data.motor_speed = Read_MotorSpeed_Adapter();
    g_measurement.debug_data.motor_state = Read_MotorState_Adapter();

    /* ---------- 3) 扭力类 ---------- */
    ret = Weight_CheckOwnCommunicationTimeout();

    if (ret != NO_ERROR) {
        return ret;
    }

    g_measurement.debug_data.current_weight = Read_CurrentWeight_Adapter();

    /* ---------- 4) 姿态角（陀螺仪） ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (Sensor_SupportsGyroChannel()) {
        ret = Sensor_ReadGyroAngle(&ax, &ay);
        if ((g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) &&
            (ret == SENSOR_DATA_STALE)) {
            /* V4姿态角尚未生成或当前无效时，仅跳过诊断项，不覆盖已有快照。 */
            (void)Sensor_ReportV4PartParamsFailure("读取姿态角", ret);
            printf("读取部件参数\tV4姿态角当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if (ret != NO_ERROR) {
            return Sensor_ReportV4PartParamsFailure("读取姿态角", ret);
        } else {
            /* 你 Read_Gyro_Angle() 里已经写了 debug_data.angle_x/y，这里再确保一遍 */
            g_measurement.debug_data.angle_x = (int32_t)(ax * 100.0f);
            g_measurement.debug_data.angle_y = (int32_t)(ay * 100.0f);
        }
    } else {
        g_measurement.debug_data.angle_x = 0;
        g_measurement.debug_data.angle_y = 0;
        printf("读取部件参数\t当前传感器类型不支持姿态角读取，已跳过\r\n");
    }

    /* ---------- 5) 密度/温度/频率 ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (is_ltd_sensor) {
        ret = Sensor_PrepareLtdDensityModeForPartParams();
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
    }

    if (ret == NO_ERROR) {
        ret = Read_Density(&freq, &dens, &temp);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
    }

    if (ret != NO_ERROR) {
        if (is_ltd_sensor) {
            g_measurement.debug_data.frequency = 0U;
            printf("读取部件参数\tLTD密度/频率暂未读到，按部分成功处理，其他部件参数保留有效。错误码=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if ((g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) &&
                   (ret == SENSOR_DATA_STALE)) {
            /* V4测量值尚未生成或当前无效时，仅跳过诊断项，避免维护读取进入全局错误态。 */
            (void)Sensor_ReportV4PartParamsFailure("读取密度/温度/频率", ret);
            printf("读取部件参数\tV4密度/温度/频率当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else {
            return Sensor_ReportV4PartParamsFailure("读取密度/温度/频率", ret);
        }
    } else {
        /* 你 Read_Density() 里已写 debug_data.temperature/frequency，这里保证一致 */
        g_measurement.debug_data.frequency    = (uint32_t)freq;   /* 若你要保留小数频率，可改成 ×100 或另存 */
        /* debug_data.temperature 在 Read_Density 内已 TEMP_TO_RAW，避免重复计算 */
    }

    /* ---------- 6) DSM 传感器电压、密度分析参数 ---------- */
    if (g_deviceParams.sensorType == DSM_SENSOR) {
        /* CK/CM 是扩展诊断数据；命令切换仍立即退出，读取失败不阻断核心参数和故障恢复。 */
        voltage_ret = Read_Sensor_Voltage(&supply_voltage);
        voltage_ret = Sensor_DiagnoseCommTimeout(voltage_ret, "读取DSM传感器电压");
        if (voltage_ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        if (voltage_ret != NO_ERROR) {
            printf("读取部件参数\tDSM传感器电压未更新，保留核心参数结果。错误码=0x%08lX\r\n",
                   (unsigned long)voltage_ret);
        }

        density_analysis_ret = DSM_ReadDensityAnalysis(&period_225,
                                                       &period_45,
                                                       &dynamic_viscosity);
        density_analysis_ret = Sensor_DiagnoseCommTimeout(density_analysis_ret,
                                                          "读取DSM密度分析参数");
        if (density_analysis_ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
        if (density_analysis_ret != NO_ERROR) {
            printf("读取部件参数\tDSM密度分析参数未更新，保留核心参数结果。错误码=0x%08lX\r\n",
                   (unsigned long)density_analysis_ret);
        }

        if (voltage_ret == NO_ERROR) {
            printf("DSM扩展参数 | 传感器电压=%.4fV\r\n", supply_voltage);
        }
        if (density_analysis_ret == NO_ERROR) {
            printf("DSM扩展参数 | 22.5度周期平方=%.0f | 45度周期平方=%.0f | 动态黏度=%.1f\r\n",
                   period_225,
                   period_45,
                   dynamic_viscosity);
        }
    }
    /* ---------- 7) V4零点霍尔 ---------- */
    if (g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) {
        ret = Sensor_ReadMagneticZeroVoltage(&magnetic_zero_voltage);
        if (ret == SENSOR_DATA_STALE) {
            (void)Sensor_ReportV4PartParamsFailure("读取零点霍尔", ret);
            printf("读取部件参数\tV4零点霍尔当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if (ret != NO_ERROR) {
            return Sensor_ReportV4PartParamsFailure("读取零点霍尔", ret);
        } else {
            printf("V4零点霍尔 | 磁零点电压=%.6f V\r\n",
                   (double)magnetic_zero_voltage);
        }
    }

    /* ---------- 8) 水位电容 ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (Sensor_SupportsWaterCapChannel()) {
        ret = Sensor_ReadWaterCapacitance(&cap);
        if ((g_deviceParams.sensorType == MULTIPARAM_V4_SENSOR) &&
            (ret == SENSOR_DATA_STALE)) {
            /* V4水位电容尚未生成或当前无效时，仅跳过诊断项，不覆盖已有快照。 */
            (void)Sensor_ReportV4PartParamsFailure("读取水位电容", ret);
            printf("读取部件参数\tV4水位电容当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if (ret != NO_ERROR) {
            return Sensor_ReportV4PartParamsFailure("读取水位电容", ret);
        } else {
            /* 水位电容快照按 0.1pF 保存，避免和历史“电压”语义混淆
               若传感器返回单位变化，需要同步调整字段名和显示小数位。 */
            g_measurement.debug_data.water_capacitance_x10 = (uint32_t)(cap * 10.0f); /* 0.1pF */
        }
    } else {
        g_measurement.debug_data.water_capacitance_x10 = 0;
        printf("读取部件参数\t当前传感器类型不支持水位电容读取，已跳过\r\n");
    }

    /* ---------- 9) 蓝牙连接 RSSI ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }
    ret = Sensor_UpdateWirelessRssiForPartParams(update_command_state);
    if (ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    if (ret != NO_ERROR) {
        printf("读取部件参数\t蓝牙状态刷新失败，UART6透明传输未确认，停止后续流程。错误码=0x%08lX\r\n",
               (unsigned long)ret);
        return ret;
    }
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    /* ---------- 10) 预留接口：后续新增部件参数统一挂这里 ---------- */
    /* TODO:
       - 读非DSM传感器电源电压/驱动电压
       - 读TMC5130错误寄存器(GSTAT/DRV_STATUS)
       - 读非DSM传感器版本号
       - 读温度2/环境温度
    */

    /* ---------- 10) 打印汇总：正常信息集中一行 ---------- */
    printf("读取部件参数完成 | 编码值=%ld 位置=%ld 缆长=%ld 步数=%ld 距离=%ld(0.1mm) "
           "| freq=%lu temp=%lu | cap=%lu | w=%lu | ang_x=%ld ang_y=%ld(0.01deg) | mspd=%lu mstate=%lu | rssi_valid=%lu rssi=%ld\r\n",
           (long)g_measurement.debug_data.current_encoder_value,
           (long)g_measurement.debug_data.sensor_position,
           (long)g_measurement.debug_data.cable_length,
           (long)g_measurement.debug_data.motor_step,
           (long)g_measurement.debug_data.motor_distance,
           (unsigned long)g_measurement.debug_data.frequency,
           (unsigned long)g_measurement.debug_data.temperature,
           (unsigned long)g_measurement.debug_data.water_capacitance_x10,
           (unsigned long)g_measurement.debug_data.current_weight,
           (long)g_measurement.debug_data.angle_x,
           (long)g_measurement.debug_data.angle_y,
           (unsigned long)g_measurement.debug_data.motor_speed,
           (unsigned long)g_measurement.debug_data.motor_state,
           (unsigned long)g_measurement.wireless_pairing_status.rssi_valid,
           (long)g_measurement.wireless_pairing_status.rssi);

    if (update_command_state) {
        g_measurement.device_status.device_state = STATE_READPARAMETEROVER;
    }

    return NO_ERROR;
}

/**
 * @brief 逐项读取并核对传感器部件参数完整性。
 * @return 返回整机错误码；NO_ERROR 表示全部传感器部件参数均已读取并通过核对，其他值定位首个失败项。
 */
uint32_t Sensor_CheckAllPartParams(void)
{
    return Sensor_ReadPartParamsInternal(0U);
}

/**
 * @brief 执行部件参数读取命令并更新测量快照。
 */
void CMD_ReadPartParams(void)
{
    uint32_t ret = Sensor_ReadPartParamsInternal(1U);

    SET_ERROR(ret);

    /* 读取部件参数完成态是长驻刷新态：不回到主循环轮询，直接在命令内按周期刷新；
       等待过程保留命令切换检查，避免新命令被 1s 刷新周期阻塞。 */
    while (g_measurement.device_status.device_state == STATE_READPARAMETEROVER) {
        ret = AbortableDelay_CommandSwitch(READ_PART_PARAMS_REFRESH_INTERVAL_MS, 100U);
        if (ret == STATE_SWITCH) {
            /* 命令切换不是传感器故障，记录切换结果后退出本命令，让主循环执行新命令。 */
            SET_ERROR(ret);
            break;
        }

        ret = Sensor_CheckAllPartParams();
        SET_ERROR(ret);
    }
}
