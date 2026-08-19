/*
 * 模块职责：按固定顺序通过无线滑环识别DSM、多参数V3或DM4唯一物理传感器。
 * 识别顺序：监听V4主动帧、检查蓝牙链路、广播读取V4 R01，再尝试V3和DSM协议。
 * DM4约束：Safe不作为独立设备参与上电识别；DM4识别成功后持久化物理类型14。
 */
#include "sensor_service.h"

#include "Protocols/Dsm/dsm_sensor_communication.h"
#include "error_log.h"
#include "Protocols/MultiparamV3/multiparam_v3_communication.h"
#include "Protocols/Dm4/V4/multiparam_v4_communication.h"
#include "Protocols/Dm4/V4/multiparam_v4_measurement.h"
#include "sensor_runtime.h"
#include "Wireless/wireless_pairing.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
/* 上电识别阶段快速监听V4主动帧的窗口，单位ms。 */
#define SENSOR_V4_ACTIVE_FAST_PROBE_MS 750U
/*
 * 函数用途：从各候选协议结果中选择最能解释识别失败的最终错误码。
 * 调用场景：V4主动前后、R01版本和DSM CN探测全部未成功之后。
 * 关键约束：具体协议错误优先于纯超时；全部超时时返回通信超时，不把未命中改成成功。
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


/*
 * 函数用途：把传感器识别最终错误发布到测量全局状态。
 * 调用场景：识别流程已经确认失败并准备退出时。
 * 关键约束：NO_ERROR和STATE_SWITCH都不是故障，不写全局错误码，避免命令切换被误报。
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
    SensorRuntime_SetDetectionResult(err);
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
/*
 * 函数用途：通过CH9141详细状态查询确认无线主从链路可用于传感器探测。
 * 调用场景：第一次V4主动监听未命中之后、同步协议探测之前。
 * 关键约束：会切入AT模式并阻塞等待，只能在线程态调用；错误保留主机、从机或命令切换语义。
 */
static uint32_t Sensor_ProbeWirelessLink(WirelessConnectionStatus *status)
{
    return WirelessPairing_CheckBluetoothLinkDetailed(status);
}

/*
 * 函数用途：打印最近一次无线链路检查得到的从机MAC和RSSI快照。
 * 调用场景：自动识别确认蓝牙链路正常之后。
 * 关键约束：只读快照、不再次访问UART6；RSSI无效不等于链路断开，分别打印有效状态。
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

/*
 * 函数用途：从DSM CN文本标识中提取可持久化的十进制传感器编号。
 * 调用场景：DSM编号命令成功返回形如N2009924H的字符串之后。
 * 关键约束：至少包含一个数字且结果非0；累加溢出时返回格式异常，不允许uint32回卷。
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

/*
 * 函数用途：通过V3 R22静默读取编号确认多参数V3传感器。
 * 调用场景：公共R01已经明确返回3.0协议版本之后。
 * 关键约束：候选未命中不打印最终故障；只有读取成功才写调用方输出，V4路径禁止调用。
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

/*
 * 函数用途：把DSM RAM会话版本分类转换为诊断名称。
 * 调用场景：DSM兼容探测输出CPU1/CPU0版本上下文时。
 * 关键约束：只返回静态文字，不参与协议选择、编号解析或持久化判断。
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

/*
 * 函数用途：读取DSM版本上下文和CN振动管编号以确认DSM一代传感器。
 * 调用场景：V4及V3候选均未识别成功后的最后兼容探测。
 * 关键约束：Cv失败仍继续CN；编号成功后必须切密度模式，任一步失败都不得提交身份。
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
    MULTIPARAM_V4_SetIdentificationProbeMode(1U);
    result = MULTIPARAM_V4_StartActiveReceive();
    if (result != NO_ERROR) {
        MULTIPARAM_V4_PrintActiveProbePacket(operation, result);
        MULTIPARAM_V4_Deinit();
        MULTIPARAM_V4_SetIdentificationProbeMode(0U);
        return result;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (HasEffectiveCommandSwitchRequest()) {
            MULTIPARAM_V4_PrintActiveProbePacket(operation, STATE_SWITCH);
            MULTIPARAM_V4_Deinit();
            MULTIPARAM_V4_SetIdentificationProbeMode(0U);
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
            MULTIPARAM_V4_SetIdentificationProbeMode(0U);
            return NO_ERROR;
        }
        HAL_Delay(1U);
    }

    MULTIPARAM_V4_PrintActiveProbePacket(operation, SENSOR_DEVICE_COMM_TIMEOUT);
    MULTIPARAM_V4_Deinit();
    MULTIPARAM_V4_SetIdentificationProbeMode(0U);
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

    /* R67通信成功即确认DM4身份；sensor_id为0表示未初始化，仍是合法传感器编号。 */
    SensorRuntime_CommitIdentity(DM4_SENSOR, sensor_id);
    printf("识别成功：多参数V4传感器 | 通信=%s | 地址=%u | 编号=%lu\r\n",
           (communication != NULL) ? communication : "未知",
           (unsigned int)sensor_address,
           (unsigned long)sensor_id);
    printf("====================================\r\n");
    return NO_ERROR;
}
/*
 * 函数用途：按固定顺序自动识别DM4-V4、多参数V3和DSM物理传感器。
 * 调用场景：CPU2上电初始化和维护综合通信测试重新识别。
 * 关键约束：顺序固定为V4主动前、蓝牙、V4主动后、R01分流、V3 R22、DSM CN；Safe不参与上电识别。
 */
uint32_t SensorService_Detect(void) {
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
    SensorRuntime_BeginDetection();

    printf("========== 传感器识别开始 ==========\r\n");
    printf("[1/6] 蓝牙查询前监听多参数V4主动帧\r\n");
    active_v4_before_link_ret = Sensor_ProbeV4Active("主动帧-蓝牙查询前",
                                                     &sensor_address,
                                                     SENSOR_V4_ACTIVE_FAST_PROBE_MS);
    if (active_v4_before_link_ret == NO_ERROR) {
        return Sensor_CompleteV4Detection("主动上报", sensor_address);
    }
    if (active_v4_before_link_ret == STATE_SWITCH) {
        SensorRuntime_SetDetectionResult(STATE_SWITCH);
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
        SensorRuntime_SetDetectionResult(STATE_SWITCH);
        return STATE_SWITCH;
    }

    printf("[4/6] 读取R01区分多参数协议V3.0和V4.0\r\n");
    version_ret = Sensor_ProbeMultiparamVersion(&protocol_version);
    /* R01只在明确返回V4协议版本时结束探测；不再用其它寄存器猜测协议代次。 */
    if (version_ret == NO_ERROR) {
        return Sensor_CompleteV4Detection("交互", 0U);
    }
    if (version_ret == STATE_SWITCH) {
        SensorRuntime_SetDetectionResult(STATE_SWITCH);
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
            SensorRuntime_CommitIdentity(LTD_SENSOR, sensor_id);
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
        SensorRuntime_CommitIdentity(DSM_SENSOR, sensor_id);
        printf("识别成功：DSM传感器 | 编号=%lu | 密度模式握手成功\r\n",
               (unsigned long)sensor_id);
        printf("====================================\r\n");
        return NO_ERROR;
    }
    if (dsm_ret == STATE_SWITCH) {
        SensorRuntime_SetDetectionResult(STATE_SWITCH);
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
