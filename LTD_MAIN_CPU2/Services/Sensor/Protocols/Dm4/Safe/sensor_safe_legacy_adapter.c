/*
 * 模块职责：把Safe服务适配为现有SensorDriverOps使用的密度、液位、水位和姿态接口。
 * 业务边界：DM4上电仍按V4识别；只有显式进入Safe会话后，本适配器才参与业务分派。
 * 错误边界：Safe结果统一映射为现有CPU2错误码，同时保留会话和传输诊断上下文。
 */
#include "sensor_safe_legacy_adapter.h"

#include <string.h>

#include "abortable_delay.h"
#include "error_log.h"
#include "measure.h"
#include "sensor_safe_identity_platform.h"
#include "sensor_safe_service.h"
#include "sensor_safe_transport_uart6.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"

/* 旧传感器接口适配层持有的安全协议服务上下文。 */
static SensorSafeServiceContext s_safe_service;

/**
 * @brief 查询整机命令切换请求，供阻塞式 UART 等待和稳定等待及时退出。
 *
 * @return 1 表示整机存在有效命令切换请求，阻塞式 UART 或稳定等待应立即退出；0 表示当前没有命令切换请求。
 */
static uint8_t SensorSafeAdapter_ShouldAbort(void)
{
    return (uint8_t)(HasEffectiveCommandSwitchRequest() ? 1U : 0U);
}

/**
 * @brief 向安全协议层提供 HAL 单调毫秒节拍。
 *
 * @return 返回 HAL 单调毫秒节拍值，允许 32 位自然回绕。
 */
static uint32_t SensorSafeAdapter_NowMs(void)
{
    return HAL_GetTick();
}

/**
 * @brief 把本地协议校验结果映射为现场可区分的故障原因。
 *
 * @param result 安全协议本地帧、会话、序号、能力或数据一致性校验结果。
 * @return 返回与本地安全协议失败原因对应的整机错误码，覆盖 CRC、版本、能力、地址、会话、身份、序号、重放、参数 CRC、配置代次、流状态和数据时效；未知结果返回
 *         SENSOR_RESP_FORMAT_ERROR。
 */
static uint32_t SensorSafeAdapter_MapProtocolResult(SensorSafeResult result)
{
    switch (result) {
    case SENSOR_SAFE_BAD_CRC:
        return SENSOR_BCC_ERROR;
    case SENSOR_SAFE_BAD_VERSION:
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    case SENSOR_SAFE_BAD_CAPABILITY:
        return SENSOR_CAPABILITY_UNSUPPORTED;
    case SENSOR_SAFE_BAD_ADDRESS:
        return SENSOR_ADDRESS_MISMATCH;
    case SENSOR_SAFE_BAD_SESSION:
        return SENSOR_SESSION_INVALID;
    case SENSOR_SAFE_BAD_SENSOR:
        return SENSOR_IDENTITY_MISMATCH;
    case SENSOR_SAFE_BAD_SEQUENCE:
        return SENSOR_SEQUENCE_ERROR;
    case SENSOR_SAFE_NEEDS_HELLO:
        return SENSOR_HANDSHAKE_REQUIRED;
    case SENSOR_SAFE_REPLAY_DETECTED:
        return SENSOR_REPLAY_DETECTED;
    case SENSOR_SAFE_BAD_PARAM_CRC:
        return SENSOR_PARAM_CRC_ERROR;
    case SENSOR_SAFE_BAD_CONFIG_EPOCH:
        return SENSOR_CONFIG_EPOCH_MISMATCH;
    case SENSOR_SAFE_BAD_SAMPLE_COUNTER:
        return SENSOR_SAMPLE_COUNTER_ERROR;
    case SENSOR_SAFE_BAD_STREAM_STATE:
        return SENSOR_STREAM_STATE_ERROR;
    case SENSOR_SAFE_STREAM_STOPPED:
        return SENSOR_STREAM_STOPPED;
    case SENSOR_SAFE_DATA_STALE:
        return SENSOR_DATA_STALE;
    case SENSOR_SAFE_MODE_MISMATCH:
        return SENSOR_MODE_MISMATCH;
    case SENSOR_SAFE_TRANSACTION_PENDING:
        return SENSOR_TRANSACTION_PENDING;
    default:
        return SENSOR_RESP_FORMAT_ERROR;
    }
}

/**
 * @brief 把传感器明确返回的在线结果码映射为对应硬件、状态或配置故障。
 *
 * @param result 传感器在安全协议响应中明确返回的 16 位在线结果码。
 * @param invalid_data_error 远端返回 SENSOR_SAFE_WIRE_DATA_INVALID 时应映射的具体业务数据错误码。
 * @return 返回远端在线结果对应的整机错误码；DATA_INVALID 使用 invalid_data_error，已知版本、命令、参数、会话、状态及硬件错误逐项映射，未知结果返回
 *         SENSOR_REMOTE_INTERNAL_ERROR。
 */
static uint32_t SensorSafeAdapter_MapWireResult(uint16_t result, uint32_t invalid_data_error)
{
    switch ((SensorSafeWireResult)result) {
    case SENSOR_SAFE_WIRE_UNSUPPORTED_VERSION:
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    case SENSOR_SAFE_WIRE_UNSUPPORTED_COMMAND:
        return SENSOR_COMMAND_UNSUPPORTED;
    case SENSOR_SAFE_WIRE_BAD_ARGUMENT:
        return SENSOR_ARGUMENT_REJECTED;
    case SENSOR_SAFE_WIRE_BAD_LENGTH:
        return SENSOR_RESP_FORMAT_ERROR;
    case SENSOR_SAFE_WIRE_BAD_CRC:
        return SENSOR_BCC_ERROR;
    case SENSOR_SAFE_WIRE_BAD_SEQUENCE:
        return SENSOR_SEQUENCE_ERROR;
    case SENSOR_SAFE_WIRE_BAD_SESSION:
        return SENSOR_SESSION_INVALID;
    case SENSOR_SAFE_WIRE_BAD_ADDRESS:
        return SENSOR_ADDRESS_MISMATCH;
    case SENSOR_SAFE_WIRE_MODE_NOT_READY:
        return SENSOR_MODE_NOT_READY;
    case SENSOR_SAFE_WIRE_MODE_NOT_ALLOWED:
        return SENSOR_MODE_NOT_ALLOWED;
    case SENSOR_SAFE_WIRE_DEVICE_BUSY:
        return SENSOR_DEVICE_BUSY;
    case SENSOR_SAFE_WIRE_DATA_INVALID:
        return invalid_data_error;
    case SENSOR_SAFE_WIRE_DATA_STALE:
        return SENSOR_DATA_STALE;
    case SENSOR_SAFE_WIRE_CONFIG_MISMATCH:
        return SENSOR_CONFIG_EPOCH_MISMATCH;
    case SENSOR_SAFE_WIRE_PARAM_CRC_ERROR:
        return SENSOR_PARAM_CRC_ERROR;
    case SENSOR_SAFE_WIRE_SELF_TEST_FAILED:
        return SENSOR_SELF_TEST_FAILED;
    case SENSOR_SAFE_WIRE_POWER_ERROR:
        return SENSOR_POWER_SUPPLY_ERROR;
    case SENSOR_SAFE_WIRE_TEMPERATURE_ERROR:
        return SENSOR_TEMPERATURE_RANGE_ERROR;
    case SENSOR_SAFE_WIRE_STREAM_NOT_ACTIVE:
        return SENSOR_STREAM_NOT_ACTIVE;
    case SENSOR_SAFE_WIRE_STREAM_ALREADY_ACTIVE:
        return SENSOR_STREAM_ALREADY_ACTIVE;
    case SENSOR_SAFE_WIRE_STREAM_EXIT_FAILED:
        return SENSOR_STREAM_EXIT_FAILED;
    default:
        return SENSOR_REMOTE_INTERNAL_ERROR;
    }
}

/**
 * @brief 把新协议的分层错误映射到整机错误码，不在适配层直接置全局故障。
 *
 * @param result 安全服务层返回的 SensorSafeServiceResult；函数映射为现有整机错误码。
 * @param invalid_data_error 上层为当前数据无效场景指定的整机错误码。
 * @return NO_ERROR 表示安全服务操作成功；参数或服务状态无效返回 SYSTEM_CALL_CONDITION_ERROR，传输、线格式、协议、数据陈旧、能力缺失、身份、会话及远端故障分别映射为对应整机错误码，无法细分的数据错误使用 invalid_data_error。
 */
static uint32_t SensorSafeAdapter_MapResult(SensorSafeServiceResult result,
                                             uint32_t invalid_data_error)
{
    switch (result) {
    case SENSOR_SAFE_SERVICE_OK:
        return NO_ERROR;
    case SENSOR_SAFE_SERVICE_INVALID_ARGUMENT:
        return SYSTEM_CALL_CONDITION_ERROR;
    case SENSOR_SAFE_SERVICE_TRANSPORT_ERROR:
        if (s_safe_service.client.last_transport_result == SENSOR_SAFE_TRANSPORT_TIMEOUT) {
            return SENSOR_DEVICE_COMM_TIMEOUT;
        }
        return COMM_UART_TRANSFER_ERROR;
    case SENSOR_SAFE_SERVICE_PROTOCOL_ERROR:
        return SensorSafeAdapter_MapProtocolResult(s_safe_service.client.last_protocol_result);
    case SENSOR_SAFE_SERVICE_REMOTE_ERROR:
        return SensorSafeAdapter_MapWireResult(s_safe_service.client.last_wire_result,
                                               invalid_data_error);
    case SENSOR_SAFE_SERVICE_DATA_INVALID:
        return invalid_data_error;
    case SENSOR_SAFE_SERVICE_DATA_STALE:
        return SENSOR_DATA_STALE;
    case SENSOR_SAFE_SERVICE_UNSUPPORTED:
        return SENSOR_CAPABILITY_UNSUPPORTED;
    case SENSOR_SAFE_SERVICE_BUFFER_TOO_SMALL:
        return SYSTEM_BUFFER_CAPACITY_ERROR;
    case SENSOR_SAFE_SERVICE_IDENTITY_ERROR:
        return SENSOR_IDENTITY_MISMATCH;
    case SENSOR_SAFE_SERVICE_SESSION_ERROR:
        return SENSOR_SESSION_INVALID;
    default:
        return SENSOR_REMOTE_INTERNAL_ERROR;
    }
}

/**
 * @brief 把触发重新 HELLO 的原始 client 结果映射为可检索的整机错误码。
 *
 * @return 传输恢复触发返回通信超时或 UART 传输错误，协议和远端触发分别调用对应映射；没有可识别触发原因时返回 SENSOR_SESSION_INVALID。
 */
static uint32_t SensorSafeAdapter_MapRecoveryError(void)
{
    if (s_safe_service.last_recovery_trigger == SENSOR_SAFE_CLIENT_TRANSPORT_ERROR) {
        return (s_safe_service.last_recovery_transport_result == SENSOR_SAFE_TRANSPORT_TIMEOUT)
                   ? SENSOR_DEVICE_COMM_TIMEOUT
                   : COMM_UART_TRANSFER_ERROR;
    }
    if (s_safe_service.last_recovery_trigger == SENSOR_SAFE_CLIENT_PROTOCOL_ERROR) {
        return SensorSafeAdapter_MapProtocolResult(s_safe_service.last_recovery_protocol_result);
    }
    if (s_safe_service.last_recovery_trigger == SENSOR_SAFE_CLIENT_REMOTE_ERROR) {
        return SensorSafeAdapter_MapWireResult(s_safe_service.last_recovery_wire_result,
                                               SENSOR_REMOTE_INTERNAL_ERROR);
    }
    return SENSOR_SESSION_INVALID;
}

/**
 * @brief 适配层在业务调用返回后统一记录恢复尝试和最终成功，不重复承担最终故障出口。
 *
 * @param attempts_before 本次操作开始前累计的安全协议恢复尝试次数。
 * @param recoveries_before 本次操作开始前累计完成的安全协议恢复次数。
 */
static void SensorSafeAdapter_LogRecovery(uint32_t attempts_before,
                                          uint32_t recoveries_before)
{
    uint32_t error_code;

    if (s_safe_service.recovery_attempt_count == attempts_before) {
        return;
    }
    error_code = SensorSafeAdapter_MapRecoveryError();
    ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                   ERROR_LOG_OP_COMM_DIAG,
                   ErrorLog_GetReasonByCode(error_code),
                   1U,
                   1U,
                   error_code);
    if (s_safe_service.recovery_count != recoveries_before) {
        ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                         ERROR_LOG_OP_COMM_DIAG,
                         ErrorLog_GetReasonByCode(error_code),
                         1U,
                         1U);
    }
}

/**
 * @brief 同一上电周期只初始化一次身份计数和 client，避免重复探测消耗启动计数。
 *
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
static SensorSafeServiceResult SensorSafeAdapter_EnsureInitialized(void)
{
    SensorSafeIdentityOps identity_ops;
    SensorSafeClientTransportOps transport_ops;
    SensorSafeTransportUart6Config uart_config;

    if (s_safe_service.initialized != 0U) {
        return SENSOR_SAFE_SERVICE_OK;
    }
    (void)memset(&uart_config, 0, sizeof(uart_config));
    uart_config.should_abort = SensorSafeAdapter_ShouldAbort;
    uart_config.transmit_timeout_ms = SENSOR_SAFE_CLIENT_DEFAULT_TIMEOUT_MS;
    SensorSafeTransportUart6_Init(&uart_config);

    SensorSafeIdentityPlatform_GetOps(&identity_ops);
    (void)memset(&transport_ops, 0, sizeof(transport_ops));
    transport_ops.exchange = SensorSafeTransportUart6_Exchange;
    transport_ops.receive = SensorSafeTransportUart6_ReceiveFrame;
    transport_ops.now_ms = SensorSafeAdapter_NowMs;
    return SensorSafeService_Init(&s_safe_service, &identity_ops, &transport_ops);
}

/**
 * @brief 探测并建立安全传感器会话，输出传感器唯一标识。
 *
 * @details 调用场景：传感器自动识别在蓝牙链路失败后、旧协议探测前调用。
 * @note 关键约束：任一失败均撤销活动态并释放 UART6，保证多参数传感器通信协议 V3.0（当前LTD使用）与 DSM 可继续回退。
 *
 * @param sensor_id 用于返回探测到的传感器编号。
 * @return NO_ERROR 表示 HELLO、状态读取和传感器编号输出均完成；sensor_id 为空返回 SYSTEM_CALL_CONDITION_ERROR，其他值由安全服务结果映射为具体通信、协议、身份、会话或远端错误。
 */
uint32_t SensorSafeAdapter_Probe(uint32_t *sensor_id)
{
    SensorSafeServiceResult result;

    if (sensor_id == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = SensorSafeAdapter_EnsureInitialized();
    if (result != SENSOR_SAFE_SERVICE_OK) {
        SensorSafeAdapter_Deactivate();
        return SensorSafeAdapter_MapResult(result, SENSOR_REMOTE_INTERNAL_ERROR);
    }
    SensorSafeTransportUart6_Abort();
    result = SensorSafeService_Probe(&s_safe_service, sensor_id);
    if (result != SENSOR_SAFE_SERVICE_OK) {
        SensorSafeAdapter_Deactivate();
    }
    return SensorSafeAdapter_MapResult(result, SENSOR_REMOTE_INTERNAL_ERROR);
}

/**
 * @brief 撤销安全会话并释放 UART6 DMA；不清除身份模块持久化启动计数。
 */
void SensorSafeAdapter_Deactivate(void)
{
    SensorSafeTransportUart6_Abort();
    SensorSafeService_Deactivate(&s_safe_service);
}

/**
 * @brief 仅中止当前 UART6 DMA 事务，用于命令切换快速退出，不修改会话对象。
 */
void SensorSafeAdapter_AbortTransport(void)
{
    SensorSafeTransportUart6_Abort();
}

/**
 * @brief 查询安全服务是否已完成 HELLO 且可接收业务命令。
 *
 * @return 1 表示安全服务已完成探测并处于 active 状态，可接收业务命令；0 表示服务上下文尚未激活。
 */
uint8_t SensorSafeAdapter_IsActive(void)
{
    return SensorSafeService_IsActive(&s_safe_service);
}

/**
 * @brief 根据已验收 HELLO 能力位判断水位电容命令是否可用。
 *
 * @return 1 表示安全服务已激活且 HELLO 能力位声明支持水位电容；否则返回 0。
 */
uint8_t SensorSafeAdapter_SupportsWaterCap(void)
{
    return (uint8_t)(((s_safe_service.active != 0U) &&
                      ((s_safe_service.identity.capability_flags & SENSOR_SAFE_CAP_WATER_CAP) != 0U))
                         ? 1U
                         : 0U);
}

/**
 * @brief 根据已验收 HELLO 能力位判断陀螺仪命令是否可用。
 *
 * @return 1 表示安全服务已激活且 HELLO 能力位声明支持陀螺仪；否则返回 0。
 */
uint8_t SensorSafeAdapter_SupportsGyro(void)
{
    return (uint8_t)(((s_safe_service.active != 0U) &&
                      ((s_safe_service.identity.capability_flags & SENSOR_SAFE_CAP_GYRO) != 0U))
                         ? 1U
                         : 0U);
}

/**
 * @brief 查询当前安全传感器是否声明参数维护和完整参数表读取能力。
 *
 * @return 1 表示安全服务处于活动态，且身份能力位包含 SENSOR_SAFE_CAP_PARAM_RW；0 表示服务未激活，或传感器未声明参数读写能力。
 */
uint8_t SensorSafeAdapter_SupportsParamTable(void)
{
    return (uint8_t)(((s_safe_service.active != 0U) &&
                      ((s_safe_service.identity.capability_flags & SENSOR_SAFE_CAP_PARAM_RW) != 0U))
                         ? 1U
                         : 0U);
}

/**
 * @brief 模式响应声明的稳定时间在返回旧业务层前完成可打断等待。
 *
 * @param mode 旧业务入口请求的安全协议测量模式码；成功后还需执行传感器声明的稳定等待。
 * @return NO_ERROR 表示模式切换已确认且传感器声明的稳定等待完成；模式请求、协议事务或可打断等待失败时返回映射后的具体错误码。
 */
static uint32_t SensorSafeAdapter_SetMode(uint8_t mode)
{
    SensorSafeServiceResult result;
    uint16_t settle_time_ms = 0U;
    uint32_t mapped;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_SetMeasureMode(&s_safe_service, mode, &settle_time_ms);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    mapped = SensorSafeAdapter_MapResult(result, SENSOR_MODE_NOT_READY);
    if ((mapped == NO_ERROR) && (settle_time_ms != 0U)) {
        mapped = AbortableDelay_CommandSwitch((uint32_t)settle_time_ms, 50U);
    }
    return mapped;
}

/**
 * @brief 把旧业务的密度模式入口映射到安全协议模式命令。
 *
 * @return NO_ERROR 表示安全传感器已确认密度模式且稳定等待完成；命令切换、传输、协议、远端、能力或模式错误返回映射后的整机错误码。
 */
uint32_t SensorSafeAdapter_EnableDensityMode(void)
{
    return SensorSafeAdapter_SetMode((uint8_t)SENSOR_SAFE_MEASURE_DENSITY);
}

/**
 * @brief 把旧业务的液位模式入口映射到安全协议模式命令。
 *
 * @return 返回安全液位模式切换映射后的整机错误码；NO_ERROR 表示模式确认和稳定等待均完成。
 */
uint32_t SensorSafeAdapter_EnableLevelMode(void)
{
    return SensorSafeAdapter_SetMode((uint8_t)SENSOR_SAFE_MEASURE_LEVEL);
}

/**
 * @brief 读取安全协议密度组合值并保持旧业务层的浮点单位。
 *
 * @details 调用场景：现有密度测量流程识别为安全传感器后调用。
 * @note 关键约束：恢复日志只由适配层记录，最终故障仍交给原测量流程统一出口。
 *
 * @param frequency_hz 传感器频率，单位 Hz。
 * @param density_kg_m3 用于返回实时密度的输出参数，单位 kg/m3。
 * @param temperature_c 温度值，单位 ℃。
 * @return NO_ERROR 表示频率、密度和温度三个输出均已更新；数据无效映射为 DENSITY_INVALID，其他服务、传输、协议或远端失败返回对应整机错误码。
 */
uint32_t SensorSafeAdapter_ReadDensity(float *frequency_hz,
                                       float *density_kg_m3,
                                       float *temperature_c)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadDensity(&s_safe_service,
                                           frequency_hz,
                                           density_kg_m3,
                                           temperature_c);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, DENSITY_INVALID);
}

/**
 * @brief 读取并映射液位频率，失败时保留原业务的 SONIC_FREQ_ABNORMAL 口径。
 *
 * @param frequency_hz 传感器频率，单位 Hz。
 * @return NO_ERROR 表示液位频率输出已更新；数据无效映射为 SONIC_FREQ_ABNORMAL，其他服务、传输、协议或远端失败返回对应整机错误码。
 */
uint32_t SensorSafeAdapter_ReadLevelFrequency(uint32_t *frequency_hz)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadLevelFrequency(&s_safe_service, frequency_hz);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, SONIC_FREQ_ABNORMAL);
}

/**
 * @brief 读取水位电容并把安全服务结果映射为现有整机错误码。
 *
 * @param capacitance_pf 用于返回水位通道电容值的输出参数，单位 pF。
 * @return 返回安全水位电容读取映射后的整机错误码；NO_ERROR 表示输出值有效，不支持、远端或通信失败返回对应错误。
 */
uint32_t SensorSafeAdapter_ReadWaterCapacitance(float *capacitance_pf)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadWaterCapacitance(&s_safe_service, capacitance_pf);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, SENSOR_REMOTE_INTERNAL_ERROR);
}

/**
 * @brief 读取双轴姿态角并把安全服务结果映射为现有整机错误码。
 *
 * @param angle_x_deg 用于返回陀螺仪 X 轴角度的输出参数，单位度。
 * @param angle_y_deg 用于返回陀螺仪 Y 轴角度的输出参数，单位度。
 * @return NO_ERROR 表示 X、Y 双轴角度均已更新；数据无效映射为 SENSOR_GYRO_ANGLE_ERROR，其他服务、传输、协议或远端失败返回对应整机错误码。
 */
uint32_t SensorSafeAdapter_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadGyroAngle(&s_safe_service, angle_x_deg, angle_y_deg);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, SENSOR_GYRO_ANGLE_ERROR);
}

/**
 * @brief 把服务层完整参数表接口公开给 CPU2 上层，并保持整机错误码口径。
 *
 * @details 调用场景：上电同步或维护导出显式调用，普通测量流程不自动触发。
 * @note 关键约束：调用期间可能占用链路 7 到 10 秒，恢复日志仍由本适配层统一记录。
 *
 * @param values 用于保存连续参数值或测量值的数组。
 * @param value_capacity 调用方结果数组可容纳的元素数量。
 * @param value_count_out 用于返回实际写入结果数组的元素数量。
 * @return 返回整机错误码；NO_ERROR 表示完整参数表已写入调用方数组，其他值表示容量、会话、通信、协议或参数 CRC 失败。
 */
uint32_t SensorSafeAdapter_ReadAllParams(SensorSafeParameterValue *values,
                                         uint16_t value_capacity,
                                         uint16_t *value_count_out)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadAllParams(&s_safe_service,
                                             values,
                                             value_capacity,
                                             value_count_out);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, SENSOR_PARAM_CRC_ERROR);
}
