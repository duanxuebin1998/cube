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

static SensorSafeServiceContext s_safe_service;

/* 查询整机命令切换请求，供阻塞式 UART 等待和稳定等待及时退出。 */
static uint8_t SensorSafeAdapter_ShouldAbort(void)
{
    return (uint8_t)(HasEffectiveCommandSwitchRequest() ? 1U : 0U);
}

/* 向安全协议层提供 HAL 单调毫秒节拍。 */
static uint32_t SensorSafeAdapter_NowMs(void)
{
    return HAL_GetTick();
}

/* 把新协议的分层错误映射到现有整机错误码，不在适配层直接置全局故障。 */
static uint32_t SensorSafeAdapter_MapResult(SensorSafeServiceResult result,
                                            uint32_t invalid_data_error)
{
    switch (result) {
    case SENSOR_SAFE_SERVICE_OK:
        return NO_ERROR;
    case SENSOR_SAFE_SERVICE_INVALID_ARGUMENT:
        return PARAM_ADDRESS_OVERFLOW;
    case SENSOR_SAFE_SERVICE_TRANSPORT_ERROR:
        if (s_safe_service.client.last_transport_result == SENSOR_SAFE_TRANSPORT_TIMEOUT) {
            return SENSOR_DEVICE_COMM_TIMEOUT;
        }
        return COMM_UART_TRANSFER_ERROR;
    case SENSOR_SAFE_SERVICE_PROTOCOL_ERROR:
        if (s_safe_service.client.last_protocol_result == SENSOR_SAFE_BAD_CRC) {
            return SENSOR_BCC_ERROR;
        }
        return SENSOR_RESP_FORMAT_ERROR;
    case SENSOR_SAFE_SERVICE_REMOTE_ERROR:
        return SENSOR_DEVICE_REPORTED_ERROR;
    case SENSOR_SAFE_SERVICE_DATA_INVALID:
    case SENSOR_SAFE_SERVICE_DATA_STALE:
        return invalid_data_error;
    case SENSOR_SAFE_SERVICE_UNSUPPORTED:
        return PARAM_ERROR;
    case SENSOR_SAFE_SERVICE_BUFFER_TOO_SMALL:
        return PARAM_ADDRESS_OVERFLOW;
    case SENSOR_SAFE_SERVICE_IDENTITY_ERROR:
    case SENSOR_SAFE_SERVICE_SESSION_ERROR:
    default:
        return SENSOR_RESP_FORMAT_ERROR;
    }
}

/* 把触发重新 HELLO 的原始 client 结果映射为可检索的整机错误码。 */
static uint32_t SensorSafeAdapter_MapRecoveryError(void)
{
    if (s_safe_service.last_recovery_trigger == SENSOR_SAFE_CLIENT_TRANSPORT_ERROR) {
        return (s_safe_service.last_recovery_transport_result == SENSOR_SAFE_TRANSPORT_TIMEOUT)
                   ? SENSOR_DEVICE_COMM_TIMEOUT
                   : COMM_UART_TRANSFER_ERROR;
    }
    if ((s_safe_service.last_recovery_trigger == SENSOR_SAFE_CLIENT_PROTOCOL_ERROR) &&
        (s_safe_service.last_recovery_protocol_result == SENSOR_SAFE_BAD_CRC)) {
        return SENSOR_BCC_ERROR;
    }
    return SENSOR_RESP_FORMAT_ERROR;
}

/* 适配层在业务调用返回后统一记录恢复尝试和最终成功，不重复承担最终故障出口。 */
static void SensorSafeAdapter_LogRecovery(uint32_t attempts_before,
                                          uint32_t recoveries_before)
{
    uint32_t error_code;

    if (s_safe_service.recovery_attempt_count == attempts_before) {
        return;
    }
    error_code = SensorSafeAdapter_MapRecoveryError();
    /* 错误 阶段：错误重试 模块：传感器 操作：通信诊断 原因：ErrorLog_GetReasonByCode(error_code) 尝试：1/1 错误码：error_code 错误名：ErrorLog_GetCodeName(error_code) */
    ErrorLog_Retry(ERROR_LOG_MODULE_SENSOR,
                   ERROR_LOG_OP_COMM_DIAG,
                   ErrorLog_GetReasonByCode(error_code),
                   1U,
                   1U,
                   error_code);
    if (s_safe_service.recovery_count != recoveries_before) {
        /* 错误 阶段：重试成功 模块：传感器 操作：通信诊断 原因：ErrorLog_GetReasonByCode(error_code) 尝试：1/1 */
        ErrorLog_Recover(ERROR_LOG_MODULE_SENSOR,
                         ERROR_LOG_OP_COMM_DIAG,
                         ErrorLog_GetReasonByCode(error_code),
                         1U,
                         1U);
    }
}

/* 同一上电周期只初始化一次身份计数和 client，避免重复探测消耗启动计数。 */
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

/*
 * 函数用途：探测并建立安全传感器会话，输出传感器唯一标识。
 * 调用场景：传感器自动识别在蓝牙链路失败后、旧协议探测前调用。
 * 关键约束：任一失败均撤销活动态并释放 UART6，保证 LTD/V2 与 DSM 可继续回退。
 */
uint32_t SensorSafeAdapter_Probe(uint32_t *sensor_id)
{
    SensorSafeServiceResult result;

    if (sensor_id == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }
    result = SensorSafeAdapter_EnsureInitialized();
    if (result != SENSOR_SAFE_SERVICE_OK) {
        SensorSafeAdapter_Deactivate();
        return SensorSafeAdapter_MapResult(result, SENSOR_DEVICE_REPORTED_ERROR);
    }
    SensorSafeTransportUart6_Abort();
    result = SensorSafeService_Probe(&s_safe_service, sensor_id);
    if (result != SENSOR_SAFE_SERVICE_OK) {
        SensorSafeAdapter_Deactivate();
    }
    return SensorSafeAdapter_MapResult(result, SENSOR_DEVICE_REPORTED_ERROR);
}

/* 撤销安全会话并释放 UART6 DMA；不清除身份模块持久化启动计数。 */
void SensorSafeAdapter_Deactivate(void)
{
    SensorSafeTransportUart6_Abort();
    SensorSafeService_Deactivate(&s_safe_service);
}

/* 仅中止当前 UART6 DMA 事务，用于命令切换快速退出，不修改会话对象。 */
void SensorSafeAdapter_AbortTransport(void)
{
    SensorSafeTransportUart6_Abort();
}

/* 查询安全服务是否已完成 HELLO 且可接收业务命令。 */
uint8_t SensorSafeAdapter_IsActive(void)
{
    return SensorSafeService_IsActive(&s_safe_service);
}

/* 根据已验收 HELLO 能力位判断水位电容命令是否可用。 */
uint8_t SensorSafeAdapter_SupportsWaterCap(void)
{
    return (uint8_t)(((s_safe_service.active != 0U) &&
                      ((s_safe_service.identity.capability_flags & SENSOR_SAFE_CAP_WATER_CAP) != 0U))
                         ? 1U
                         : 0U);
}

/* 根据已验收 HELLO 能力位判断陀螺仪命令是否可用。 */
uint8_t SensorSafeAdapter_SupportsGyro(void)
{
    return (uint8_t)(((s_safe_service.active != 0U) &&
                      ((s_safe_service.identity.capability_flags & SENSOR_SAFE_CAP_GYRO) != 0U))
                         ? 1U
                         : 0U);
}

/* 查询当前安全传感器是否声明参数维护和完整参数表读取能力。 */
uint8_t SensorSafeAdapter_SupportsParamTable(void)
{
    return (uint8_t)(((s_safe_service.active != 0U) &&
                      ((s_safe_service.identity.capability_flags & SENSOR_SAFE_CAP_PARAM_RW) != 0U))
                         ? 1U
                         : 0U);
}

/* 模式响应声明的稳定时间在返回旧业务层前完成可打断等待。 */
static uint32_t SensorSafeAdapter_SetMode(uint8_t mode)
{
    SensorSafeServiceResult result;
    uint16_t settle_time_ms = 0U;
    uint32_t mapped;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_SetMeasureMode(&s_safe_service, mode, &settle_time_ms);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    mapped = SensorSafeAdapter_MapResult(result, SENSOR_DEVICE_REPORTED_ERROR);
    if ((mapped == NO_ERROR) && (settle_time_ms != 0U)) {
        mapped = AbortableDelay_CommandSwitch((uint32_t)settle_time_ms, 50U);
    }
    return mapped;
}

/* 把旧业务的密度模式入口映射到安全协议模式命令。 */
uint32_t SensorSafeAdapter_EnableDensityMode(void)
{
    return SensorSafeAdapter_SetMode((uint8_t)SENSOR_SAFE_MEASURE_DENSITY);
}

/* 把旧业务的液位模式入口映射到安全协议模式命令。 */
uint32_t SensorSafeAdapter_EnableLevelMode(void)
{
    return SensorSafeAdapter_SetMode((uint8_t)SENSOR_SAFE_MEASURE_LEVEL);
}

/*
 * 函数用途：读取安全协议密度组合值并保持旧业务层的浮点单位。
 * 调用场景：现有密度测量流程识别为安全传感器后调用。
 * 关键约束：恢复日志只由适配层记录，最终故障仍交给原测量流程统一出口。
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

/* 读取并映射液位频率，失败时保留原业务的 SONIC_FREQ_ABNORMAL 口径。 */
uint32_t SensorSafeAdapter_ReadLevelFrequency(uint32_t *frequency_hz)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadLevelFrequency(&s_safe_service, frequency_hz);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, SONIC_FREQ_ABNORMAL);
}

/* 读取水位电容并把安全服务结果映射为现有整机错误码。 */
uint32_t SensorSafeAdapter_ReadWaterCapacitance(float *capacitance_pf)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadWaterCapacitance(&s_safe_service, capacitance_pf);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, SENSOR_DEVICE_REPORTED_ERROR);
}

/* 读取双轴姿态角并把安全服务结果映射为现有整机错误码。 */
uint32_t SensorSafeAdapter_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg)
{
    SensorSafeServiceResult result;
    uint32_t attempts_before = s_safe_service.recovery_attempt_count;
    uint32_t recoveries_before = s_safe_service.recovery_count;

    result = SensorSafeService_ReadGyroAngle(&s_safe_service, angle_x_deg, angle_y_deg);
    SensorSafeAdapter_LogRecovery(attempts_before, recoveries_before);
    return SensorSafeAdapter_MapResult(result, SENSOR_DEVICE_REPORTED_ERROR);
}

/*
 * 函数用途：把服务层完整参数表接口公开给 CPU2 上层，并保持整机错误码口径。
 * 调用场景：上电同步或维护导出显式调用，普通测量流程不自动触发。
 * 关键约束：调用期间可能占用链路 7 到 10 秒，恢复日志仍由本适配层统一记录。
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
    return SensorSafeAdapter_MapResult(result, SENSOR_DEVICE_REPORTED_ERROR);
}
