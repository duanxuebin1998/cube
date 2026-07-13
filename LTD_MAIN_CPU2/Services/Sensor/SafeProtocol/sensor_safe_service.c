#include "sensor_safe_service.h"

#include <string.h>

/* 区分可安全重发的只读事务和结果不确定时禁止重发的副作用事务。 */
typedef enum {
    SENSOR_SAFE_RECOVERY_READ_ONLY = 0,
    SENSOR_SAFE_RECOVERY_SIDE_EFFECT
} SensorSafeRecoveryPolicy;

/* 服务级诊断计数饱和保持，避免长稳运行后回绕成较小值。 */
static void SensorSafeService_IncrementCounter(uint32_t *counter)
{
    if ((counter != NULL) && (*counter != UINT32_MAX)) {
        (*counter)++;
    }
}

/* 把 client 分层结果收敛为设备适配层使用的稳定服务结果。 */
static SensorSafeServiceResult SensorSafeService_MapClientResult(SensorSafeClientResult result)
{
    switch (result) {
    case SENSOR_SAFE_CLIENT_OK:
        return SENSOR_SAFE_SERVICE_OK;
    case SENSOR_SAFE_CLIENT_INVALID_ARGUMENT:
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    case SENSOR_SAFE_CLIENT_TRANSPORT_ERROR:
        return SENSOR_SAFE_SERVICE_TRANSPORT_ERROR;
    case SENSOR_SAFE_CLIENT_REMOTE_ERROR:
        return SENSOR_SAFE_SERVICE_REMOTE_ERROR;
    case SENSOR_SAFE_CLIENT_DATA_INVALID:
        return SENSOR_SAFE_SERVICE_DATA_INVALID;
    case SENSOR_SAFE_CLIENT_DATA_STALE:
        return SENSOR_SAFE_SERVICE_DATA_STALE;
    case SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY:
        return SENSOR_SAFE_SERVICE_UNSUPPORTED;
    case SENSOR_SAFE_CLIENT_BUFFER_TOO_SMALL:
        return SENSOR_SAFE_SERVICE_BUFFER_TOO_SMALL;
    case SENSOR_SAFE_CLIENT_NEEDS_HELLO:
    case SENSOR_SAFE_CLIENT_CONTROL_WINDOW_CLOSED:
        return SENSOR_SAFE_SERVICE_SESSION_ERROR;
    case SENSOR_SAFE_CLIENT_PROTOCOL_ERROR:
    default:
        return SENSOR_SAFE_SERVICE_PROTOCOL_ERROR;
    }
}

/*
 * 函数用途：按旧 LTD 参数码判断冻结映射中的整数参数类型。
 * 调用场景：完整参数表读取成功后，服务层逐项核对传感器返回的解释元数据。
 * 关键约束：未列出的参数均来自旧协议单浮点，统一转换为六位小数定点数。
 */
static uint8_t SensorSafeService_LtdParamType(uint16_t legacy_param_code)
{
    uint8_t param_type = (uint8_t)SENSOR_SAFE_PARAM_TYPE_SIGNED;

    if (legacy_param_code == 2U) {
        param_type = (uint8_t)SENSOR_SAFE_PARAM_TYPE_BITMASK;
    } else if ((legacy_param_code == 20U) ||
               (legacy_param_code == 107U) ||
               (legacy_param_code == 108U)) {
        param_type = (uint8_t)SENSOR_SAFE_PARAM_TYPE_ENUM;
    } else if ((legacy_param_code == 4U) ||
               ((legacy_param_code >= 10U) && (legacy_param_code <= 16U)) ||
               ((legacy_param_code >= 21U) && (legacy_param_code <= 24U))) {
        param_type = (uint8_t)SENSOR_SAFE_PARAM_TYPE_UNSIGNED;
    } else {
        /* 旧协议单浮点统一使用 SIGNED、scale=-6 的十进制定点表示。 */
    }
    return param_type;
}

/*
 * 函数用途：核对完整 LTD 参数表的项数、连续地址和冻结解释元数据。
 * 调用场景：client 完成所有分页事务后、服务层向业务调用方发布结果之前。
 * 关键约束：任一项不符都使整表无效，防止自洽 CRC 掩盖参数缺失或解释漂移。
 */
static uint8_t SensorSafeService_IsLtdParamTableValid(const SensorSafeParameterValue *values,
                                                      uint16_t value_count)
{
    uint16_t index;

    if ((values == NULL) || (value_count != SENSOR_SAFE_LTD_PARAM_COUNT)) {
        return 0U;
    }
    for (index = 0U; index < SENSOR_SAFE_LTD_PARAM_COUNT; index++) {
        uint8_t expected_access = (index < 20U)
                                      ? SENSOR_SAFE_PARAM_ACCESS_READ
                                      : (SENSOR_SAFE_PARAM_ACCESS_READ |
                                         SENSOR_SAFE_PARAM_ACCESS_WRITE);
        uint8_t expected_type = SensorSafeService_LtdParamType(index);
        int16_t expected_scale = (expected_type == (uint8_t)SENSOR_SAFE_PARAM_TYPE_SIGNED)
                                     ? (int16_t)SENSOR_SAFE_LTD_FLOAT_SCALE
                                     : 0;

        if ((values[index].param_id != (uint16_t)(SENSOR_SAFE_LTD_PARAM_BASE + index)) ||
            (values[index].param_index != 0U) ||
            (values[index].param_type != expected_type) ||
            (values[index].access != expected_access) ||
            (values[index].scale != expected_scale) ||
            (values[index].unit_id != SENSOR_SAFE_UNIT_LEGACY_NATIVE)) {
            return 0U;
        }
    }
    return 1U;
}

/*
 * 函数用途：领取新挑战、建立会话、核对必需能力并读取配置摘要。
 * 调用场景：首次探测以及允许恢复的控制事务失败后调用。
 * 关键约束：每次新 HELLO 领取新挑战，同一事务内重试由 client 复用原挑战。
 */
static SensorSafeServiceResult SensorSafeService_Hello(SensorSafeServiceContext *context)
{
    SensorSafeIdentityResult identity_result;
    SensorSafeClientResult client_result;
    SensorSafeConfigDigest digest;
    uint32_t nonce = 0U;

    context->active = 0U;
    (void)memset(&context->config_digest, 0, sizeof(context->config_digest));
    identity_result = SensorSafeIdentity_GenerateNonce(&context->startup_identity,
                                                       &nonce,
                                                       &context->last_nonce_source);
    context->last_identity_result = identity_result;
    if ((identity_result != SENSOR_SAFE_IDENTITY_OK) &&
        (identity_result != SENSOR_SAFE_IDENTITY_OK_FALLBACK_NONCE)) {
        context->active = 0U;
        return SENSOR_SAFE_SERVICE_IDENTITY_ERROR;
    }
    client_result = SensorSafeClient_Hello(&context->client, nonce, &context->identity);
    context->last_client_result = client_result;
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        context->active = 0U;
        return SensorSafeService_MapClientResult(client_result);
    }
    /* 设备侧基础测量依赖的能力必须全部声明，缺失时不能冒充可用传感器。 */
    if ((context->identity.capability_flags & SENSOR_SAFE_SERVICE_REQUIRED_CAPABILITIES) !=
        SENSOR_SAFE_SERVICE_REQUIRED_CAPABILITIES) {
        context->active = 0U;
        return SENSOR_SAFE_SERVICE_UNSUPPORTED;
    }
    (void)memset(&digest, 0, sizeof(digest));
    client_result = SensorSafeClient_GetConfigDigest(&context->client, &digest);
    context->last_client_result = client_result;
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        context->client.session.state = SENSOR_SAFE_SESSION_RECOVERING;
        context->active = 0U;
        return SensorSafeService_MapClientResult(client_result);
    }
    context->config_digest = digest;
    context->active = 1U;
    SensorSafeService_IncrementCounter(&context->hello_count);
    return SENSOR_SAFE_SERVICE_OK;
}

/*
 * 函数用途：确认服务已初始化、处于活动态且会话允许普通业务。
 * 调用场景：所有请求响应测量和模式切换接口的共同前置检查。
 * 关键约束：周期流活动时禁止隐式切回控制业务；离线或恢复态只能通过新 HELLO 恢复。
 */
static SensorSafeServiceResult SensorSafeService_EnsureReady(SensorSafeServiceContext *context)
{
    if ((context == NULL) || (context->initialized == 0U) || (context->active == 0U)) {
        return SENSOR_SAFE_SERVICE_SESSION_ERROR;
    }
    if ((context->client.session.state == SENSOR_SAFE_SESSION_READY) ||
        (context->client.session.state == SENSOR_SAFE_SESSION_MEASURE_READY)) {
        return SENSOR_SAFE_SERVICE_OK;
    }
    if (context->client.session.state == SENSOR_SAFE_SESSION_PERIODIC_ACTIVE) {
        return SENSOR_SAFE_SERVICE_SESSION_ERROR;
    }
    return SensorSafeService_Hello(context);
}

/*
 * 函数用途：按事务副作用策略决定是否允许一次重新 HELLO。
 * 调用场景：业务控制命令首次失败后、决定是否进行第二次尝试时调用。
 * 关键约束：结果不确定的副作用命令禁止重发；可信 ERR 明确拒绝时允许恢复后重试。
 */
static uint8_t SensorSafeService_TryRecover(SensorSafeServiceContext *context,
                                           SensorSafeClientResult result,
                                           SensorSafeRecoveryPolicy policy)
{
    SensorSafeServiceResult hello_result;

    if (result == SENSOR_SAFE_CLIENT_NEEDS_HELLO) {
        /* 可信 ERR 已明确拒绝原事务，副作用命令也允许重新建立会话。 */
    } else if ((policy == SENSOR_SAFE_RECOVERY_READ_ONLY) &&
               ((result == SENSOR_SAFE_CLIENT_TRANSPORT_ERROR) ||
                (result == SENSOR_SAFE_CLIENT_PROTOCOL_ERROR))) {
        /* 只读事务可在结果不确定时重新建立会话并重发。 */
    } else {
        if ((policy == SENSOR_SAFE_RECOVERY_SIDE_EFFECT) &&
            ((result == SENSOR_SAFE_CLIENT_TRANSPORT_ERROR) ||
             (result == SENSOR_SAFE_CLIENT_PROTOCOL_ERROR))) {
            SensorSafeService_IncrementCounter(&context->recovery_blocked_count);
            context->client.session.state = SENSOR_SAFE_SESSION_RECOVERING;
        }
        return 0U;
    }
    SensorSafeService_IncrementCounter(&context->recovery_attempt_count);
    context->last_recovery_trigger = result;
    context->last_recovery_transport_result = context->client.last_transport_result;
    context->last_recovery_protocol_result = context->client.last_protocol_result;
    context->last_recovery_wire_result = context->client.last_wire_result;
    hello_result = SensorSafeService_Hello(context);
    if (hello_result != SENSOR_SAFE_SERVICE_OK) {
        return 0U;
    }
    return 1U;
}

/* 只有重新 HELLO 后的业务重试也成功，才把本次恢复计为成功。 */
static void SensorSafeService_RecordRecoveryResult(SensorSafeServiceContext *context,
                                                   uint8_t attempt,
                                                   SensorSafeClientResult result)
{
    if ((context != NULL) && (attempt != 0U) && (result == SENSOR_SAFE_CLIENT_OK)) {
        SensorSafeService_IncrementCounter(&context->recovery_count);
    }
}

/* 周期快报失败原因与本次 client 结果绑定，禁止沿用上一帧的协议错误。 */
static SensorSafeResult SensorSafeService_PeriodicFailureReason(const SensorSafeServiceContext *context,
                                                                SensorSafeClientResult result)
{
    if (result == SENSOR_SAFE_CLIENT_DATA_STALE) {
        return SENSOR_SAFE_DATA_STALE;
    }
    if (result == SENSOR_SAFE_CLIENT_NEEDS_HELLO) {
        return SENSOR_SAFE_NEEDS_HELLO;
    }
    if (result == SENSOR_SAFE_CLIENT_PROTOCOL_ERROR) {
        return (context->client.last_protocol_result == SENSOR_SAFE_OK)
                   ? SENSOR_SAFE_DATA_INVALID
                   : context->client.last_protocol_result;
    }
    if (result == SENSOR_SAFE_CLIENT_TRANSPORT_ERROR) {
        return SENSOR_SAFE_DATA_STALE;
    }
    return SENSOR_SAFE_DATA_INVALID;
}

/*
 * 函数用途：初始化持久化身份、client、会话和周期快照监控器。
 * 调用场景：设备适配层首次尝试安全协议探测前调用。
 * 关键约束：同一上电周期重复调用不再推进 FRAM 启动计数；本函数不发送 HELLO。
 */
SensorSafeServiceResult SensorSafeService_Init(SensorSafeServiceContext *context,
                                               const SensorSafeIdentityOps *identity_ops,
                                               const SensorSafeClientTransportOps *transport_ops)
{
    SensorSafeClientConfig client_config;
    uint32_t boot_counter = 0U;

    if ((context == NULL) || (identity_ops == NULL) || (transport_ops == NULL) ||
        (transport_ops->exchange == NULL) || (transport_ops->receive == NULL) ||
        (transport_ops->now_ms == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    if (context->initialized != 0U) {
        return SENSOR_SAFE_SERVICE_OK;
    }
    (void)memset(context, 0, sizeof(*context));
    context->last_identity_result = SensorSafeIdentity_Initialize(&context->startup_identity,
                                                                 identity_ops);
    if (context->last_identity_result != SENSOR_SAFE_IDENTITY_OK) {
        return SENSOR_SAFE_SERVICE_IDENTITY_ERROR;
    }
    context->last_identity_result = SensorSafeIdentity_GetBootCounter(&context->startup_identity,
                                                                     &boot_counter);
    if (context->last_identity_result != SENSOR_SAFE_IDENTITY_OK) {
        return SENSOR_SAFE_SERVICE_IDENTITY_ERROR;
    }

    (void)memset(&client_config, 0, sizeof(client_config));
    client_config.cpu_node_id = SENSOR_SAFE_SERVICE_CPU_NODE_ID;
    client_config.sensor_node_id = SENSOR_SAFE_SERVICE_SENSOR_NODE_ID;
    client_config.cpu_boot_counter = boot_counter;
    client_config.max_data_age_ms = SENSOR_SAFE_SERVICE_MAX_DATA_AGE_MS;
    client_config.request_timeout_ms = SENSOR_SAFE_CLIENT_DEFAULT_TIMEOUT_MS;
    client_config.max_attempts = SENSOR_SAFE_CLIENT_DEFAULT_MAX_ATTEMPTS;
    SensorSafeClient_Init(&context->client, &client_config, transport_ops);
    SensorSafeMonitor_Init(&context->monitor);
    context->initialized = 1U;
    return SENSOR_SAFE_SERVICE_OK;
}

/*
 * 函数用途：执行完整 HELLO 与配置摘要验收并返回安全传感器 ID。
 * 调用场景：传感器自动识别的安全协议探测阶段。
 * 关键约束：必需能力缺失或摘要读取失败时 active 保持为 0，允许外层回退旧协议。
 */
SensorSafeServiceResult SensorSafeService_Probe(SensorSafeServiceContext *context,
                                                uint32_t *sensor_id)
{
    SensorSafeServiceResult result;

    if ((context == NULL) || (sensor_id == NULL) || (context->initialized == 0U)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    result = SensorSafeService_Hello(context);
    if (result != SENSOR_SAFE_SERVICE_OK) {
        return result;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_SERVICE_REQUIRED_CAPABILITIES) !=
        SENSOR_SAFE_SERVICE_REQUIRED_CAPABILITIES) {
        context->active = 0U;
        return SENSOR_SAFE_SERVICE_UNSUPPORTED;
    }
    *sensor_id = context->identity.sensor_id;
    return SENSOR_SAFE_SERVICE_OK;
}

/* 撤销服务活动态、client 会话及周期快照，不重复初始化持久化身份。 */
void SensorSafeService_Deactivate(SensorSafeServiceContext *context)
{
    if (context == NULL) {
        return;
    }
    context->active = 0U;
    SensorSafeClient_Deactivate(&context->client);
    SensorSafeMonitor_Invalidate(&context->monitor, SENSOR_SAFE_STREAM_STOPPED);
}

/* 查询服务是否已通过探测并保持活动态。 */
uint8_t SensorSafeService_IsActive(const SensorSafeServiceContext *context)
{
    return (uint8_t)(((context != NULL) && (context->active != 0U)) ? 1U : 0U);
}

/*
 * 函数用途：切换测量模式并返回传感器声明的稳定等待时间。
 * 调用场景：旧测量流程进入密度或液位阶段时由适配层调用。
 * 关键约束：模式切换属于副作用事务，响应不确定时禁止自动重发。
 */
SensorSafeServiceResult SensorSafeService_SetMeasureMode(SensorSafeServiceContext *context,
                                                         uint8_t mode,
                                                         uint16_t *settle_time_ms)
{
    SensorSafeServiceResult ready_result;
    SensorSafeClientResult client_result;
    uint8_t attempt;

    if ((context == NULL) || (settle_time_ms == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    ready_result = SensorSafeService_EnsureReady(context);
    if (ready_result != SENSOR_SAFE_SERVICE_OK) {
        return ready_result;
    }
    client_result = SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    for (attempt = 0U; attempt < 2U; attempt++) {
        client_result = SensorSafeClient_SetMeasureMode(&context->client, mode, settle_time_ms);
        context->last_client_result = client_result;
        if ((client_result == SENSOR_SAFE_CLIENT_OK) || (attempt != 0U) ||
            (SensorSafeService_TryRecover(context,
                                          client_result,
                                          SENSOR_SAFE_RECOVERY_SIDE_EFFECT) == 0U)) {
            break;
        }
    }
    SensorSafeService_RecordRecoveryResult(context, attempt, client_result);
    return SensorSafeService_MapClientResult(client_result);
}

/*
 * 函数用途：读取密度组合原始定点数并转换为旧业务层浮点单位。
 * 调用场景：密度测量流程读取频率、密度和温度时调用。
 * 关键约束：只读事务最多允许一次重新 HELLO；失败时不修改三个输出值。
 */
SensorSafeServiceResult SensorSafeService_ReadDensity(SensorSafeServiceContext *context,
                                                      float *frequency_hz,
                                                      float *density_kg_m3,
                                                      float *temperature_c)
{
    SensorSafeServiceResult ready_result;
    SensorSafeClientResult client_result;
    SensorSafeMeasurementCombo measurement;
    uint8_t attempt;

    if ((context == NULL) || (frequency_hz == NULL) ||
        (density_kg_m3 == NULL) || (temperature_c == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    ready_result = SensorSafeService_EnsureReady(context);
    if (ready_result != SENSOR_SAFE_SERVICE_OK) {
        return ready_result;
    }
    client_result = SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    for (attempt = 0U; attempt < 2U; attempt++) {
        client_result = SensorSafeClient_ReadMeasurementCombo(&context->client, &measurement);
        context->last_client_result = client_result;
        if ((client_result == SENSOR_SAFE_CLIENT_OK) || (attempt != 0U) ||
            (SensorSafeService_TryRecover(context,
                                          client_result,
                                          SENSOR_SAFE_RECOVERY_READ_ONLY) == 0U)) {
            break;
        }
    }
    SensorSafeService_RecordRecoveryResult(context, attempt, client_result);
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        return SensorSafeService_MapClientResult(client_result);
    }
    *frequency_hz = (float)measurement.frequency_hz_x1000 / 1000.0f;
    *density_kg_m3 = (float)measurement.density_kg_m3_x100 / 100.0f;
    *temperature_c = (float)measurement.temperature_c_x100 / 100.0f;
    return SENSOR_SAFE_SERVICE_OK;
}

/*
 * 函数用途：读取液位频率并按四舍五入转换为整数 Hz。
 * 调用场景：液位测量流程读取探头频率时调用。
 * 关键约束：加 500 前检查 UINT32 上界，避免溢出后产生虚假低频值。
 */
SensorSafeServiceResult SensorSafeService_ReadLevelFrequency(SensorSafeServiceContext *context,
                                                             uint32_t *frequency_hz)
{
    SensorSafeServiceResult ready_result;
    SensorSafeClientResult client_result;
    SensorSafeLevelMeasurement measurement;
    uint8_t attempt;

    if ((context == NULL) || (frequency_hz == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    ready_result = SensorSafeService_EnsureReady(context);
    if (ready_result != SENSOR_SAFE_SERVICE_OK) {
        return ready_result;
    }
    client_result = SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    for (attempt = 0U; attempt < 2U; attempt++) {
        client_result = SensorSafeClient_ReadLevelFrequency(&context->client, &measurement);
        context->last_client_result = client_result;
        if ((client_result == SENSOR_SAFE_CLIENT_OK) || (attempt != 0U) ||
            (SensorSafeService_TryRecover(context,
                                          client_result,
                                          SENSOR_SAFE_RECOVERY_READ_ONLY) == 0U)) {
            break;
        }
    }
    SensorSafeService_RecordRecoveryResult(context, attempt, client_result);
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        return SensorSafeService_MapClientResult(client_result);
    }
    if (measurement.level_frequency_hz_x1000 > (UINT32_MAX - 500U)) {
        return SENSOR_SAFE_SERVICE_DATA_INVALID;
    }
    *frequency_hz = (measurement.level_frequency_hz_x1000 + 500U) / 1000U;
    return SENSOR_SAFE_SERVICE_OK;
}

/* 读取水位电容定点数并转换为 pF；能力缺失由 client 明确返回不支持。 */
SensorSafeServiceResult SensorSafeService_ReadWaterCapacitance(SensorSafeServiceContext *context,
                                                               float *capacitance_pf)
{
    SensorSafeServiceResult ready_result;
    SensorSafeClientResult client_result;
    SensorSafeWaterCapMeasurement measurement;
    uint8_t attempt;

    if ((context == NULL) || (capacitance_pf == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    ready_result = SensorSafeService_EnsureReady(context);
    if (ready_result != SENSOR_SAFE_SERVICE_OK) {
        return ready_result;
    }
    client_result = SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    for (attempt = 0U; attempt < 2U; attempt++) {
        client_result = SensorSafeClient_ReadWaterCap(&context->client, &measurement);
        context->last_client_result = client_result;
        if ((client_result == SENSOR_SAFE_CLIENT_OK) || (attempt != 0U) ||
            (SensorSafeService_TryRecover(context,
                                          client_result,
                                          SENSOR_SAFE_RECOVERY_READ_ONLY) == 0U)) {
            break;
        }
    }
    SensorSafeService_RecordRecoveryResult(context, attempt, client_result);
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        return SensorSafeService_MapClientResult(client_result);
    }
    *capacitance_pf = (float)measurement.capacitance_pf_x10 / 10.0f;
    return SENSOR_SAFE_SERVICE_OK;
}

/* 读取双轴姿态定点数并转换为度；失败时不覆盖调用方输出。 */
SensorSafeServiceResult SensorSafeService_ReadGyroAngle(SensorSafeServiceContext *context,
                                                        float *angle_x_deg,
                                                        float *angle_y_deg)
{
    SensorSafeServiceResult ready_result;
    SensorSafeClientResult client_result;
    SensorSafeGyroMeasurement measurement;
    uint8_t attempt;

    if ((context == NULL) || (angle_x_deg == NULL) || (angle_y_deg == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    ready_result = SensorSafeService_EnsureReady(context);
    if (ready_result != SENSOR_SAFE_SERVICE_OK) {
        return ready_result;
    }
    client_result = SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    for (attempt = 0U; attempt < 2U; attempt++) {
        client_result = SensorSafeClient_ReadGyro(&context->client, &measurement);
        context->last_client_result = client_result;
        if ((client_result == SENSOR_SAFE_CLIENT_OK) || (attempt != 0U) ||
            (SensorSafeService_TryRecover(context,
                                          client_result,
                                          SENSOR_SAFE_RECOVERY_READ_ONLY) == 0U)) {
            break;
        }
    }
    SensorSafeService_RecordRecoveryResult(context, attempt, client_result);
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        return SensorSafeService_MapClientResult(client_result);
    }
    *angle_x_deg = (float)measurement.angle_x_deg_x100 / 100.0f;
    *angle_y_deg = (float)measurement.angle_y_deg_x100 / 100.0f;
    return SENSOR_SAFE_SERVICE_OK;
}

/*
 * 函数用途：在服务层以一次调用读取完整 LTD 参数表并隐藏底层分页事务。
 * 调用场景：参数同步、维护导出和诊断快照需要取得完整参数集合时调用。
 * 关键约束：固定校验 115 项映射；属于只读事务，允许会话失效后恢复一次。
 */
SensorSafeServiceResult SensorSafeService_ReadAllParams(SensorSafeServiceContext *context,
                                                        SensorSafeParameterValue *values,
                                                        uint16_t value_capacity,
                                                        uint16_t *value_count_out)
{
    SensorSafeServiceResult ready_result;
    SensorSafeClientResult client_result;
    uint8_t attempt;

    if ((context == NULL) || (values == NULL) || (value_count_out == NULL) ||
        (value_capacity == 0U)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    *value_count_out = 0U;
    if (value_capacity < SENSOR_SAFE_LTD_PARAM_COUNT) {
        *value_count_out = SENSOR_SAFE_LTD_PARAM_COUNT;
        return SENSOR_SAFE_SERVICE_BUFFER_TOO_SMALL;
    }
    ready_result = SensorSafeService_EnsureReady(context);
    if (ready_result != SENSOR_SAFE_SERVICE_OK) {
        return ready_result;
    }
    if (context->identity.param_table_version != SENSOR_SAFE_LTD_PARAM_TABLE_VERSION) {
        return SENSOR_SAFE_SERVICE_PROTOCOL_ERROR;
    }
    client_result = SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    for (attempt = 0U; attempt < 2U; attempt++) {
        client_result = SensorSafeClient_ReadAllParams(&context->client,
                                                      values,
                                                      SENSOR_SAFE_LTD_PARAM_COUNT,
                                                      value_count_out);
        context->last_client_result = client_result;
        if ((client_result == SENSOR_SAFE_CLIENT_OK) ||
            (client_result == SENSOR_SAFE_CLIENT_BUFFER_TOO_SMALL) ||
            (attempt != 0U) ||
            (SensorSafeService_TryRecover(context,
                                          client_result,
                                          SENSOR_SAFE_RECOVERY_READ_ONLY) == 0U)) {
            break;
        }
    }
    SensorSafeService_RecordRecoveryResult(context, attempt, client_result);
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        if ((client_result == SENSOR_SAFE_CLIENT_BUFFER_TOO_SMALL) &&
            (*value_count_out != SENSOR_SAFE_LTD_PARAM_COUNT)) {
            *value_count_out = 0U;
            return SENSOR_SAFE_SERVICE_PROTOCOL_ERROR;
        }
        return SensorSafeService_MapClientResult(client_result);
    }
    if (SensorSafeService_IsLtdParamTableValid(values, *value_count_out) == 0U) {
        *value_count_out = 0U;
        return SENSOR_SAFE_SERVICE_PROTOCOL_ERROR;
    }
    return SENSOR_SAFE_SERVICE_OK;
}

/*
 * 函数用途：协商周期、静默预算、流标识和控制保护窗并启动主动上报。
 * 调用场景：显式启用安全协议周期模式时调用，默认测量流程不会自动进入。
 * 关键约束：启动属于副作用事务；成功后清空旧快照，必须等待本流首帧重新发布。
 */
SensorSafeServiceResult SensorSafeService_StartPeriodic(SensorSafeServiceContext *context,
                                                        uint16_t period_ms,
                                                        uint16_t max_silent_ms,
                                                        uint16_t stream_id,
                                                        uint16_t control_guard_ms,
                                                        uint8_t measure_mode,
                                                        uint16_t *start_after_ms)
{
    SensorSafeServiceResult ready_result;
    SensorSafeClientResult client_result;
    uint8_t attempt;

    if ((context == NULL) || (start_after_ms == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    ready_result = SensorSafeService_EnsureReady(context);
    if (ready_result != SENSOR_SAFE_SERVICE_OK) {
        return ready_result;
    }
    client_result = SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    for (attempt = 0U; attempt < 2U; attempt++) {
        client_result = SensorSafeClient_StartPeriodic(&context->client,
                                                       period_ms,
                                                       max_silent_ms,
                                                       stream_id,
                                                       control_guard_ms,
                                                       measure_mode,
                                                       start_after_ms);
        context->last_client_result = client_result;
        if ((client_result == SENSOR_SAFE_CLIENT_OK) || (attempt != 0U) ||
            (SensorSafeService_TryRecover(context,
                                          client_result,
                                          SENSOR_SAFE_RECOVERY_SIDE_EFFECT) == 0U)) {
            break;
        }
    }
    SensorSafeService_RecordRecoveryResult(context, attempt, client_result);
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        return SensorSafeService_MapClientResult(client_result);
    }
    SensorSafeMonitor_Init(&context->monitor);
    return SENSOR_SAFE_SERVICE_OK;
}

/*
 * 函数用途：接收并验收一帧周期快报，再发布为一致性监控快照。
 * 调用场景：周期模式任务轮询入口。
 * 关键约束：任一 client 或 monitor 错误立即撤销旧快照，禁止陈旧值继续被读取。
 */
SensorSafeServiceResult SensorSafeService_PollPeriodic(SensorSafeServiceContext *context,
                                                       uint32_t timeout_ms)
{
    SensorSafeFastReport report;
    SensorSafeClientResult client_result;
    SensorSafeResult monitor_result;
    uint32_t now_ms;

    if ((context == NULL) || (context->client.transport.now_ms == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    client_result = SensorSafeClient_PollPeriodic(&context->client, &report, timeout_ms);
    context->last_client_result = client_result;
    if (client_result != SENSOR_SAFE_CLIENT_OK) {
        SensorSafeMonitor_Invalidate(&context->monitor,
                                     SensorSafeService_PeriodicFailureReason(context, client_result));
        return SensorSafeService_MapClientResult(client_result);
    }
    now_ms = context->client.transport.now_ms();
    monitor_result = SensorSafeMonitor_Publish(&context->monitor,
                                               &report,
                                               context->client.last_rx_timestamp_ms,
                                               now_ms,
                                               context->client.config.max_data_age_ms);
    if (monitor_result != SENSOR_SAFE_OK) {
        return (monitor_result == SENSOR_SAFE_DATA_STALE)
                   ? SENSOR_SAFE_SERVICE_DATA_STALE
                   : SENSOR_SAFE_SERVICE_DATA_INVALID;
    }
    return SENSOR_SAFE_SERVICE_OK;
}

/*
 * 函数用途：按当前时刻读取仍在新鲜度预算内的周期快照。
 * 调用场景：业务层消费最近一次已验收快报时调用。
 * 关键约束：读取失败时由 monitor 清零输出，服务层只映射失效类别。
 */
SensorSafeServiceResult SensorSafeService_ReadPeriodicSnapshot(SensorSafeServiceContext *context,
                                                               SensorSafePeriodicSnapshot *snapshot)
{
    SensorSafeResult monitor_result;
    uint32_t now_ms;

    if ((context == NULL) || (snapshot == NULL) || (context->client.transport.now_ms == NULL)) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    now_ms = context->client.transport.now_ms();
    monitor_result = SensorSafeMonitor_Read(&context->monitor,
                                            now_ms,
                                            context->client.config.max_data_age_ms,
                                            snapshot);
    if (monitor_result == SENSOR_SAFE_OK) {
        return SENSOR_SAFE_SERVICE_OK;
    }
    if (monitor_result == SENSOR_SAFE_DATA_STALE) {
        return SENSOR_SAFE_SERVICE_DATA_STALE;
    }
    return SENSOR_SAFE_SERVICE_DATA_INVALID;
}

/*
 * 函数用途：请求退出周期模式，并无条件撤销当前快照。
 * 调用场景：回到请求响应模式、命令切换或业务结束时调用。
 * 关键约束：即使停流响应失败也不得继续使用旧周期数据。
 */
SensorSafeServiceResult SensorSafeService_StopPeriodic(SensorSafeServiceContext *context)
{
    SensorSafeClientResult client_result;

    if (context == NULL) {
        return SENSOR_SAFE_SERVICE_INVALID_ARGUMENT;
    }
    client_result = SensorSafeClient_StopPeriodic(&context->client);
    context->last_client_result = client_result;
    SensorSafeMonitor_Invalidate(&context->monitor,
                                 (client_result == SENSOR_SAFE_CLIENT_OK)
                                     ? SENSOR_SAFE_STREAM_STOPPED
                                     : SensorSafeService_PeriodicFailureReason(context, client_result));
    return SensorSafeService_MapClientResult(client_result);
}
