#include "sensor_safe_client.h"

#include <limits.h>
#include <string.h>

#include "sensor_safe_crc32c.h"
#include "sensor_safe_frame.h"

#define SENSOR_SAFE_HELLO_REQUEST_LEN       12U
#define SENSOR_SAFE_HELLO_APPEND_LEN        52U
#define SENSOR_SAFE_CONFIG_APPEND_LEN       18U
#define SENSOR_SAFE_SET_COMM_REQUEST_LEN    14U
#define SENSOR_SAFE_SET_COMM_APPEND_LEN     16U

/* 已通过线格式、会话和公共载荷校验的控制响应视图；extra 指向 client 接收缓冲区。 */
typedef struct {
    SensorSafeControlFrame frame;
    SensorSafeCommonResponse common;
    const uint8_t *extra;
    uint16_t extra_len;
    uint32_t rx_timestamp_ms;
} SensorSafeClientResponse;

/* 参数分页响应的固定页头；参数记录暂存在调用方提供的三项页缓冲区。 */
typedef struct {
    uint16_t total_count;
    uint16_t next_index;
    uint8_t value_count;
    uint8_t last_page;
} SensorSafeClientParamPage;

/* 从协议冻结的小端线格式读取 16 位字段。 */
static uint16_t SensorSafeClient_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/* 从协议冻结的小端线格式读取 32 位字段。 */
static uint32_t SensorSafeClient_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/* 按位组合小端有符号 64 位参数值，避免未对齐访问和实现相关移位。 */
static int64_t SensorSafeClient_ReadI64(const uint8_t *data)
{
    uint64_t value = (uint64_t)SensorSafeClient_ReadU32(data) |
                     ((uint64_t)SensorSafeClient_ReadU32(&data[4]) << 32U);
    return (int64_t)value;
}

/* 把 16 位字段逐字节写入冻结线格式。 */
static void SensorSafeClient_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/* 把 32 位字段逐字节写入冻结线格式。 */
static void SensorSafeClient_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/* 保留二进制补码位模式写入 64 位参数字段。 */
static void SensorSafeClient_WriteI64(uint8_t *data, int64_t value)
{
    uint64_t bits = (uint64_t)value;
    SensorSafeClient_WriteU32(data, (uint32_t)(bits & 0xFFFFFFFFULL));
    SensorSafeClient_WriteU32(&data[4], (uint32_t)(bits >> 32U));
}

/* 接受协议卷定义的业务、诊断和故障模式，拒绝其余保留值。 */
static uint8_t SensorSafeClient_IsMeasureModeValid(uint8_t mode)
{
    return (uint8_t)(((mode <= (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM) ||
                      (mode == (uint8_t)SENSOR_SAFE_MEASURE_SELF_TEST) ||
                      (mode == (uint8_t)SENSOR_SAFE_MEASURE_FAULT)) ? 1U : 0U);
}

/*
 * 函数用途：把测量模式映射到 HELLO 声明的能力位。
 * 调用场景：发送模式切换或启动周期上报前的本地能力校验。
 * 关键约束：IDLE 不要求测量能力；未知模式返回 0，由调用方按参数错误拒绝。
 */
static uint32_t SensorSafeClient_ModeCapability(uint8_t mode)
{
    switch (mode) {
    case (uint8_t)SENSOR_SAFE_MEASURE_IDLE:
        return 0U;
    case (uint8_t)SENSOR_SAFE_MEASURE_DENSITY:
        return SENSOR_SAFE_CAP_DENSITY;
    case (uint8_t)SENSOR_SAFE_MEASURE_LEVEL:
        return SENSOR_SAFE_CAP_LEVEL;
    case (uint8_t)SENSOR_SAFE_MEASURE_WATER_CAP:
        return SENSOR_SAFE_CAP_WATER_CAP;
    case (uint8_t)SENSOR_SAFE_MEASURE_GYRO:
        return SENSOR_SAFE_CAP_GYRO;
    case (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM:
        return SENSOR_SAFE_CAP_BOTTOM;
    case (uint8_t)SENSOR_SAFE_MEASURE_SELF_TEST:
        return SENSOR_SAFE_CAP_SELF_TEST;
    default:
        return 0U;
    }
}

/* 诊断计数采用饱和加一，长稳运行后不得因回绕伪装成较小故障次数。 */
static void SensorSafeClient_IncrementCounter(uint32_t *counter)
{
    if ((counter != NULL) && (*counter != UINT32_MAX)) {
        (*counter)++;
    }
}

/* 清除只对当前周期流有效的协商值，诊断计数和会话防重放历史不受影响。 */
static void SensorSafeClient_ClearPeriodicAcceptance(SensorSafeClientContext *context)
{
    context->accepted_period_ms = 0U;
    context->accepted_max_silent_ms = 0U;
    context->accepted_control_guard_ms = 0U;
    context->accepted_start_after_ms = 0U;
    context->periodic_start_ms = 0U;
}

/* 计算首次快报或最近有效快报之后还剩多少协商静默预算。 */
static uint32_t SensorSafeClient_PeriodicRemainingMs(const SensorSafeClientContext *context,
                                                     uint32_t now_ms)
{
    uint32_t baseline_ms;
    uint32_t allowed_ms;
    uint32_t elapsed_ms;

    if (context->session.sample_counter_valid != 0U) {
        baseline_ms = context->session.last_valid_rx_ms;
        allowed_ms = (uint32_t)context->accepted_max_silent_ms;
    } else {
        baseline_ms = context->periodic_start_ms;
        allowed_ms = (uint32_t)context->accepted_start_after_ms +
                     (uint32_t)context->accepted_max_silent_ms;
    }
    elapsed_ms = now_ms - baseline_ms;
    return (elapsed_ms >= allowed_ms) ? 0U : (allowed_ms - elapsed_ms);
}

/* 把 frame/session 层结果稳定映射为 client 对外结果类别。 */
static SensorSafeClientResult SensorSafeClient_MapProtocolResult(SensorSafeResult result)
{
    if (result == SENSOR_SAFE_NEEDS_HELLO) {
        return SENSOR_SAFE_CLIENT_NEEDS_HELLO;
    }
    if (result == SENSOR_SAFE_DATA_STALE) {
        return SENSOR_SAFE_CLIENT_DATA_STALE;
    }
    if (result == SENSOR_SAFE_DATA_INVALID) {
        return SENSOR_SAFE_CLIENT_DATA_INVALID;
    }
    return SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
}

/* 可信远端错误中只有会话或配置身份失效允许进入单次 HELLO 恢复。 */
static uint8_t SensorSafeClient_WireResultNeedsHello(uint16_t wire_result)
{
    return (uint8_t)(((wire_result == (uint16_t)SENSOR_SAFE_WIRE_BAD_SESSION) ||
                      (wire_result == (uint16_t)SENSOR_SAFE_WIRE_CONFIG_MISMATCH))
                         ? 1U
                         : 0U);
}

/* 记录最近一次协议拒绝原因，并对 CRC 和地址类故障单独累计。 */
static void SensorSafeClient_RecordProtocolReject(SensorSafeClientContext *context,
                                                  SensorSafeResult result)
{
    context->last_protocol_result = result;
    SensorSafeClient_IncrementCounter(&context->counters.protocol_reject_count);
    if (result == SENSOR_SAFE_BAD_CRC) {
        SensorSafeClient_IncrementCounter(&context->counters.crc_reject_count);
    } else if (result == SENSOR_SAFE_BAD_ADDRESS) {
        SensorSafeClient_IncrementCounter(&context->counters.address_reject_count);
    }
}

/* 先记录具体协议原因，再返回对外 client 结果，避免上层丢失诊断证据。 */
static SensorSafeClientResult SensorSafeClient_FromProtocolResult(SensorSafeClientContext *context,
                                                                   SensorSafeResult result)
{
    SensorSafeClient_RecordProtocolReject(context, result);
    return SensorSafeClient_MapProtocolResult(result);
}

/*
 * 函数用途：校验所有控制响应共有的状态位、诊断位、模式和载荷版本。
 * 调用场景：会话字段通过后、命令专属 extra 载荷解析之前调用。
 * 关键约束：任何保留位或未知枚举均拒绝，避免不同协议版本产生歧义解释。
 */
static SensorSafeClientResult SensorSafeClient_CheckCommon(SensorSafeClientContext *context,
                                                           const SensorSafeCommonResponse *common)
{
    if ((common->status_flags & ~((uint32_t)SENSOR_SAFE_ALLOWED_FAST_STATUS16)) != 0U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_RESERVED_STATUS);
    }
    if ((common->diag_flags & ~((uint32_t)SENSOR_SAFE_ALLOWED_FAST_DIAG16)) != 0U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_RESERVED_STATUS);
    }
    if ((SensorSafeClient_IsMeasureModeValid(common->measure_mode) == 0U) ||
        (common->comm_mode > (uint8_t)SENSOR_SAFE_COMM_PERIODIC_UPLINK) ||
        (common->payload_version != 1U) ||
        (common->data_quality > 100U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_HEADER);
    }
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：执行普通控制命令的同序号有限次请求响应事务。
 * 调用场景：除 HELLO 外的测量、模式、配置、参数和诊断命令统一复用。
 * 关键约束：重试复用原 cmd/seq 并置 RETRY；只有绑定待决事务且公共字段合法的响应可结束事务。
 */
static SensorSafeClientResult SensorSafeClient_ExchangeCommand(SensorSafeClientContext *context,
                                                               uint8_t cmd,
                                                               uint16_t flags,
                                                               const uint8_t *payload,
                                                               uint16_t payload_len,
                                                               uint32_t timeout_ms,
                                                               SensorSafeClientResponse *response)
{
    SensorSafeControlFields fields;
    SensorSafeResult protocol_result;
    SensorSafeTransportResult transport_result = SENSOR_SAFE_TRANSPORT_TIMEOUT;
    uint32_t seq = 0U;
    uint8_t attempt;

    protocol_result = SensorSafeSession_BeginRequest(&context->session, cmd, &seq);
    if (protocol_result != SENSOR_SAFE_OK) {
        return SensorSafeClient_FromProtocolResult(context, protocol_result);
    }
    SensorSafeClient_IncrementCounter(&context->counters.control_request_count);

    /* 同一逻辑事务的所有尝试复用 seq，使传感器能够识别重复请求并抑制副作用。 */
    for (attempt = 0U; attempt < context->config.max_attempts; attempt++) {
        uint16_t tx_len = 0U;
        uint16_t rx_len = 0U;
        uint32_t rx_timestamp = 0U;

        SensorSafeClient_IncrementCounter(&context->counters.control_attempt_count);
        if (attempt != 0U) {
            SensorSafeClient_IncrementCounter(&context->counters.control_retry_count);
        }

        (void)memset(&fields, 0, sizeof(fields));
        fields.src_id = context->config.cpu_node_id;
        fields.dst_id = context->config.sensor_node_id;
        fields.msg_type = (uint8_t)SENSOR_SAFE_MSG_REQ;
        fields.cmd = cmd;
        fields.flags = (uint16_t)(flags | SENSOR_SAFE_FLAG_ACK_REQUIRED);
        if (attempt != 0U) {
            fields.flags = (uint16_t)(fields.flags | SENSOR_SAFE_FLAG_RETRY_FRAME);
        }
        fields.seq = seq;
        fields.session_id = context->session.session_id;
        fields.sensor_id = context->session.sensor_id;
        fields.safety_param_crc = context->session.safety_param_crc;
        fields.payload = payload;
        fields.payload_len = payload_len;

        protocol_result = SensorSafeFrame_EncodeControl(&fields,
                                                        context->tx_buffer,
                                                        sizeof(context->tx_buffer),
                                                        &tx_len);
        if (protocol_result != SENSOR_SAFE_OK) {
            (void)SensorSafeSession_CompleteRequest(&context->session);
            return SensorSafeClient_FromProtocolResult(context, protocol_result);
        }

        transport_result = context->transport.exchange(context->tx_buffer,
                                                       tx_len,
                                                       context->rx_buffer,
                                                       sizeof(context->rx_buffer),
                                                       &rx_len,
                                                       &rx_timestamp,
                                                       timeout_ms);
        context->last_transport_result = transport_result;
        if (transport_result != SENSOR_SAFE_TRANSPORT_OK) {
            SensorSafeClient_IncrementCounter(&context->counters.transport_failure_count);
            continue;
        }
        protocol_result = SensorSafeFrame_DecodeControl(context->rx_buffer, rx_len, &response->frame);
        if (protocol_result != SENSOR_SAFE_OK) {
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }
        protocol_result = SensorSafeSession_CheckControlResponse(&context->session, &response->frame);
        if (protocol_result != SENSOR_SAFE_OK) {
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }
        protocol_result = SensorSafeFrame_DecodeCommonResponse(response->frame.fields.payload,
                                                               response->frame.fields.payload_len,
                                                               &response->common);
        if (protocol_result != SENSOR_SAFE_OK) {
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }
        if (SensorSafeClient_CheckCommon(context, &response->common) != SENSOR_SAFE_CLIENT_OK) {
            continue;
        }

        response->extra = &response->frame.fields.payload[SENSOR_SAFE_COMMON_RESPONSE_LEN];
        response->extra_len = (uint16_t)(response->frame.fields.payload_len - SENSOR_SAFE_COMMON_RESPONSE_LEN);
        response->rx_timestamp_ms = rx_timestamp;
        context->last_rx_timestamp_ms = rx_timestamp;
        context->last_wire_result = response->common.result_code;
        (void)SensorSafeSession_CompleteRequest(&context->session);
        SensorSafeClient_IncrementCounter(&context->counters.control_response_count);
        /* 只有完整验收的远端 ERR 才能证明事务已被拒绝；超时仍属于结果不确定。 */
        if ((response->frame.fields.msg_type == (uint8_t)SENSOR_SAFE_MSG_ERR) ||
            (response->common.result_code != (uint16_t)SENSOR_SAFE_WIRE_OK)) {
            SensorSafeClient_IncrementCounter(&context->counters.remote_error_count);
            if (SensorSafeClient_WireResultNeedsHello(response->common.result_code) != 0U) {
                context->session.state = SENSOR_SAFE_SESSION_RECOVERING;
                return SENSOR_SAFE_CLIENT_NEEDS_HELLO;
            }
            return SENSOR_SAFE_CLIENT_REMOTE_ERROR;
        }
        return SENSOR_SAFE_CLIENT_OK;
    }

    (void)SensorSafeSession_CompleteRequest(&context->session);
    /* 尝试耗尽后旧会话可信度不足，禁止继续分配普通业务序号。 */
    context->session.state = SENSOR_SAFE_SESSION_RECOVERING;
    if (transport_result != SENSOR_SAFE_TRANSPORT_OK) {
        return SENSOR_SAFE_CLIENT_TRANSPORT_ERROR;
    }
    return SensorSafeClient_MapProtocolResult(context->last_protocol_result);
}

/*
 * 函数用途：校验请求响应测量的模式、状态、诊断、新鲜度和样本单调性。
 * 调用场景：各测量命令解析定点数据前共同调用。
 * 关键约束：每种测量使用独立样本槽；未通过全部检查前不得更新防重复基准。
 */
static SensorSafeClientResult SensorSafeClient_CheckMeasurement(SensorSafeClientContext *context,
                                                                const SensorSafeClientResponse *response,
                                                                uint8_t expected_mode,
                                                                uint16_t channel_diag_mask,
                                                                uint8_t sample_slot,
                                                                uint32_t *effective_age_ms)
{
    uint32_t now_ms = context->transport.now_ms();
    uint32_t age = SensorSafeSession_EffectiveAgeMs(response->common.data_age_ms,
                                                    response->rx_timestamp_ms,
                                                    now_ms);

    if ((response->common.measure_mode != expected_mode) ||
        (response->common.comm_mode != (uint8_t)SENSOR_SAFE_COMM_REQUEST_RESPONSE) ||
        ((response->common.status_flags & SENSOR_SAFE_STATUS_MODE_MATCH) == 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_MODE_MISMATCH);
    }
    if (((response->common.status_flags & SENSOR_SAFE_STATUS_DATA_VALID) == 0U) ||
        ((response->common.status_flags & (SENSOR_SAFE_STATUS_MODE_SETTLING |
                                           SENSOR_SAFE_STATUS_CONFIG_MISMATCH)) != 0U) ||
        ((response->common.diag_flags & ((uint32_t)SENSOR_SAFE_DIAG_GLOBAL_INVALID_MASK |
                                         (uint32_t)channel_diag_mask)) != 0U)) {
        return SENSOR_SAFE_CLIENT_DATA_INVALID;
    }
    if (((response->common.status_flags & SENSOR_SAFE_STATUS_DATA_STALE) != 0U) ||
        (age > context->config.max_data_age_ms)) {
        return SENSOR_SAFE_CLIENT_DATA_STALE;
    }
    if ((sample_slot >= 6U) ||
        ((context->last_control_sample_valid[sample_slot] != 0U) &&
         (response->common.sample_counter <= context->last_control_sample[sample_slot]))) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SAMPLE_COUNTER);
    }
    if (SensorSafeSession_SetMeasureReady(&context->session, expected_mode) != SENSOR_SAFE_OK) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SESSION);
    }

    context->last_control_sample[sample_slot] = response->common.sample_counter;
    context->last_control_sample_valid[sample_slot] = 1U;
    *effective_age_ms = age;
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：保存 client 配置和传输接口并初始化离线会话。
 * 调用场景：服务层完成持久化身份初始化后调用。
 * 关键约束：零超时或零尝试次数替换为受控默认值；本函数不访问总线。
 */
void SensorSafeClient_Init(SensorSafeClientContext *context,
                           const SensorSafeClientConfig *config,
                           const SensorSafeClientTransportOps *transport)
{
    if ((context == NULL) || (config == NULL) || (transport == NULL)) {
        return;
    }
    (void)memset(context, 0, sizeof(*context));
    context->config = *config;
    context->transport = *transport;
    if (context->config.request_timeout_ms == 0U) {
        context->config.request_timeout_ms = SENSOR_SAFE_CLIENT_DEFAULT_TIMEOUT_MS;
    }
    if (context->config.max_attempts == 0U) {
        context->config.max_attempts = SENSOR_SAFE_CLIENT_DEFAULT_MAX_ATTEMPTS;
    }
    SensorSafeSession_Init(&context->session,
                           context->config.cpu_node_id,
                           context->config.sensor_node_id);
}

/* 撤销 client 会话、周期协商值和本会话样本历史，但保留累计诊断计数。 */
void SensorSafeClient_Deactivate(SensorSafeClientContext *context)
{
    if (context == NULL) {
        return;
    }
    SensorSafeSession_Deactivate(&context->session);
    SensorSafeClient_ClearPeriodicAcceptance(context);
    (void)memset(&context->identity, 0, sizeof(context->identity));
    (void)memset(context->last_control_sample_valid, 0,
                 sizeof(context->last_control_sample_valid));
}

/*
 * 函数用途：发送 HELLO 挑战并验收传感器回显、身份、能力及单调会话计数。
 * 调用场景：首次探测和服务层允许的故障恢复。
 * 关键约束：同一 HELLO 的重试复用 nonce 和 seq；任何未绑定响应不得激活会话。
 */
SensorSafeClientResult SensorSafeClient_Hello(SensorSafeClientContext *context,
                                              uint32_t cpu_nonce,
                                              SensorSafeIdentity *identity_out)
{
    SensorSafeControlFields fields;
    SensorSafeClientResponse response;
    SensorSafeHelloAcceptance hello;
    SensorSafeResult protocol_result;
    uint8_t payload[SENSOR_SAFE_HELLO_REQUEST_LEN] = {0U};
    uint32_t seq = 0U;
    uint8_t attempt;

    if ((context == NULL) || (identity_out == NULL) ||
        (context->transport.exchange == NULL) || (context->transport.now_ms == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    protocol_result = SensorSafeSession_BeginHello(&context->session,
                                                   context->config.cpu_boot_counter,
                                                   cpu_nonce,
                                                   &seq);
    if (protocol_result != SENSOR_SAFE_OK) {
        return SensorSafeClient_FromProtocolResult(context, protocol_result);
    }
    SensorSafeClient_IncrementCounter(&context->counters.control_request_count);

    payload[0] = SENSOR_SAFE_PROTOCOL_VERSION;
    payload[1] = SENSOR_SAFE_PROTOCOL_VERSION;
    payload[2] = context->config.cpu_node_id;
    payload[3] = 0U;
    SensorSafeClient_WriteU32(&payload[4], context->config.cpu_boot_counter);
    SensorSafeClient_WriteU32(&payload[8], cpu_nonce);

    /* HELLO 重试保持同一挑战和序号，新的挑战只由上层发起新的 HELLO 事务。 */
    for (attempt = 0U; attempt < context->config.max_attempts; attempt++) {
        uint16_t tx_len = 0U;
        uint16_t rx_len = 0U;
        uint32_t rx_timestamp = 0U;

        SensorSafeClient_IncrementCounter(&context->counters.control_attempt_count);
        if (attempt != 0U) {
            SensorSafeClient_IncrementCounter(&context->counters.control_retry_count);
        }

        (void)memset(&fields, 0, sizeof(fields));
        fields.src_id = context->config.cpu_node_id;
        fields.dst_id = context->config.sensor_node_id;
        fields.msg_type = (uint8_t)SENSOR_SAFE_MSG_REQ;
        fields.cmd = (uint8_t)SENSOR_SAFE_CMD_HELLO;
        fields.flags = (uint16_t)(SENSOR_SAFE_FLAG_ACK_REQUIRED | SENSOR_SAFE_FLAG_COLD_START);
        if (attempt != 0U) {
            fields.flags = (uint16_t)(fields.flags | SENSOR_SAFE_FLAG_RETRY_FRAME);
        }
        fields.seq = seq;
        fields.payload = payload;
        fields.payload_len = sizeof(payload);

        protocol_result = SensorSafeFrame_EncodeControl(&fields,
                                                        context->tx_buffer,
                                                        sizeof(context->tx_buffer),
                                                        &tx_len);
        if (protocol_result != SENSOR_SAFE_OK) {
            break;
        }
        context->last_transport_result = context->transport.exchange(context->tx_buffer,
                                                                     tx_len,
                                                                     context->rx_buffer,
                                                                     sizeof(context->rx_buffer),
                                                                     &rx_len,
                                                                     &rx_timestamp,
                                                                     context->config.request_timeout_ms);
        if (context->last_transport_result != SENSOR_SAFE_TRANSPORT_OK) {
            SensorSafeClient_IncrementCounter(&context->counters.transport_failure_count);
            continue;
        }
        protocol_result = SensorSafeFrame_DecodeControl(context->rx_buffer, rx_len, &response.frame);
        if (protocol_result != SENSOR_SAFE_OK) {
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }
        protocol_result = SensorSafeSession_CheckControlResponse(&context->session, &response.frame);
        if (protocol_result != SENSOR_SAFE_OK) {
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }
        protocol_result = SensorSafeFrame_DecodeCommonResponse(response.frame.fields.payload,
                                                               response.frame.fields.payload_len,
                                                               &response.common);
        if (protocol_result != SENSOR_SAFE_OK) {
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }
        if (SensorSafeClient_CheckCommon(context, &response.common) != SENSOR_SAFE_CLIENT_OK) {
            continue;
        }
        context->last_wire_result = response.common.result_code;
        if ((response.frame.fields.msg_type == (uint8_t)SENSOR_SAFE_MSG_ERR) ||
            (response.common.result_code != (uint16_t)SENSOR_SAFE_WIRE_OK)) {
            (void)SensorSafeSession_CompleteRequest(&context->session);
            SensorSafeClient_IncrementCounter(&context->counters.control_response_count);
            SensorSafeClient_IncrementCounter(&context->counters.remote_error_count);
            return SENSOR_SAFE_CLIENT_REMOTE_ERROR;
        }
        if (response.frame.fields.payload_len != (SENSOR_SAFE_COMMON_RESPONSE_LEN + SENSOR_SAFE_HELLO_APPEND_LEN)) {
            protocol_result = SENSOR_SAFE_BAD_LENGTH;
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }

        response.extra = &response.frame.fields.payload[SENSOR_SAFE_COMMON_RESPONSE_LEN];
        (void)memset(&hello, 0, sizeof(hello));
        hello.selected_protocol = response.extra[0];
        hello.sensor_node_id = response.extra[1];
        hello.sensor_type = SensorSafeClient_ReadU16(&response.extra[2]);
        hello.sensor_id = SensorSafeClient_ReadU32(&response.extra[4]);
        context->identity.sensor_sw_version = SensorSafeClient_ReadU32(&response.extra[8]);
        context->identity.sensor_hw_version = SensorSafeClient_ReadU32(&response.extra[12]);
        hello.sensor_boot_counter = SensorSafeClient_ReadU32(&response.extra[16]);
        hello.sensor_session_counter = SensorSafeClient_ReadU32(&response.extra[20]);
        hello.echo_cpu_boot_counter = SensorSafeClient_ReadU32(&response.extra[24]);
        hello.echo_cpu_nonce = SensorSafeClient_ReadU32(&response.extra[28]);
        hello.new_session_id = SensorSafeClient_ReadU32(&response.extra[32]);
        hello.capability_flags = SensorSafeClient_ReadU32(&response.extra[36]);
        hello.scale_version = SensorSafeClient_ReadU16(&response.extra[40]);
        hello.calibration_version = SensorSafeClient_ReadU16(&response.extra[42]);
        hello.param_table_version = SensorSafeClient_ReadU16(&response.extra[44]);
        hello.digest_schema_version = SensorSafeClient_ReadU16(&response.extra[46]);
        hello.safety_param_crc = SensorSafeClient_ReadU32(&response.extra[48]);
        protocol_result = SensorSafeSession_AcceptHello(&context->session, &response.frame, &hello);
        if (protocol_result != SENSOR_SAFE_OK) {
            SensorSafeClient_RecordProtocolReject(context, protocol_result);
            continue;
        }

        context->identity.sensor_type = hello.sensor_type;
        context->identity.sensor_id = hello.sensor_id;
        context->identity.sensor_boot_counter = hello.sensor_boot_counter;
        context->identity.sensor_session_counter = hello.sensor_session_counter;
        context->identity.capability_flags = hello.capability_flags;
        context->identity.scale_version = hello.scale_version;
        context->identity.calibration_version = hello.calibration_version;
        context->identity.param_table_version = hello.param_table_version;
        context->identity.digest_schema_version = hello.digest_schema_version;
        context->identity.safety_param_crc = hello.safety_param_crc;
        SensorSafeClient_ClearPeriodicAcceptance(context);
        (void)memset(context->last_control_sample_valid, 0, sizeof(context->last_control_sample_valid));
        *identity_out = context->identity;
        context->last_rx_timestamp_ms = rx_timestamp;
        SensorSafeClient_IncrementCounter(&context->counters.control_response_count);
        return SENSOR_SAFE_CLIENT_OK;
    }

    if (protocol_result != SENSOR_SAFE_OK) {
        context->last_protocol_result = protocol_result;
    }
    (void)SensorSafeSession_CompleteRequest(&context->session);
    context->session.state = SENSOR_SAFE_SESSION_OFFLINE;
    if (context->last_transport_result != SENSOR_SAFE_TRANSPORT_OK) {
        return SENSOR_SAFE_CLIENT_TRANSPORT_ERROR;
    }
    return SensorSafeClient_MapProtocolResult(context->last_protocol_result);
}

/*
 * 函数用途：执行无副作用在线检查并返回传感器运行时间。
 * 调用场景：安全会话健康检查或周期流控制保护窗内诊断。
 * 关键约束：响应长度和公共状态必须通过统一校验后才写输出。
 */
SensorSafeClientResult SensorSafeClient_Ping(SensorSafeClientContext *context,
                                             uint32_t token,
                                             uint32_t *sensor_uptime_ms)
{
    uint8_t payload[4];
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (sensor_uptime_ms == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    SensorSafeClient_WriteU32(payload, token);
    result = SensorSafeClient_ExchangeCommand(context,
                                              (uint8_t)SENSOR_SAFE_CMD_PING,
                                              0U,
                                              payload,
                                              sizeof(payload),
                                              context->config.request_timeout_ms,
                                              &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) || (SensorSafeClient_ReadU32(response.extra) != token)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SEQUENCE);
    }
    *sensor_uptime_ms = SensorSafeClient_ReadU32(&response.extra[4]);
    return SENSOR_SAFE_CLIENT_OK;
}

/* 读取传感器公共状态及复位原因，供诊断流程判断当前运行态。 */
SensorSafeClientResult SensorSafeClient_GetStatus(SensorSafeClientContext *context,
                                                  SensorSafeStatus *status_out)
{
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (status_out == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    result = SensorSafeClient_ExchangeCommand(context,
                                              (uint8_t)SENSOR_SAFE_CMD_GET_STATUS,
                                              0U,
                                              NULL,
                                              0U,
                                              context->config.request_timeout_ms,
                                              &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if (response.extra_len != 0U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    status_out->common = response.common;
    status_out->rx_timestamp_ms = response.rx_timestamp_ms;
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：读取并核对安全参数摘要、参数表版本和配置代次。
 * 调用场景：HELLO 成功后由服务层立即调用，建立配置身份基线。
 * 关键约束：响应中的 safety_param_crc 必须与控制帧身份字段一致且非 0。
 */
SensorSafeClientResult SensorSafeClient_GetConfigDigest(SensorSafeClientContext *context,
                                                        SensorSafeConfigDigest *digest_out)
{
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (digest_out == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    result = SensorSafeClient_ExchangeCommand(context,
                                              (uint8_t)SENSOR_SAFE_CMD_GET_CONFIG_DIGEST,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT,
                                              NULL,
                                              0U,
                                              context->config.request_timeout_ms,
                                              &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if (response.extra_len != SENSOR_SAFE_CONFIG_APPEND_LEN) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    digest_out->common = response.common;
    digest_out->safety_param_crc = SensorSafeClient_ReadU32(&response.extra[0]);
    digest_out->capability_flags = SensorSafeClient_ReadU32(&response.extra[4]);
    digest_out->scale_version = SensorSafeClient_ReadU16(&response.extra[8]);
    digest_out->calibration_version = SensorSafeClient_ReadU16(&response.extra[10]);
    digest_out->param_table_version = SensorSafeClient_ReadU16(&response.extra[12]);
    digest_out->digest_schema_version = SensorSafeClient_ReadU16(&response.extra[14]);
    digest_out->config_epoch = SensorSafeClient_ReadU16(&response.extra[16]);
    if ((digest_out->safety_param_crc != context->session.safety_param_crc) ||
        (digest_out->capability_flags != context->identity.capability_flags) ||
        (digest_out->scale_version != context->identity.scale_version) ||
        (digest_out->calibration_version != context->identity.calibration_version) ||
        (digest_out->param_table_version != context->identity.param_table_version) ||
        (digest_out->digest_schema_version != context->identity.digest_schema_version) ||
        (digest_out->config_epoch == 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_CONFIG_EPOCH);
    }
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：请求传感器撤销当前会话并返回复位接受状态。
 * 调用场景：显式维护或不可继续使用当前会话时调用。
 * 关键约束：命令成功后本地也立即下线，后续业务必须重新 HELLO。
 */
SensorSafeClientResult SensorSafeClient_ResetSession(SensorSafeClientContext *context,
                                                     uint16_t reset_reason)
{
    uint8_t payload[4] = {0U};
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if (context == NULL) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    SensorSafeClient_WriteU16(payload, reset_reason);
    result = SensorSafeClient_ExchangeCommand(context,
                                              (uint8_t)SENSOR_SAFE_CMD_RESET_SESSION,
                                              0U,
                                              payload,
                                              sizeof(payload),
                                              context->config.request_timeout_ms,
                                              &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) || (SensorSafeClient_ReadU32(response.extra) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SESSION);
    }
    context->session.state = SENSOR_SAFE_SESSION_OFFLINE;
    context->session.session_id = 0U;
    context->session.stream_valid = 0U;
    SensorSafeClient_ClearPeriodicAcceptance(context);
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：切换测量模式并验收模式回显、能力位和稳定等待时间。
 * 调用场景：密度、液位、水位电容、陀螺仪等测量前调用。
 * 关键约束：未声明能力的模式在发送前拒绝；模式未匹配时不得进入 MEASURE_READY。
 */
SensorSafeClientResult SensorSafeClient_SetMeasureMode(SensorSafeClientContext *context,
                                                       uint8_t target_mode,
                                                       uint16_t *settle_time_ms)
{
    uint8_t payload[4] = {0U};
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (settle_time_ms == NULL) ||
        (target_mode > (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((target_mode != (uint8_t)SENSOR_SAFE_MEASURE_IDLE) &&
        ((context->identity.capability_flags & SensorSafeClient_ModeCapability(target_mode)) == 0U)) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    payload[0] = target_mode;
    result = SensorSafeClient_ExchangeCommand(context,
                                              (uint8_t)SENSOR_SAFE_CMD_SET_MEASURE_MODE,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT,
                                              payload,
                                              sizeof(payload),
                                              SENSOR_SAFE_CLIENT_MODE_TIMEOUT_MS,
                                              &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) || (response.extra[0] != target_mode) ||
        (response.extra[1] != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_MODE_MISMATCH);
    }
    if ((response.common.measure_mode != target_mode) ||
        (response.common.comm_mode != (uint8_t)SENSOR_SAFE_COMM_REQUEST_RESPONSE) ||
        ((response.common.status_flags & SENSOR_SAFE_STATUS_MODE_MATCH) == 0U) ||
        ((response.common.status_flags & (SENSOR_SAFE_STATUS_MODE_SETTLING |
                                          SENSOR_SAFE_STATUS_CONFIG_MISMATCH)) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_MODE_MISMATCH);
    }
    if (SensorSafeSession_SetMeasureReady(&context->session, target_mode) != SENSOR_SAFE_OK) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SESSION);
    }
    *settle_time_ms = SensorSafeClient_ReadU16(&response.extra[2]);
    return SENSOR_SAFE_CLIENT_OK;
}

/* 读取密度模式的频率、密度、温度组合值，并执行样本单调性与新鲜度检查。 */
SensorSafeClientResult SensorSafeClient_ReadMeasurementCombo(SensorSafeClientContext *context,
                                                             SensorSafeMeasurementCombo *measurement)
{
    uint8_t payload[4] = {0U};
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (measurement == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_MEAS_COMBO) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    result = SensorSafeClient_ExchangeCommand(context,
                                              (uint8_t)SENSOR_SAFE_CMD_READ_MEAS_COMBO,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT,
                                              payload,
                                              sizeof(payload),
                                              context->config.request_timeout_ms,
                                              &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 24U) || (SensorSafeClient_ReadU16(&response.extra[22]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    measurement->temperature_c_x100 = (int32_t)SensorSafeClient_ReadU32(&response.extra[0]);
    measurement->density_kg_m3_x100 = (int32_t)SensorSafeClient_ReadU32(&response.extra[4]);
    measurement->frequency_hz_x1000 = SensorSafeClient_ReadU32(&response.extra[8]);
    measurement->freq_45_hz_x1000 = SensorSafeClient_ReadU32(&response.extra[12]);
    measurement->freq_225_hz_x1000 = SensorSafeClient_ReadU32(&response.extra[16]);
    measurement->signal_quality = SensorSafeClient_ReadU16(&response.extra[20]);
    if (measurement->signal_quality > 1000U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_DATA_INVALID);
    }
    result = SensorSafeClient_CheckMeasurement(context,
                                               &response,
                                               (uint8_t)SENSOR_SAFE_MEASURE_DENSITY,
                                               (uint16_t)SENSOR_SAFE_DIAG_DENSITY_CHANNEL_ERROR,
                                               0U,
                                               &measurement->effective_age_ms);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    measurement->sample_counter = response.common.sample_counter;
    return SENSOR_SAFE_CLIENT_OK;
}

/* 读取液位频率定点值，只有液位通道诊断和样本检查全部通过才写输出。 */
SensorSafeClientResult SensorSafeClient_ReadLevelFrequency(SensorSafeClientContext *context,
                                                          SensorSafeLevelMeasurement *measurement)
{
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (measurement == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_LEVEL) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_READ_LEVEL_FREQ,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT, NULL, 0U,
                                              context->config.request_timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 12U) || (SensorSafeClient_ReadU16(&response.extra[10]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    measurement->level_frequency_hz_x1000 = SensorSafeClient_ReadU32(&response.extra[0]);
    measurement->signal_quality = SensorSafeClient_ReadU16(&response.extra[4]);
    measurement->valid_min_hz = SensorSafeClient_ReadU16(&response.extra[6]);
    measurement->valid_max_hz = SensorSafeClient_ReadU16(&response.extra[8]);
    if ((measurement->signal_quality > 1000U) ||
        (measurement->valid_min_hz > measurement->valid_max_hz) ||
        (measurement->level_frequency_hz_x1000 < ((uint32_t)measurement->valid_min_hz * 1000U)) ||
        (measurement->level_frequency_hz_x1000 > ((uint32_t)measurement->valid_max_hz * 1000U))) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_DATA_INVALID);
    }
    result = SensorSafeClient_CheckMeasurement(context, &response,
                                               (uint8_t)SENSOR_SAFE_MEASURE_LEVEL,
                                               (uint16_t)SENSOR_SAFE_DIAG_LEVEL_CHANNEL_ERROR,
                                               1U, &measurement->effective_age_ms);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    measurement->sample_counter = response.common.sample_counter;
    return SENSOR_SAFE_CLIENT_OK;
}

/* 读取水位电容定点值；未声明 WATER_CAP 能力时不发送总线请求。 */
SensorSafeClientResult SensorSafeClient_ReadWaterCap(SensorSafeClientContext *context,
                                                     SensorSafeWaterCapMeasurement *measurement)
{
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (measurement == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_WATER_CAP) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_READ_WATER_CAP,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT, NULL, 0U,
                                              context->config.request_timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) || (SensorSafeClient_ReadU16(&response.extra[6]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    measurement->capacitance_pf_x10 = (int32_t)SensorSafeClient_ReadU32(&response.extra[0]);
    measurement->signal_quality = SensorSafeClient_ReadU16(&response.extra[4]);
    if (measurement->signal_quality > 1000U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_DATA_INVALID);
    }
    result = SensorSafeClient_CheckMeasurement(context, &response,
                                               (uint8_t)SENSOR_SAFE_MEASURE_WATER_CAP,
                                               (uint16_t)SENSOR_SAFE_DIAG_WATER_CAP_ERROR,
                                               2U, &measurement->effective_age_ms);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    measurement->sample_counter = response.common.sample_counter;
    return SENSOR_SAFE_CLIENT_OK;
}

/* 读取双轴姿态定点值；未声明 GYRO 能力时不发送总线请求。 */
SensorSafeClientResult SensorSafeClient_ReadGyro(SensorSafeClientContext *context,
                                                 SensorSafeGyroMeasurement *measurement)
{
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (measurement == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_GYRO) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_READ_GYRO,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT, NULL, 0U,
                                              context->config.request_timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 12U) || (SensorSafeClient_ReadU16(&response.extra[10]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    result = SensorSafeClient_CheckMeasurement(context, &response,
                                               (uint8_t)SENSOR_SAFE_MEASURE_GYRO,
                                               (uint16_t)SENSOR_SAFE_DIAG_GYRO_CHANNEL_ERROR,
                                               3U, &measurement->effective_age_ms);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    measurement->angle_x_deg_x100 = (int32_t)SensorSafeClient_ReadU32(&response.extra[0]);
    measurement->angle_y_deg_x100 = (int32_t)SensorSafeClient_ReadU32(&response.extra[4]);
    measurement->gyro_status = SensorSafeClient_ReadU16(&response.extra[8]);
    measurement->sample_counter = response.common.sample_counter;
    return SENSOR_SAFE_CLIENT_OK;
}

/* 读取电源电压、电流及电源状态，作为诊断数据而非直接控制依据。 */
SensorSafeClientResult SensorSafeClient_ReadPower(SensorSafeClientContext *context,
                                                  SensorSafePowerMeasurement *measurement)
{
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (measurement == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_POWER) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_READ_POWER,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT, NULL, 0U,
                                              context->config.request_timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) || (SensorSafeClient_ReadU16(&response.extra[6]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    result = SensorSafeClient_CheckMeasurement(context, &response,
                                               response.common.measure_mode,
                                               (uint16_t)(SENSOR_SAFE_DIAG_POWER_LOW | SENSOR_SAFE_DIAG_POWER_HIGH),
                                               4U, &measurement->effective_age_ms);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    measurement->voltage_mv = SensorSafeClient_ReadU32(&response.extra[0]);
    measurement->power_status = SensorSafeClient_ReadU16(&response.extra[4]);
    measurement->sample_counter = response.common.sample_counter;
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：请求进入周期主动上报并锁存传感器实际接受的时序参数。
 * 调用场景：上层显式选择周期模式且已处于对应测量就绪态时调用。
 * 关键约束：周期、静默预算、保护窗、stream_id、配置代次和模式必须逐项回显一致。
 */
SensorSafeClientResult SensorSafeClient_StartPeriodic(SensorSafeClientContext *context,
                                                      uint16_t period_ms,
                                                      uint16_t max_silent_ms,
                                                      uint16_t stream_id,
                                                      uint16_t control_guard_ms,
                                                      uint8_t measure_mode,
                                                      uint16_t *start_after_ms)
{
    uint8_t payload[SENSOR_SAFE_SET_COMM_REQUEST_LEN] = {0U};
    SensorSafeClientResponse response;
    SensorSafeClientResult result;
    SensorSafeResult stream_result;
    uint16_t accepted_period_ms;
    uint16_t accepted_max_silent_ms;
    uint16_t accepted_stream_id;
    uint16_t accepted_control_guard_ms;
    uint16_t accepted_epoch;

    if ((context == NULL) || (start_after_ms == NULL) ||
        (period_ms < SENSOR_SAFE_CLIENT_MIN_PERIOD_MS) ||
        (period_ms > SENSOR_SAFE_CLIENT_MAX_PERIOD_MS) ||
        ((uint32_t)max_silent_ms < ((uint32_t)period_ms * 3U)) ||
        (stream_id == 0U) ||
        (control_guard_ms < SENSOR_SAFE_CLIENT_MIN_CONTROL_GUARD_MS) ||
        (measure_mode > (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_PERIODIC_UPLINK) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    if ((measure_mode != (uint8_t)SENSOR_SAFE_MEASURE_IDLE) &&
        ((context->identity.capability_flags & SensorSafeClient_ModeCapability(measure_mode)) == 0U)) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    payload[0] = (uint8_t)SENSOR_SAFE_COMM_PERIODIC_UPLINK;
    payload[1] = (uint8_t)SENSOR_SAFE_CMD_REPORT_MEAS_FAST;
    SensorSafeClient_WriteU16(&payload[4], period_ms);
    SensorSafeClient_WriteU16(&payload[6], max_silent_ms);
    SensorSafeClient_WriteU16(&payload[8], stream_id);
    SensorSafeClient_WriteU16(&payload[10], control_guard_ms);

    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_SET_COMM_MODE,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT, payload, sizeof(payload),
                                              SENSOR_SAFE_CLIENT_MODE_TIMEOUT_MS, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if (response.extra_len != SENSOR_SAFE_SET_COMM_APPEND_LEN) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    accepted_period_ms = SensorSafeClient_ReadU16(&response.extra[2]);
    accepted_max_silent_ms = SensorSafeClient_ReadU16(&response.extra[4]);
    accepted_stream_id = SensorSafeClient_ReadU16(&response.extra[6]);
    accepted_control_guard_ms = SensorSafeClient_ReadU16(&response.extra[8]);
    /* 以远端明确回显的接受值建立静默和退出窗口，不能沿用本地请求假设。 */
    if ((response.extra[0] != (uint8_t)SENSOR_SAFE_COMM_PERIODIC_UPLINK) ||
        (response.extra[1] != (uint8_t)SENSOR_SAFE_CMD_REPORT_MEAS_FAST) ||
        (accepted_period_ms < SENSOR_SAFE_CLIENT_MIN_PERIOD_MS) ||
        (accepted_period_ms > SENSOR_SAFE_CLIENT_MAX_PERIOD_MS) ||
        ((uint32_t)accepted_max_silent_ms < ((uint32_t)accepted_period_ms * 3U)) ||
        (accepted_stream_id != stream_id) ||
        (accepted_control_guard_ms < SENSOR_SAFE_CLIENT_MIN_CONTROL_GUARD_MS) ||
        (SensorSafeClient_ReadU16(&response.extra[14]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_STREAM_STATE);
    }
    accepted_epoch = SensorSafeClient_ReadU16(&response.extra[10]);
    *start_after_ms = SensorSafeClient_ReadU16(&response.extra[12]);
    if ((response.common.measure_mode != measure_mode) ||
        (response.common.comm_mode != (uint8_t)SENSOR_SAFE_COMM_PERIODIC_UPLINK) ||
        ((response.common.status_flags & SENSOR_SAFE_STATUS_MODE_MATCH) == 0U) ||
        ((response.common.status_flags & (SENSOR_SAFE_STATUS_MODE_SETTLING |
                                          SENSOR_SAFE_STATUS_CONFIG_MISMATCH)) != 0U) ||
        ((response.common.status_flags & SENSOR_SAFE_STATUS_STREAM_ACTIVE) == 0U) ||
        (SensorSafeSession_SetMeasureReady(&context->session, measure_mode) != SENSOR_SAFE_OK)) {
        SensorSafeSession_InvalidateStream(&context->session);
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_MODE_MISMATCH);
    }
    stream_result = SensorSafeSession_StartStream(&context->session,
                                                  accepted_stream_id,
                                                  accepted_epoch,
                                                  measure_mode);
    if (stream_result != SENSOR_SAFE_OK) {
        return SensorSafeClient_FromProtocolResult(context, stream_result);
    }
    context->accepted_period_ms = accepted_period_ms;
    context->accepted_max_silent_ms = accepted_max_silent_ms;
    context->accepted_control_guard_ms = accepted_control_guard_ms;
    context->accepted_start_after_ms = *start_after_ms;
    context->periodic_start_ms = response.rx_timestamp_ms;
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：在协商静默预算内接收并验收一帧周期快报。
 * 调用场景：周期模式轮询任务调用，每次最多交付一帧。
 * 关键约束：接收等待不超过剩余静默预算；迟到、跳号或身份异常立即使流失效。
 */
SensorSafeClientResult SensorSafeClient_PollPeriodic(SensorSafeClientContext *context,
                                                     SensorSafeFastReport *report,
                                                     uint32_t timeout_ms)
{
    uint16_t frame_len = 0U;
    uint32_t rx_timestamp = 0U;
    uint32_t remaining_ms;
    uint32_t receive_timeout_ms;
    SensorSafeResult protocol_result;

    if ((context == NULL) || (report == NULL) ||
        (context->transport.receive == NULL) || (timeout_ms == 0U)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->session.state != SENSOR_SAFE_SESSION_PERIODIC_ACTIVE) ||
        (context->session.stream_valid == 0U) ||
        (context->accepted_max_silent_ms == 0U)) {
        return SENSOR_SAFE_CLIENT_NEEDS_HELLO;
    }
    remaining_ms = SensorSafeClient_PeriodicRemainingMs(context,
                                                        context->transport.now_ms());
    if (remaining_ms == 0U) {
        context->last_protocol_result = SENSOR_SAFE_DATA_STALE;
        SensorSafeSession_InvalidateStream(&context->session);
        return SENSOR_SAFE_CLIENT_DATA_STALE;
    }
    /* 调用方超时只能缩短等待，不能突破与传感器协商的最大静默安全边界。 */
    receive_timeout_ms = (timeout_ms < remaining_ms) ? timeout_ms : remaining_ms;
    SensorSafeClient_IncrementCounter(&context->counters.fast_report_receive_count);
    context->last_transport_result = context->transport.receive(context->rx_buffer,
                                                                sizeof(context->rx_buffer),
                                                                &frame_len,
                                                                &rx_timestamp,
                                                                receive_timeout_ms);
    if (context->last_transport_result != SENSOR_SAFE_TRANSPORT_OK) {
        SensorSafeClient_IncrementCounter(&context->counters.transport_failure_count);
        if ((context->last_transport_result == SENSOR_SAFE_TRANSPORT_TIMEOUT) &&
            (SensorSafeClient_PeriodicRemainingMs(context,
                                                  context->transport.now_ms()) != 0U)) {
            return SENSOR_SAFE_CLIENT_TRANSPORT_ERROR;
        }
        if (context->last_transport_result == SENSOR_SAFE_TRANSPORT_TIMEOUT) {
            context->last_protocol_result = SENSOR_SAFE_DATA_STALE;
            SensorSafeSession_InvalidateStream(&context->session);
            return SENSOR_SAFE_CLIENT_DATA_STALE;
        }
        SensorSafeSession_InvalidateStream(&context->session);
        return SENSOR_SAFE_CLIENT_TRANSPORT_ERROR;
    }
    /*
     * 底层即使因调度抖动晚于超时才返回成功，也不得让迟到快报跨过协商静默边界。
     * 使用实际接收时间戳二次判定，避免仅依赖传输层严格兑现等待超时。
     */
    if (SensorSafeClient_PeriodicRemainingMs(context, rx_timestamp) == 0U) {
        context->last_protocol_result = SENSOR_SAFE_DATA_STALE;
        SensorSafeSession_InvalidateStream(&context->session);
        return SENSOR_SAFE_CLIENT_DATA_STALE;
    }
    protocol_result = SensorSafeFrame_DecodeFastReport(context->rx_buffer, frame_len, report);
    if (protocol_result != SENSOR_SAFE_OK) {
        SensorSafeSession_InvalidateStream(&context->session);
        return SensorSafeClient_FromProtocolResult(context, protocol_result);
    }
    protocol_result = SensorSafeSession_AcceptFastReport(&context->session,
                                                        report,
                                                        rx_timestamp,
                                                        context->config.max_data_age_ms);
    if (protocol_result != SENSOR_SAFE_OK) {
        return SensorSafeClient_FromProtocolResult(context, protocol_result);
    }
    context->last_rx_timestamp_ms = rx_timestamp;
    SensorSafeClient_IncrementCounter(&context->counters.fast_report_accept_count);
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：只在最近快报后的控制保护窗内请求退出周期模式。
 * 调用场景：业务结束或需要返回请求响应模式时调用。
 * 关键约束：预留完整发送预算；窗口关闭或响应异常时流立即失效，禁止假定已停流。
 */
SensorSafeClientResult SensorSafeClient_StopPeriodic(SensorSafeClientContext *context)
{
    uint8_t payload[SENSOR_SAFE_SET_COMM_REQUEST_LEN] = {0U};
    SensorSafeClientResponse response;
    SensorSafeClientResult result;
    uint32_t elapsed_ms;
    uint32_t latest_send_offset_ms;

    if ((context == NULL) || (context->transport.now_ms == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->session.state != SENSOR_SAFE_SESSION_PERIODIC_ACTIVE) ||
        (context->session.stream_valid == 0U) ||
        (context->session.sample_counter_valid == 0U) ||
        (context->accepted_control_guard_ms < SENSOR_SAFE_CLIENT_MIN_CONTROL_GUARD_MS) ||
        (context->accepted_control_guard_ms < SENSOR_SAFE_CLIENT_EXIT_REQUEST_BUDGET_MS)) {
        return SENSOR_SAFE_CLIENT_CONTROL_WINDOW_CLOSED;
    }
    elapsed_ms = context->transport.now_ms() - context->session.last_valid_rx_ms;
    latest_send_offset_ms = (uint32_t)context->accepted_control_guard_ms -
                            SENSOR_SAFE_CLIENT_EXIT_REQUEST_BUDGET_MS;
    /* 超过最晚发送点不再抢占总线，避免退出命令与下一主动快报碰撞。 */
    if (elapsed_ms > latest_send_offset_ms) {
        return SENSOR_SAFE_CLIENT_CONTROL_WINDOW_CLOSED;
    }
    payload[0] = (uint8_t)SENSOR_SAFE_COMM_REQUEST_RESPONSE;
    SensorSafeClient_WriteU16(&payload[8], context->session.stream_id);
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_SET_COMM_MODE,
                                              SENSOR_SAFE_FLAG_STOP_STREAM_REQUEST,
                                              payload, sizeof(payload),
                                              SENSOR_SAFE_CLIENT_MODE_TIMEOUT_MS, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        SensorSafeSession_InvalidateStream(&context->session);
        return result;
    }
    if ((response.extra_len != SENSOR_SAFE_SET_COMM_APPEND_LEN) ||
        (response.common.measure_mode != context->session.expected_measure_mode) ||
        (response.common.comm_mode != (uint8_t)SENSOR_SAFE_COMM_REQUEST_RESPONSE) ||
        ((response.common.status_flags & SENSOR_SAFE_STATUS_MODE_MATCH) == 0U) ||
        ((response.common.status_flags & (SENSOR_SAFE_STATUS_MODE_SETTLING |
                                          SENSOR_SAFE_STATUS_CONFIG_MISMATCH |
                                          SENSOR_SAFE_STATUS_STREAM_ACTIVE)) != 0U) ||
        (response.extra[0] != (uint8_t)SENSOR_SAFE_COMM_REQUEST_RESPONSE) ||
        (response.extra[1] != 0U) ||
        (SensorSafeClient_ReadU16(&response.extra[2]) != 0U) ||
        (SensorSafeClient_ReadU16(&response.extra[4]) != 0U) ||
        (SensorSafeClient_ReadU16(&response.extra[6]) != 0U) ||
        (SensorSafeClient_ReadU16(&response.extra[8]) != 0U) ||
        (SensorSafeClient_ReadU16(&response.extra[10]) != context->session.config_epoch) ||
        (SensorSafeClient_ReadU16(&response.extra[12]) != 0U) ||
        (SensorSafeClient_ReadU16(&response.extra[14]) != 0U)) {
        SensorSafeSession_InvalidateStream(&context->session);
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_STREAM_STATE);
    }
    if (SensorSafeSession_StopStream(&context->session) != SENSOR_SAFE_OK) {
        return SENSOR_SAFE_CLIENT_PROTOCOL_ERROR;
    }
    SensorSafeClient_ClearPeriodicAcceptance(context);
    return SENSOR_SAFE_CLIENT_OK;
}

/* 按参数表版本和冻结字段顺序计算单项参数 CRC32C，禁止直接校验结构体内存。 */
static uint32_t SensorSafeClient_ParamCrc(uint16_t schema_version,
                                         const SensorSafeParameterValue *value)
{
    uint8_t normalized[20];

    SensorSafeClient_WriteU16(&normalized[0], schema_version);
    SensorSafeClient_WriteU16(&normalized[2], value->param_id);
    SensorSafeClient_WriteU16(&normalized[4], value->param_index);
    normalized[6] = value->param_type;
    normalized[7] = value->access;
    SensorSafeClient_WriteU16(&normalized[8], (uint16_t)value->scale);
    SensorSafeClient_WriteU16(&normalized[10], value->unit_id);
    SensorSafeClient_WriteI64(&normalized[12], value->value_i64);
    return SensorSafeCrc32c_Calculate(normalized, sizeof(normalized));
}

static uint8_t SensorSafeClient_IsParamSemanticValid(const SensorSafeParameterValue *value);

/* 从 22 字节参数记录解码解释字段，并在返回前完成单项 CRC 验收。 */
static SensorSafeClientResult SensorSafeClient_DecodeParamRecord(
    SensorSafeClientContext *context,
    const uint8_t *record,
    SensorSafeParameterValue *value_out)
{
    value_out->param_id = SensorSafeClient_ReadU16(&record[0]);
    value_out->param_index = SensorSafeClient_ReadU16(&record[2]);
    value_out->param_type = record[4];
    value_out->access = record[5];
    value_out->scale = (int16_t)SensorSafeClient_ReadU16(&record[6]);
    value_out->unit_id = SensorSafeClient_ReadU16(&record[8]);
    value_out->value_i64 = SensorSafeClient_ReadI64(&record[10]);
    value_out->param_crc = SensorSafeClient_ReadU32(&record[18]);
    if (SensorSafeClient_IsParamSemanticValid(value_out) == 0U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_INVALID_ARGUMENT);
    }
    if (SensorSafeClient_ParamCrc(context->identity.param_table_version, value_out) !=
        value_out->param_crc) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_PARAM_CRC);
    }
    return SENSOR_SAFE_CLIENT_OK;
}

/* 判断当前参数键是否严格位于上一参数键之后，用于拒绝重复项和乱序页。 */
static uint8_t SensorSafeClient_IsParamAfter(const SensorSafeParameterValue *previous,
                                             const SensorSafeParameterValue *current)
{
    return (uint8_t)(((current->param_id > previous->param_id) ||
                      ((current->param_id == previous->param_id) &&
                       (current->param_index > previous->param_index)))
                         ? 1U
                         : 0U);
}

/* 校验参数类型、访问属性和值域，阻止未知解释或无符号负值进入完整参数表。 */
static uint8_t SensorSafeClient_IsParamSemanticValid(const SensorSafeParameterValue *value)
{
    if ((value->param_type < (uint8_t)SENSOR_SAFE_PARAM_TYPE_BOOLEAN) ||
        (value->param_type > (uint8_t)SENSOR_SAFE_PARAM_TYPE_BITMASK) ||
        ((value->access & SENSOR_SAFE_PARAM_ACCESS_READ) == 0U) ||
        ((value->access & (uint8_t)(~SENSOR_SAFE_PARAM_ACCESS_ALLOWED)) != 0U)) {
        return 0U;
    }
    if ((value->param_type == (uint8_t)SENSOR_SAFE_PARAM_TYPE_BOOLEAN) &&
        (value->value_i64 != 0LL) && (value->value_i64 != 1LL)) {
        return 0U;
    }
    if (((value->param_type == (uint8_t)SENSOR_SAFE_PARAM_TYPE_UNSIGNED) ||
         (value->param_type == (uint8_t)SENSOR_SAFE_PARAM_TYPE_ENUM) ||
         (value->param_type == (uint8_t)SENSOR_SAFE_PARAM_TYPE_BITMASK)) &&
        (value->value_i64 < 0LL)) {
        return 0U;
    }
    return 1U;
}

/*
 * 函数用途：读取参数表的一个有界页并验证页游标、总数、末页标志和逐项 CRC。
 * 调用场景：完整参数表读取接口内部循环调用，不直接暴露给业务层。
 * 关键约束：页内最多三项且必须严格升序；非末页不得返回空页或停滞游标。
 */
static SensorSafeClientResult SensorSafeClient_ReadParamPage(
    SensorSafeClientContext *context,
    uint16_t start_index,
    uint16_t expected_total_count,
    SensorSafeParameterValue values[SENSOR_SAFE_PARAM_PAGE_MAX_ITEMS],
    SensorSafeClientParamPage *page_out)
{
    uint8_t payload[8];
    SensorSafeClientResponse response;
    SensorSafeClientResult result;
    uint16_t expected_length;
    uint16_t record_offset;
    uint8_t page_flags;
    uint8_t index;
    uint32_t expected_next_index;

    SensorSafeClient_WriteU16(&payload[0], start_index);
    payload[2] = SENSOR_SAFE_PARAM_PAGE_MAX_ITEMS;
    payload[3] = 0U;
    SensorSafeClient_WriteU16(&payload[4], context->identity.param_table_version);
    SensorSafeClient_WriteU16(&payload[6], expected_total_count);
    result = SensorSafeClient_ExchangeCommand(context,
                                              (uint8_t)SENSOR_SAFE_CMD_READ_PARAM_PAGE,
                                              0U,
                                              payload,
                                              sizeof(payload),
                                              context->config.request_timeout_ms,
                                              &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if (response.extra_len < SENSOR_SAFE_PARAM_PAGE_HEADER_LEN) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }

    page_out->total_count = SensorSafeClient_ReadU16(&response.extra[0]);
    page_out->next_index = SensorSafeClient_ReadU16(&response.extra[2]);
    page_out->value_count = response.extra[4];
    page_flags = response.extra[5];
    page_out->last_page = (uint8_t)(((page_flags & SENSOR_SAFE_PARAM_PAGE_FLAG_LAST) != 0U)
                                        ? 1U
                                        : 0U);
    expected_length = (uint16_t)(SENSOR_SAFE_PARAM_PAGE_HEADER_LEN +
                                 ((uint16_t)page_out->value_count * SENSOR_SAFE_PARAM_RECORD_LEN));
    if ((page_out->value_count > SENSOR_SAFE_PARAM_PAGE_MAX_ITEMS) ||
        (page_out->total_count > SENSOR_SAFE_PARAM_TABLE_MAX_ITEMS) ||
        ((page_flags & (uint8_t)(~SENSOR_SAFE_PARAM_PAGE_ALLOWED_FLAGS)) != 0U) ||
        (response.extra_len != expected_length) ||
        ((expected_total_count != SENSOR_SAFE_PARAM_TOTAL_UNKNOWN) &&
         (page_out->total_count != expected_total_count))) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }

    expected_next_index = (uint32_t)start_index + (uint32_t)page_out->value_count;
    if ((expected_next_index > UINT16_MAX) ||
        (page_out->next_index != (uint16_t)expected_next_index) ||
        (page_out->next_index > page_out->total_count) ||
        ((page_out->last_page != 0U) && (page_out->next_index != page_out->total_count)) ||
        ((page_out->last_page == 0U) &&
         ((page_out->value_count == 0U) || (page_out->next_index >= page_out->total_count))) ||
        ((page_out->value_count == 0U) &&
         ((start_index != 0U) || (page_out->total_count != 0U) ||
          (page_out->last_page == 0U)))) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SEQUENCE);
    }

    record_offset = SENSOR_SAFE_PARAM_PAGE_HEADER_LEN;
    for (index = 0U; index < page_out->value_count; index++) {
        result = SensorSafeClient_DecodeParamRecord(context,
                                                    &response.extra[record_offset],
                                                    &values[index]);
        if (result != SENSOR_SAFE_CLIENT_OK) {
            return result;
        }
        if ((index != 0U) &&
            (SensorSafeClient_IsParamAfter(&values[index - 1U], &values[index]) == 0U)) {
            return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SEQUENCE);
        }
        record_offset = (uint16_t)(record_offset + SENSOR_SAFE_PARAM_RECORD_LEN);
    }
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：按参数 ID 和索引读取单项参数并核对参数级 CRC。
 * 调用场景：维护诊断明确请求参数读取时调用，基础测量流程不使用。
 * 关键约束：能力位、回显标识和 CRC 任一不符时不接受返回值。
 */
SensorSafeClientResult SensorSafeClient_ReadParam(SensorSafeClientContext *context,
                                                  uint16_t param_id,
                                                  uint16_t param_index,
                                                  SensorSafeParameterValue *value_out)
{
    uint8_t payload[4];
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (value_out == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_PARAM_RW) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    SensorSafeClient_WriteU16(&payload[0], param_id);
    SensorSafeClient_WriteU16(&payload[2], param_index);
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_READ_PARAM,
                                              0U, payload, sizeof(payload),
                                              context->config.request_timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if (response.extra_len != 22U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    result = SensorSafeClient_DecodeParamRecord(context, response.extra, value_out);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((value_out->param_id != param_id) || (value_out->param_index != param_index)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_PARAM_CRC);
    }
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：通过自动分页读取完整 LTD 参数表，对上保持一次逻辑调用。
 * 调用场景：设备识别完成后的参数同步、维护导出或诊断快照。
 * 关键约束：首帧确认总数后才写调用方数组；任一页失败时 value_count_out 保持 0。
 */
SensorSafeClientResult SensorSafeClient_ReadAllParams(SensorSafeClientContext *context,
                                                      SensorSafeParameterValue *values,
                                                      uint16_t value_capacity,
                                                      uint16_t *value_count_out)
{
    SensorSafeParameterValue page_values[SENSOR_SAFE_PARAM_PAGE_MAX_ITEMS];
    SensorSafeClientParamPage page;
    SensorSafeClientResult result;
    uint16_t start_index = 0U;
    uint16_t total_count = SENSOR_SAFE_PARAM_TOTAL_UNKNOWN;
    uint16_t output_count = 0U;
    uint8_t index;

    if ((context == NULL) || (values == NULL) || (value_count_out == NULL) ||
        (value_capacity == 0U)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    *value_count_out = 0U;
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_PARAM_RW) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }

    do {
        (void)memset(page_values, 0, sizeof(page_values));
        (void)memset(&page, 0, sizeof(page));
        result = SensorSafeClient_ReadParamPage(context,
                                                start_index,
                                                total_count,
                                                page_values,
                                                &page);
        if (result != SENSOR_SAFE_CLIENT_OK) {
            return result;
        }
        if (total_count == SENSOR_SAFE_PARAM_TOTAL_UNKNOWN) {
            total_count = page.total_count;
            if (total_count > value_capacity) {
                *value_count_out = total_count;
                return SENSOR_SAFE_CLIENT_BUFFER_TOO_SMALL;
            }
        }
        if ((output_count != start_index) ||
            ((output_count != 0U) && (page.value_count != 0U) &&
             (SensorSafeClient_IsParamAfter(&values[output_count - 1U], &page_values[0]) == 0U))) {
            *value_count_out = 0U;
            return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SEQUENCE);
        }
        for (index = 0U; index < page.value_count; index++) {
            values[output_count] = page_values[index];
            output_count++;
        }
        start_index = page.next_index;
    } while (page.last_page == 0U);

    if (output_count != total_count) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SEQUENCE);
    }
    *value_count_out = output_count;
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：携带访问保护和期望参数 CRC 写入暂存参数值。
 * 调用场景：受控维护流程修改单项参数时调用。
 * 关键约束：这是副作用事务；调用方必须使用返回的新参数 CRC 继续提交闭环。
 */
SensorSafeClientResult SensorSafeClient_WriteParam(SensorSafeClientContext *context,
                                                   const SensorSafeParameterValue *value,
                                                   uint8_t access_guard,
                                                   uint32_t expected_param_crc,
                                                   uint32_t *new_param_crc)
{
    uint8_t payload[20];
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (value == NULL) || (new_param_crc == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_PARAM_RW) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    SensorSafeClient_WriteU16(&payload[0], value->param_id);
    SensorSafeClient_WriteU16(&payload[2], value->param_index);
    payload[4] = value->param_type;
    payload[5] = access_guard;
    SensorSafeClient_WriteU16(&payload[6], (uint16_t)value->scale);
    SensorSafeClient_WriteI64(&payload[8], value->value_i64);
    SensorSafeClient_WriteU32(&payload[16], expected_param_crc);
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_WRITE_PARAM,
                                              SENSOR_SAFE_FLAG_PARAM_WRITE, payload, sizeof(payload),
                                              context->config.request_timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) ||
        (SensorSafeClient_ReadU16(&response.extra[0]) != value->param_id) ||
        (SensorSafeClient_ReadU16(&response.extra[2]) != value->param_index)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    *new_param_crc = SensorSafeClient_ReadU32(&response.extra[4]);
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：以当前安全参数 CRC 为前置条件提交参数事务。
 * 调用场景：一组 WRITE_PARAM 完成后由受控维护流程调用。
 * 关键约束：提交成功会改变配置身份，本地立即下线并要求重新 HELLO 获取新摘要。
 */
SensorSafeClientResult SensorSafeClient_CommitParam(SensorSafeClientContext *context,
                                                    uint16_t commit_scope,
                                                    uint32_t expected_safety_param_crc,
                                                    uint32_t *new_safety_param_crc)
{
    uint8_t payload[8] = {0U};
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (new_safety_param_crc == NULL) ||
        (expected_safety_param_crc != context->session.safety_param_crc)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_PARAM_RW) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    SensorSafeClient_WriteU16(&payload[0], commit_scope);
    SensorSafeClient_WriteU32(&payload[4], expected_safety_param_crc);
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_COMMIT_PARAM,
                                              SENSOR_SAFE_FLAG_PARAM_WRITE, payload, sizeof(payload),
                                              SENSOR_SAFE_CLIENT_MODE_TIMEOUT_MS, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) || (SensorSafeClient_ReadU16(&response.extra[6]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    *new_safety_param_crc = SensorSafeClient_ReadU32(&response.extra[0]);
    /* 配置提交后的旧 session_id 和 safety_param_crc 不再可信，禁止继续复用。 */
    context->session.state = SENSOR_SAFE_SESSION_OFFLINE;
    context->session.stream_valid = 0U;
    SensorSafeClient_ClearPeriodicAcceptance(context);
    return SENSOR_SAFE_CLIENT_OK;
}

/*
 * 函数用途：启动指定掩码的传感器自检并返回 test_id 与预计完成时间。
 * 调用场景：受控诊断流程显式调用。
 * 关键约束：必须声明 SELF_TEST 能力；timeout_ms 同时约束远端和本地等待。
 */
SensorSafeClientResult SensorSafeClient_RunSelfTest(SensorSafeClientContext *context,
                                                    uint32_t test_mask,
                                                    uint16_t timeout_ms,
                                                    uint32_t *test_id,
                                                    uint16_t *estimated_time_ms)
{
    uint8_t payload[8] = {0U};
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (test_id == NULL) || (estimated_time_ms == NULL) || (timeout_ms == 0U)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_SELF_TEST) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    SensorSafeClient_WriteU32(&payload[0], test_mask);
    SensorSafeClient_WriteU16(&payload[4], timeout_ms);
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_RUN_SELF_TEST,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT, payload, sizeof(payload),
                                              (uint32_t)timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if ((response.extra_len != 8U) || (SensorSafeClient_ReadU16(&response.extra[6]) != 0U)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    *test_id = SensorSafeClient_ReadU32(&response.extra[0]);
    *estimated_time_ms = SensorSafeClient_ReadU16(&response.extra[4]);
    return SENSOR_SAFE_CLIENT_OK;
}

/* 读取指定自检任务结果，并核对非零 test_id 回显防止串用旧任务结果。 */
SensorSafeClientResult SensorSafeClient_ReadSelfTestResult(SensorSafeClientContext *context,
                                                           uint32_t test_id,
                                                           SensorSafeSelfTestResult *result_out)
{
    uint8_t payload[4];
    SensorSafeClientResponse response;
    SensorSafeClientResult result;

    if ((context == NULL) || (result_out == NULL)) {
        return SENSOR_SAFE_CLIENT_INVALID_ARGUMENT;
    }
    if ((context->identity.capability_flags & SENSOR_SAFE_CAP_SELF_TEST) == 0U) {
        return SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY;
    }
    SensorSafeClient_WriteU32(payload, test_id);
    result = SensorSafeClient_ExchangeCommand(context, (uint8_t)SENSOR_SAFE_CMD_READ_SELF_TEST_RESULT,
                                              SENSOR_SAFE_FLAG_SAFETY_RELEVANT, payload, sizeof(payload),
                                              context->config.request_timeout_ms, &response);
    if (result != SENSOR_SAFE_CLIENT_OK) {
        return result;
    }
    if (response.extra_len != 12U) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_LENGTH);
    }
    result_out->test_id = SensorSafeClient_ReadU32(&response.extra[0]);
    result_out->test_result = SensorSafeClient_ReadU16(&response.extra[4]);
    result_out->failed_item = SensorSafeClient_ReadU16(&response.extra[6]);
    result_out->detail_code = SensorSafeClient_ReadU32(&response.extra[8]);
    if ((test_id != 0U) && (result_out->test_id != test_id)) {
        return SensorSafeClient_FromProtocolResult(context, SENSOR_SAFE_BAD_SEQUENCE);
    }
    return SENSOR_SAFE_CLIENT_OK;
}

/* 复制 client 累计诊断计数快照，不清零计数且不访问传输硬件。 */
void SensorSafeClient_GetCounters(const SensorSafeClientContext *context,
                                  SensorSafeClientCounters *counters_out)
{
    if ((context != NULL) && (counters_out != NULL)) {
        *counters_out = context->counters;
    }
}
