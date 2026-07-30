#include "sensor_safe_client.h"

#include <limits.h>
#include <string.h>

#include "sensor_safe_crc32c.h"
#include "sensor_safe_frame.h"

/* HELLO 控制请求的完整帧长度 12 字节；只含握手所需固定字段和 CRC。 */
#define SENSOR_SAFE_HELLO_REQUEST_LEN       12U
/* HELLO 响应在通用控制头之后追加的数据长度 52 字节；包含身份、能力和参数表摘要等固定字段。 */
#define SENSOR_SAFE_HELLO_APPEND_LEN        52U
/* CONFIG 响应追加数据长度 18 字节；解析器必须在读取配置字段前精确校验。 */
#define SENSOR_SAFE_CONFIG_APPEND_LEN       18U
/* SET_COMM 请求完整帧长度 14 字节；包含目标通信配置和 CRC。 */
#define SENSOR_SAFE_SET_COMM_REQUEST_LEN    14U
/* SET_COMM 响应追加数据长度 16 字节；用于回显实际生效通信参数。 */
#define SENSOR_SAFE_SET_COMM_APPEND_LEN     16U

/* 已通过线格式、会话和公共载荷校验的控制响应视图；extra 指向 client 接收缓冲区。 */
typedef struct {
    /* 安全协议客户端解析后的一次控制响应；公共字段、命令专属载荷和接收时间戳来自同一帧。 */
    SensorSafeControlFrame frame; /* 已经通过基础解码的完整控制帧视图。 */
    SensorSafeCommonResponse common; /* 从同一响应载荷解析出的公共结果、模式、状态和数据质量字段。 */
    const uint8_t *extra; /* 指向命令专属响应载荷的只读视图，其生命周期不超过接收缓冲区。 */
    uint16_t extra_len; /* 命令专属响应载荷的字节长度。 */
    uint32_t rx_timestamp_ms; /* 本地接收时间戳，使用单调毫秒节拍；仅可通过无符号差值比较超时。 */
} SensorSafeClientResponse;

/* 参数分页响应的固定页头；参数记录暂存在调用方提供的三项页缓冲区。 */
typedef struct {
    /* 安全参数分页响应的总数、下一索引、本页数量和末页标志。 */
    uint16_t total_count; /* 传感器参数表声明的参数总数，用于校验分页边界和结束条件。 */
    uint16_t next_index; /* 下一页应请求的参数索引；达到 total_count 时不再继续分页。 */
    uint8_t value_count; /* 当前参数页实际返回的参数数量，不得超过本地页面缓冲容量。 */
    uint8_t last_page; /* 本页是否为参数表末页的标志。 */
} SensorSafeClientParamPage;

/**
 * @brief 从协议冻结的小端线格式读取 16 位字段。
 *
 * @param data 安全协议载荷中的连续小端字段首地址；具体读写宽度由当前 U16、U32 或 I64 辅助函数固定。
 * @return 返回从协议冻结的小端线格式读取 16 位字段得到的主机整数值；函数只处理既定字节序或编码，不执行范围校验。
 */
static uint16_t SensorSafeClient_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/**
 * @brief 从协议冻结的小端线格式读取 32 位字段。
 *
 * @param data 安全协议载荷中的连续小端字段首地址；具体读写宽度由当前 U16、U32 或 I64 辅助函数固定。
 * @return 返回从协议冻结的小端线格式读取 32 位字段得到的主机整数值；函数只处理既定字节序或编码，不执行范围校验。
 */
static uint32_t SensorSafeClient_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/**
 * @brief 按位组合小端有符号 64 位参数值，避免未对齐访问和实现相关移位。
 *
 * @param data 安全协议载荷中的连续小端字段首地址；具体读写宽度由当前 U16、U32 或 I64 辅助函数固定。
 * @return 返回按小端字节序还原的 64 位有符号参数值。
 */
static int64_t SensorSafeClient_ReadI64(const uint8_t *data)
{
    uint64_t value = (uint64_t)SensorSafeClient_ReadU32(data) |
                     ((uint64_t)SensorSafeClient_ReadU32(&data[4]) << 32U);
    return (int64_t)value;
}

/**
 * @brief 把 16 位字段逐字节写入冻结线格式。
 *
 * @param data 安全协议载荷中的连续小端字段首地址；具体读写宽度由当前 U16、U32 或 I64 辅助函数固定。
 * @param value 待编码写入线格式的 16 位数值。
 */
static void SensorSafeClient_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/**
 * @brief 把 32 位字段逐字节写入冻结线格式。
 *
 * @param data 安全协议载荷中的连续小端字段首地址；具体读写宽度由当前 U16、U32 或 I64 辅助函数固定。
 * @param value 待编码写入线格式的 32 位数值。
 */
static void SensorSafeClient_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/**
 * @brief 保留二进制补码位模式写入 64 位参数字段。
 *
 * @param data 安全协议载荷中的连续小端字段首地址；具体读写宽度由当前 U16、U32 或 I64 辅助函数固定。
 * @param value 待保留补码位模式写入线格式的 64 位有符号数值。
 */
static void SensorSafeClient_WriteI64(uint8_t *data, int64_t value)
{
    uint64_t bits = (uint64_t)value;
    SensorSafeClient_WriteU32(data, (uint32_t)(bits & 0xFFFFFFFFULL));
    SensorSafeClient_WriteU32(&data[4], (uint32_t)(bits >> 32U));
}

/**
 * @brief 接受协议卷定义的业务、诊断和故障模式，拒绝其余保留值。
 *
 * @param mode 安全协议测量模式码，允许业务、诊断和故障模式，拒绝协议保留值。
 * @return 1 表示 mode 属于 SENSOR_SAFE_MEASURE_BOTTOM 及其之前的业务测量模式，或为自检或故障模式；0 表示 mode 为协议保留值或未定义模式。
 */
static uint8_t SensorSafeClient_IsMeasureModeValid(uint8_t mode)
{
    return (uint8_t)(((mode <= (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM) ||
                      (mode == (uint8_t)SENSOR_SAFE_MEASURE_SELF_TEST) ||
                      (mode == (uint8_t)SENSOR_SAFE_MEASURE_FAULT)) ? 1U : 0U);
}

/**
 * @brief 把测量模式映射到 HELLO 声明的能力位。
 *
 * @details 调用场景：发送模式切换或启动周期上报前的本地能力校验。
 * @note 关键约束：IDLE 不要求测量能力；未知模式返回 0，由调用方按参数错误拒绝。
 *
 * @param mode 安全协议测量模式码，用于选择 HELLO 中必须具备的能力位。
 * @return 返回测量模式对应的 HELLO 能力位；密度、液位、水位电容、陀螺仪、探底和自检分别映射固定掩码，未知模式返回 0。
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

/**
 * @brief 诊断计数采用饱和加一，长稳运行后不得因回绕伪装成较小故障次数。
 *
 * @param counter 本次节拍、重试或统计使用的计数值。指针非 NULL 且当前值未达到 UINT32_MAX 时原地加一，达到上限后饱和保持。
 */
static void SensorSafeClient_IncrementCounter(uint32_t *counter)
{
    if ((counter != NULL) && (*counter != UINT32_MAX)) {
        (*counter)++;
    }
}

/**
 * @brief 清除只对当前周期流有效的协商值，诊断计数和会话防重放历史不受影响。
 *
 * @param context 待清理的客户端上下文；函数只复位已接受周期、启动延迟、控制保护、最大静默时间和周期起始时刻。
 */
static void SensorSafeClient_ClearPeriodicAcceptance(SensorSafeClientContext *context)
{
    context->accepted_period_ms = 0U;
    context->accepted_max_silent_ms = 0U;
    context->accepted_control_guard_ms = 0U;
    context->accepted_start_after_ms = 0U;
    context->periodic_start_ms = 0U;
}

/**
 * @brief 计算首次快报或最近有效快报之后还剩多少协商静默预算。
 *
 * @param context 已完成 HELLO 协商的客户端上下文，提供最近有效快报时间和协商静默时限。
 * @param now_ms 当前系统节拍，单位 ms。
 * @return 返回距协商静默时限还剩的毫秒数；已有快报时以 last_valid_rx_ms 为基准，首次快报前以 periodic_start_ms 加启动等待预算为基准，预算耗尽返回 0。
 */
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

/**
 * @brief 把 frame/session 层结果稳定映射为 client 对外结果类别。
 *
 * @param result frame 或 session 层返回的 SensorSafeResult，映射时保留参数、CRC、会话、序号和数据类失败。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 可信远端错误中只有会话或配置身份失效允许进入单次 HELLO 恢复。
 *
 * @param wire_result 线格式结果。
 * @return 1 表示远端结果为会话失效或配置身份不匹配，可执行一次 HELLO 恢复；其他结果返回 0。
 */
static uint8_t SensorSafeClient_WireResultNeedsHello(uint16_t wire_result)
{
    return (uint8_t)(((wire_result == (uint16_t)SENSOR_SAFE_WIRE_BAD_SESSION) ||
                      (wire_result == (uint16_t)SENSOR_SAFE_WIRE_CONFIG_MISMATCH))
                         ? 1U
                         : 0U);
}

/**
 * @brief 记录最近一次协议拒绝原因，并对 CRC 和地址类故障单独累计。
 *
 * @param context 客户端诊断上下文；函数写入 last_protocol_result，并按拒绝原因递增认证、序号、CRC、长度或语义错误计数。
 * @param result 本次被拒绝的具体 SensorSafeResult；函数写入最近协议原因并更新 CRC 或地址错误计数。
 */
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

/**
 * @brief 先记录具体协议原因，再返回对外 client 结果，避免上层丢失诊断证据。
 *
 * @param context 接收协议解析结果的客户端上下文；失败时用于记录最近协议结果和分类计数，成功时不改变既有错误现场。
 * @param result 准备记录并转换为客户端类别的具体 SensorSafeResult。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
static SensorSafeClientResult SensorSafeClient_FromProtocolResult(SensorSafeClientContext *context,
                                                                   SensorSafeResult result)
{
    SensorSafeClient_RecordProtocolReject(context, result);
    return SensorSafeClient_MapProtocolResult(result);
}

/**
 * @brief 校验所有控制响应共有的状态位、诊断位、模式和载荷版本。
 *
 * @details 调用场景：会话字段通过后、命令专属 extra 载荷解析之前调用。
 * @note 关键约束：任何保留位或未知枚举均拒绝，避免不同协议版本产生歧义解释。
 *
 * @param context 待校验的客户端上下文；函数确认指针、传输接口、会话对象和客户端基础配置已经建立，不启动新的协议事务。
 * @param common 已经解码的安全协议控制响应公共字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 执行普通控制命令的同序号有限次请求响应事务。
 *
 * @details 调用场景：除 HELLO 外的测量、模式、配置、参数和诊断命令统一复用。
 * @note 关键约束：重试复用原 cmd/seq 并置 RETRY；只有绑定待决事务且公共字段合法的响应可结束事务。
 *
 * @param context 单次控制事务使用的客户端上下文；函数占用固定收发缓冲区，推进会话序号，并更新最近传输、线格式、协议结果和诊断计数。
 * @param cmd 安全协议控制命令字节；写入请求帧并与待确认会话状态绑定，响应必须回显同一命令才可接受。
 * @param flags 安全协议控制帧标志位，必须满足保留位约束。
 * @param payload 待封装到安全请求帧的只读业务载荷；payload_len 为 0 时允许传入 NULL。
 * @param payload_len 协议载荷有效长度，单位字节。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param response 安全协议客户端响应输出对象；成功交换后写入已校验控制帧、载荷长度、接收时间以及传输和协议结果。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 校验请求响应测量的模式、状态、诊断、新鲜度和样本单调性。
 *
 * @details 调用场景：各测量命令解析定点数据前共同调用。
 * @note 关键约束：每种测量使用独立样本槽；未通过全部检查前不得更新防重复基准。
 *
 * @param context 测量一致性检查使用的客户端上下文；函数读取会话、传输状态和最近控制样本，核对周期样本是否属于同一受控测量。
 * @param response 已经通过帧级校验的安全协议客户端响应；函数只读其命令、载荷、接收时间和协议结果以继续业务一致性检查。
 * @param expected_mode 期望模式。
 * @param channel_diag_mask 通道。
 * @param sample_slot 槽位。
 * @param effective_age_ms 用于返回同时计入传感器样本年龄和本机传输耗时后的有效数据年龄，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 保存 client 配置和传输接口并初始化离线会话。
 *
 * @details 调用场景：服务层完成持久化身份初始化后调用。
 * @note 关键约束：零超时或零尝试次数替换为受控默认值；本函数不访问总线。
 *
 * @param context 待初始化的客户端对象；函数清零运行态，绑定传输接口和会话对象，并复制固定客户端配置，不要求调用前已有有效会话。
 * @param config 安全协议客户端静态配置；包含 CPU2 与传感器节点号、CPU2 单调启动计数、最大数据年龄、请求超时和最大尝试次数。
 * @param transport 安全协议客户端使用的请求响应传输接口表。
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

/**
 * @brief 撤销 client 会话、周期协商值和本会话样本历史，但保留累计诊断计数。
 *
 * @param context 待停用的客户端上下文；函数发送停用事务后清除会话、远端身份以及最近控制样本有效标志。
 */
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

/**
 * @brief 发送 HELLO 挑战并验收传感器回显、身份、能力及单调会话计数。
 *
 * @details 调用场景：首次探测和服务层允许的故障恢复。
 * @note 关键约束：同一 HELLO 的重试复用 nonce 和 seq；任何未绑定响应不得激活会话。
 *
 * @param context 客户端握手上下文；函数使用固定收发缓冲区和启动身份建立远端身份、会话序号、能力配置及最近事务诊断。
 * @param cpu_nonce CPU随机数。
 * @param identity_out 用于接收已经完成身份、能力和单调计数验收的 HELLO 响应。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 执行无副作用在线检查并返回传感器运行时间。
 *
 * @details 调用场景：安全会话健康检查或周期流控制保护窗内诊断。
 * @note 关键约束：响应长度和公共状态必须通过统一校验后才写输出。
 *
 * @param context 活动客户端上下文；函数按当前事务配置发送 PING，用于验证会话和传输链路仍可完成控制往返。
 * @param token 用于串行化访问共享资源的控制标记。
 * @param sensor_uptime_ms 用于返回传感器报告的本次启动运行时长，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 读取传感器公共状态及复位原因，供诊断流程判断当前运行态。
 *
 * @param context 已初始化的客户端上下文；函数通过其中的事务配置和当前会话读取远端状态字、模式及诊断摘要。
 * @param status_out 用于接收本次查询得到的状态快照。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 读取并核对安全参数摘要、参数表版本和配置代次。
 *
 * @details 调用场景：HELLO 成功后由服务层立即调用，建立配置身份基线。
 * @note 关键约束：响应中的 safety_param_crc 必须与控制帧身份字段一致且非 0。
 *
 * @param context 活动客户端上下文；函数使用当前会话和远端身份读取配置摘要，供服务层判断参数契约是否变化。
 * @param digest_out 用于接收通过核对的安全参数摘要、参数表版本和配置代次。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 请求传感器撤销当前会话并返回复位接受状态。
 *
 * @details 调用场景：显式维护或不可继续使用当前会话时调用。
 * @note 关键约束：命令成功后本地也立即下线，后续业务必须重新 HELLO。
 *
 * @param context 待重置会话的客户端上下文；函数按本地配置重新初始化会话序号和防重放窗口，同时保留传输接口。
 * @param reset_reason 复位。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 切换测量模式并验收模式回显、能力位和稳定等待时间。
 *
 * @details 调用场景：密度、液位、水位电容、陀螺仪等测量前调用。
 * @note 关键约束：未声明能力的模式在发送前拒绝；模式未匹配时不得进入 MEASURE_READY。
 *
 * @param context 活动客户端上下文；函数使用当前会话和远端身份发送模式切换，并核对响应中的实际模式和稳定等待时间。
 * @param target_mode 目标模式。
 * @param settle_time_ms 传感器切换测量模式后要求的稳定等待时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 读取密度模式的频率、密度、温度组合值，并执行样本单调性与新鲜度检查。
 *
 * @param context 活动客户端上下文；函数依据协商能力和远端身份读取同一采样时刻的频率、密度、温度组合值。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议组合测量中的密度、温度、频率、位置、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 读取液位频率定点值，只有液位通道诊断和样本检查全部通过才写输出。
 *
 * @param context 活动客户端上下文；函数检查液位频率能力和远端身份后，使用当前会话读取定点频率值。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议液位频率、位置、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 读取水位电容定点值；未声明 WATER_CAP 能力时不发送总线请求。
 *
 * @param context 活动客户端上下文；函数检查水位电容能力后通过当前会话读取定点电容值。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议水位电容、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 读取双轴姿态定点值；未声明 GYRO 能力时不发送总线请求。
 *
 * @param context 活动客户端上下文；函数检查姿态能力并通过当前会话读取X、Y 双轴定点角度。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议陀螺角度、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 读取电源电压、电流及电源状态，作为诊断数据而非直接控制依据。
 *
 * @param context 活动客户端上下文；函数检查电源诊断能力和远端身份后读取电压、电流及电源状态定点值。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议电源电压、电流、状态、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 请求进入周期主动上报并锁存传感器实际接受的时序参数。
 *
 * @details 调用场景：上层显式选择周期模式且已处于对应测量就绪态时调用。
 * @note 关键约束：周期、静默预算、保护窗、stream_id、配置代次和模式必须逐项回显一致。
 *
 * @param context 活动会话的客户端上下文；函数写入远端接受的周期、启动延迟、控制保护和最大静默时间，并记录周期流起始时刻。
 * @param period_ms 传感器周期主动上报的目标周期，单位 ms。
 * @param max_silent_ms 周期主动上报允许连续未收到有效快报的最长静默时间，单位 ms。
 * @param stream_id 数据流编号。
 * @param control_guard_ms 周期主动上报期间预留给控制请求的保护窗口，单位 ms。
 * @param measure_mode 测量模式。
 * @param start_after_ms 传感器接受周期主动上报请求后延迟开始快报的时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 在协商静默预算内接收并验收一帧周期快报。
 *
 * @details 调用场景：周期模式轮询任务调用，每次最多交付一帧。
 * @note 关键约束：接收等待不超过剩余静默预算；迟到、跳号或身份异常立即使流失效。
 *
 * @param context 周期流轮询上下文；函数使用接收缓冲区、会话和最大静默门限接收快报，并更新最近收包时刻、错误结果和诊断计数。
 * @param report 用于接收本次诊断或测量结果的报告对象。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 只在最近快报后的控制保护窗内请求退出周期模式。
 *
 * @details 调用场景：业务结束或需要返回请求响应模式时调用。
 * @note 关键约束：预留完整发送预算；窗口关闭或响应异常时流立即失效，禁止假定已停流。
 *
 * @param context 正在运行周期流的客户端上下文；函数在控制保护时间满足后发送停止请求，并清除周期流接受参数。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 按参数表版本和冻结字段顺序计算单项参数 CRC32C，禁止直接校验结构体内存。
 *
 * @param schema_version 版本。
 * @param value 待原地读取或更新的数值对象。该参数实际是只读安全参数记录，包含参数 ID、类型、访问属性、长度和值字节，用于 CRC、语义校验或写入请求。
 * @return 返回按冻结字段顺序计算的单项参数 CRC32C。
 */
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

/**
 * @brief 校验参数类型、访问属性和值域，阻止未知解释或无符号负值进入完整参数表。
 *
 * @param value 待原地读取或更新的数值对象。该参数实际是只读安全参数记录，包含参数 ID、类型、访问属性、长度和值字节，用于 CRC、语义校验或写入请求。
 * @return 1 表示参数类型、访问属性和值域语义均合法；未知类型、非法属性或越界值返回 0。
 */
static uint8_t SensorSafeClient_IsParamSemanticValid(const SensorSafeParameterValue *value);

/**
 * @brief 从 22 字节参数记录解码解释字段，并在返回前完成单项 CRC 验收。
 *
 * @param context 包含已确认远端身份的客户端上下文；函数用身份中的参数契约校验并解码单条参数记录。
 * @param record 一条固定 22 字节安全参数记录的只读线格式缓冲区。
 * @param value_out 用于接收通过类型、范围和参数 CRC 校验的参数值。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 判断当前参数键是否严格位于上一参数键之后，用于拒绝重复项和乱序页。
 *
 * @param previous 参数表中前一个已解码条目，用于校验地址和索引严格递增。
 * @param current 紧随 previous 的当前参数条目；地址或索引未递增时判为非法顺序。
 * @return 1 表示 current 的 param_id 大于 previous，或 ID 相同但 param_index 更大，参数键严格递增；0 表示参数键重复或逆序，应拒绝该分页结果。
 */
static uint8_t SensorSafeClient_IsParamAfter(const SensorSafeParameterValue *previous,
                                             const SensorSafeParameterValue *current)
{
    return (uint8_t)(((current->param_id > previous->param_id) ||
                      ((current->param_id == previous->param_id) &&
                       (current->param_index > previous->param_index)))
                         ? 1U
                         : 0U);
}

/**
 * @brief 校验参数类型、访问属性和值域，阻止未知解释或无符号负值进入完整参数表。
 *
 * @param value 待原地读取或更新的数值对象。该参数实际是只读安全参数记录，包含参数 ID、类型、访问属性、长度和值字节，用于 CRC、语义校验或写入请求。
 * @return 1 表示参数类型、访问属性和值域语义均合法；未知类型、非法属性或越界值返回 0。
 */
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

/**
 * @brief 读取参数表的一个有界页并验证页游标、总数、末页标志和逐项 CRC。
 *
 * @details 调用场景：完整参数表读取接口内部循环调用，不直接暴露给业务层。
 * @note 关键约束：页内最多三项且必须严格升序；非末页不得返回空页或停滞游标。
 *
 * @param context 已建立远端身份的客户端上下文；函数按页号和容量发送参数页读取，并保持同一会话的防重放序号连续。
 * @param start_index 起始位置索引。
 * @param expected_total_count 首个参数页声明的参数表总项数；后续页面必须保持一致。
 * @param values 用于保存连续参数值或测量值的数组。
 * @param page_out 用于接收通过页游标、总数、末页标志和逐项 CRC 校验的参数页。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 按参数 ID 和索引读取单项参数并核对参数级 CRC。
 *
 * @details 调用场景：维护诊断明确请求参数读取时调用，基础测量流程不使用。
 * @note 关键约束：能力位、回显标识和 CRC 任一不符时不接受返回值。
 *
 * @param context 已完成 HELLO 的客户端上下文；函数使用远端身份、能力配置和当前会话发送单参数读取事务。
 * @param param_id 参数编号。
 * @param param_index 参数索引。
 * @param value_out 用于接收通过类型、范围和参数 CRC 校验的参数值。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 通过自动分页读取完整 LTD 参数表，对上保持一次逻辑调用。
 *
 * @details 调用场景：设备识别完成后的参数同步、维护导出或诊断快照。
 * @note 关键约束：首帧确认总数后才写调用方数组；任一页失败时 value_count_out 保持 0。
 *
 * @param context 已建立身份和会话的客户端上下文；函数使用其中的远端身份、事务序号和固定收发缓冲区连续读取所有参数页。
 * @param values 调用方提供的参数值数组；全部分页成功后才写入。
 * @param value_capacity 参数值数组容量，单位为 64 位参数值个数。
 * @param value_count_out 用于返回完整参数表的实际元素数量；任一页失败时保持为 0。
 * @return 返回安全协议结果码；SENSOR_SAFE_RESULT_OK 表示完整参数表读取成功，其他值表示分页、容量或通信失败。
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

/**
 * @brief 携带访问保护和期望参数 CRC 写入暂存参数值。
 *
 * @details 调用场景：受控维护流程修改单项参数时调用。
 * @note 关键约束：这是副作用事务；调用方必须使用返回的新参数 CRC 继续提交闭环。
 *
 * @param context 已建立远端身份的客户端上下文；函数按参数元数据和能力约束发送暂存写入，不在此步骤提交持久化。
 * @param value 待原地读取或更新的数值对象。该参数实际是只读安全参数记录，包含参数 ID、类型、访问属性、长度和值字节，用于 CRC、语义校验或写入请求。
 * @param access_guard 参数写入使用的访问保护值，必须与当前协议授权条件一致。
 * @param expected_param_crc 期望参数CRC。
 * @param new_param_crc 用于返回远端暂存参数表更新后的 CRC32C。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 以当前安全参数 CRC 为前置条件提交参数事务。
 *
 * @details 调用场景：一组 WRITE_PARAM 完成后由受控维护流程调用。
 * @note 关键约束：提交成功会改变配置身份，本地立即下线并要求重新 HELLO 获取新摘要。
 *
 * @param context 包含当前会话和远端身份的客户端上下文；函数提交此前暂存的参数写入，并通过响应确认远端持久化结果。
 * @param commit_scope 安全参数提交范围，决定传感器应用哪些暂存修改。
 * @param expected_safety_param_crc 期望参数CRC。
 * @param new_safety_param_crc 用于返回远端提交后生效的安全参数 CRC32C。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 启动指定掩码的传感器自检并返回 test_id 与预计完成时间。
 *
 * @details 调用场景：受控诊断流程显式调用。
 * @note 关键约束：必须声明 SELF_TEST 能力；timeout_ms 同时约束远端和本地等待。
 *
 * @param context 已确认远端身份的客户端上下文；函数发送自检启动事务，不在本次调用中等待完整自检结果。
 * @param test_mask 测试。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param test_id 用于返回远端本次自检事务的标识号。
 * @param estimated_time_ms 用于返回传感器自检预计持续时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
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

/**
 * @brief 读取指定自检任务结果，并核对非零 test_id 回显防止串用旧任务结果。
 *
 * @param context 已完成身份协商的客户端上下文；函数依据远端能力位和当前会话读取最近一次自检结果。
 * @param test_id 测试编号。
 * @param result_out 用于接收与请求 test_id 一致的自检结果。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
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

/**
 * @brief 复制 client 累计诊断计数快照，不清零计数且不访问传输硬件。
 *
 * @param context 安全协议客户端只读运行上下文；用于读取当前会话、周期流协商结果、最近错误和诊断计数，函数不会修改上下文字段。
 * @param counters_out 用于接收客户端累计诊断计数快照；读取不会清零计数。
 */
void SensorSafeClient_GetCounters(const SensorSafeClientContext *context,
                                  SensorSafeClientCounters *counters_out)
{
    if ((context != NULL) && (counters_out != NULL)) {
        *counters_out = context->counters;
    }
}
