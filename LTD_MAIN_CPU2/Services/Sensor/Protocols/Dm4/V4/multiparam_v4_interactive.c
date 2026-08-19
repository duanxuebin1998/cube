/*
 * 模块职责：执行DM4-V4的8字节读写事务、模式切换和密度模式功能开关。
 * 半双工约束：主动帧结束后至少等待15ms才可发送切换指令；过晚则放弃当前窗口，
 * 等待下一主动帧重新计时。停流事务中若主动帧抢占，必须先保存该帧再重试。
 * 重试边界：只有读操作和明确值写入可按幂等策略重试，L/M翻转命令不得盲目重发。
 */
#include "multiparam_v4_internal.h"

#include "error_log.h"
#include "main.h"
#include "my_crc.h"
#include "sensor_transport_config.h"
#include "sensor_uart6_owner.h"
#include "system_parameter.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
/*
 * 函数用途：返回V4交互请求使用的总线地址。
 * 调用场景：普通V4读写、模式控制和识别阶段R67读取。
 * 关键约束：当前为规避无线桥本地地址应答而临时强制广播；现场问题解决后把临时开关改为0恢复锁定地址。
 */
static uint8_t MULTIPARAM_V4_GetRequestAddress(void)
{
#if (MULTIPARAM_V4_TEMP_FORCE_BROADCAST_TX != 0U)
    return MULTIPARAM_V4_BROADCAST_ADDRESS;
#else
    return (s_expected_address == MULTIPARAM_V4_ANY_ADDRESS) ?
        MULTIPARAM_V4_BROADCAST_ADDRESS : s_expected_address;
#endif
}

/*
 * 函数用途：在停止主动上报的写事务中区分8字节应答和64字节优先主动帧。
 * 调用场景：主动模式写参数65为1的唯一事务。
 * 关键约束：主动帧必须收满、保留并恢复常驻接收；调用方随后等待下一安全窗口重试。
 */
static uint32_t MULTIPARAM_V4_TransceiveStopActiveOwned(const uint8_t request[8],
                                                        uint8_t reply[8],
                                                        uint32_t timeout_ms,
                                                        uint8_t *active_preempted)
{
    uint8_t receive_buffer[MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE];
    uint8_t canonical_frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE];
    multiparam_v4_snapshot_t candidate;
    uint32_t start_tick;
    uint32_t result;
    uint16_t received_length;
    uint16_t active_consumed_length;
    uint8_t wireless_padding_used;
    multiparam_v4_active_candidate_state_t candidate_state;
    const char *stage;

    if ((request == NULL) || (reply == NULL) || (active_preempted == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    memset(receive_buffer, 0, sizeof(receive_buffer));
    memset(reply, 0, 8U);
    *active_preempted = 0U;
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    start_tick = HAL_GetTick();
    MULTIPARAM_V4_BeginTransactionDiagnostic(request);

    if (HAL_UART_Receive_DMA(&huart6, receive_buffer,
                             MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE) != HAL_OK) {
        MULTIPARAM_V4_RecordTransactionDiagnostic(
            receive_buffer, 0U, start_tick, "接收DMA启动");
        return COMM_UART_TRANSFER_ERROR;
    }
    if (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)request, 8U) != HAL_OK) {
        MULTIPARAM_V4_RecordTransactionDiagnostic(
            receive_buffer, 0U, start_tick, "发送DMA启动");
        (void)HAL_UART_DMAStop(&huart6);
        return COMM_UART_TRANSFER_ERROR;
    }
    if (s_probe_trace_enabled != 0U) {
        s_probe_trace_transaction++;
        MULTIPARAM_V4_PrintProbePacket(request, "TX", request, 8U,
                                       NO_ERROR, "发送DMA已启动");
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        received_length = (huart6.hdmarx == NULL)
                              ? 0U
                              : (uint16_t)(MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE -
                                           __HAL_DMA_GET_COUNTER(huart6.hdmarx));
        if (HasEffectiveCommandSwitchRequest()) {
            stage = "命令切换";
            MULTIPARAM_V4_RecordTransactionDiagnostic(
                receive_buffer, received_length, start_tick, stage);
            (void)HAL_UART_DMAStop(&huart6);
            MULTIPARAM_V4_PrintProbePacket(request, "RX", receive_buffer,
                                           received_length, STATE_SWITCH, stage);
            return STATE_SWITCH;
        }
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            stage = "UART错误";
            MULTIPARAM_V4_RecordTransactionDiagnostic(
                receive_buffer, received_length, start_tick, stage);
            (void)HAL_UART_DMAStop(&huart6);
            MULTIPARAM_V4_PrintProbePacket(
                request, "RX", receive_buffer, received_length,
                COMM_UART_TRANSFER_ERROR, stage);
            return COMM_UART_TRANSFER_ERROR;
        }
        if ((received_length >= 2U) &&
            (receive_buffer[0] == MULTIPARAM_V4_ACTIVE_HEADER0) &&
            (receive_buffer[1] == MULTIPARAM_V4_ACTIVE_HEADER1)) {
            candidate_state = MULTIPARAM_V4_DecodeActiveCandidate(
                receive_buffer, received_length, &candidate, canonical_frame,
                &active_consumed_length, &wireless_padding_used, &result);
            if (candidate_state == MULTIPARAM_V4_ACTIVE_CANDIDATE_NEED_MORE) {
                HAL_Delay(1U);
                continue;
            }
            /* 停流窗口收到完整主动帧时，以该帧结束时刻重新计算15ms保护期。 */
            MULTIPARAM_V4_MarkActiveFrameEnd();
            (void)HAL_UART_DMAStop(&huart6);
            stage = (candidate_state == MULTIPARAM_V4_ACTIVE_CANDIDATE_VALID)
                        ? ((wireless_padding_used != 0U)
                               ? "主动帧抢占-无线补零已还原"
                               : "主动帧抢占")
                        : "主动帧校验失败";
            MULTIPARAM_V4_RecordTransactionDiagnostic(
                receive_buffer, received_length, start_tick, stage);
            MULTIPARAM_V4_PrintProbePacket(request, "RX", receive_buffer,
                                           received_length, result, stage);
            if (candidate_state != MULTIPARAM_V4_ACTIVE_CANDIDATE_VALID) {
                MULTIPARAM_V4_RecordQualityAnomaly(
                    result, 1U, receive_buffer, received_length);
                s_diagnostics.interactive_transactions++;
                s_diagnostics.interactive_errors++;
                return result;
            }
            if (wireless_padding_used != 0U) {
                s_diagnostics.wireless_zero_padding_frames++;
            }
            result = MULTIPARAM_V4_ResumeActiveReceivePreservingSnapshot();
            MULTIPARAM_V4_FeedBytes(canonical_frame,
                                    MULTIPARAM_V4_ACTIVE_FRAME_SIZE);
            *active_preempted = 1U;
            s_diagnostics.interactive_transactions++;
            return result;
        }
        if ((received_length >= 8U) && (huart6.gState == HAL_UART_STATE_READY)) {
            (void)HAL_UART_DMAStop(&huart6);
            memcpy(reply, receive_buffer, 8U);
            result = MULTIPARAM_V4_ValidateReply(request, reply);
            stage = (result == NO_ERROR) ? "校验通过" : "应答校验失败";
            MULTIPARAM_V4_RecordTransactionDiagnostic(
                reply, 8U, start_tick, stage);
            MULTIPARAM_V4_PrintProbePacket(request, "RX", reply, 8U,
                                           result, stage);
            s_diagnostics.interactive_transactions++;
            if (result != NO_ERROR) {
                s_diagnostics.interactive_errors++;
            }
            return result;
        }
        HAL_Delay(1U);
    }

    received_length = (huart6.hdmarx == NULL)
                          ? 0U
                          : (uint16_t)(MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE -
                                       __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    stage = (received_length == 0U) ? "零字节超时" : "响应长度不足";
    result = (received_length == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT
                                     : SENSOR_RESP_FORMAT_ERROR;
    MULTIPARAM_V4_RecordTransactionDiagnostic(
        receive_buffer, received_length, start_tick, stage);
    (void)HAL_UART_DMAStop(&huart6);
    MULTIPARAM_V4_PrintProbePacket(request, "RX", receive_buffer,
                                   received_length, result, stage);
    s_diagnostics.interactive_transactions++;
    s_diagnostics.interactive_errors++;
    return result;
}

/*
 * 函数用途：停止其它V4 DMA后完成一次8字节请求和8字节应答事务。
 * 调用场景：交互读取、明确值写入、模式选择和L/M单次翻转。
 * 关键约束：函数不自动重试；幂等性策略由上层按命令类别决定。
 */
static uint32_t MULTIPARAM_V4_TransceiveStopActiveOnce(const uint8_t request[8],
                                                        uint8_t reply[8],
                                                        uint32_t timeout_ms,
                                                        uint8_t *active_preempted)
{
    uint32_t result;

    /* 主动DMA已停，但仍需独占UART6，防止AT或Safe事务插入停流窗口。 */
    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = MULTIPARAM_V4_TransceiveStopActiveOwned(
        request, reply, timeout_ms, active_preempted);
    if (SensorUart6Owner_Is(SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE) != 0U) {
        (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE);
    }
    return result;
}

/*
 * 函数用途：在已持有UART6交互所有权时完成一次八字节请求应答事务。
 * 调用场景：普通读写、模式命令和停流后的事实恢复。
 * 关键约束：会启动DMA并轮询等待，只能在线程态调用；是否允许命令打断及地址策略由参数决定。
 */
static uint32_t MULTIPARAM_V4_TransceiveOnceOwned(
    const uint8_t request[8],
    uint8_t reply[8],
    uint32_t timeout_ms,
    uint8_t check_command_switch,
    multiparam_v4_reply_address_policy_t address_policy)
{
    uint32_t start_tick;
    uint32_t result;
    uint16_t received_length;
    const char *stage;

    if ((request == NULL) || (reply == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    memset(reply, 0, 8U);
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    start_tick = HAL_GetTick();
    MULTIPARAM_V4_BeginTransactionDiagnostic(request);

    if (HAL_UART_Receive_DMA(&huart6, reply, 8U) != HAL_OK) {
        MULTIPARAM_V4_RecordTransactionDiagnostic(
            reply, 0U, start_tick, "接收DMA启动");
        return COMM_UART_TRANSFER_ERROR;
    }
    if (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)request, 8U) != HAL_OK) {
        MULTIPARAM_V4_RecordTransactionDiagnostic(
            reply, 0U, start_tick, "发送DMA启动");
        (void)HAL_UART_DMAStop(&huart6);
        return COMM_UART_TRANSFER_ERROR;
    }
    if (s_probe_trace_enabled != 0U) {
        s_probe_trace_transaction++;
        MULTIPARAM_V4_PrintProbePacket(request, "TX", request, 8U,
                                       NO_ERROR, "发送DMA已启动");
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        received_length = (huart6.hdmarx == NULL)
                              ? 0U
                              : (uint16_t)(8U - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
        if ((check_command_switch != 0U) && HasEffectiveCommandSwitchRequest()) {
            MULTIPARAM_V4_RecordTransactionDiagnostic(
                reply, received_length, start_tick, "命令切换");
            (void)HAL_UART_DMAStop(&huart6);
            MULTIPARAM_V4_PrintProbePacket(request, "RX", reply,
                                           received_length, STATE_SWITCH,
                                           "命令切换");
            return STATE_SWITCH;
        }
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            MULTIPARAM_V4_RecordTransactionDiagnostic(
                reply, received_length, start_tick, "UART错误");
            (void)HAL_UART_DMAStop(&huart6);
            MULTIPARAM_V4_PrintProbePacket(request, "RX", reply,
                                           received_length,
                                           COMM_UART_TRANSFER_ERROR,
                                           "UART错误");
            return COMM_UART_TRANSFER_ERROR;
        }
        if ((received_length >= 8U) && (huart6.gState == HAL_UART_STATE_READY)) {
            result = MULTIPARAM_V4_ValidateReplyInternal(
                request, reply, address_policy, &stage);
            MULTIPARAM_V4_RecordTransactionDiagnostic(reply, received_length,
                                                       start_tick, stage);
            (void)HAL_UART_DMAStop(&huart6);
            MULTIPARAM_V4_PrintProbePacket(request, "RX", reply,
                                           received_length, result, stage);
            s_diagnostics.interactive_transactions++;
            return result;
        }
        HAL_Delay(1U);
    }

    received_length = (huart6.hdmarx == NULL)
                          ? 0U
                          : (uint16_t)(8U - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    if (received_length == 0U) {
        stage = "零字节超时";
    } else if (received_length < 8U) {
        stage = "响应长度不足";
    } else {
        stage = "事务等待超时";
    }
    MULTIPARAM_V4_RecordTransactionDiagnostic(reply, received_length,
                                               start_tick, stage);
    (void)HAL_UART_DMAStop(&huart6);
    MULTIPARAM_V4_PrintProbePacket(
        request, "RX", reply, received_length,
        (received_length == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT
                                : SENSOR_RESP_FORMAT_ERROR,
        stage);
    s_diagnostics.interactive_transactions++;
    s_diagnostics.interactive_errors++;
    return (received_length == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
}

/*
 * 函数用途：申请UART6交互所有权后执行一次可配置的底层事务。
 * 调用场景：普通交互请求和R65恢复主动模式的必要收尾。
 * 关键约束：始终配对释放所有权；普通事务允许命令打断，R65收尾可禁止打断。
 */
static uint32_t MULTIPARAM_V4_TransceiveOnceInternal(
    const uint8_t request[8],
    uint8_t reply[8],
    uint32_t timeout_ms,
    uint8_t check_command_switch,
    multiparam_v4_reply_address_policy_t address_policy)
{
    uint32_t result;

    /* 普通交互事务同样必须串行占用UART6，失败时不启动任何DMA。 */
    if (SensorUart6Owner_Acquire(SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE) == 0U) {
        return SENSOR_MODE_NOT_READY;
    }
    result = MULTIPARAM_V4_TransceiveOnceOwned(
        request, reply, timeout_ms, check_command_switch, address_policy);
    (void)SensorUart6Owner_Release(SENSOR_UART6_OWNER_DM4_V4_INTERACTIVE);
    return result;
}

/*
 * 函数用途：以严格地址校验和可命令打断策略执行一次普通交互事务。
 * 调用场景：生产读写、模式和功能命令的单次发送。
 * 关键约束：本函数不重试；调用方必须按命令幂等性决定是否允许再次发送。
 */
static uint32_t MULTIPARAM_V4_TransceiveOnce(const uint8_t request[8],
                                             uint8_t reply[8],
                                             uint32_t timeout_ms)
{
    return MULTIPARAM_V4_TransceiveOnceInternal(
        request, reply, timeout_ms, 1U, MULTIPARAM_V4_REPLY_ADDRESS_STRICT);
}

/*
 * 函数用途：按统一次数重试可安全重复执行的V4交互事务。
 * 调用场景：读取、明确值写入以及可重复的D/S模式命令。
 * 关键约束：成功或命令切换立即结束；L/M翻转命令禁止调用本函数，避免状态二次翻转。
 */
static uint32_t MULTIPARAM_V4_TransceiveIdempotent(const uint8_t request[8], uint8_t reply[8])
{
    uint32_t last_error = SENSOR_DEVICE_COMM_TIMEOUT;

    for (uint32_t attempt = 0U; attempt < MULTIPARAM_V4_MAX_RETRY; attempt++) {
        last_error = MULTIPARAM_V4_TransceiveOnce(request,
                                                  reply,
                                                  MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
        if ((last_error == NO_ERROR) || (last_error == STATE_SWITCH)) {
            return last_error;
        }
        HAL_Delay(DSM_BCC_DELAY);
    }
    return last_error;
}

/*
 * 函数用途：不经过通信方式门禁读取一个V4参数原始值。
 * 调用场景：普通交互读取和参数65写应答丢失后的事实恢复。
 * 关键约束：调用方决定是否重试及超时；参数必须属于可读范围，成功后才写输出。
 */
static uint32_t MULTIPARAM_V4_ReadParamRawInternal(uint8_t parameter,
                                                   uint32_t *raw_value,
                                                   uint8_t retry_allowed,
                                                   uint32_t timeout_ms)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t result;

    if ((raw_value == NULL) || (MULTIPARAM_V4_IsReadableParameter(parameter) == 0U)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(),
                                    MULTIPARAM_V4_FUNCTION_READ,
                                    0U,
                                    parameter,
                                    request);
    result = (retry_allowed != 0U)
                 ? MULTIPARAM_V4_TransceiveIdempotent(request, reply)
                 : MULTIPARAM_V4_TransceiveOnce(request, reply, timeout_ms);
    if (result == NO_ERROR) {
        *raw_value = MULTIPARAM_V4_DecodeU32Le(reply + 2U);
    }
    return result;
}

/*
 * 函数用途：在交互模式下按幂等重试策略读取一个V4参数原始值。
 * 调用场景：整数、浮点、状态字和功能确认等上层读取入口。
 * 关键约束：主动接收仍运行时拒绝事务，避免同一UART6同时存在主动DMA和交互DMA。
 */
uint32_t MULTIPARAM_V4_ReadParamRaw(uint8_t parameter, uint32_t *raw_value)
{
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    return MULTIPARAM_V4_ReadParamRawInternal(parameter,
                                              raw_value,
                                              1U,
                                              MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
}

/*
 * 函数用途：读取一个V4参数并按有符号32位整数解释数据区。
 * 调用场景：R67编号、R02状态字及其它整数寄存器读取。
 * 关键约束：只在通信成功后写输出；不额外判断具体寄存器的业务范围。
 */
uint32_t MULTIPARAM_V4_ReadIntParam(uint8_t parameter, int32_t *value)
{
    uint32_t raw;
    uint32_t result;

    if (value == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRaw(parameter, &raw);
    if (result == NO_ERROR) {
        *value = (int32_t)raw;
    }
    return result;
}

/*
 * 函数用途：读取多参数传感器通信协议 V4.0 的 R67 传感器号。
 * 调用场景：CPU2 已确认 V4 协议并处于交互通信后读取物理传感器编号。
 * 关键约束：V4 的 R22 是密度扫频原始量，不能沿用 V3.0 的 R22 编号语义；编号0表示未初始化且仍为合法编号，负数无效。
 */
uint32_t MULTIPARAM_V4_ReadSensorID(uint32_t *sensor_id)
{
    int32_t value;
    uint32_t result;

    if (sensor_id == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadIntParam(MULTIPARAM_V4_PARAM_SENSOR_ID, &value);
    if (result != NO_ERROR) {
        return result;
    }
    if (value < 0) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *sensor_id = (uint32_t)value;
    return NO_ERROR;
}

/*
 * 函数用途：读取一个V4参数并按IEEE 754单精度浮点解释数据区。
 * 调用场景：交互模式读取温度、密度、频率和扩展量。
 * 关键约束：NaN表示数据未就绪并返回STALE；无穷值返回格式异常，均不写输出。
 */
uint32_t MULTIPARAM_V4_ReadFloatParam(uint8_t parameter, float *value)
{
    uint32_t raw;
    uint32_t result;

    if (value == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRaw(parameter, &raw);
    if (result == NO_ERROR) {
        float parsed = MULTIPARAM_V4_RawToFloat(raw);
        if (isnan(parsed)) {
            /* 传感器以NaN表示当前参数尚无有效值，按数据未就绪上报而不是协议格式错误。 */
            return SENSOR_DATA_STALE;
        }
        if (!isfinite(parsed)) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        *value = parsed;
    }
    return result;
}

/*
 * 函数用途：以单次短事务读取一个有限浮点参数。
 * 调用场景：交互核心测量完成后的非关键调试量刷新。
 * 关键约束：不重试，避免附加量缺失时把业务流程阻塞数秒。
 */
uint32_t MULTIPARAM_V4_ReadFloatParamOnce(uint8_t parameter, float *value)
{
    uint32_t raw;
    uint32_t result;

    if (value == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRawInternal(parameter,
                                                &raw,
                                                0U,
                                                MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS);
    if (result == NO_ERROR) {
        float parsed = MULTIPARAM_V4_RawToFloat(raw);
        if (!isfinite(parsed)) {
            return SENSOR_RESP_FORMAT_ERROR;
        }
        *value = parsed;
    }
    return result;
}

/*
 * 函数用途：读取R02并统一解析测量模式、功能状态和异常位原值。
 * 调用场景：交互测量读取R04、R05、R11和R12之前。
 * 关键约束：无法识别的低四位状态视为响应格式错误。
 */
uint32_t MULTIPARAM_V4_ReadOperatingState(uint32_t *status_word,
                                          multiparam_v4_measurement_mode_t *mode,
                                          multiparam_v4_feature_state_t *feature)
{
    uint32_t raw_status;
    uint32_t result;

    if ((status_word == NULL) || (mode == NULL) || (feature == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    result = MULTIPARAM_V4_ReadParamRaw(MULTIPARAM_V4_PARAM_STATUS, &raw_status);
    if (result != NO_ERROR) {
        return result;
    }
    MULTIPARAM_V4_DecodeOperatingState(raw_status, mode, feature);
    if ((*mode == MULTIPARAM_V4_MEASUREMENT_INVALID) ||
        (*feature == MULTIPARAM_V4_FEATURE_INVALID)) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    *status_word = raw_status;
    return NO_ERROR;
}

/*
 * 函数用途：向允许写入的V4参数写入明确32位值，并在应答丢失时读回确认事实。
 * 调用场景：交互维护命令写配置参数，不包括R65通信方式。
 * 关键约束：主动接收运行时拒绝；写命令只发一次，后续重试仅用于幂等读回而非盲目重写。
 */
uint32_t MULTIPARAM_V4_WriteParamRaw(uint8_t parameter, uint32_t raw_value)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t readback = 0U;
    uint32_t result;

    /*
     * Raw register writes are valid only inside an established interactive window.
     * R65 and active-stream timing remain owned by EnterInteractive/EnterActive.
     */
    if (s_communication_mode != MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    if ((parameter < 65U) || (parameter > MULTIPARAM_V4_PARAM_MAX) ||
        (parameter == MULTIPARAM_V4_PARAM_COMMUNICATION_MODE) ||
        (s_active_receive_requested != 0U) ||
        (s_expected_address == MULTIPARAM_V4_ANY_ADDRESS) ||
        (s_expected_address == MULTIPARAM_V4_BROADCAST_ADDRESS)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    /* Force writes to the learned unicast address even while reads use broadcast. */
    MULTIPARAM_V4_BuildRequestFrame(s_expected_address,
                                    MULTIPARAM_V4_FUNCTION_WRITE,
                                    raw_value,
                                    parameter,
                                    request);
    result = MULTIPARAM_V4_TransceiveOnce(request, reply, MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
    if (result == NO_ERROR) {
        return NO_ERROR;
    }

    if (MULTIPARAM_V4_ReadParamRawInternal(parameter,
                                           &readback,
                                           1U,
                                           MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS) == NO_ERROR) {
        if (readback == raw_value) {
            return NO_ERROR;
        }
    }
    return result;
}

/*
 * 函数用途：用广播R01探测V4协议版本并锁定应答中的实际设备地址。
 * 调用场景：自动识别在主动帧监听未命中后区分V3和V4。
 * 关键约束：仅4.0返回成功；探测允许广播请求地址与应答地址不同，普通事务仍严格校验地址。
 */
uint32_t MULTIPARAM_V4_ProbeProtocolVersion(float *protocol_version)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t raw;
    uint32_t result;
    float value;

    if (protocol_version == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_BROADCAST_ADDRESS,
                                    MULTIPARAM_V4_FUNCTION_READ,
                                    0U,
                                    MULTIPARAM_V4_PARAM_PROTOCOL_VERSION,
                                    request);
    result = MULTIPARAM_V4_TransceiveOnceInternal(
        request, reply, MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS, 1U,
        MULTIPARAM_V4_REPLY_ADDRESS_PROTOCOL_PROBE);
    if (result != NO_ERROR) {
        if (result != STATE_SWITCH) {
            MULTIPARAM_V4_LogProtocolProbeFailure(result);
        }
        return result;
    }
    raw = MULTIPARAM_V4_DecodeU32Le(reply + 2U);
    value = MULTIPARAM_V4_RawToFloat(raw);
    if (!isfinite(value)) {
        s_last_transaction_diagnostic.stage = "R01数值校验";
        MULTIPARAM_V4_LogProtocolProbeFailure(SENSOR_PROTOCOL_VERSION_INCOMPATIBLE);
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    }
    *protocol_version = value;
    if (fabsf(value - MULTIPARAM_V4_PROTOCOL_VERSION_VALUE) > 0.01f) {
        /* 向自动识别层保留实际R01值，由其判断是否继续按V3.0读取R22。 */
        return SENSOR_PROTOCOL_VERSION_INCOMPATIBLE;
    }
    /* V4广播R01应答携带实际设备地址；后续R67和普通交互必须定向到该地址。 */
    s_expected_address = reply[0];
    s_communication_mode = MULTIPARAM_V4_COMMUNICATION_INTERACTIVE;
    return NO_ERROR;
}

/*
 * 函数用途：把D/S测量模式调整到目标状态，并用状态字确认最终模式。
 * 调用场景：交互通信模式下切换密度或液位测量过程。
 * 关键约束：已在密度模式时不得重复发送D，避免关闭已保持开启的L/M功能。
 */
static uint32_t MULTIPARAM_V4_SelectMeasurementMode(uint8_t function,
                                                    multiparam_v4_measurement_mode_t expected_mode)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t status_word;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t result;

    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
    if (result != NO_ERROR) {
        return result;
    }
    if ((mode == expected_mode) &&
        ((expected_mode == MULTIPARAM_V4_MEASUREMENT_DENSITY) ||
         (feature == MULTIPARAM_V4_FEATURE_NONE))) {
        return NO_ERROR;
    }
    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(), function, 0U, 0U, request);
    result = MULTIPARAM_V4_TransceiveIdempotent(request, reply);
    if (result != NO_ERROR) {
        return result;
    }
    result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
    if (result != NO_ERROR) {
        return result;
    }
    return ((mode == expected_mode) && (feature == MULTIPARAM_V4_FEATURE_NONE))
               ? NO_ERROR
               : SENSOR_STREAM_STATE_ERROR;
}

/*
 * 函数用途：把V4传感器调整到密度测量模式。
 * 调用场景：密度业务以及启用水位或零点霍尔功能之前。
 * 关键约束：复用状态确认逻辑；已经处于密度模式时不重复发送D，避免关闭保持中的功能。
 */
uint32_t MULTIPARAM_V4_SelectDensityMode(void)
{
    return MULTIPARAM_V4_SelectMeasurementMode(MULTIPARAM_V4_FUNCTION_DENSITY,
                                               MULTIPARAM_V4_MEASUREMENT_DENSITY);
}

/*
 * 函数用途：把V4传感器调整到液位测量模式。
 * 调用场景：液位搜索、跟随和模式恢复。
 * 关键约束：最终状态必须为液位且无附加功能；传感器进入液位时会自动关闭L/M功能。
 */
uint32_t MULTIPARAM_V4_SelectLevelMode(void)
{
    return MULTIPARAM_V4_SelectMeasurementMode(MULTIPARAM_V4_FUNCTION_LEVEL,
                                               MULTIPARAM_V4_MEASUREMENT_LEVEL);
}

/*
 * 函数用途：发送一次非幂等L/M翻转命令并读取状态字恢复事实。
 * 调用场景：目标状态与当前状态不同，且UART6处于交互通信模式时。
 * 关键约束：应答丢失后禁止直接重发；无论应答结果都先读状态字。
 */
static uint32_t MULTIPARAM_V4_ToggleFeatureOnce(uint8_t function,
                                                multiparam_v4_feature_state_t expected_feature)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t command_result;
    int32_t status_word;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t read_result;

    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(), function, 0U, 0U, request);
    command_result = MULTIPARAM_V4_TransceiveOnce(request,
                                                  reply,
                                                  MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS);
    read_result = MULTIPARAM_V4_ReadIntParam(MULTIPARAM_V4_PARAM_STATUS, &status_word);
    if (read_result != NO_ERROR) {
        return (command_result == NO_ERROR) ? read_result : command_result;
    }
    MULTIPARAM_V4_DecodeOperatingState((uint32_t)status_word, &mode, &feature);
    if ((mode == MULTIPARAM_V4_MEASUREMENT_DENSITY) && (feature == expected_feature)) {
        return NO_ERROR;
    }
    return (command_result == NO_ERROR) ? SENSOR_STREAM_STATE_ERROR : command_result;
}

/*
 * 函数用途：把测水或磁零点功能调整到明确目标状态。
 * 调用场景：业务层请求使能或关闭功能，不允许业务层直接盲发L/M翻转。
 * 关键约束：L/M只能在密度模式开启且互斥；每次翻转后必须读R02确认事实。
 */
static uint32_t MULTIPARAM_V4_EnsureFeature(uint8_t enabled,
                                           multiparam_v4_feature_state_t target_feature,
                                           uint8_t target_function,
                                           multiparam_v4_feature_state_t opposite_feature,
                                           uint8_t opposite_function)
{
    uint32_t status_word;
    multiparam_v4_measurement_mode_t mode;
    multiparam_v4_feature_state_t feature;
    uint32_t result;

    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    result = MULTIPARAM_V4_ReadOperatingState(&status_word, &mode, &feature);
    if (result != NO_ERROR) {
        return result;
    }
    if (mode != MULTIPARAM_V4_MEASUREMENT_DENSITY) {
        if ((enabled == 0U) && (mode == MULTIPARAM_V4_MEASUREMENT_LEVEL)) {
            /* 进入液位模式会自动关闭L/M，关闭请求已经满足。 */
            return NO_ERROR;
        }
        if ((enabled == 0U) || (mode != MULTIPARAM_V4_MEASUREMENT_LEVEL)) {
            return SENSOR_STREAM_STATE_ERROR;
        }
        /* L/M只能在密度模式开启；仅在确实需要时发送D，避免D关闭已开启功能。 */
        result = MULTIPARAM_V4_SelectDensityMode();
        if (result != NO_ERROR) {
            return result;
        }
        feature = MULTIPARAM_V4_FEATURE_NONE;
    }

    if (enabled == 0U) {
        if (feature != target_feature) {
            return NO_ERROR;
        }
        return MULTIPARAM_V4_ToggleFeatureOnce(target_function, MULTIPARAM_V4_FEATURE_NONE);
    }
    if (feature == target_feature) {
        return NO_ERROR;
    }
    if (feature == opposite_feature) {
        result = MULTIPARAM_V4_ToggleFeatureOnce(opposite_function, MULTIPARAM_V4_FEATURE_NONE);
        if (result != NO_ERROR) {
            return result;
        }
    } else if (feature != MULTIPARAM_V4_FEATURE_NONE) {
        return SENSOR_STREAM_STATE_ERROR;
    }
    return MULTIPARAM_V4_ToggleFeatureOnce(target_function, target_feature);
}

/*
 * 函数用途：把V4测水功能调整为调用方要求的明确状态。
 * 调用场景：水位电容读取前使能或串口调试关闭。
 * 关键约束：只能在密度模式开启；开启前先关闭互斥的零点霍尔，完成后保持当前状态。
 */
uint32_t MULTIPARAM_V4_EnsureWaterEnabled(uint8_t enabled)
{
    return MULTIPARAM_V4_EnsureFeature(enabled,
                                       MULTIPARAM_V4_FEATURE_WATER,
                                       MULTIPARAM_V4_FUNCTION_WATER,
                                       MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO,
                                       MULTIPARAM_V4_FUNCTION_MAGNETIC_ZERO);
}

/*
 * 函数用途：把V4零点霍尔功能调整为调用方要求的明确状态。
 * 调用场景：磁零点电压读取前使能或串口调试关闭。
 * 关键约束：只能在密度模式开启；开启前先关闭互斥的测水功能，完成后保持当前状态。
 */
uint32_t MULTIPARAM_V4_EnsureMagneticZeroEnabled(uint8_t enabled)
{
    return MULTIPARAM_V4_EnsureFeature(enabled,
                                       MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO,
                                       MULTIPARAM_V4_FUNCTION_MAGNETIC_ZERO,
                                       MULTIPARAM_V4_FEATURE_WATER,
                                       MULTIPARAM_V4_FUNCTION_WATER);
}

/*
 * 函数用途：在主动帧后的安全窗口内写R65=1，把V4从主动上发切到交互通信。
 * 调用场景：模式切换、功能控制和交互参数读取需要暂时停流时。
 * 关键约束：完整主动帧后至少等待15ms且不晚于100ms；主动帧抢占时保存帧并重新等待窗口。
 */
uint32_t MULTIPARAM_V4_EnterInteractive(void)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t result;
    uint32_t readback = 0U;
    uint32_t operation_start;
    uint32_t restore_result;
    uint8_t active_preempted;

    if ((s_communication_mode == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE) &&
        (s_active_receive_requested == 0U)) {
        return NO_ERROR;
    }
    if (s_active_receive_requested == 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    operation_start = HAL_GetTick();
    for (;;) {
        uint32_t event_age;

        MULTIPARAM_V4_Service();
        if (s_last_active_frame_end_valid != 0U) {
            event_age = HAL_GetTick() - s_last_active_frame_end_tick;
            /* 15ms是最早发送边界；100ms是本周期最晚边界，超出后等待下一帧以免撞帧。 */
            if ((event_age >= MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS) &&
                (event_age <= MULTIPARAM_V4_ACTIVE_WINDOW_LATEST_MS)) {
                MULTIPARAM_V4_StopActiveReceive();
                /* 停止DMA前可能刚完成另一帧解析，发送前再次执行15ms半双工门禁。 */
                MULTIPARAM_V4_WaitActiveFrameGuard();
                MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(),
                                                MULTIPARAM_V4_FUNCTION_WRITE,
                                                1U,
                                                MULTIPARAM_V4_PARAM_COMMUNICATION_MODE,
                                                request);
                result = MULTIPARAM_V4_TransceiveStopActiveOnce(
                    request, reply, MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS,
                    &active_preempted);
                if (active_preempted != 0U) {
                    if (result != NO_ERROR) {
                        return result;
                    }
                    /* 主动帧优先，保留该帧后等待它的15ms保护期再重试停流。 */
                    continue;
                }
                break;
            }
        }
        if ((HAL_GetTick() - operation_start) >=
            MULTIPARAM_V4_STOP_ACTIVE_TOTAL_TIMEOUT_MS) {
            return SENSOR_DEVICE_COMM_TIMEOUT;
        }
        HAL_Delay(1U);
    }

    if (result == NO_ERROR) {
        s_communication_mode = MULTIPARAM_V4_COMMUNICATION_INTERACTIVE;
        return NO_ERROR;
    }
    if (result == STATE_SWITCH) {
        /* 新命令优先：只恢复原主动接收，不再追加读回事务。 */
        restore_result = MULTIPARAM_V4_ResumeActiveReceivePreservingSnapshot();
        return (restore_result == NO_ERROR) ? STATE_SWITCH : restore_result;
    }

    /* 失败事务若刚收到主动帧，读回通信方式前仍需满足同一半双工保护期。 */
    MULTIPARAM_V4_WaitActiveFrameGuard();
    if ((MULTIPARAM_V4_ReadParamRawInternal(MULTIPARAM_V4_PARAM_COMMUNICATION_MODE,
                                            &readback,
                                            0U,
                                            MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS) == NO_ERROR) &&
        (readback == 1U)) {
        s_communication_mode = MULTIPARAM_V4_COMMUNICATION_INTERACTIVE;
        return NO_ERROR;
    }

    if (MULTIPARAM_V4_StartActiveReceive() != NO_ERROR) {
        return COMM_UART_TRANSFER_ERROR;
    }
    return result;
}

/*
 * 函数用途：写R65=0并重新建立V4主动上发常驻接收。
 * 调用场景：交互模式命令或功能操作完成后的统一收尾。
 * 关键约束：恢复属于必要收尾，不被已到达的新命令提前取消；应答丢失时用首个主动帧确认事实。
 */
uint32_t MULTIPARAM_V4_EnterActive(void)
{
    uint8_t request[8];
    uint8_t reply[8];
    uint32_t result;
    uint32_t start_result;
    uint32_t start_tick;
    uint32_t previous_generation = s_latest_snapshot.generation;

    if ((s_communication_mode == MULTIPARAM_V4_COMMUNICATION_ACTIVE) &&
        (s_active_receive_requested != 0U)) {
        return NO_ERROR;
    }
    if (s_active_receive_requested != 0U) {
        return SENSOR_STREAM_STATE_ERROR;
    }

    MULTIPARAM_V4_BuildRequestFrame(MULTIPARAM_V4_GetRequestAddress(),
                                    MULTIPARAM_V4_FUNCTION_WRITE,
                                    0U,
                                    MULTIPARAM_V4_PARAM_COMMUNICATION_MODE,
                                    request);
    /* 恢复原主动通信属于必要收尾，不能被已经到达的新命令在发送前打断。 */
    result = MULTIPARAM_V4_TransceiveOnceInternal(
        request, reply, MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS, 0U,
        MULTIPARAM_V4_REPLY_ADDRESS_STRICT);
    start_result = MULTIPARAM_V4_StartActiveReceive();
    if (start_result != NO_ERROR) {
        return start_result;
    }
    /* 写入已确认时，主动接收启动后即可让位给新命令；帧接收继续由中断链推进。 */
    if ((result == NO_ERROR) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }
    /* 应答丢失时仍等待首帧恢复事实，避免把未确认的通信方式当作主动模式。 */
    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < MULTIPARAM_V4_ACTIVE_TIMEOUT_MS) {
        if ((result == NO_ERROR) && HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }
        MULTIPARAM_V4_Service();
        if (s_latest_snapshot.generation != previous_generation) {
            return NO_ERROR;
        }
        HAL_Delay(1U);
    }
    return (result == NO_ERROR) ? SENSOR_DEVICE_COMM_TIMEOUT : result;
}
