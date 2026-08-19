/*
 * 模块职责：实现Safe会话状态机、命令序号、防重放计数和周期流启停约束。
 * 分层边界：本文件只验收状态转换和会话语义，不收发UART字节，也不解释测量物理量。
 * 安全约束：传感器启动计数或会话计数倒退、响应不匹配及序号异常都必须拒绝。
 */
#include "sensor_safe_session.h"

#include <limits.h>
#include <string.h>

/**
 * @brief 清除当前会话和周期流运行态，但保留跨会话的传感器防重放历史。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 */
static void SensorSafeSession_ClearRuntime(SensorSafeSessionContext *context)
{
    context->session_id = 0U;
    context->sensor_id = 0U;
    context->safety_param_crc = 0U;
    context->capability_flags = 0U;
    context->pending_seq = 0U;
    context->pending_cmd = 0U;
    context->pending_active = 0U;
    context->hello_active = 0U;
    context->stream_id = 0U;
    context->config_epoch = 0U;
    context->expected_stream_seq = 0U;
    context->last_sample_counter = 0U;
    context->sample_counter_valid = 0U;
    context->expected_measure_mode = (uint8_t)SENSOR_SAFE_MEASURE_IDLE;
    context->stream_valid = 0U;
    context->last_valid_rx_ms = 0U;
}

/**
 * @brief 根据当前测量模式选择必须判无效的通道诊断位。
 *
 * @param measure_mode 测量模式。
 * @return 返回根据当前测量模式选择必须判无效的通道诊断位对应的位掩码；各位含义由相邻枚举或宏定义。
 */
static uint16_t SensorSafeSession_ChannelDiagnosticMask(uint8_t measure_mode)
{
    switch (measure_mode) {
    case (uint8_t)SENSOR_SAFE_MEASURE_DENSITY:
        return (uint16_t)SENSOR_SAFE_DIAG_DENSITY_CHANNEL_ERROR;
    case (uint8_t)SENSOR_SAFE_MEASURE_LEVEL:
        return (uint16_t)SENSOR_SAFE_DIAG_LEVEL_CHANNEL_ERROR;
    case (uint8_t)SENSOR_SAFE_MEASURE_WATER_CAP:
        return (uint16_t)SENSOR_SAFE_DIAG_WATER_CAP_ERROR;
    case (uint8_t)SENSOR_SAFE_MEASURE_GYRO:
        return (uint16_t)SENSOR_SAFE_DIAG_GYRO_CHANNEL_ERROR;
    default:
        return 0U;
    }
}

/**
 * @brief 为新事务分配单调序号并锁存待应答命令。
 *
 * @details 调用场景：HELLO 和全部控制请求在编码前调用。
 * @note 关键约束：同一时刻只允许一个待决事务；序号临近回绕时强制重新 HELLO。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param cmd 安全协议控制命令字节；写入请求帧并与待确认会话状态绑定，响应必须回显同一命令才可接受。
 * @param seq_out 用于返回本次新分配或重试沿用的协议事务序号。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
static SensorSafeResult SensorSafeSession_AllocateSequence(SensorSafeSessionContext *context,
                                                           uint8_t cmd,
                                                           uint32_t *seq_out)
{
    if (context->pending_active != 0U) {
        return SENSOR_SAFE_TRANSACTION_PENDING;
    }
    /*
     * UINT32_MAX 本身虽可编码，但使用后无法在同一会话继续分配新序号。
     * 提前一个事务要求重建会话，可消除回绕复用和临界分支。
     */
    if ((context->next_seq == 0U) || (context->next_seq == UINT32_MAX)) {
        return SENSOR_SAFE_NEEDS_HELLO;
    }

    context->pending_seq = context->next_seq;
    context->pending_cmd = cmd;
    context->pending_active = 1U;
    context->next_seq++;
    *seq_out = context->pending_seq;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 初始化会话状态机和本地、远端节点地址。
 *
 * @details 调用场景：client 初始化时调用，不建立实际安全会话。
 * @note 关键约束：初态为 OFFLINE，首个合法业务必须先完成 HELLO。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param local_node_id 编号。该值是 CPU2 在安全协议中的本机节点号，初始化会话后写入每个控制请求的源节点字段。
 * @param remote_node_id 编号。该值是目标安全传感器节点号，响应源节点必须与该值一致才可通过会话校验。
 */
void SensorSafeSession_Init(SensorSafeSessionContext *context,
                            uint8_t local_node_id,
                            uint8_t remote_node_id)
{
    if (context == NULL) {
        return;
    }
    (void)memset(context, 0, sizeof(*context));
    context->state = SENSOR_SAFE_SESSION_OFFLINE;
    context->local_node_id = local_node_id;
    context->remote_node_id = remote_node_id;
    context->next_seq = 1U;
}

/**
 * @brief 撤销当前会话、待决事务和周期流运行态。
 *
 * @details 调用场景：协议探测失败、旧协议回退或显式下线时调用。
 * @note 关键约束：保留传感器启动/会话计数历史，防止下次 HELLO 接受重放。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 */
void SensorSafeSession_Deactivate(SensorSafeSessionContext *context)
{
    if (context == NULL) {
        return;
    }
    context->state = SENSOR_SAFE_SESSION_OFFLINE;
    SensorSafeSession_ClearRuntime(context);
    context->next_seq = 1U;
}

/**
 * @brief 开始 HELLO 事务并锁存 CPU 启动计数与挑战随机数。
 *
 * @details 调用场景：首次探测或故障恢复需要重建安全会话时调用。
 * @note 关键约束：禁止复用当前或已有防重放历史中的 nonce；开始时清除旧运行态。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param cpu_boot_counter CPU2 本次启动计数；写入 HELLO 会话上下文，用于和传感器共同建立防重放基线。
 * @param cpu_nonce CPU2 为本次 HELLO 生成的 32 位挑战随机数；不得复用当前会话或历史防重放记录中的值。
 * @param seq_out 用于返回本次新分配或重试沿用的协议事务序号。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_BeginHello(SensorSafeSessionContext *context,
                                              uint32_t cpu_boot_counter,
                                              uint32_t cpu_nonce,
                                              uint32_t *seq_out)
{
    SensorSafeResult result;

    if ((context == NULL) || (seq_out == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((context->cpu_nonce == cpu_nonce) &&
        ((context->hello_active != 0U) || (context->sensor_counter_valid != 0U))) {
        return SENSOR_SAFE_REPLAY_DETECTED;
    }

    context->state = SENSOR_SAFE_SESSION_OFFLINE;
    SensorSafeSession_ClearRuntime(context);
    context->next_seq = 1U;
    context->cpu_boot_counter = cpu_boot_counter;
    context->cpu_nonce = cpu_nonce;
    context->hello_active = 1U;
    result = SensorSafeSession_AllocateSequence(context, (uint8_t)SENSOR_SAFE_CMD_HELLO, seq_out);
    if (result != SENSOR_SAFE_OK) {
        context->hello_active = 0U;
    }
    return result;
}

/**
 * @brief 依据会话状态和周期流命令白名单开始普通控制事务。
 *
 * @details 调用场景：client 发送测量、配置、诊断及通信模式命令前调用。
 * @note 关键约束：周期流活动或恢复态仅允许停流、PING、RESET_SESSION 类控制命令。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param cmd 安全协议控制命令字节；写入请求帧并与待确认会话状态绑定，响应必须回显同一命令才可接受。
 * @param seq_out 用于返回本次新分配或重试沿用的协议事务序号。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_BeginRequest(SensorSafeSessionContext *context,
                                                uint8_t cmd,
                                                uint32_t *seq_out)
{
    if ((context == NULL) || (seq_out == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((context->state != SENSOR_SAFE_SESSION_READY) &&
        (context->state != SENSOR_SAFE_SESSION_MEASURE_READY) &&
        (context->state != SENSOR_SAFE_SESSION_PERIODIC_ACTIVE) &&
        (context->state != SENSOR_SAFE_SESSION_RECOVERING)) {
        return SENSOR_SAFE_NEEDS_HELLO;
    }
    if (cmd == (uint8_t)SENSOR_SAFE_CMD_HELLO) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((context->state == SENSOR_SAFE_SESSION_PERIODIC_ACTIVE) ||
        (context->state == SENSOR_SAFE_SESSION_RECOVERING)) {
        if ((cmd != (uint8_t)SENSOR_SAFE_CMD_SET_COMM_MODE) &&
            (cmd != (uint8_t)SENSOR_SAFE_CMD_PING) &&
            (cmd != (uint8_t)SENSOR_SAFE_CMD_RESET_SESSION)) {
            return SENSOR_SAFE_INVALID_ARGUMENT;
        }
    }
    return SensorSafeSession_AllocateSequence(context, cmd, seq_out);
}

/**
 * @brief 读取待决命令和原序号，确保超时重发不分配新序号而破坏幂等识别。
 *
 * @param context 安全协议会话只读上下文；用于核对会话标识、待确认命令和序号、防重放计数及周期流边界，不修改状态机。
 * @param cmd_out 用于返回当前待重试的协议命令码。
 * @param seq_out 用于返回本次新分配或重试沿用的协议事务序号。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_RetryPending(const SensorSafeSessionContext *context,
                                                uint8_t *cmd_out,
                                                uint32_t *seq_out)
{
    if ((context == NULL) || (cmd_out == NULL) || (seq_out == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (context->pending_active == 0U) {
        return SENSOR_SAFE_BAD_SEQUENCE;
    }
    *cmd_out = context->pending_cmd;
    *seq_out = context->pending_seq;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 核对控制响应的类型、地址、命令、序号及会话身份字段。
 *
 * @details 调用场景：控制帧 CRC 和线格式通过后、解释业务载荷之前调用。
 * @note 关键约束：HELLO 与普通事务采用不同身份规则；ERR 也必须绑定当前待决事务。
 *
 * @param context 安全协议会话只读上下文；用于核对会话标识、待确认命令和序号、防重放计数及周期流边界，不修改状态机。
 * @param frame 待解析、校验或发送的协议帧缓冲区。该参数实际是已经完成帧级解码的只读安全协议控制帧，用于会话、命令和序号核对。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_CheckControlResponse(const SensorSafeSessionContext *context,
                                                        const SensorSafeControlFrame *frame)
{
    if ((context == NULL) || (frame == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((frame->fields.msg_type != (uint8_t)SENSOR_SAFE_MSG_RSP) &&
        (frame->fields.msg_type != (uint8_t)SENSOR_SAFE_MSG_ERR)) {
        return SENSOR_SAFE_BAD_MESSAGE_TYPE;
    }
    if ((frame->fields.src_id != context->remote_node_id) ||
        (frame->fields.dst_id != context->local_node_id)) {
        return SENSOR_SAFE_BAD_ADDRESS;
    }
    if ((context->pending_active == 0U) ||
        (frame->fields.seq != context->pending_seq) ||
        (frame->fields.cmd != context->pending_cmd)) {
        return SENSOR_SAFE_BAD_SEQUENCE;
    }

    if (context->pending_cmd == (uint8_t)SENSOR_SAFE_CMD_HELLO) {
        if (frame->fields.msg_type == (uint8_t)SENSOR_SAFE_MSG_ERR) {
            return SENSOR_SAFE_OK;
        }
        if ((frame->fields.session_id == 0U) ||
            (frame->fields.sensor_id == 0U) ||
            (frame->fields.safety_param_crc == 0U)) {
            return SENSOR_SAFE_BAD_SESSION;
        }
        return SENSOR_SAFE_OK;
    }

    if (frame->fields.session_id != context->session_id) {
        return SENSOR_SAFE_BAD_SESSION;
    }
    if (frame->fields.sensor_id != context->sensor_id) {
        return SENSOR_SAFE_BAD_SENSOR;
    }
    if (frame->fields.safety_param_crc != context->safety_param_crc) {
        return SENSOR_SAFE_BAD_PARAM_CRC;
    }
    return SENSOR_SAFE_OK;
}

/**
 * @brief 验收 HELLO 回显、传感器身份、能力位和单调计数并激活会话。
 *
 * @details 调用场景：client 完成 HELLO 响应载荷解析后调用。
 * @note 关键约束：启动计数不得倒退，同一启动周期的会话计数必须严格递增。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param frame 待解析、校验或发送的协议帧缓冲区。该参数实际是已经完成帧级解码的只读安全协议控制帧，用于会话、命令和序号核对。
 * @param hello 已经解码并等待会话层验收的 HELLO 响应载荷。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_AcceptHello(SensorSafeSessionContext *context,
                                               const SensorSafeControlFrame *frame,
                                               const SensorSafeHelloAcceptance *hello)
{
    SensorSafeResult result;

    if ((context == NULL) || (frame == NULL) || (hello == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    result = SensorSafeSession_CheckControlResponse(context, frame);
    if (result != SENSOR_SAFE_OK) {
        return result;
    }
    if ((context->hello_active == 0U) ||
        (hello->selected_protocol != SENSOR_SAFE_PROTOCOL_VERSION) ||
        (hello->sensor_node_id != context->remote_node_id)) {
        return SENSOR_SAFE_BAD_VERSION;
    }
    if ((hello->echo_cpu_boot_counter != context->cpu_boot_counter) ||
        (hello->echo_cpu_nonce != context->cpu_nonce)) {
        return SENSOR_SAFE_REPLAY_DETECTED;
    }
    if ((hello->new_session_id == 0U) ||
        (hello->new_session_id != frame->fields.session_id)) {
        return SENSOR_SAFE_BAD_SESSION;
    }
    if ((context->sensor_counter_valid != 0U) &&
        (hello->new_session_id == context->last_session_id)) {
        return SENSOR_SAFE_REPLAY_DETECTED;
    }
    if ((hello->sensor_id == 0U) ||
        (hello->sensor_id != frame->fields.sensor_id)) {
        return SENSOR_SAFE_BAD_SENSOR;
    }
    if ((hello->safety_param_crc == 0U) ||
        (hello->safety_param_crc != frame->fields.safety_param_crc)) {
        return SENSOR_SAFE_BAD_PARAM_CRC;
    }
    if ((hello->capability_flags & ~SENSOR_SAFE_ALLOWED_CAPABILITIES) != 0U) {
        return SENSOR_SAFE_BAD_CAPABILITY;
    }
    if (hello->sensor_session_counter == 0U) {
        return SENSOR_SAFE_REPLAY_DETECTED;
    }

    /* 跨会话保留的单调历史是重放防护边界，不能随普通下线清零。 */
    if (context->sensor_counter_valid != 0U) {
        if (hello->sensor_boot_counter < context->last_sensor_boot_counter) {
            return SENSOR_SAFE_REPLAY_DETECTED;
        }
        if ((hello->sensor_boot_counter == context->last_sensor_boot_counter) &&
            (hello->sensor_session_counter <= context->last_sensor_session_counter)) {
            return SENSOR_SAFE_REPLAY_DETECTED;
        }
    }

    context->last_sensor_boot_counter = hello->sensor_boot_counter;
    context->last_sensor_session_counter = hello->sensor_session_counter;
    context->last_session_id = hello->new_session_id;
    context->sensor_counter_valid = 1U;
    context->session_id = hello->new_session_id;
    context->sensor_id = hello->sensor_id;
    context->safety_param_crc = hello->safety_param_crc;
    context->capability_flags = hello->capability_flags;
    context->pending_active = 0U;
    context->hello_active = 0U;
    context->state = SENSOR_SAFE_SESSION_READY;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 成功完成当前控制事务并释放待决锁；无待决事务时拒绝误清状态。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_CompleteRequest(SensorSafeSessionContext *context)
{
    if (context == NULL) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (context->pending_active == 0U) {
        return SENSOR_SAFE_BAD_SEQUENCE;
    }
    context->pending_active = 0U;
    context->pending_seq = 0U;
    context->pending_cmd = 0U;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 把已确认的测量模式锁存为请求响应和后续周期流的预期模式。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param measure_mode 测量模式。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_SetMeasureReady(SensorSafeSessionContext *context,
                                                   uint8_t measure_mode)
{
    if (context == NULL) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((context->state != SENSOR_SAFE_SESSION_READY) &&
        (context->state != SENSOR_SAFE_SESSION_MEASURE_READY)) {
        return SENSOR_SAFE_BAD_SESSION;
    }
    if (measure_mode > (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    context->expected_measure_mode = measure_mode;
    context->state = SENSOR_SAFE_SESSION_MEASURE_READY;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 以已协商的 stream_id、配置代次和测量模式进入周期接收态。
 *
 * @details 调用场景：SET_COMM_MODE 启动响应验收成功后调用。
 * @note 关键约束：标识和配置代次不得为 0，模式必须与此前 SET_MEASURE_MODE 一致。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param stream_id 数据流编号。
 * @param config_epoch 配置。
 * @param measure_mode 测量模式。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_StartStream(SensorSafeSessionContext *context,
                                               uint16_t stream_id,
                                               uint16_t config_epoch,
                                               uint8_t measure_mode)
{
    if (context == NULL) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (context->state != SENSOR_SAFE_SESSION_MEASURE_READY) {
        return SENSOR_SAFE_BAD_SESSION;
    }
    if ((stream_id == 0U) || (config_epoch == 0U) ||
        (measure_mode > (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM) ||
        (measure_mode != context->expected_measure_mode)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }

    context->stream_id = stream_id;
    context->config_epoch = config_epoch;
    context->expected_stream_seq = 1U;
    context->last_sample_counter = 0U;
    context->sample_counter_valid = 0U;
    context->expected_measure_mode = measure_mode;
    context->stream_valid = 1U;
    context->last_valid_rx_ms = 0U;
    context->state = SENSOR_SAFE_SESSION_PERIODIC_ACTIVE;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 正常结束周期流并回到测量就绪态，同时清除全部流连续性基准。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_StopStream(SensorSafeSessionContext *context)
{
    if (context == NULL) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((context->state != SENSOR_SAFE_SESSION_PERIODIC_ACTIVE) &&
        (context->state != SENSOR_SAFE_SESSION_RECOVERING)) {
        return SENSOR_SAFE_BAD_SESSION;
    }
    context->stream_id = 0U;
    context->config_epoch = 0U;
    context->expected_stream_seq = 0U;
    context->last_sample_counter = 0U;
    context->sample_counter_valid = 0U;
    context->stream_valid = 0U;
    context->state = SENSOR_SAFE_SESSION_MEASURE_READY;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 周期帧异常时立即撤销流有效性并转入恢复态，禁止继续发布快照。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 */
void SensorSafeSession_InvalidateStream(SensorSafeSessionContext *context)
{
    if (context == NULL) {
        return;
    }
    context->stream_valid = 0U;
    context->state = SENSOR_SAFE_SESSION_RECOVERING;
}

/**
 * @brief 验收周期快报的地址、会话、流序号、样本计数、模式和数据状态。
 *
 * @details 调用场景：快报线格式及 CRC 通过后、发布监控快照之前调用。
 * @note 关键约束：身份或连续性异常立即使流失效；质量无效只拒绝数据但推进已验证序号。
 *
 * @param context 安全协议会话状态机上下文；保存本机与传感器节点号、会话和能力协商结果、待确认命令与序号、防重放计数、周期流标识和最近有效接收时间。
 * @param report 用于接收本次诊断或测量结果的报告对象。
 * @param rx_timestamp_ms 完整接收本帧时记录的本机 HAL 毫秒节拍；作为指针传入时由函数写回。
 * @param max_data_age_ms 允许样本保持有效的最大数据年龄，单位 ms。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeSession_AcceptFastReport(SensorSafeSessionContext *context,
                                                    const SensorSafeFastReport *report,
                                                    uint32_t rx_timestamp_ms,
                                                    uint32_t max_data_age_ms)
{
    uint8_t data_valid;

    if ((context == NULL) || (report == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((context->state != SENSOR_SAFE_SESSION_PERIODIC_ACTIVE) ||
        (context->stream_valid == 0U)) {
        return SENSOR_SAFE_NEEDS_HELLO;
    }
    if ((report->src_id != context->remote_node_id) ||
        (report->dst_id != context->local_node_id)) {
        SensorSafeSession_InvalidateStream(context);
        return SENSOR_SAFE_BAD_ADDRESS;
    }
    if (report->session_id != context->session_id) {
        SensorSafeSession_InvalidateStream(context);
        return SENSOR_SAFE_BAD_SESSION;
    }
    if ((report->stream_id != context->stream_id) ||
        (report->stream_seq != context->expected_stream_seq)) {
        SensorSafeSession_InvalidateStream(context);
        return SENSOR_SAFE_BAD_SEQUENCE;
    }
    if (report->config_epoch != context->config_epoch) {
        SensorSafeSession_InvalidateStream(context);
        return SENSOR_SAFE_BAD_CONFIG_EPOCH;
    }
    if ((context->sample_counter_valid != 0U) &&
        (report->sample_counter <= context->last_sample_counter)) {
        SensorSafeSession_InvalidateStream(context);
        return SENSOR_SAFE_BAD_SAMPLE_COUNTER;
    }
    if ((report->measure_mode != context->expected_measure_mode) ||
        (report->comm_mode != (uint8_t)SENSOR_SAFE_COMM_PERIODIC_UPLINK)) {
        SensorSafeSession_InvalidateStream(context);
        return SENSOR_SAFE_MODE_MISMATCH;
    }

    /*
     * 完整性和连续性字段通过后才推进序号。数据质量无效时仍推进已验证的线上序号，
     * 避免下一帧被误判为跳号；安全快照是否更新由返回值决定。
     */
    context->expected_stream_seq++;
    if (context->expected_stream_seq == 0U) {
        SensorSafeSession_InvalidateStream(context);
        return SENSOR_SAFE_NEEDS_HELLO;
    }
    context->last_sample_counter = report->sample_counter;
    context->sample_counter_valid = 1U;
    /*
     * 完整性、身份、连续性和模式已通过即证明链路收到一帧协议合法快报。
     * 即使后续状态或数据年龄判无效，也要刷新通信静默基准；安全快照仍由返回值阻断。
     */
    context->last_valid_rx_ms = rx_timestamp_ms;

    data_valid = (uint8_t)(((report->status_flags16 & SENSOR_SAFE_STATUS_DATA_VALID) != 0U) &&
                           ((report->status_flags16 & SENSOR_SAFE_STATUS_DATA_STALE) == 0U) &&
                           ((report->status_flags16 & SENSOR_SAFE_STATUS_MODE_MATCH) != 0U) &&
                           ((report->status_flags16 & SENSOR_SAFE_STATUS_MODE_SETTLING) == 0U) &&
                           ((report->status_flags16 & SENSOR_SAFE_STATUS_CONFIG_MISMATCH) == 0U) &&
                           ((report->status_flags16 & SENSOR_SAFE_STATUS_FAST_INVALID_MASK) == 0U) &&
                           ((report->status_flags16 & SENSOR_SAFE_STATUS_STREAM_ACTIVE) != 0U));
    if (data_valid == 0U) {
        return SENSOR_SAFE_DATA_INVALID;
    }
    if ((report->diag_flags16 &
         (uint16_t)((uint16_t)SENSOR_SAFE_DIAG_FAST_INVALID_MASK |
                    SensorSafeSession_ChannelDiagnosticMask(report->measure_mode))) != 0U) {
        return SENSOR_SAFE_DATA_INVALID;
    }
    if ((uint32_t)report->data_age_ms > max_data_age_ms) {
        return SENSOR_SAFE_DATA_STALE;
    }

    return SENSOR_SAFE_OK;
}

/**
 * @brief 把传感器声明的数据年龄与本地接收后经过时间相加。
 *
 * @details 调用场景：周期帧首次验收和业务读取快照时调用。
 * @note 关键约束：加法溢出时饱和为 UINT32_MAX，确保数据必然被判过期而非变新。
 *
 * @param data_age_ms 传感器在快速上报帧中给出的样本年龄，单位 ms。
 * @param rx_timestamp_ms 完整接收本帧时记录的本机 HAL 毫秒节拍；作为指针传入时由函数写回。
 * @param now_ms 当前系统节拍，单位 ms。
 * @return 返回远端声明数据年龄与本地接收后经过时间之和，单位 ms；加法溢出时饱和到 UINT32_MAX。
 */
uint32_t SensorSafeSession_EffectiveAgeMs(uint16_t data_age_ms,
                                          uint32_t rx_timestamp_ms,
                                          uint32_t now_ms)
{
    uint32_t local_age = now_ms - rx_timestamp_ms;
    uint32_t wire_age = (uint32_t)data_age_ms;

    if (local_age > (UINT32_MAX - wire_age)) {
        return UINT32_MAX;
    }
    return wire_age + local_age;
}
