/*
 * 模块职责：逐字段编码和解码Safe控制帧、固定快报帧，并完成线格式基础校验。
 * 分层边界：本文件只处理字节序、长度、类型、保留位和CRC，不判断会话时序或业务能力。
 * 安全约束：禁止对C结构体内存直接求CRC，所有多字节字段均按冻结小端格式显式读写。
 */
#include "sensor_safe_frame.h"

#include <string.h>

#include "sensor_safe_crc32c.h"

/**
 * @brief 从冻结的小端线格式读取 16 位字段，避免依赖 CPU 对齐和主机字节序。
 *
 * @param data 安全协议帧中的连续小端字段首地址；具体读写宽度由当前 U16 或 U32 辅助函数固定。
 * @return 返回从冻结的小端线格式读取 16 位字段，避免依赖 CPU 对齐和主机字节序得到的主机整数值；函数只处理既定字节序或编码，不执行范围校验。
 */
static uint16_t SensorSafe_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/**
 * @brief 从冻结的小端线格式读取 32 位字段，调用方必须先完成边界检查。
 *
 * @param data 安全协议帧中的连续小端字段首地址；具体读写宽度由当前 U16 或 U32 辅助函数固定。
 * @return 返回从冻结的小端线格式读取 32 位字段，调用方必须先完成边界检查得到的主机整数值；函数只处理既定字节序或编码，不执行范围校验。
 */
static uint32_t SensorSafe_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/**
 * @brief 把 16 位字段逐字节写入线格式，禁止直接强制转换未对齐指针。
 *
 * @param data 安全协议帧中的连续小端字段首地址；具体读写宽度由当前 U16 或 U32 辅助函数固定。
 * @param value 待编码写入线格式的 16 位数值。
 */
static void SensorSafe_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/**
 * @brief 把 32 位字段逐字节写入线格式，保证不同编译器生成相同帧。
 *
 * @param data 安全协议帧中的连续小端字段首地址；具体读写宽度由当前 U16 或 U32 辅助函数固定。
 * @param value 待编码写入线格式的 32 位数值。
 */
static void SensorSafe_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/**
 * @brief 只接受协议卷冻结的请求、响应和错误三类控制消息。
 *
 * @param msg_type 类型。
 * @return 1 表示消息类型是冻结的请求、响应或错误控制帧；其他保留类型返回 0。
 */
static uint8_t SensorSafe_IsControlMessageType(uint8_t msg_type)
{
    return (uint8_t)(((msg_type == (uint8_t)SENSOR_SAFE_MSG_REQ) ||
                      (msg_type == (uint8_t)SENSOR_SAFE_MSG_RSP) ||
                      (msg_type == (uint8_t)SENSOR_SAFE_MSG_ERR)) ? 1U : 0U);
}

/**
 * @brief 仅凭已接收前缀判定候选帧类型和完整帧长度。
 *
 * @details 调用场景：UART6 流式收包在等待完整帧前调用，不解析业务字段。
 * @note 关键约束：先检查 SOF 和最小前缀；异常长度不得驱动后续缓冲区访问。
 *
 * @param data 安全协议帧中的连续小端字段首地址；具体读写宽度由当前 U16 或 U32 辅助函数固定。
 * @param available 当前接收缓冲区中已经可供判帧的字节数。
 * @param frame_len_out 用于返回根据当前帧前缀判定的完整候选帧长度，单位字节。
 * @param is_fast_frame_out 用于返回候选帧是否为周期快速上报帧。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeFrame_PeekLength(const uint8_t *data,
                                            size_t available,
                                            uint16_t *frame_len_out,
                                            uint8_t *is_fast_frame_out)
{
    uint16_t frame_len;

    if ((data == NULL) || (frame_len_out == NULL) || (is_fast_frame_out == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (available < 4U) {
        return SENSOR_SAFE_BUFFER_TOO_SMALL;
    }
    if ((data[0] != SENSOR_SAFE_SOF0) || (data[1] != SENSOR_SAFE_SOF1)) {
        return SENSOR_SAFE_BAD_SOF;
    }

    /* 快报帧使用固定 8 位长度；其余帧按控制帧 16 位长度解释。 */
    if (data[SENSOR_SAFE_FAST_OFFSET_FRAME_TYPE] == SENSOR_SAFE_FAST_FRAME_TYPE) {
        frame_len = (uint16_t)data[SENSOR_SAFE_FAST_OFFSET_FRAME_LEN];
        if (frame_len != SENSOR_SAFE_FAST_FRAME_LEN) {
            return SENSOR_SAFE_BAD_LENGTH;
        }
        *frame_len_out = frame_len;
        *is_fast_frame_out = 1U;
        return SENSOR_SAFE_OK;
    }

    if (available < 6U) {
        return SENSOR_SAFE_BUFFER_TOO_SMALL;
    }
    frame_len = SensorSafe_ReadU16(&data[SENSOR_SAFE_CTRL_OFFSET_FRAME_LEN]);
    if ((frame_len < SENSOR_SAFE_CTRL_MIN_FRAME_LEN) ||
        (frame_len > SENSOR_SAFE_CTRL_MAX_FRAME_LEN)) {
        return SENSOR_SAFE_BAD_LENGTH;
    }
    *frame_len_out = frame_len;
    *is_fast_frame_out = 0U;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 把控制字段编码为冻结的小端线格式并追加 CRC32C。
 *
 * @details 调用场景：client 发起 HELLO、测量、配置和诊断控制事务时调用。
 * @note 关键约束：保留标志、消息类型和载荷上限必须在写缓冲区前完成校验。
 *
 * @param fields 待编码为安全协议控制帧的字段集合。
 * @param output 用于接收控制帧头、载荷和 CRC32C 的目标字节缓冲区。
 * @param output_capacity 输出容量。
 * @param output_len 用于返回实际编码后的完整控制帧长度，单位字节。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeFrame_EncodeControl(const SensorSafeControlFields *fields,
                                               uint8_t *output,
                                               size_t output_capacity,
                                               uint16_t *output_len)
{
    uint16_t frame_len;
    uint32_t crc;

    if ((fields == NULL) || (output == NULL) || (output_len == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if ((fields->payload == NULL) && (fields->payload_len != 0U)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (fields->payload_len > SENSOR_SAFE_CTRL_MAX_PAYLOAD_LEN) {
        return SENSOR_SAFE_BAD_LENGTH;
    }
    if ((fields->flags & (uint16_t)(~SENSOR_SAFE_ALLOWED_FLAGS)) != 0U) {
        return SENSOR_SAFE_BAD_RESERVED_FLAGS;
    }
    if (SensorSafe_IsControlMessageType(fields->msg_type) == 0U) {
        return SENSOR_SAFE_BAD_MESSAGE_TYPE;
    }

    frame_len = (uint16_t)(SENSOR_SAFE_CTRL_HEADER_LEN +
                           fields->payload_len +
                           SENSOR_SAFE_CTRL_CRC_LEN);
    if (output_capacity < (size_t)frame_len) {
        return SENSOR_SAFE_BUFFER_TOO_SMALL;
    }

    output[0] = SENSOR_SAFE_SOF0;
    output[1] = SENSOR_SAFE_SOF1;
    output[SENSOR_SAFE_CTRL_OFFSET_PROTOCOL_VERSION] = SENSOR_SAFE_PROTOCOL_VERSION;
    output[SENSOR_SAFE_CTRL_OFFSET_HEADER_LEN] = SENSOR_SAFE_CTRL_HEADER_LEN;
    SensorSafe_WriteU16(&output[SENSOR_SAFE_CTRL_OFFSET_FRAME_LEN], frame_len);
    output[SENSOR_SAFE_CTRL_OFFSET_SRC_ID] = fields->src_id;
    output[SENSOR_SAFE_CTRL_OFFSET_DST_ID] = fields->dst_id;
    output[SENSOR_SAFE_CTRL_OFFSET_MSG_TYPE] = fields->msg_type;
    output[SENSOR_SAFE_CTRL_OFFSET_CMD] = fields->cmd;
    SensorSafe_WriteU16(&output[SENSOR_SAFE_CTRL_OFFSET_FLAGS], fields->flags);
    SensorSafe_WriteU32(&output[SENSOR_SAFE_CTRL_OFFSET_SEQ], fields->seq);
    SensorSafe_WriteU32(&output[SENSOR_SAFE_CTRL_OFFSET_SESSION_ID], fields->session_id);
    SensorSafe_WriteU32(&output[SENSOR_SAFE_CTRL_OFFSET_SENSOR_ID], fields->sensor_id);
    SensorSafe_WriteU32(&output[SENSOR_SAFE_CTRL_OFFSET_PARAM_CRC], fields->safety_param_crc);
    if (fields->payload_len != 0U) {
        (void)memcpy(&output[SENSOR_SAFE_CTRL_OFFSET_PAYLOAD], fields->payload, fields->payload_len);
    }

    /* SOF 只用于定界，不进入 CRC；CRC 字段本身同样不参与计算。 */
    crc = SensorSafeCrc32c_Calculate(&output[SENSOR_SAFE_CTRL_OFFSET_PROTOCOL_VERSION],
                                     (size_t)frame_len - 2U - SENSOR_SAFE_CTRL_CRC_LEN);
    SensorSafe_WriteU32(&output[frame_len - SENSOR_SAFE_CTRL_CRC_LEN], crc);
    *output_len = frame_len;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 校验并解码控制帧，输出只引用调用方持有的原始载荷区。
 *
 * @details 调用场景：client 在会话语义校验之前执行线格式验收。
 * @note 关键约束：先校验长度和 CRC，再解释版本、标志及业务字段，失败时不得信任载荷。
 *
 * @param frame 待解析、校验或发送的协议帧缓冲区。该只读线端帧有效范围由 frame_len 指定，函数在解码业务字段前完成长度、帧头和 CRC32C 校验。
 * @param frame_len 协议帧有效长度，单位字节。
 * @param decoded 用于接收通过完整性校验后的解码帧或响应字段。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeFrame_DecodeControl(const uint8_t *frame,
                                               size_t frame_len,
                                               SensorSafeControlFrame *decoded)
{
    uint16_t wire_len;
    uint16_t flags;
    uint32_t expected_crc;
    uint32_t actual_crc;

    if ((frame == NULL) || (decoded == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (frame_len < SENSOR_SAFE_CTRL_MIN_FRAME_LEN) {
        return SENSOR_SAFE_BAD_LENGTH;
    }
    if ((frame[0] != SENSOR_SAFE_SOF0) || (frame[1] != SENSOR_SAFE_SOF1)) {
        return SENSOR_SAFE_BAD_SOF;
    }
    wire_len = SensorSafe_ReadU16(&frame[SENSOR_SAFE_CTRL_OFFSET_FRAME_LEN]);
    if ((wire_len != frame_len) || (wire_len > SENSOR_SAFE_CTRL_MAX_FRAME_LEN)) {
        return SENSOR_SAFE_BAD_LENGTH;
    }

    /* CRC 先于其余可控字段校验，避免使用传输损坏的版本和状态信息。 */
    expected_crc = SensorSafe_ReadU32(&frame[wire_len - SENSOR_SAFE_CTRL_CRC_LEN]);
    actual_crc = SensorSafeCrc32c_Calculate(&frame[SENSOR_SAFE_CTRL_OFFSET_PROTOCOL_VERSION],
                                            (size_t)wire_len - 2U - SENSOR_SAFE_CTRL_CRC_LEN);
    if (actual_crc != expected_crc) {
        return SENSOR_SAFE_BAD_CRC;
    }
    if (frame[SENSOR_SAFE_CTRL_OFFSET_PROTOCOL_VERSION] != SENSOR_SAFE_PROTOCOL_VERSION) {
        return SENSOR_SAFE_BAD_VERSION;
    }
    if (frame[SENSOR_SAFE_CTRL_OFFSET_HEADER_LEN] != SENSOR_SAFE_CTRL_HEADER_LEN) {
        return SENSOR_SAFE_BAD_HEADER;
    }
    if (SensorSafe_IsControlMessageType(frame[SENSOR_SAFE_CTRL_OFFSET_MSG_TYPE]) == 0U) {
        return SENSOR_SAFE_BAD_MESSAGE_TYPE;
    }
    flags = SensorSafe_ReadU16(&frame[SENSOR_SAFE_CTRL_OFFSET_FLAGS]);
    if ((flags & (uint16_t)(~SENSOR_SAFE_ALLOWED_FLAGS)) != 0U) {
        return SENSOR_SAFE_BAD_RESERVED_FLAGS;
    }

    decoded->fields.src_id = frame[SENSOR_SAFE_CTRL_OFFSET_SRC_ID];
    decoded->fields.dst_id = frame[SENSOR_SAFE_CTRL_OFFSET_DST_ID];
    decoded->fields.msg_type = frame[SENSOR_SAFE_CTRL_OFFSET_MSG_TYPE];
    decoded->fields.cmd = frame[SENSOR_SAFE_CTRL_OFFSET_CMD];
    decoded->fields.flags = flags;
    decoded->fields.seq = SensorSafe_ReadU32(&frame[SENSOR_SAFE_CTRL_OFFSET_SEQ]);
    decoded->fields.session_id = SensorSafe_ReadU32(&frame[SENSOR_SAFE_CTRL_OFFSET_SESSION_ID]);
    decoded->fields.sensor_id = SensorSafe_ReadU32(&frame[SENSOR_SAFE_CTRL_OFFSET_SENSOR_ID]);
    decoded->fields.safety_param_crc = SensorSafe_ReadU32(&frame[SENSOR_SAFE_CTRL_OFFSET_PARAM_CRC]);
    decoded->fields.payload = &frame[SENSOR_SAFE_CTRL_OFFSET_PAYLOAD];
    decoded->fields.payload_len = (uint16_t)(wire_len - SENSOR_SAFE_CTRL_HEADER_LEN - SENSOR_SAFE_CTRL_CRC_LEN);
    decoded->frame_len = wire_len;
    decoded->received_crc = expected_crc;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 校验固定长度周期快报并解码测量值、状态和诊断位。
 *
 * @details 调用场景：周期上报接收链路在会话防重放检查之前调用。
 * @note 关键约束：CRC、通信模式、测量模式、保留位和质量范围任一异常均拒绝整帧。
 *
 * @param frame 待解析、校验或发送的协议帧缓冲区。该只读线端帧有效范围由 frame_len 指定，函数在解码业务字段前完成长度、帧头和 CRC32C 校验。
 * @param frame_len 协议帧有效长度，单位字节。
 * @param decoded 用于接收通过完整性校验后的解码帧或响应字段。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeFrame_DecodeFastReport(const uint8_t *frame,
                                                  size_t frame_len,
                                                  SensorSafeFastReport *decoded)
{
    uint8_t stream_state;
    uint16_t status_flags;
    uint16_t diag_flags;
    uint32_t expected_crc;
    uint32_t actual_crc;

    if ((frame == NULL) || (decoded == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (frame_len != SENSOR_SAFE_FAST_FRAME_LEN) {
        return SENSOR_SAFE_BAD_LENGTH;
    }
    if ((frame[0] != SENSOR_SAFE_SOF0) || (frame[1] != SENSOR_SAFE_SOF1)) {
        return SENSOR_SAFE_BAD_SOF;
    }
    if (frame[SENSOR_SAFE_FAST_OFFSET_FRAME_TYPE] != SENSOR_SAFE_FAST_FRAME_TYPE) {
        return SENSOR_SAFE_BAD_HEADER;
    }
    if (frame[SENSOR_SAFE_FAST_OFFSET_FRAME_LEN] != SENSOR_SAFE_FAST_FRAME_LEN) {
        return SENSOR_SAFE_BAD_LENGTH;
    }

    expected_crc = SensorSafe_ReadU32(&frame[SENSOR_SAFE_FAST_OFFSET_CRC]);
    actual_crc = SensorSafeCrc32c_Calculate(&frame[SENSOR_SAFE_FAST_OFFSET_FRAME_TYPE],
                                            SENSOR_SAFE_FAST_OFFSET_CRC - SENSOR_SAFE_FAST_OFFSET_FRAME_TYPE);
    if (actual_crc != expected_crc) {
        return SENSOR_SAFE_BAD_CRC;
    }

    stream_state = frame[SENSOR_SAFE_FAST_OFFSET_STREAM_STATE];
    if ((stream_state & 0x0FU) != (uint8_t)SENSOR_SAFE_COMM_PERIODIC_UPLINK) {
        return SENSOR_SAFE_BAD_STREAM_STATE;
    }
    if ((stream_state >> 4U) > (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM) {
        return SENSOR_SAFE_BAD_STREAM_STATE;
    }

    /* 未冻结的状态位一律拒绝，防止新旧实现对同一位产生不同安全解释。 */
    status_flags = SensorSafe_ReadU16(&frame[SENSOR_SAFE_FAST_OFFSET_STATUS_FLAGS]);
    diag_flags = SensorSafe_ReadU16(&frame[SENSOR_SAFE_FAST_OFFSET_DIAG_FLAGS]);
    if (((status_flags & (uint16_t)(~SENSOR_SAFE_ALLOWED_FAST_STATUS16)) != 0U) ||
        ((diag_flags & (uint16_t)(~SENSOR_SAFE_ALLOWED_FAST_DIAG16)) != 0U)) {
        return SENSOR_SAFE_BAD_RESERVED_STATUS;
    }
    if (frame[SENSOR_SAFE_FAST_OFFSET_QUALITY] > 100U) {
        return SENSOR_SAFE_DATA_INVALID;
    }

    decoded->src_id = frame[SENSOR_SAFE_FAST_OFFSET_SRC_ID];
    decoded->dst_id = frame[SENSOR_SAFE_FAST_OFFSET_DST_ID];
    decoded->measure_mode = (uint8_t)(stream_state >> 4U);
    decoded->comm_mode = (uint8_t)(stream_state & 0x0FU);
    decoded->session_id = SensorSafe_ReadU32(&frame[SENSOR_SAFE_FAST_OFFSET_SESSION_ID]);
    decoded->stream_id = SensorSafe_ReadU16(&frame[SENSOR_SAFE_FAST_OFFSET_STREAM_ID]);
    decoded->stream_seq = SensorSafe_ReadU32(&frame[SENSOR_SAFE_FAST_OFFSET_STREAM_SEQ]);
    decoded->sample_counter = SensorSafe_ReadU32(&frame[SENSOR_SAFE_FAST_OFFSET_SAMPLE_COUNTER]);
    decoded->data_age_ms = SensorSafe_ReadU16(&frame[SENSOR_SAFE_FAST_OFFSET_DATA_AGE_MS]);
    decoded->status_flags16 = status_flags;
    decoded->diag_flags16 = diag_flags;
    decoded->temperature_c_x100 = (int16_t)SensorSafe_ReadU16(&frame[SENSOR_SAFE_FAST_OFFSET_TEMPERATURE]);
    decoded->density_kg_m3_x100 = (int32_t)SensorSafe_ReadU32(&frame[SENSOR_SAFE_FAST_OFFSET_DENSITY]);
    decoded->frequency_hz_x1000 = SensorSafe_ReadU32(&frame[SENSOR_SAFE_FAST_OFFSET_FREQUENCY]);
    decoded->signal_quality = frame[SENSOR_SAFE_FAST_OFFSET_QUALITY];
    decoded->config_epoch = SensorSafe_ReadU16(&frame[SENSOR_SAFE_FAST_OFFSET_CONFIG_EPOCH]);
    decoded->received_crc = expected_crc;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 解码控制响应载荷共有的结果、模式、状态和数据新鲜度字段。
 *
 * @details 调用场景：各业务命令在控制帧和会话校验通过后复用此入口。
 * @note 关键约束：只接受完整公共前缀；命令专属尾部仍由对应 client 接口检查。
 *
 * @param payload 已经通过帧头、长度和 CRC32C 校验的响应载荷起始地址。
 * @param payload_len 协议载荷有效长度，单位字节。
 * @param decoded 用于接收通过完整性校验后的解码帧或响应字段。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeFrame_DecodeCommonResponse(const uint8_t *payload,
                                                      size_t payload_len,
                                                      SensorSafeCommonResponse *decoded)
{
    if ((payload == NULL) || (decoded == NULL)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (payload_len < SENSOR_SAFE_COMMON_RESPONSE_LEN) {
        return SENSOR_SAFE_BAD_LENGTH;
    }

    decoded->result_code = SensorSafe_ReadU16(&payload[0]);
    decoded->measure_mode = payload[2];
    decoded->comm_mode = payload[3];
    decoded->status_flags = SensorSafe_ReadU32(&payload[4]);
    decoded->diag_flags = SensorSafe_ReadU32(&payload[8]);
    decoded->sample_counter = SensorSafe_ReadU32(&payload[12]);
    decoded->data_age_ms = SensorSafe_ReadU16(&payload[16]);
    decoded->payload_version = payload[18];
    decoded->data_quality = payload[19];
    return SENSOR_SAFE_OK;
}
