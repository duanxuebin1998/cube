#include "protocol_switch_frame.h"

#include <stddef.h>
#include "my_crc.h"

/* 协议切换异常响应固定长度 5 字节。 */
#define PROTOCOL_SWITCH_EXCEPTION_LENGTH    5U
/* 协议切换请求字段非法时返回的 Modbus 异常值 0x03，即非法数据值。 */
#define PROTOCOL_SWITCH_EXCEPTION_VALUE     0x03U

/**
 * @brief 判断切换帧中的目标协议是否属于当前固件已实现的外部协议。
 *
 * @details 调用场景：统一切换帧通过地址、功能码和魔术字识别后调用。
 * @note 关键约束：预留协议值 4 不得通过远程切换入口写入 FRAM。
 *
 * @param target 本次比较、运动或写入的目标值。该值是协议切换目标枚举，函数只接受当前固件明确支持的外部协议类型。
 * @return 1 表示切换帧中的目标协议属于当前固件已实现的外部协议；0 表示切换帧中的目标协议不属于当前固件已实现的外部协议。
 */
static uint8_t ProtocolSwitchFrame_IsTargetSupported(uint8_t target)
{
    switch ((ComProtocolType)target)
    {
    case COM_PROTO_DSM:
    case COM_PROTO_WARTSILA:
    case COM_PROTO_LTD:
    case COM_PROTO_LH:
    case COM_PROTO_SI:
        return 1U;

    default:
        return 0U;
    }
}

/**
 * @brief 为统一切换帧应答追加 Modbus RTU CRC16。
 *
 * @details 调用场景：构造非法目标协议异常应答时调用。
 * @note 关键约束：CRC 低字节在前、高字节在后。
 *
 * @param frame 待解析、校验或发送的协议帧缓冲区。该可写缓冲区已包含 payload_len 个有效字节，函数在尾部追加低字节在前的 CRC16。
 * @param payload_len 协议载荷有效长度，单位字节。
 * @param frame_len 协议帧有效长度，单位字节。
 */
static void ProtocolSwitchFrame_AppendCrc(uint8_t *frame,
                                         uint16_t payload_len,
                                         uint16_t *frame_len)
{
    uint16_t crc = CRC16_Calculate(frame, payload_len);

    frame[payload_len] = (uint8_t)(crc & 0x00FFU);
    frame[payload_len + 1U] = (uint8_t)(crc >> 8U);
    *frame_len = (uint16_t)(payload_len + 2U);
}

/**
 * @brief 识别并处理独立协议切换帧，返回是否接收及是否应答。
 *
 * @param slave_address 从站地址。
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx 独立协议切换确认帧的输出缓冲区；请求合法并需要应答时由函数填充。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @param target_protocol 用于返回请求切换到的外部协议枚举；未接受切换时保持调用方原值。
 * @return 返回协议切换帧处理结果；PROTOCOL_SWITCH_FRAME_NOT_MATCHED 表示不是切换帧，PROTOCOL_SWITCH_FRAME_HANDLED
 *         表示已识别并处理但未接受切换，PROTOCOL_SWITCH_FRAME_ACCEPTED 表示请求合法且已生成接受应答。
 */
ProtocolSwitchFrameResult ProtocolSwitchFrame_Process(uint8_t slave_address,
                                                       const uint8_t *rx,
                                                       uint16_t rx_len,
                                                       uint8_t *tx,
                                                       uint16_t *tx_len,
                                                       ComProtocolType *target_protocol)
{
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((rx == NULL) || (tx == NULL) || (tx_len == NULL) || (target_protocol == NULL))
    {
        return PROTOCOL_SWITCH_FRAME_NOT_MATCHED;
    }

    *tx_len = 0U;

    if ((slave_address < 1U) ||
        (slave_address > 247U) ||
        (rx_len != PROTOCOL_SWITCH_REQUEST_LENGTH) ||
        (rx[0] != slave_address) ||
        (rx[1] != PROTOCOL_SWITCH_FUNCTION_CODE) ||
        (rx[2] != PROTOCOL_SWITCH_MAGIC_L) ||
        (rx[3] != PROTOCOL_SWITCH_MAGIC_T) ||
        (rx[4] != PROTOCOL_SWITCH_MAGIC_D))
    {
        return PROTOCOL_SWITCH_FRAME_NOT_MATCHED;
    }

    received_crc = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8U);
    calculated_crc = CRC16_Calculate(rx, PROTOCOL_SWITCH_REQUEST_LENGTH - 2U);
    if (received_crc != calculated_crc)
    {
        /* 已识别为管理帧但 CRC 错误时静默丢弃，避免落入当前业务协议产生误应答。 */
        return PROTOCOL_SWITCH_FRAME_HANDLED;
    }

    if (ProtocolSwitchFrame_IsTargetSupported(rx[5]) == 0U)
    {
        tx[0] = slave_address;
        tx[1] = (uint8_t)(PROTOCOL_SWITCH_FUNCTION_CODE | 0x80U);
        tx[2] = PROTOCOL_SWITCH_EXCEPTION_VALUE;
        ProtocolSwitchFrame_AppendCrc(tx, 3U, tx_len);
        return PROTOCOL_SWITCH_FRAME_HANDLED;
    }

    tx[0] = slave_address;
    tx[1] = PROTOCOL_SWITCH_FUNCTION_CODE;
    tx[2] = PROTOCOL_SWITCH_ACK_STATUS_OK;
    tx[3] = rx[5];
    ProtocolSwitchFrame_AppendCrc(tx, 4U, tx_len);
    *target_protocol = (ComProtocolType)rx[5];
    return PROTOCOL_SWITCH_FRAME_ACCEPTED;
}
