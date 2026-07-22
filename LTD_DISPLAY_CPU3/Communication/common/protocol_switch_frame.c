#include "protocol_switch_frame.h"

#include <stddef.h>
#include "my_crc.h"

#define PROTOCOL_SWITCH_EXCEPTION_LENGTH    5U
#define PROTOCOL_SWITCH_EXCEPTION_VALUE     0x03U

/*
 * 函数用途：判断切换帧中的目标协议是否属于当前固件已实现的外部协议。
 * 调用场景：统一切换帧通过地址、功能码和魔术字识别后调用。
 * 关键约束：预留协议值 4 不得通过远程切换入口写入 FRAM。
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

/*
 * 函数用途：为统一切换帧应答追加 Modbus RTU CRC16。
 * 调用场景：构造非法目标协议异常应答时调用。
 * 关键约束：CRC 低字节在前、高字节在后。
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
