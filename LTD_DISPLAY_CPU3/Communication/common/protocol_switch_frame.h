#ifndef PROTOCOL_SWITCH_FRAME_H_
#define PROTOCOL_SWITCH_FRAME_H_

#include <stdint.h>
#include "com_port_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_SWITCH_FUNCTION_CODE       0x46U
#define PROTOCOL_SWITCH_REQUEST_LENGTH      8U
#define PROTOCOL_SWITCH_ACK_LENGTH          6U
#define PROTOCOL_SWITCH_ACK_STATUS_OK       0x00U
#define PROTOCOL_SWITCH_MAGIC_L             0x4CU
#define PROTOCOL_SWITCH_MAGIC_T             0x54U
#define PROTOCOL_SWITCH_MAGIC_D             0x44U

typedef enum
{
    PROTOCOL_SWITCH_FRAME_NOT_MATCHED = 0,
    PROTOCOL_SWITCH_FRAME_HANDLED,
    PROTOCOL_SWITCH_FRAME_ACCEPTED
} ProtocolSwitchFrameResult;

/*
 * 函数用途：在当前外部协议解析前识别统一协议切换帧并生成旧串口参数下的应答。
 * 调用场景：CPU3 COM1、COM2、COM3 收到完整帧后，由统一协议分发入口优先调用。
 * 关键约束：CRC 错误静默丢弃，非法目标协议返回 Modbus 非法数据值异常，合法请求返回独立 ACK。
 */
ProtocolSwitchFrameResult ProtocolSwitchFrame_Process(uint8_t slave_address,
                                                       const uint8_t *rx,
                                                       uint16_t rx_len,
                                                       uint8_t *tx,
                                                       uint16_t *tx_len,
                                                       ComProtocolType *target_protocol);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_SWITCH_FRAME_H_ */
