#ifndef PROTOCOL_SWITCH_FRAME_H_
/* PROTOCOL_SWITCH_FRAME_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define PROTOCOL_SWITCH_FRAME_H_

#include <stdint.h>
#include "com_port_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 快速协议切换私有功能码 0x46；仅用于带 LTD 魔术字的切换帧，不能与标准 Modbus 功能码混用。 */
#define PROTOCOL_SWITCH_FUNCTION_CODE       0x46U
/* 协议切换请求帧固定长度 8 字节；解析器必须精确匹配后再读取目标协议字段。 */
#define PROTOCOL_SWITCH_REQUEST_LENGTH      8U
/* 协议切换成功确认帧固定长度 6 字节。 */
#define PROTOCOL_SWITCH_ACK_LENGTH          6U
/* 协议切换确认状态 0x00：请求已接受并完成必要校验。 */
#define PROTOCOL_SWITCH_ACK_STATUS_OK       0x00U
/* 协议切换请求魔术字第 1 字节，ASCII 字符 'L'（0x4C）。 */
#define PROTOCOL_SWITCH_MAGIC_L             0x4CU
/* 协议切换请求魔术字第 2 字节，ASCII 字符 'T'（0x54）。 */
#define PROTOCOL_SWITCH_MAGIC_T             0x54U
/* 协议切换请求魔术字第 3 字节，ASCII 字符 'D'（0x44）。 */
#define PROTOCOL_SWITCH_MAGIC_D             0x44U

/* 协议切换专用帧解析结果；区分“并非切换帧”“已识别但未接受”和“已经接受并进入切换流程”。 */
typedef enum
{
    /* 协议切换专用帧的识别和受理结果。 */
    PROTOCOL_SWITCH_FRAME_NOT_MATCHED = 0, /* 当前帧不符合协议切换专用帧格式，继续交给普通协议处理。 */
    PROTOCOL_SWITCH_FRAME_HANDLED, /* 切换帧已识别并完成应答，但未接受新的目标协议。 */
    PROTOCOL_SWITCH_FRAME_ACCEPTED /* 切换帧校验通过，目标协议将在应答发送完成后应用。 */
} ProtocolSwitchFrameResult;

/**
 * @brief 在当前外部协议解析前识别统一协议切换帧并生成旧串口参数下的应答。
 *
 * @details 调用场景：CPU3 COM1、COM2、COM3 收到完整帧后，由统一协议分发入口优先调用。
 * @note 关键约束：CRC 错误静默丢弃，非法目标协议返回 Modbus 非法数据值异常，合法请求返回独立 ACK。
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
