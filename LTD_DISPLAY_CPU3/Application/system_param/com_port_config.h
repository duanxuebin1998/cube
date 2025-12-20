/*
 * com_port_config.h
 *
 *  Created on: 2025年12月11日
 *      Author: Duan Xuebin
 */

#ifndef __COM_PORT_CONFIG_H__
#define __COM_PORT_CONFIG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 协议类型 */
typedef enum {
	COM_PROTO_DSM        = 0,
	COM_PROTO_WARTSILA  = 1,
    COM_PROTO_LTD         = 2,
    COM_PROTO_1    = 3,
    COM_PROTO_2 = 4,
} ComProtocolType;

/* 校验 */
typedef enum {
    COM_PARITY_NONE = 0,
    COM_PARITY_EVEN = 1,
    COM_PARITY_ODD  = 2,
} ComParityType;

/* 停止位 */
typedef enum {
    COM_STOPBITS_1 = 0,
    COM_STOPBITS_2 = 1,
} ComStopBitsType;

/* 每个串口一份的完整配置 */
typedef struct
{
    uint32_t        baudrate;   /* 波特率：4800/9600/115200 等 */
    uint8_t         databits;   /* 数据位：一般是 8 或 9 */
    ComParityType   parity;     /* 校验位 */
    ComStopBitsType stopbits;   /* 停止位 */
    ComProtocolType protocol;   /* 当前协议类型 */
} ComPortConfig;

#ifdef __cplusplus
}
#endif

#endif /* __COM_PORT_CONFIG_H__ */

