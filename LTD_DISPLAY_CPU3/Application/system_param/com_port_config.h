/*
 * com_port_config.h
 *
 *  Created on: 2025年12月11日
 *      Author: Duan Xuebin
 */

#ifndef __COM_PORT_CONFIG_H__
/* __COM_PORT_CONFIG_H__ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define __COM_PORT_CONFIG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 协议类型 */
typedef enum {
	/* 外部 COM 口协议选择值；该数值写入持久化参数，并决定串口帧分发处理器。 */
	COM_PROTO_DSM        = 0, /* 该串口使用 DSM 外部 Modbus 协议。 */
	COM_PROTO_WARTSILA  = 1, /* 该串口使用 Wartsila 外部 Modbus 协议。 */
    COM_PROTO_LTD         = 2, /* 该串口使用 LTD 外部 Modbus 协议。 */
    COM_PROTO_LH          = 3, /* 该串口使用 LH 外部 Modbus 协议。 */
    COM_PROTO_2 = 4, /* 历史保留协议编号 2；当前不得解释为其它协议。 */
    COM_PROTO_SI = 5, /* SI协议选项，切换协议时带出 9600 8O1 默认值。 */
} ComProtocolType;

/* 校验 */
typedef enum {
    /* 外部 COM 口串行校验方式。 */
    COM_PARITY_NONE = 0, /* 串口不使用奇偶校验。 */
    COM_PARITY_EVEN = 1, /* 串口使用偶校验。 */
    COM_PARITY_ODD  = 2, /* 串口使用奇校验。 */
} ComParityType;

/* 停止位 */
typedef enum {
    /* 外部 COM 口停止位配置。 */
    COM_STOPBITS_1 = 0, /* 串口使用 1 个停止位。 */
    COM_STOPBITS_2 = 1, /* 串口使用 2 个停止位。 */
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

