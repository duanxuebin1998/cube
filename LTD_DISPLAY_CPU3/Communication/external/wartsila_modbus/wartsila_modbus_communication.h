/*
 * wartsila_modbus_communication.h
 *
 *  Created on: Nov 15, 2025
 *      Author: Duan Xuebin
 */

#ifndef WARTSILA_MODBUS_WARTSILA_MODBUS_COMMUNICATION_H_
/* WARTSILA_MODBUS_WARTSILA_MODBUS_COMMUNICATION_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define WARTSILA_MODBUS_WARTSILA_MODBUS_COMMUNICATION_H_

#pragma once
#include <stdint.h>
#include <stddef.h>
#include "wartsila_register_map.h"

/* ============= 对外状态码（返回处理结果）============ */
typedef enum {
    /* Wartsila Modbus 帧处理结果。 */
    MODBUS_OK = 0, /* 处理成功。 */
    MODBUS_ERR_BADLEN,        /* 帧长度错误 */
    MODBUS_ERR_CRC,           /* CRC 错误 */
    MODBUS_ERR_ADDR_MISMATCH, /* 从站地址不匹配（丢弃） */
    MODBUS_ERR_FUNC_UNSUPPORT /* 功能码不支持（理论上不会触发，因为仅实现03/10） */
} ModbusResult;

/* ============= 对外接口（核心处理函数）============ */
/* 输入：rx_buf/rx_len（含CRC），输出：tx_buf/tx_len（含CRC） */
/**
 * @brief 校验瓦锡兰 Modbus RTU 请求并分发 0x03、0x10 功能码。
 *
 * @param rx_buf 接收到的原始请求帧首地址；函数只读取 rx_buf[0..rx_len-1]，不会修改请求内容。
 * @param rx_len 请求帧有效长度，单位字节。
 * @param tx_buf 正常或异常响应帧的输出缓冲区。
 * @param tx_len 用于返回响应帧有效长度的输出指针，单位字节。
 * @return MODBUS_OK 表示已完成处理；MODBUS_ERR_ADDR_MISMATCH 表示请求不属于本机，其他值区分帧长、CRC 和功能码错误。tx_len 非零时包含可发送的响应。
 */
ModbusResult modbus_rtu_process(const uint8_t* rx_buf, uint16_t rx_len,
                                uint8_t* tx_buf, uint16_t* tx_len);

/* 保持寄存器池（16位寄存器数组）——由应用层可直接读写 */
extern uint16_t g_holding_regs[HOLDREG_COUNT];



#endif /* WARTSILA_MODBUS_WARTSILA_MODBUS_COMMUNICATION_H_ */
