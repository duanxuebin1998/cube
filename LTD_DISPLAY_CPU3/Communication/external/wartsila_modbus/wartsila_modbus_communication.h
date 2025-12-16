/*
 * wartsila_modbus_communication.h
 *
 *  Created on: Nov 15, 2025
 *      Author: Duan Xuebin
 */

#ifndef WARTSILA_MODBUS_WARTSILA_MODBUS_COMMUNICATION_H_
#define WARTSILA_MODBUS_WARTSILA_MODBUS_COMMUNICATION_H_

#pragma once
#include <stdint.h>
#include <stddef.h>
#include "wartsila_register_map.h"

// ============= 对外状态码（返回处理结果）============
typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERR_BADLEN,        // 帧长度错误
    MODBUS_ERR_CRC,           // CRC 错误
    MODBUS_ERR_ADDR_MISMATCH, // 从站地址不匹配（丢弃）
    MODBUS_ERR_FUNC_UNSUPPORT // 功能码不支持（理论上不会触发，因为仅实现03/10）
} ModbusResult;

// ============= 对外接口（核心处理函数）============
// 输入：rx_buf/rx_len（含CRC），输出：tx_buf/tx_len（含CRC）
// 返回：处理状态（OK 表示已生成应答；ADDR_MISMATCH 表示不是本机；其他为异常）
ModbusResult modbus_rtu_process(const uint8_t* rx_buf, uint16_t rx_len,
                                uint8_t* tx_buf, uint16_t* tx_len);

// 保持寄存器池（16位寄存器数组）——由应用层可直接读写
extern uint16_t g_holding_regs[HOLDREG_COUNT];



#endif /* WARTSILA_MODBUS_WARTSILA_MODBUS_COMMUNICATION_H_ */
