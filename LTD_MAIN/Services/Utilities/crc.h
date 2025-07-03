/*
 * crc.h
 *
 *  Created on: Mar 19, 2025
 *      Author: 1
 */

#ifndef UTILITIES_CRC_H_
#define UTILITIES_CRC_H_

#include <stdint.h>

// 选择CRC16标准（例如CRC-16-CCITT）
#define CRC16_POLYNOMIAL  0x1021  // 多项式
#define CRC16_INIT        0xFFFF  // 初始值
#define CRC16_XOR_OUT     0x0000  // 输出异或值

// 计算CRC16校验值
uint16_t CRC16_Calculate(const uint8_t *data, uint32_t length);
// 计算CRC32校验值
uint32_t CRC32_Calculate(const uint8_t *data, uint32_t len);


#endif /* UTILITIES_CRC_H_ */
