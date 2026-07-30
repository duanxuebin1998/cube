/*
 * crc.h
 *
 *  Created on: Mar 19, 2025
 *      Author: Duan Xuebin
 */

#ifndef UTILITIES_MY_CRC_H_
/* UTILITIES_MY_CRC_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define UTILITIES_MY_CRC_H_

#include <stdint.h>
#include <stdbool.h>

/* 选择CRC16标准（例如CRC-16-CCITT） */
#define CRC16_POLYNOMIAL  0x1021  /* 多项式 */
#define CRC16_INIT        0xFFFF  /* 初始值 */
#define CRC16_XOR_OUT     0x0000  /* 输出异或值 */

/**
 * @brief CRC16/MODBUS 校验计算
 *
 * 计算CRC16校验值。
 *
 * @param data 输入数据指针。指向参与 CRC16 计算的连续只读字节序列，有效范围为 data[0..length-1]。
 * @param length 数据长度（字节）
 * @return 16位CRC校验值
 */
uint16_t CRC16_Calculate(const uint8_t *data, uint32_t length);
/**
 * @brief 重新计算接收帧的 CRC16，并与帧尾低字节在前的校验值比较。
 *
 * @param revframe 待校验 CRC16 的 Modbus RTU 完整帧只读缓冲区；末两字节按低字节在前保存线端 CRC。
 * @param framelen 参与地址或 CRC 校验的完整帧长度，单位字节。
 * @return true 表示帧内 CRC16 与重新计算值一致；false 表示帧长度不足或 CRC 不一致。
 */
bool SlaveCheckCRC(uint8_t const *revframe, int framelen) ;
/**
 * @brief 使用 STM32 硬件 CRC 单元计算 32 位 CRC，不足整字时以零补齐。
 *
 * 计算CRC32校验值。
 *
 * @param buf 参与 STM32 硬件 CRC32 计算的只读字节序列。
 * @param lenBytes 必须是 4 的倍数，不足时用 0 填充。
 * @return 返回 STM32 硬件 CRC 单元计算得到的 32 位 CRC 值。
 */
uint32_t CRC32_HAL(const uint8_t *buf, uint32_t lenBytes);
/**
 * @brief 使用三组固定测试向量验证硬件 CRC32 计算结果并打印逐例通过情况。
 */
void CRC32_HAL_Test(void); /* CRC32测试函数 */
#endif /* UTILITIES_MY_CRC_H_ */
