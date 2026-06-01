/*
 * wireless_pairing.h
 *
 * CH9141K 主机模块无线滑环匹配调试与正式命令接口。
 */

#ifndef SENSOR_WIRELESS_PAIRING_H_
#define SENSOR_WIRELESS_PAIRING_H_

#include <stdint.h>

/**
 * @brief 扫描并打印 CH9141K 主机可见的从机候选。
 *
 * 串口调试命令 SPS 使用该接口，只扫描不保存默认连接。
 */
uint32_t WirelessPairing_DebugScan(void);

/**
 * @brief 按 RSSI 近距离策略选择从机并保存为默认连接。
 *
 * 串口调试命令 SPR 使用该接口；多候选时必须满足最强 RSSI 阈值和差值条件。
 */
uint32_t WirelessPairing_RunByRssi(void);

/**
 * @brief 按扫描结果中的蓝牙名称选择从机并保存为默认连接。
 *
 * 串口调试命令 SPN=<name> 使用该接口；如果扫描输出不包含名称字段，会拒绝匹配。
 */
uint32_t WirelessPairing_RunByName(const char *target_name);

/**
 * @brief 查询当前 CH9141K 连接状态并打印模式、状态、MAC、RSSI 和缓存名称。
 *
 * 串口调试命令 SPC 使用该接口；只读查询，不扫描、不断开、不保存默认连接。
 */
uint32_t WirelessPairing_PrintConnectionStatus(void);

#endif /* SENSOR_WIRELESS_PAIRING_H_ */
