/*
 * 模块职责：声明无线滑环连接快照、配对结果和扫描/匹配/链路诊断接口。
 * 数据边界：连接状态、对端MAC、RSSI和错误码作为同一代快照整体发布。
 * 调用约束：查询接口不等于传感器通信成功；业务仍需独立完成具体协议事务。
 */

#ifndef SENSOR_WIRELESS_PAIRING_H_
/* SENSOR_WIRELESS_PAIRING_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_WIRELESS_PAIRING_H_

#include <stdint.h>

/* 无线模块连接状态快照；将连接有效性、对端 MAC、RSSI 及最近错误作为同一份可发布数据保存。 */
typedef struct {
    /* 无线连接、MAC、RSSI 和错误码的一致性状态快照。 */
    uint32_t connection_valid; /* 连接查询已得到可信结果的标志；与是否已连接的业务值分开。 */
    uint32_t mac_valid; /* mac_high、mac_mid 和 mac_low 已组成完整对端 MAC 的标志。 */
    uint32_t mac_high; /* 对端 MAC 地址高 16 位/字段，按既定寄存器布局发布。 */
    uint32_t mac_mid; /* 对端 MAC 地址中间字段，按既定寄存器布局发布。 */
    uint32_t mac_low; /* 对端 MAC 地址低 16 位/字段，按既定寄存器布局发布。 */
    uint32_t rssi_valid; /* rssi 已由本次有效查询取得的标志；为假时不得显示旧值。 */
    int32_t rssi; /* 最近一次有效的无线 RSSI，单位为 dBm。 */
    uint32_t error_code; /* 最近一次无线查询、配对或连接流程的完整统一故障码。 */
} WirelessConnectionStatus;

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
 * @brief 查询当前 CH9141K 连接状态和 RSSI，不扫描、不断开、不保存默认连接。
 */
uint32_t WirelessPairing_ReadConnectionStatus(WirelessConnectionStatus *status);

/**
 * @brief 查询当前 CH9141K 连接状态和 RSSI，并发布到 CPU2/CPU3 共享快照。
 */
uint32_t WirelessPairing_UpdateConnectionStatusSnapshot(void);

/**
 * @brief 查询 CH9141K 蓝牙主机状态，发布共享快照并判断从机链路是否有效。
 *
 * 该接口用于上电识别、传感器通信超时归因、配对收尾和串口维护测试；只允许在任务上下文调用，不执行扫描、断开或默认连接保存。
 *
 * @param status 用于返回本次 AT 查询得到的临时连接状态；可读取 MAC、RSSI、有效标志和业务错误码。
 * @return NO_ERROR 表示蓝牙主机可查询且从机连接有效；其他值区分 AT 查询失败、主机未连接从机和命令切换。
 */
uint32_t WirelessPairing_CheckBluetoothLinkDetailed(WirelessConnectionStatus *status);

/**
 * @brief 检查蓝牙主机是否可查询且蓝牙从机是否已连接。
 *
 * 返回 WIRELESS_HOST_COMM_TIMEOUT 表示蓝牙主机状态查询失败，返回 WIRELESS_SLAVE_COMM_TIMEOUT 表示主机正常但未连接从机。
 */
uint32_t WirelessPairing_CheckBluetoothLink(void);
/**
 * @brief 查询当前 CH9141K 连接状态并打印模式、状态、MAC、RSSI 和缓存名称。
 *
 * 串口调试命令 SPC 使用该接口；只读查询，不扫描、不断开、不保存默认连接。
 */
uint32_t WirelessPairing_PrintConnectionStatus(void);

#endif /* SENSOR_WIRELESS_PAIRING_H_ */
