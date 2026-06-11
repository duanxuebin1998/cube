/*
 * wireless_host_communication.h
 *
 *  Created on: 2025年12月8日
 *      Author: Duan Xuebin
 */

#ifndef WIRELESS_HOST_COMMUNICATION_H_
#define WIRELESS_HOST_COMMUNICATION_H_

#include "main.h"
#include "stdint.h"

/**
 * @brief 处理无线主机通信中的 Wireless_Handle_MasterFrame 逻辑。
 *
 * @param req 业务参数。
 */
void Wireless_Handle_MasterFrame(const uint8_t req[8]);

#endif /* WIRELESS_HOST_COMMUNICATION_H_ */
