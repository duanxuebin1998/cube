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

void Wireless_Handle_MasterFrame(const uint8_t req[8]);

#endif /* WIRELESS_HOST_COMMUNICATION_H_ */
