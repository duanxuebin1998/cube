/*
 * CH9141EVT.c
 *
 *  Created on: Mar 20, 2025
 *      Author: Duan Xuebin
 */

#include "CH9141EVT.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

char BLE_MAC[18] = "36:1D:1A:26:3B:38";

void Init_MAC()
{
	strcpy(BLE_MAC, "36:1D:1A:26:3B:38");  // 正确方式
}

// 发送 AT 指令
void Send_AT_Command(char *cmd)
{
	HAL_UART_Transmit(CH9141_UART, (uint8_t*) cmd, strlen(cmd), 100);
	HAL_UART_Transmit(CH9141_UART, (uint8_t*) "\r\n", 2, 100);
	HAL_Delay(100);
}

// 接收模块返回的数据
void Receive_Response(uint8_t *buffer, uint16_t size)
{
	memset(buffer, 0, size);
	HAL_UART_Receive(CH9141_UART, buffer, size, 500);
}

// 初始化 CH9141EVT
void CH9141_Init()
{
	uint8_t rxBuffer[100];
	char atCommand[50];

	// 1. 进入 AT 模式
	Send_AT_Command("AT");
	Receive_Response(rxBuffer, sizeof(rxBuffer));

	// 2. 设置 BLE 为主机模式
	Send_AT_Command("AT+BLEMODE=1");
	Receive_Response(rxBuffer, sizeof(rxBuffer));

	// 3. 复位模块
	Send_AT_Command("AT+RESET");
	HAL_Delay(500);

	// 4. 再次进入 AT 模式
	Send_AT_Command("AT");
	Receive_Response(rxBuffer, sizeof(rxBuffer));

	// 5. 设置 MAC 地址
	snprintf(atCommand, sizeof(atCommand), "AT+CONADD=%s,000000", BLE_MAC);
	Send_AT_Command(atCommand);
	Receive_Response(rxBuffer, sizeof(rxBuffer));

	// 6. 再次复位，使配置生效
	Send_AT_Command("AT+RESET");
	HAL_Delay(500);
}
//
