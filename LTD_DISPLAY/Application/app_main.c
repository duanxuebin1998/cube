/*
 * app_main.c
 *
 *  Created on: Nov 18, 2025
 *      Author: admin
 */
#include "Display.h"
#include "modbus_agreement.h"
#include "system_parameter.h"
#include "DSM_communication.h"
#include "wartsila_modbus_communication.h"
void App_Init(void) {
	printf("LTD demo restart!\r\n");
	DisplayInit(); // Initialize the OLED display
	DisplayAubonLogo(); /* 刚上电显示AUBON LOGO */
	DSM_CommunicationInit(); // 初始化通信模块
}

void App_MainLoop(void) {
	uint32_t ret = 0;
	volatile uint16_t send_len = 0;              // 接收一帧数据的长度
	static uint8_t sendbuff[256] = { 0 };   // 接收数据缓冲区
//	const char *msg = "UART Test Message\r\n";
	if (com1_rx_ready == 1) { // 如果接收到数据
		printf("com1_rx_ready == 1\r\n");
		ret = modbus_rtu_process(UART6_RX_BUF, UART6_RX_LEN, sendbuff, &send_len);  // 处理接收到的数据
		if (ret != 0) {
			printf("com1 modbus_rtu_process ret=%ld\r\n", ret);
			HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE); // 重新启用DMA接收
			COM1_SET_RECV_MODE();  //切换接收模式
		} else {
			if (send_len > 0) {
				COM1_SET_SEND_MODE();  //切换发送模式
				HAL_UART_Transmit_DMA(&huart6, sendbuff, send_len);  //返回接受到的包
			}
		}
		com1_rx_ready = 0; // 重置标志
	} else if (com2_rx_ready == 1) { // 如果接收到数据
		printf("com2_rx_ready == 1\r\n");
		ret = modbus_rtu_process(UART2_RX_BUF, UART2_RX_LEN, sendbuff, &send_len);  // 处理接收到的数据
		if (ret != 0) {
			printf("com2 modbus_rtu_process ret=%ld\r\n", ret);
			HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE); // 重新启用DMA接收
			COM2_SET_RECV_MODE();  //切换接收模式
		} else {
			if (send_len > 0) {
				COM2_SET_SEND_MODE();  //切换发送模式

			    printf("COM2 TX (%d): ", send_len);
			    for (int i = 0; i < send_len; i++) {
			        printf("%02X ", sendbuff[i]);
			    }
			    printf("\r\n");
				HAL_UART_Transmit_DMA(&huart2, sendbuff, send_len);  //返回接受到的包
			}
		}
		com2_rx_ready = 0; // 重置标志
	} else if (com3_rx_ready == 1) { // 如果接收到数据
		printf("com3_rx_ready == 1\r\n");
		ret = DSM_CommunicationProcess(UART3_RX_BUF, UART3_RX_LEN,sendbuff, &send_len);  // 处理接收到的数据
		if (ret != 0) {
			HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE); // 重新启用DMA接收
			COM3_SET_RECV_MODE();  //切换接收模式
		}
		else
		{
			HAL_UART_Transmit_DMA(&huart3, sendbuff, send_len);  //返回响应包
		}
		com3_rx_ready = 0; // 重置标志
	} else {

//			g_measurement.device_status.device_state =0; // 正常工作模式
//			cnt_commutoCPU2 = 0;
		PollingInputData(); // Polling for input data
	}
//		print_device_params();
//		HAL_Delay(1000); //
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)msg, strlen(msg));
}
