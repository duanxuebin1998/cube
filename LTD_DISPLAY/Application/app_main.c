/*
 * app_main.c
 *
 *  Created on: Nov 18, 2025
 *      Author: admin
 */
#include "Display.h"
#include "modbus_agreement.h"
#include "system_parameter.h"


void App_Init(void) {
	printf("LTD demo restart!\r\n");
	DisplayInit(); // Initialize the OLED display
	DisplayAubonLogo(); /* 刚上电显示AUBON LOGO */
	DSM_CommunicationInit();
}

void App_MainLoop(void) {
		if (uart3_rx_ready == 1) { // 如果接收到数据
			printf("usart3_rx_ready == 1\r\n");
			uart3_rx_ready = 0; // 重置标志
//			HAL_UART_Transmit_DMA(&huart3, UART3_RX_BUF, UART3_RX_LEN);//返回接受到的包
			DSM_CommunicationProcess(UART3_RX_BUF, UART3_RX_LEN);  // 处理接收到的数据
			HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE); // 重新启用DMA接收
		}
		else
		{
//			g_measurement.device_status.device_state =0; // 正常工作模式
//			cnt_commutoCPU2 = 0;
			PollingInputData(); // Polling for input data
		}
//		print_device_params();
//		HAL_Delay(1000); //
//		HAL_UART_Transmit_DMA(&huart1, "123", 3);
}
