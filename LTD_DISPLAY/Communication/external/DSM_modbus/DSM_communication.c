#include "DSM_communication.h"
#include <stdlib.h>
#include <string.h>
#include "spi.h"
#include "DSM_comm.h"
#include "DSM_DataAnalysis_modbus2.h"
#include "DSM_stateformodbus2.h"
#include "DSM_system_parameter.h"
#include "DSM_SlaveModbus_modbus2.h"
#include "my_crc.h"

#define DEBUG_COMM 1
/**********************************************************************************************
 **函数名称：	CommunicationInit()
 **函数功能：	与上位机通信初始化:串口4初始化；DMA初始化；定时器2初始化；地址初始化并读取当前地址
 **参数:			无
 **返回值:		0
 **********************************************************************************************/
int DSM_CommunicationInit(void) {
	int LocalAddress = 129;
//	uart4_init(36, 4800, 8, 2, 'N', 20);
//	MYDMA2CH5_Config((u32)&UART4->DR); // 初始化DMA通道
//	Timer2Init();					   // 初始化定时器2
//	AddressInit();
//	delay_ms(10);
	LocalAddress = Get_Device_Address();
	SetSlaveaddress(LocalAddress);
//用于权限设置
	MaxNum_Coil = ENDADDRESS1_COM;					   // 线圈工作模式有效值
	MaxNum_HoldingRegister = ENDADDRESS1_HOLDREGISTER; // 保持寄存器工作模式有效值
	MaxNum_InputRegister = ENDADDRESS2_INPUTREGISTER;  // 输入寄存器工作模式有效值
	WriteOneInputRegister(INPUTREGISTER_SYSTEMSTATE, 1, DSM_STATE_INIT);
#ifdef DEBUG_COMM
	printf("Address = %d\r\n", LocalAddress);
#endif
	return 0;
}
/**********************************************************************************************
 **函数名称：	CommunicationProcess(high)
 **函数功能：	处理数据包
 **参数:			无
 **返回值:		0
 **********************************************************************************************/
int DSM_CommunicationProcess(unsigned char *rcvbuff, int rcvcount, uint8_t* tx, uint16_t* tx_len) {
#ifdef DEBUG_COMM
	int i;
#endif
	int functioncode, ret;
#if DEBUG_COMM
	printf("CPU3_RCV %d : ", rcvcount);
	for (i = 0; i < rcvcount; i++)
		printf("%02X ", rcvbuff[i]);
	printf("\r\n");
#endif
	if (rcvcount <= 3) {
		printf("COMM:<=3\t");

		for (i = 0; i < rcvcount; i++) {
			printf("0x%02X\t", rcvbuff[i]);
		}
		printf("\r\n");
		return;
	}
	//校验地址
	if (rcvbuff[0] != SlaveAddress && rcvbuff[0] != 0) {
		return -1;
	}
	if (SlaveCheckCRC(rcvbuff, rcvcount) == false) {
		return -1;
	}

	if (GetFunctioncode(rcvbuff, &functioncode) == false) {
		tx[0] = SlaveAddress;
		tx[1] = 0x80 + functioncode;
		tx[2] = 0x01; // 非法功能码
		// 缺少CRC校验
		*tx_len = 5;
	} else {
		switch (functioncode) {
		case FUNCTIONCODE_READ_HOLDREGISTER: {
			SystemParameterSet(); // 更新参数,每一个保持寄存器必须写入，否则读出来会不变；
			*tx_len = Response03(rcvbuff, tx);
			break;
		}

		case FUNCTIONCODE_READ_INPUTREGISTER: {
			Input_Write(); // 更新数据,每一个输入寄存器的参数必须写入，否则读出来会不变；
			*tx_len = Response04(rcvbuff, tx);
			break;
		}

		case FUNCTIONCODE_WRITE_COIL: {
			*tx_len = Response05(rcvbuff, tx);
			break;
		}

		case FUNCTIONCODE_WRITE_MULREGISTER: {
			*tx_len = Response16(rcvbuff, tx);
			break;
		}

		default: {
			break;
		}
		}
	}
	return 0;
}

/*向上位机发送数据包*/
void SendPacketToHost(uint8_t *arr, uint16_t len) {
	HAL_UART_Transmit_DMA(&huart3, arr, len);
#if DEBUG_COMMUCPU2
    {
        int i;
        for(i = 0;i < len;i++)
            printf("%02X ",arr[i]);
        printf("\n");
    }
    #endif
}
