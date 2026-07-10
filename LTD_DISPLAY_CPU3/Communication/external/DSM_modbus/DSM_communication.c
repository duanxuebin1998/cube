#include "DSM_communication.h"
#include <stdlib.h>
#include <string.h>
#include "spi.h"
#include "DSM_comm.h"
#include "DSM_DataAnalysis_modbus2.h"
#include "DSM_stateformodbus2.h"
#include "DSM_SlaveModbus_modbus2.h"
#include "my_crc.h"
#include "address.h"

#define DEBUG_COMM 0

/*
 * 函数用途：校验 DSM 请求帧的实际长度是否与功能码和 byteCount 一致。
 * 调用场景：地址和 CRC 通过后、分发到各 Response 函数前调用。
 * 关键约束：FC10 读取 byteCount 前必须先确认缓冲区至少包含该字段。
 */
static bool DSM_IsRequestLengthValid(const unsigned char *rcvbuff, int rcvcount)
{
	if ((rcvbuff == NULL) || (rcvcount < 2))
	{
		return false;
	}

	switch (rcvbuff[1])
	{
	case FUNCTIONCODE_READ_COIL:
	case FUNCTIONCODE_READ_HOLDREGISTER:
	case FUNCTIONCODE_READ_INPUTREGISTER:
	case FUNCTIONCODE_WRITE_COIL:
		return (rcvcount == 8);

	case FUNCTIONCODE_WRITE_MULREGISTER:
		if (rcvcount < 9)
		{
			return false;
		}
		return (rcvcount == (9 + (int)rcvbuff[6]));

	default:
		return true;
	}
}

/**********************************************************************************************
 **函数名称：	CommunicationInit()
 **函数功能：	与上位机通信初始化:串口4初始化；DMA初始化；定时器2初始化；地址初始化并读取当前地址
 **参数:			无
 **返回值:		0
 **********************************************************************************************/
int DSM_CommunicationInit(void) {
	int LocalAddress = 129;
	LocalAddress = Get_Device_Address();
	SetSlaveaddress(LocalAddress);
/* 用于权限设置 */
	MaxNum_Coil = ENDADDRESS3_COM;					   /* 线圈工作模式有效值 */
	MaxNum_HoldingRegister = ENDADDRESS6_HOLDREGISTER; /* 保持寄存器工作模式有效值 */
	MaxNum_InputRegister = ENDADDRESS5_INPUTREGISTER;  /* 输入寄存器工作模式有效值 */
	WriteOneInputRegister(INPUTREGISTER_SYSTEMSTATE, 1, STATE_INIT);
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
	int functioncode;
	unsigned short crc;
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
		return -1;
	}
	/* 校验地址 */
	if (rcvbuff[0] != SlaveAddress && rcvbuff[0] != 0) {
		return -1;
	}
	if (SlaveCheckCRC(rcvbuff, rcvcount) == false) {
		return -1;
	}
	if (!DSM_IsRequestLengthValid(rcvbuff, rcvcount)) {
		*tx_len = (uint16_t)ResponseException(rcvbuff[1], EXCEPTIONCODE_ERRORDATA, tx);
		return 0;
	}

	if (GetFunctioncode(rcvbuff, &functioncode) == false) {
		tx[0] = SlaveAddress;
		tx[1] = 0x80 + functioncode;
		tx[2] = 0x01; /* 非法功能码 */
		/* 缺少CRC校验 */
		crc = CRC16_Calculate(tx, 3);
		tx[3] = crc & 0xff;
		tx[4] = (crc >> 8) & 0xff;
		*tx_len = 5;
	} else {
		switch (functioncode) {
		case FUNCTIONCODE_READ_COIL: {
			*tx_len = Response01(rcvbuff, tx);
			break;
		}

		case FUNCTIONCODE_READ_HOLDREGISTER: {
			SystemParameterSet(); /* 更新参数,每一个保持寄存器必须写入，否则读出来会不变； */
			*tx_len = Response03(rcvbuff, tx);
			break;
		}

		case FUNCTIONCODE_READ_INPUTREGISTER: {
			Input_Write(); /* 更新数据,每一个输入寄存器的参数必须写入，否则读出来会不变； */
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
