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

/**
 * @brief 校验 DSM 请求帧的实际长度是否与功能码和 byteCount 一致。
 *
 * @details 调用场景：地址和 CRC 通过后、分发到各 Response 函数前调用。
 * @note 关键约束：FC10 读取 byteCount 前必须先确认缓冲区至少包含该字段。
 *
 * @param rcvbuff 接收到的完整 DSM Modbus RTU 请求帧。
 * @param rcvcount 接收缓冲区中的有效帧长度，单位字节。
 * @return true 表示上述校验全部通过；false 表示至少一项校验未通过。
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

/**
 * @brief 初始化 DSM Modbus 地址、寄存器访问上限和系统状态寄存器。
 *
 * 函数读取当前设备地址并调用 SetSlaveaddress，随后设置线圈、保持寄存器和输入寄存器的最大有效数量，最后把系统状态输入寄存器写为 STATE_INIT。
 *
 * @return 固定返回 0，表示既有初始化入口已执行完毕；当前返回值不反映 SetSlaveaddress 或输入寄存器写入是否成功。
 * @note 当前实现不初始化 UART4、DMA 或定时器，也不检查地址设置及寄存器写入结果；固定返回 0 仅保留既有初始化接口约定。
 */
int DSM_CommunicationInit(void) {
	int LocalAddress = 129;
	LocalAddress = Get_Device_Address();
	SetSlaveaddress(LocalAddress);
/* 用于权限设置 */
	MaxNum_Coil = ENDADDRESS3_COM;					   /* 线圈工作模式有效值 */
	MaxNum_HoldingRegister = ENDADDRESS6_HOLDREGISTER; /* 保持寄存器工作模式有效值 */
	MaxNum_InputRegister = ENDADDRESS5_INPUTREGISTER;  /* 输入寄存器工作模式有效值 */
	WriteOneInputRegister(INPUTREGISTER_SYSTEMSTATE, 1, STATE_INIT);
	return 0;
}
/**
 * @brief 校验并处理一帧 DSM Modbus RTU 请求，生成正常或标准异常响应。
 *
 * 处理顺序为最小长度、从站地址、CRC 和功能码对应帧长校验，随后分发 FC01、FC03、FC04、FC05 或 FC10 响应。
 *
 * @param rcvbuff 接收到的完整 DSM Modbus RTU 请求帧。
 * @param rcvcount 接收缓冲区中的有效帧长度，单位字节。
 * @param tx DSM Modbus RTU 响应帧输出缓冲区；正常响应和标准异常响应均写入此处。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 DSM_COMM_OK 表示请求已处理；其他值分别表示帧长、从机地址或 CRC 校验失败。
 * @note 当前实现同时接受本机地址和广播地址 0；非法功能码及数据形状由响应缓冲区返回 Modbus 异常帧。
 */
int DSM_CommunicationProcess(unsigned char *rcvbuff, int rcvcount, uint8_t* tx, uint16_t* tx_len) {
	int functioncode;
	unsigned short crc;
	if (rcvcount <= 3) {
		return DSM_COMM_ERR_BAD_LENGTH;
	}
	/* 校验地址 */
	if (rcvbuff[0] != SlaveAddress && rcvbuff[0] != 0) {
		return DSM_COMM_ERR_ADDRESS_MISMATCH;
	}
	if (SlaveCheckCRC(rcvbuff, rcvcount) == false) {
		return DSM_COMM_ERR_CRC;
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
			/* Response04 在地址分类和新鲜度门禁通过后再刷新目标影子。 */
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
	return DSM_COMM_OK;
}
