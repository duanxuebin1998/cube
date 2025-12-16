#include "hostcommu.h"
#include "hostcommu_modbus.h"
#include "usart.h"
#include <string.h>
#include "stateformodbus.h"
#include "crc.h"
#include <ctype.h>
#include <stdbool.h>
#include "my_crc.h"
#include "stdio.h"

#define DEBUG_HOSTCOMMU 0
// 常量定义
#define MAXRCVLENGTH 256 // Modbus帧最大接收长度（RTU模式一般为256字节）

// 发送缓冲区（静态分配）
static uint8_t  HCOM_SendBuff[HOSTCOMMU_SENDLENGTH]; // Modbus响应帧缓冲区
static int HCOM_SendCount = 0;                   // 发送缓冲区当前数据长度

/**
 * @brief Modbus从站初始化函数
 *
 * @return int 初始化状态（0表示成功）
 */
int HostCommuInit(void) {
	int ret = 0;

	// 设置从站地址（默认为01）
	SetSlaveaddress(01);

	return ret;
}

/**
 * @brief Modbus通信处理主函数
 *
 * @param rcvbuff 接收到的Modbus帧数据
 * @param rcvcount 接收到的数据长度
 * @param commu_num 通信通道号（用于多接口系统）
 */
void HostCommuProcess(uint8_t *rcvbuff, int rcvcount) {
	int functioncode;    // Modbus功能码
	int startaddress;    // 寄存器起始地址
	int registeramount;  // 寄存器数量
	uint16_t crc;        // CRC校验值

	// 调试输出：打印接收到的原始帧数据（仅在调试模式启用时）
#if DEBUG_HOSTCOMMU
	int i;
	printf("HostRcv %d:\t", rcvcount);
	for (i = 0; i < rcvcount; i++) {
		printf("%02X ", rcvbuff[i]);
	}
	printf("\n");
#endif

	// 检查1: 接收数据长度有效性
	if ((rcvcount <= 3) || (rcvcount >= MAXRCVLENGTH)) {
		// 帧长度过短或过长都不合法
#if DEBUG_HOSTCOMMU
		printf("HOSTCOMM:length %d error\r\n", rcvcount);
#endif
	}
	// 检查2: 目标地址校验
	else if (SlaveCheckAddress(rcvbuff, rcvcount) == false) {
		// 地址不匹配，不是发给本机的请求
#if DEBUG_HOSTCOMMU
		printf("HOSTCOMM:Address %d error\r\n", rcvbuff[0]);
#endif
	}
	// 检查3: CRC校验
	else if (SlaveCheckCRC(rcvbuff, rcvcount) == false) {
		// CRC校验失败，打印错误信息和接收到的原始数据
		printf("CRC-err -  HostRcv %d:\t", rcvcount);
		{
			int i;
			printf("HostRcv %d:\t", rcvcount);
			for (i = 0; i < rcvcount; i++) {
				printf("%x\t", rcvbuff[i]);
			}
			printf("\n");
		}
	}
	// 处理有效请求
	else {
		// 解析功能码 (第2字节)
		functioncode = rcvbuff[1];

		// 解析起始地址 (第3-4字节，高位在前)
		startaddress = (rcvbuff[2] << 8) + rcvbuff[3];

		// 解析寄存器数量 (第5-6字节，高位在前)
		registeramount = (rcvbuff[4] << 8) + rcvbuff[5];

		// 更新接收参数状态（用于调试/监控）
		UpdateRcvPara(functioncode, startaddress, registeramount);

		// 准备发送缓冲区（重置长度）
		HCOM_SendCount = 0;

		// 检查4: 功能码合法性
		if (FunctionCheckIllPack(HCOM_SendBuff, &HCOM_SendCount) == false) {
			// 非支持的功能码
#if DEBUG_HOSTCOMMU
			printf("HOSTCOMM:Function %d error\r\n", functioncode);
#endif
		}
		// 检查5: 数据地址合法性
		else if (IllegalDataAddressPack(HCOM_SendBuff, &HCOM_SendCount) == false) {
			// 请求的寄存器地址或数量超出范围
#if DEBUG_HOSTCOMMU
			printf("HOSTCOMM:Startadd %d regiscnt %d error\r\n", startaddress, registeramount);
#endif
		}
		// 处理支持的合法请求
		else {
			// 根据功能码调用对应的处理函数
			switch (functioncode) {
			case FUNCTIONCODE_READ_HOLDREGISTER:  // 03 - 读保持寄存器
				HCOM_SendCount = Response03Process(rcvbuff, HCOM_SendBuff);
				break;

			case FUNCTIONCODE_READ_INPUTREGISTER: // 04 - 读输入寄存器
				HCOM_SendCount = Response04Process(rcvbuff, HCOM_SendBuff);
				break;

			case FUNCTIONCODE_WRITE_MULREGISTER:  // 16 - 写多个寄存器
				HCOM_SendCount = Response10Process(rcvbuff, HCOM_SendBuff);
				break;
			}
		}

		//添加CRC校验到响应帧尾部
		crc = CRC16_Calculate(HCOM_SendBuff, HCOM_SendCount);
		HCOM_SendBuff[HCOM_SendCount] = crc & 0xff;    // CRC低字节
		HCOM_SendBuff[HCOM_SendCount + 1] = crc >> 8;   // CRC高字节
		HCOM_SendCount += 2;
#if DEBUG_HOSTCOMMU
		printf("HOSTCOMM: send %d bytes:", HCOM_SendCount);
		for (int i = 0; i < HCOM_SendCount; i++) {
			printf(" %02X", (unsigned char) HCOM_SendBuff[i]);
		}
		printf("\r\n");
#endif
		//切换发送模式
		RS485_SET_SEND_MODE();  // 切换到发送模式
		HAL_UART_Transmit_DMA(&huart5, (uint8_t*)HCOM_SendBuff, HCOM_SendCount);  // 通过UART发送响应帧
	}
}
