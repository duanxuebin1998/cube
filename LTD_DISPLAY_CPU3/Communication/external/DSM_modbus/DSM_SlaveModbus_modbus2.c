#include "DSM_SlaveModbus_modbus2.h"
#include "cpu2_communicate.h"
#include <stdlib.h>
#include <string.h>
#include "crc.h"
#include "DSM_comm.h"
#include "usart.h"
#include "spi.h"
#include "DSM_communication.h"
#include "DSM_DataAnalysis_modbus2.h"
#include <stdlib.h>
#include <string.h>
#include "stateformodbus.h"

extern struct MEASURE_DATA Measure_Data;
uint16_t DSM_HoldingRegisterArray[HOLDREGISTERAMOUNT] = {0}; // 保持寄存器数组
int DSM_InputRegisterArray[INPUTREGISTERAMOUNT] = {0};	// 输入寄存器数组
int MaxNum_Coil;									// 线圈最大有效值
int MaxNum_HoldingRegister;							// 保持寄存器最大有效值
int MaxNum_InputRegister;							// 输入寄存器最大有效值

const int readholdingregisterfuncode = 0x03;	// 读保持寄存器功能码
const int readinputregisterfuncode = 0x04;		// 读输入寄存器功能码
const int presetsinglecoilfuncode = 0x05;		// 写单个线圈功能码
const int presetmultipleregisterfuncode = 0x10; // 写多个寄存器功能码

const int HoldingregisterAddress = 0X00; // 已定义保持寄存器的起始地址
const int InputregisterAddress = 0X00;	 // 已定义输入寄存器的起始地址

const int IllegalFunction = 0X01;	 // 已定义异常码0x01
const int IllegalDataAddress = 0X02; // 已定义异常码0x02
const int IllegalDataValue = 0X03;	 // 已定义异常码0x03
const int SlaveFailure = 0X04;		 // 已定义异常码0x04
const int SlaveBusy = 0X06;			 // 已定义异常码0x06

int TempBuffer[1675]; // V1.116 dq2020.4.2


/******************************************************
函数功能： 获取功能码

函 数 名： GetFunctioncode
参    数： char *revframe   接收帧
		   int &funcode     功能码[out]

返 回 值：
		   true   成功
		   false  失败，功能码无法识别，返回参数无效
******************************************************/
bool GetFunctioncode(unsigned char *revframe, int *funcode)
{
	*funcode = revframe[1];

	if ((*funcode != readholdingregisterfuncode) && (*funcode != readinputregisterfuncode) && (*funcode != presetsinglecoilfuncode) && (*funcode != presetmultipleregisterfuncode))
	{
		return false;
	}
	else
	{
		return true;
	}
}


/******************************************************
函数功能： 读输入寄存器

函 数 名： ReadInputRegister
参    数： bool registertype           寄存器类型
									   true： 输入寄存器
									   false：保持寄存器
		   unsigned int startaddress   起始地址
		   unsigned int registeramount 数量
		   int *registervalue          寄存器值

返 回 值：
		   true   成功
		   false  失败,返回参数无效
******************************************************/
bool ReadInputRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue)
{
	int range;
	int i;
	int j;

	switch ((startaddress & 0x1180))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_INPUTREGISTER;
		break;

	case 0x080:
		startaddress = startaddress - STARTADDRESS2_INPUTREGISTER + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS3_INPUTREGISTER + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	default:
		break;
		//				return false;
	}

	if (startaddress >= 0x1000 && startaddress <= 0x129F) // V1.116 dq2020.4.2
		startaddress = startaddress - STARTADDRESS4_INPUTREGISTER + (ENDADDRESS3_INPUTREGISTER - STARTADDRESS3_INPUTREGISTER + 1) + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
	else if (startaddress >= 0x12A0 && startaddress <= 0x15C9) // V1.116 dq2020.4.2
		startaddress = startaddress - STARTADDRESS5_INPUTREGISTER + (ENDADDRESS4_INPUTREGISTER - STARTADDRESS4_INPUTREGISTER + 1) + (ENDADDRESS3_INPUTREGISTER - STARTADDRESS3_INPUTREGISTER + 1) + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);

	range = startaddress + registeramount;

	for (i = startaddress, j = 0; i < range; i++, j++)
	{
		registervalue[j] = DSM_InputRegisterArray[i];
	}

	return true;
}
/******************************************************
函数功能： 读单或双输入寄存器

函 数 名： ReadOneInputRegister
参    数：
		   unsigned int startaddress   起始地址
		   unsigned int registeramount 数量

返 回 值：
******************************************************/
int ReadOneInputRegister(unsigned int startaddress, u8 registeramount)
{
	int i, range, temp = 0;

	switch ((startaddress & 0x180))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_INPUTREGISTER;
		break;

	case 0x080:
		startaddress = startaddress - STARTADDRESS2_INPUTREGISTER + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS3_INPUTREGISTER + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	default:
		return false;
	}

	range = startaddress + registeramount;

	for (i = startaddress; i < range; i++)
	{
		temp <<= 16;
		temp += DSM_InputRegisterArray[i];
	}

	return temp;
}

/******************************************************
函数功能： 读单或双保持寄存器

函 数 名： ReadOneHoldingRegister
参    数： unsigned int startaddress   起始地址
		   unsigned int registeramount 数量

返 回 值：
******************************************************/
int ReadOneHoldingRegister(unsigned int startaddress, unsigned int registeramount)
{
	int range;
	int i, temp = 0;

	switch ((startaddress & 0x380))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_HOLDREGISTER;
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS2_HOLDREGISTER + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x180:
		startaddress = startaddress - STARTADDRESS3_HOLDREGISTER + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x200:
		startaddress = startaddress - STARTADDRESS4_HOLDREGISTER + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x280:
		startaddress = startaddress - STARTADDRESS5_HOLDREGISTER + (ENDADDRESS4_HOLDREGISTER - STARTADDRESS4_HOLDREGISTER + 1) + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	default:
		break;
	}

	range = startaddress + registeramount;

	for (i = startaddress; i < range; i++)
	{
		temp <<= 16;
		temp += DSM_HoldingRegisterArray[i];
	}

	return temp;
}

/******************************************************
函数功能： 读保持寄存器

函 数 名： ReadHoldingRegister
参    数： unsigned int startaddress   起始地址
		   unsigned int registeramount 数量
		   int *registervalue          寄存器值

返 回 值：
		   true   成功
		   false  失败,返回参数无效
******************************************************/
bool ReadHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue)
{
	int range;
	int i;
	int j;

	switch ((startaddress & 0x380))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_HOLDREGISTER;
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS2_HOLDREGISTER + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x180:
		startaddress = startaddress - STARTADDRESS3_HOLDREGISTER + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x200:
		startaddress = startaddress - STARTADDRESS4_HOLDREGISTER + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x280:
		startaddress = startaddress - STARTADDRESS5_HOLDREGISTER + (ENDADDRESS4_HOLDREGISTER - STARTADDRESS4_HOLDREGISTER + 1) + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	default:
		break;
	}

	range = startaddress + registeramount;

	for (i = startaddress, j = 0; i < range; i++, j++)
	{
		registervalue[j] = DSM_HoldingRegisterArray[i];
	}

	return true;
}
/******************************************************
函数功能： 写输入寄存器

函 数 名： WriteInputRegister
参    数：
		   unsigned int startaddress   起始地址
		   unsigned int registeramount 数量
		   int *registervalue          寄存器值

返 回 值：
		   true   成功
		   false  失败,返回参数无效
******************************************************/
bool WriteInputRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue)
{
	int range;
	int i;
	int j;

	switch ((startaddress & 0x180))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_INPUTREGISTER;
		break;

	case 0x080:
		startaddress = startaddress - STARTADDRESS2_INPUTREGISTER + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS3_INPUTREGISTER + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	default:
		return false;
	}

	range = startaddress + registeramount;

	for (i = startaddress, j = 0; i < range; i++, j++)
	{
		DSM_InputRegisterArray[i] = registervalue[j];
	}

	return true;
}

/******************************************************
函数功能： 写单或双输入寄存器

函 数 名： WriteOneInputRegister
参    数： unsigned int startaddress   起始地址
		   unsigned int registeramount 数量

返 回 值：
		   true   成功
		   false  失败,返回参数无效
******************************************************/
bool WriteOneInputRegister(unsigned int startaddress, unsigned int registeramount, int registervalue)
{
	int range;
	int i;

	switch ((startaddress & 0x1180))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_INPUTREGISTER;
		break;

	case 0x080:
		startaddress = startaddress - STARTADDRESS2_INPUTREGISTER + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS3_INPUTREGISTER + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
		break;

	default:
		break;
		//			return false;
	}

	if (startaddress >= 0x1000 && startaddress <= 0x129F) // V1.116 dq2020.4.2
		startaddress = startaddress - STARTADDRESS4_INPUTREGISTER + (ENDADDRESS3_INPUTREGISTER - STARTADDRESS3_INPUTREGISTER + 1) + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
	else if (startaddress >= 0x12A0 && startaddress <= 0x15C9) // V1.116 dq2020.4.2
		startaddress = startaddress - STARTADDRESS5_INPUTREGISTER + (ENDADDRESS4_INPUTREGISTER - STARTADDRESS4_INPUTREGISTER + 1) + (ENDADDRESS3_INPUTREGISTER - STARTADDRESS3_INPUTREGISTER + 1) + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);

	range = startaddress + registeramount - 1;

	for (i = range; i >= (int16_t)startaddress;i--)
	{
		DSM_InputRegisterArray[i] = registervalue % 0x10000;
		registervalue >>= 16;
	}

	return true;
}

/******************************************************
函数功能： 写单或双保持寄存器

函 数 名： WriteOneHoldingRegister
参    数： unsigned int startaddress   起始地址
		   unsigned int registeramount 数量

返 回 值：
******************************************************/
bool WriteOneHoldingRegister(unsigned int startaddress, unsigned int registeramount, int registervalue)
{
	int range;
	int16_t i;

	switch ((startaddress & 0x380))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_HOLDREGISTER;
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS2_HOLDREGISTER + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x180:
		startaddress = startaddress - STARTADDRESS3_HOLDREGISTER + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x200:
		startaddress = startaddress - STARTADDRESS4_HOLDREGISTER + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x280:
		startaddress = startaddress - STARTADDRESS5_HOLDREGISTER + (ENDADDRESS4_HOLDREGISTER - STARTADDRESS4_HOLDREGISTER + 1) + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	default:
		break;
	}

	range = startaddress + registeramount - 1;

	for (i = range; i >= (int16_t)startaddress; i--)
	{
		DSM_HoldingRegisterArray[i] = registervalue % 0x10000;
		registervalue >>= 16;
	}

	return true;
}
/******************************************************
函数功能： 写保持寄存器

函 数 名： WriteHoldingRegister
参    数： unsigned int startaddress   起始地址
		   unsigned int registeramount 数量
		   int *registervalue          寄存器值

返 回 值：
		   true   成功
		   false  失败,返回参数无效
******************************************************/
bool WriteHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue)
{
	int i;
	int j;
	int range;

	switch ((startaddress & 0x380))
	{
	case 0x000:
		startaddress = startaddress - STARTADDRESS1_HOLDREGISTER;
		break;

	case 0x100:
		startaddress = startaddress - STARTADDRESS2_HOLDREGISTER + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x180:
		startaddress = startaddress - STARTADDRESS3_HOLDREGISTER + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x200:
		startaddress = startaddress - STARTADDRESS4_HOLDREGISTER + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	case 0x280:
		startaddress = startaddress - STARTADDRESS5_HOLDREGISTER + (ENDADDRESS4_HOLDREGISTER - STARTADDRESS4_HOLDREGISTER + 1) + (ENDADDRESS3_HOLDREGISTER - STARTADDRESS3_HOLDREGISTER + 1) + (ENDADDRESS2_HOLDREGISTER - STARTADDRESS2_HOLDREGISTER + 1) + (ENDADDRESS1_HOLDREGISTER - STARTADDRESS1_HOLDREGISTER + 1);
		break;

	default:
		return false;
	}

	range = startaddress + registeramount;

	for (i = startaddress, j = 0; i < range; i++, j++)
	{
		DSM_HoldingRegisterArray[i] = registervalue[j];
	}

	return true;
}
///******************************************************
//函数功能： 写线圈
//
//函 数 名： PresetCoil
//参    数： unsigned int startaddress   起始地址
//		   unsigned int coilamount     数量
//		   unsigned int *coilvalue     线圈值 [0x0000 0xff00]
//
//返 回 值：
//		   true   成功
//		   false  失败,返回参数无效
//******************************************************/
//bool PresetCoil(unsigned int startaddress, int coilvalue)
//{
//	int coilarrayaddress;
//	int tempvalue;
//	int i;
//
//	if (startaddress < 0x0100)
//	{
//		coilarrayaddress = startaddress - STARTADDRESS1_COM;
//	}
//	else if (startaddress < 0x0200)
//	{
//		coilarrayaddress = startaddress - STARTADDRESS2_COM + (ENDADDRESS1_COM - STARTADDRESS1_COM + 1);
//	}
//	else
//	{
//		coilarrayaddress = startaddress - STARTADDRESS3_COM + (ENDADDRESS2_COM - STARTADDRESS2_COM + 1) + (ENDADDRESS1_COM - STARTADDRESS1_COM + 1);
//	}
//
//	if (coilvalue == 0x0000)
//	{
//		tempvalue = 0;
//		CoilArray[coilarrayaddress] = tempvalue;
//	}
//	else if (coilvalue == 0xff00)
//	{
//		tempvalue = 1;
//
//		for (i = 0; i < coilarrayaddress; i++)
//		{
//			CoilArray[i] = 0;
//		}
//
//		CoilArray[coilarrayaddress] = tempvalue;
//
//		for (i = coilarrayaddress + 1; i < COILAMOUNT; i++)
//		{
//			CoilArray[i] = 0;
//		}
//	}
//	return true;
//}
/******************************************************
函数功能： 下位机响应0x03请求

函 数 名： Response03
参    数： char *revframe   请求帧
		   char* sendframe  响应帧

返 回 值：
		   组帧的长度
******************************************************/
int Response03(unsigned char *revframe, unsigned char *sendframe)
{
	unsigned int functioncode;
	bool flagofaddress;
	int startaddress, registeramount, framelen, endaddress = 0;
	int i;
	int j;
	unsigned short crc;

	functioncode = revframe[1];
	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	registeramount = ((revframe[4] & 0x00ff) << 8) + revframe[5];
	endaddress = startaddress + registeramount - 1;
	flagofaddress = false;

	switch ((startaddress & 0x380))
	{
	case 0x000:
		if ((registeramount <= 0) || (endaddress > ENDADDRESS1_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x100:
		if ((startaddress > ENDADDRESS5_HOLDREGISTER) || (registeramount <= 0) || (endaddress > ENDADDRESS2_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x180:
		if ((startaddress > ENDADDRESS5_HOLDREGISTER) || (registeramount <= 0) || (endaddress > ENDADDRESS3_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x200:
		if ((startaddress > ENDADDRESS5_HOLDREGISTER) || (registeramount <= 0) || (endaddress > ENDADDRESS4_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x280:
		if ((startaddress > ENDADDRESS5_HOLDREGISTER) || (registeramount <= 0) || (endaddress > ENDADDRESS5_HOLDREGISTER))
			flagofaddress = true;

		break;

	default:
		flagofaddress = true;
		break;
	}


	memset(TempBuffer, 0, sizeof(TempBuffer));

	if (flagofaddress)
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + readholdingregisterfuncode;
		sendframe[2] = 0x02; // 超出自定义范围
		framelen = 3;
	}
	else
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = functioncode;
		sendframe[2] = registeramount * 2;
		// 读保持寄存器
		ReadHoldingRegister(startaddress, registeramount, TempBuffer);

		for (i = 0, j = 0; i < registeramount; i++, j = j + 2)
		{
			sendframe[j + 3] = (TempBuffer[i] >> 8) & 0xff;
			sendframe[j + 4] = TempBuffer[i] & 0xff;
		}

		framelen = 3 + registeramount * 2;
	}

	crc = CRC16_Calculate(sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;

	// free(registervalue);

	return (framelen + 2);
}
/******************************************************
函数功能： 下位机响应0x04请求

函 数 名： Response04
参    数： char *revframe   请求帧
		   char* sendframe  响应帧

返 回 值：
		   组帧的长度
******************************************************/
int Response04(unsigned char *revframe, unsigned char *sendframe)
{
	unsigned int functioncode;
	bool flagofaddress;
	int startaddress, registeramount, framelen, endaddress = 0;
	// int *registervalue;
	unsigned short crc;
	int i;
	int j;

	functioncode = revframe[1];
	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	registeramount = ((revframe[4] & 0x00ff) << 8) + revframe[5];
	endaddress = startaddress + registeramount - 1;
	//	Input_Write();//提到前面
	flagofaddress = false;

	switch ((startaddress & 0x1180))
	{
	case 0x000:
		if ((registeramount <= 0) || (endaddress > ENDADDRESS1_INPUTREGISTER))
			flagofaddress = true;

		break;

	case 0x080:
		if ((startaddress > MaxNum_InputRegister) || (registeramount <= 0) || (endaddress > ENDADDRESS2_INPUTREGISTER))
			flagofaddress = true;

		break;

	case 0x100:
		if ((startaddress > MaxNum_InputRegister) || (registeramount <= 0) || (endaddress > ENDADDRESS3_INPUTREGISTER))
			flagofaddress = true;

		break;

	default:
		flagofaddress = true;
		break;
	}

	if (startaddress >= 0x1000 && startaddress <= 0x129F && endaddress <= ENDADDRESS4_INPUTREGISTER) // V1.116 dq2020.4.2
		flagofaddress = false;
	else if (startaddress >= 0x12A0 && startaddress <= 0x15C9 && endaddress <= ENDADDRESS5_INPUTREGISTER) // V1.116 dq2020.4.2
		flagofaddress = false;

	memset(TempBuffer, 0, 1676);

	if (flagofaddress)
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + readinputregisterfuncode;
		sendframe[2] = 0x02; // 超出自定义范围
		framelen = 3;
	}
	else
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = functioncode;
		sendframe[2] = registeramount * 2;
		// 读输入寄存器
		ReadInputRegister(startaddress, registeramount, TempBuffer);

		for (i = 0, j = 0; i < registeramount; i++, j = j + 2)
		{
			sendframe[j + 3] = (TempBuffer[i] >> 8) & 0xff;
			sendframe[j + 4] = TempBuffer[i] & 0xff;
		}

		framelen = 3 + registeramount * 2;
	}

	crc = CRC16_Calculate(sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;

	// free(registervalue);

	return (framelen + 2);
}
typedef struct {
    uint16_t coil;   // 线圈起始地址（COM_xxx）
    uint8_t  cmd;    // 内部命令（CMD_xxx）
} CoilCmdMap;

/* 线圈地址 -> 内部命令映射表 */
static const CoilCmdMap g_coil_cmd_map[] = {
    /* ========= 工作模式区 (0x000A ~ 0x0017) ========= */
    { COM_SET_WORKPATTER,      CMD_UNKNOWN },                 // 只写工作模式，不直接触发动作

    { COM_BACK_ZERO,           CMD_BACK_ZERO },               // 回零点
    { COM_FIND_ZERO,           CMD_CALIBRATE_ZERO },          // 标定零点

    { COM_SINGLE_POINT,        CMD_MEASURE_SINGLE },          // 单点测量
    { COM_SP_TEST,             CMD_MONITOR_SINGLE },          // 单点监测

    { COM_SPREADPOINTS,        CMD_MEASURE_DISTRIBUTED },     // 分布测量（带高度）
    { COM_SPREADPOINTS_AI,     CMD_MEASURE_DISTRIBUTED },     // 自动分布测量（同一类命令）

    { COM_FIND_OIL,            CMD_FIND_OIL },                // 寻找液位
    { COM_FIND_WATER,          CMD_FIND_WATER },              // 寻找水位
    { COM_FIND_BOTTOM,         CMD_FIND_BOTTOM },             // 寻找罐底

    { COM_SYNTHETIC,           CMD_SYNTHETIC },               // 综合指令测量

    { COM_METER_DENSITY,       CMD_MEASURE_DENSITY_METER },   // 密度每米测量
    { COM_INTERVAL_DENSITY,    CMD_MEASURE_DENSITY_RANGE },   // 区间密度测量
    { COM_WATER_FOLLOW,    CMD_FOLLOW_WATER },   //水位跟随

    /* ========= 调试模式区 (0x0100 ~ 0x0107) ========= */
    { COM_CAL_OIL,             CMD_CALIBRATE_OIL },           // 液位标定
    { COM_READPARAMETER,       CMD_MAINTENANCE_MODE },        // 读取当前参数/维护动作

    { COM_RUNUP,               CMD_MOVE_UP },                 // 向上运行
    { COM_RUNDOWN,             CMD_MOVE_DOWN },               // 向下运行

    { COM_SET_ZEROCIRCLE,      CMD_UNKNOWN },          // 调整零点圈数
    { COM_SET_ZEROANGLE,       CMD_UNKNOWN },          // 调整零点角度
    { COM_CORRECTION_OIL,      CMD_CORRECT_OIL },             // 修正液位
    { COM_FORCE_ZERO,          CMD_CALIBRATE_ZERO },          // 强制零点

    /* ========= 解锁模式区 (0x0200 ~ 0x0202) ========= */
    { COM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY },      // 恢复出厂设置
    { COM_BACKUP_FILE,         CMD_RESTORE_FACTORY },        // 恢复出厂设置
    { COM_RESTORY_FILE,        CMD_RESTORE_FACTORY },        // 恢复出厂设置
};

/* 简单的数组长度宏 */
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* 线圈地址 -> CMD_ 指令 */
uint8_t GetCmdFromCoil(uint16_t coil_addr)
{
    for (size_t i = 0; i < ARRAY_SIZE(g_coil_cmd_map); i++) {
        if (g_coil_cmd_map[i].coil == coil_addr) {
            return g_coil_cmd_map[i].cmd;
        }
    }
    return CMD_UNKNOWN;
}

/******************************************************
函数功能： 下位机响应0x05请求

函 数 名： Response05
参    数： char *revframe   请求帧
		   char* sendframe  响应帧

返 回 值：
		   组帧的长度
******************************************************/
int Response05(unsigned char *revframe, unsigned char *sendframe)
{
	uint32_t cmd = 1;
	int startaddress;
	int coilvalue;
	int framelen = 0;
	unsigned short crc;
	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	coilvalue = ((revframe[4] & 0x00ff) << 8) + revframe[5];

	if ((startaddress > MaxNum_Coil) || (startaddress < STARTADDRESS1_COM) || ((startaddress > ENDADDRESS1_COM) && (startaddress < STARTADDRESS2_COM)) || ((startaddress > ENDADDRESS2_COM) && (startaddress < STARTADDRESS3_COM)) || (startaddress > ENDADDRESS3_COM))
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + presetsinglecoilfuncode;
		sendframe[2] = 0x02; // 非法数据地址
		framelen = 3;
	}
	else if ((coilvalue != 0x00) && (coilvalue != 0xff00))
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + presetsinglecoilfuncode;
		sendframe[2] = 0x03; // 超出自定义范围
		framelen = 3;
	}
	else
	{
		// 写线圈
//		PresetCoil(startaddress, coilvalue);
		//改成获取功能码
		sendframe[0] = SlaveAddress;
		sendframe[1] = presetsinglecoilfuncode;
		sendframe[2] = revframe[2];
		sendframe[3] = revframe[3];
		sendframe[4] = revframe[4];
		sendframe[5] = revframe[5];
		framelen = 6;
	}

	crc = CRC16_Calculate(sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;
	printf("startaddress=%04X,coilvalue=%04X\r\n", startaddress, coilvalue);

	cmd = GetCmdFromCoil(startaddress);
	// 发送命令给主控单元
	printf("cmd=%lu\r\n", cmd);
	CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, 2, &cmd);
	return (framelen + 2);
}
/******************************************************
函数功能： 下位机响应0x10请求

函 数 名： Response16
参    数： char *revframe   请求帧
		   char* sendframe  响应帧

返 回 值：
		   组帧的长度
******************************************************/
int Response16(unsigned char *revframe, unsigned char *sendframe)
{
	int ret;
	unsigned int functioncode;
	unsigned int stdHi;
	unsigned int stdLo;
	unsigned int amountHi;
	unsigned int amountLo;
	bool flagofaddress;
	int startaddress, endaddress;
	int registeramount;
	int framelen = 0;
	int i;
	int j;
	unsigned short crc;

	functioncode = revframe[1];
	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	registeramount = ((revframe[4] & 0x00ff) << 8) + revframe[5];
	endaddress = startaddress + registeramount - 1;
	stdHi = revframe[2];
	stdLo = revframe[3];
	amountHi = revframe[4];
	amountLo = revframe[5];
	flagofaddress = false;

	switch ((startaddress & 0x380))
	{
	case 0x000:
		if ((registeramount <= 0) || (endaddress > ENDADDRESS1_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x100:
		if ((startaddress > MaxNum_HoldingRegister) || (registeramount <= 0) || (endaddress > ENDADDRESS2_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x180:
		if ((startaddress > MaxNum_HoldingRegister) || (registeramount <= 0) || (endaddress > ENDADDRESS3_WRITE_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x200:
		if ((startaddress > MaxNum_HoldingRegister) || (registeramount <= 0) || (endaddress > ENDADDRESS4_HOLDREGISTER))
			flagofaddress = true;

		break;

	case 0x280:
		if ((startaddress > MaxNum_HoldingRegister) || (registeramount <= 0) || (endaddress > ENDADDRESS5_HOLDREGISTER))
			flagofaddress = true;

		break;

	default:
		flagofaddress = true;
		break;
	}

	// 状态识别

	// printf("flagofaddress=%d\r\n",flagofaddress);
	// registervalue = malloc(registeramount);
	memset(TempBuffer, 0, sizeof(TempBuffer));
	for (i = 0, j = 0; i < registeramount; i++, j = j + 2)
	{
		TempBuffer[i] = ((revframe[j + 7] & 0x00ff) << 8) + revframe[j + 8];
	}

	if (flagofaddress)
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + presetmultipleregisterfuncode;
		sendframe[2] = EXCEPTIONCODE_ERRORADDRESS; // 超出地址范围
		framelen = 3;
	}
	else
	{
		//写到本地保持寄存器数组
		WriteHoldingRegister(startaddress, registeramount, TempBuffer);
		//解析保持寄存器到设备参数，并下发到CPU2
		ret = UpdateDeviceParamsFromLegacyRegs(startaddress, registeramount);

		if (ret == PARAMETER_ERROR) // 设置的数据错误
		{
			sendframe[0] = SlaveAddress;
			sendframe[1] = 0x80 + presetmultipleregisterfuncode;
			sendframe[2] = EXCEPTIONCODE_ERRORDATA; // 设置数据超出范围
			framelen = 3;
		}
		else if (ret == PARAMETER_WRITE_FAIL) // V1.106待机和故障状态允许设置
		{
			sendframe[0] = SlaveAddress;
			sendframe[1] = 0x80 + presetmultipleregisterfuncode;
			sendframe[2] = EXCEPTIONCODE_ERRORDEVIVEBUSY; // 设备忙
			framelen = 3;
		}
		else // 正常情况
		{
			sendframe[0] = SlaveAddress;
			sendframe[1] = functioncode;
			sendframe[2] = stdHi;
			sendframe[3] = stdLo;
			sendframe[4] = amountHi;
			sendframe[5] = amountLo;
			framelen = 6;
		}
	}

	crc = CRC16_Calculate(sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;
	return (framelen + 2);
}

/******************************************************
函数功能：下位机响应异常请求

函数名称: ResponseException
函数参数：unsigned int functioncode   功能码
		   unsigned int exception     异常码
		   char* sendframe            响应帧

返回值：
			组帧长度
			-1组帧失败
******************************************************/
int ResponseException(unsigned int functioncode, unsigned int exception, unsigned char *sendframe)
{
	int framelen = -1;
	unsigned short crc;
	framelen = 3;
	sendframe[0] = SlaveAddress;
	sendframe[1] = functioncode + 0x80;
	sendframe[2] = exception;
	crc = CRC16_Calculate((unsigned char *)sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;

	framelen = 5;
	return framelen;
}
