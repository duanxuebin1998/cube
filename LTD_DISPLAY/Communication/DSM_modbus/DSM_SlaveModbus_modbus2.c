#include "DSM_SlaveModbus_modbus2.h"

#include <stdlib.h>
#include <string.h>
#include "crc.h"
#include "DSM_comm.h"
#include "usart.h"
#include "spi.h"
#include "DSM_communication.h"
#include "DSM_DataAnalysis_modbus2.h"
#include "DSM_system_parameter.h"
#include <stdlib.h>
#include <string.h>
#include "stateformodbus.h"

extern struct MEASURE_DATA Measure_Data;
int SlaveAddress = 0x01;							// 下位机地址
int HoldingRegisterArray[HOLDREGISTERAMOUNT] = {0}; // 保持寄存器数组
int InputRegisterArray[INPUTREGISTERAMOUNT] = {0};	// 输入寄存器数组
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
函数功能： 设置下位机地址

函 数 名： SetSlaveaddress
参    数： int address   传入地址
返 回 值： false  失败，地址超范围[1-247]
		   true   成功
******************************************************/
bool SetSlaveaddress(int address)
{
	if ((address < 1) || (address > 247))
	{
		SlaveAddress = -1;
		return false;
	}
	else
	{
		SlaveAddress = address;
		return true;
	}
}
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
		registervalue[j] = InputRegisterArray[i];
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
		temp += InputRegisterArray[i];
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
		temp += HoldingRegisterArray[i];
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
		registervalue[j] = HoldingRegisterArray[i];
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
		InputRegisterArray[i] = registervalue[j];
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
		InputRegisterArray[i] = registervalue % 0x10000;
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
		HoldingRegisterArray[i] = registervalue % 0x10000;
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
		HoldingRegisterArray[i] = registervalue[j];
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


	memset(TempBuffer, 0, 256);

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
	if (startaddress == 0x000B) // 回零点
		cmd = CMD_BACK_ZERO;
	else if (startaddress == 0x0011) // 液位测量
		cmd = CMD_FIND_OIL;
	else if (startaddress == 0x0013) // 罐底测量
		cmd = CMD_FIND_BOTTOM;
	else
		cmd = CMD_NONE;
	// 发送命令给主控单元
	printf("cmd=%d\r\n", cmd);
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
	memset(TempBuffer, 0, 256);

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
		// 写保持寄存器
		WriteHoldingRegister(startaddress, registeramount, TempBuffer);
		ret = SystemParameterSave(startaddress, registeramount);

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
