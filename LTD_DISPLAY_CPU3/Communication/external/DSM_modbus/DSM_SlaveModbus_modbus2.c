#include "DSM_SlaveModbus_modbus2.h"
#include "cpu2_communicate.h"
#include "cpu3_debug_log.h"
#include <stdlib.h>
#include <string.h>
#include "crc.h"
#include "DSM_comm.h"
#include "usart.h"
#include "spi.h"
#include "DSM_communication.h"
#include "DSM_DataAnalysis_modbus2.h"
#include "../external_read_freshness.h"
#include <stdlib.h>
#include <string.h>
#include "stateformodbus.h"

extern struct MEASURE_DATA Measure_Data;
uint16_t DSM_HoldingRegisterArray[HOLDREGISTERAMOUNT] = {0}; /* 保持寄存器数组 */
int DSM_InputRegisterArray[INPUTREGISTERAMOUNT] = {0};	/* 输入寄存器数组 */
int MaxNum_Coil;									/* 线圈最大有效值 */
int MaxNum_HoldingRegister;							/* 保持寄存器最大有效值 */
int MaxNum_InputRegister;							/* 输入寄存器最大有效值 */

const int readcoilfuncode = 0x01;				/* 读线圈功能码 */
const int readholdingregisterfuncode = 0x03;	/* 读保持寄存器功能码 */
const int readinputregisterfuncode = 0x04;		/* 读输入寄存器功能码 */
const int presetsinglecoilfuncode = 0x05;		/* 写单个线圈功能码 */
const int presetmultipleregisterfuncode = 0x10; /* 写多个寄存器功能码 */

int TempBuffer[1675]; /* V1.116 dq2020.4.2 */

static uint8_t s_dsm_self_check_placeholder_step = 0U; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */

#define DSM_MAX_WRITE_REGISTER_COUNT 123U

/* FC16 同步失败时恢复最后一次已确认的参数与 DSM 保持寄存器镜像。 */
static DeviceParameters s_dsm_parameter_snapshot;
static int s_dsm_holding_register_snapshot[DSM_MAX_WRITE_REGISTER_COUNT];

/*
 * 函数用途：刷新 DSM FC04 可离线读取的 CPU3 本地静态字段。
 * 调用场景：FC04 完成整帧新鲜度分类后、读取输入寄存器影子前调用。
 * 关键约束：本函数不得读取 g_measurement，也不得推进 0x0001 自检生命周期。
 */
static void DSM_RefreshLocalInputRegisters(void)
{
	WriteOneInputRegister(INPUTREGISTER_PATTERNOFWORK, 1U, 1);
	WriteOneInputRegister(INPUTREGISTER_SP_TDSTATE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_SPT_TDSTATE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_SPREAD_AVERAGETDSTATE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_VALUE_P1, 2U, 0);
	WriteOneInputRegister(INPUTREGISTER_VALUE_P3, 2U, 0);
	WriteOneInputRegister(INPUTREGISTER_ZEROCIRCLE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_ZEROANGLE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_TANKHIGHT, 2U, 0);
	WriteOneInputRegister(INPUTREGISTER_AMPLITUDE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_SENSORX_ANGLE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_SENSORY_ANGLE, 1U, 0);
	WriteOneInputRegister(INPUTREGISTER_WARTER_VOLTAGE, 1U, 0);
}

/*
 * 函数功能：判断保持寄存器访问是否落在 DSM V1.228 第6段占位区。
 * 说明：本次确认第6段只需要能响应并返回0，不落本地参数、不下发 CPU2。
 */
static bool IsHoldingRegisterZeroSegment(unsigned int startaddress, unsigned int registeramount)
{
	if (registeramount == 0U)
	{
		return false;
	}
	return (startaddress >= STARTADDRESS6_HOLDREGISTER) &&
		   (startaddress <= ENDADDRESS6_HOLDREGISTER) &&
		   (registeramount <= (ENDADDRESS6_HOLDREGISTER - startaddress + 1U));
}

/*
 * 函数功能：记录一次 DSM 自检占位请求。
 * 说明：当前不扩展 CPU2 共享命令，只在后续输入寄存器读取时输出一轮自检状态。
 */
void DSM_RequestSelfCheckPlaceholder(void)
{
	s_dsm_self_check_placeholder_step = 1U;
}

/*
 * 函数功能：消费 DSM 自检占位状态。
 * 说明：第一次返回自检中，第二次返回自检完成，再恢复为真实内部状态翻译。
 */
uint16_t DSM_ConsumeSelfCheckPlaceholderState(void)
{
	if (s_dsm_self_check_placeholder_step == 1U)
	{
		s_dsm_self_check_placeholder_step = 2U;
		return 0x0027U;
	}

	if (s_dsm_self_check_placeholder_step == 2U)
	{
		s_dsm_self_check_placeholder_step = 0U;
		return 0x8027U;
	}

	return 0U;
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

	if ((*funcode != readcoilfuncode) && (*funcode != readholdingregisterfuncode) && (*funcode != readinputregisterfuncode) && (*funcode != presetsinglecoilfuncode) && (*funcode != presetmultipleregisterfuncode))
	{
		return false;
	}
	else
	{
		return true;
	}
}

/* 兼容上位机读线圈请求：按协议格式正常响应，线圈值统一返回 0。 */
int Response01(unsigned char *revframe, unsigned char *sendframe)
{
	unsigned int startaddress;
	unsigned int coilamount;
	unsigned int endaddress;
	unsigned int bytecount;
	unsigned short crc;
	int framelen;
	unsigned int i;

	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	coilamount = ((revframe[4] & 0x00ff) << 8) + revframe[5];

	if ((coilamount == 0U) || (coilamount > 2000U))
	{
		return ResponseException(readcoilfuncode, EXCEPTIONCODE_ERRORDATA, sendframe);
	}

	endaddress = startaddress + coilamount - 1U;
	if ((startaddress > (unsigned int)MaxNum_Coil) || (endaddress > (unsigned int)MaxNum_Coil))
	{
		return ResponseException(readcoilfuncode, EXCEPTIONCODE_ERRORADDRESS, sendframe);
	}

	bytecount = (coilamount + 7U) / 8U;
	sendframe[0] = SlaveAddress;
	sendframe[1] = readcoilfuncode;
	sendframe[2] = (unsigned char)bytecount;
	for (i = 0; i < bytecount; i++)
	{
		sendframe[3 + i] = 0x00;
	}

	framelen = 3 + (int)bytecount;
	crc = CRC16_Calculate(sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;
	return framelen + 2;
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
		/* return false; */
	}

	if (startaddress >= 0x1000 && startaddress <= 0x129F) /* V1.116 dq2020.4.2 */
		startaddress = startaddress - STARTADDRESS4_INPUTREGISTER + (ENDADDRESS3_INPUTREGISTER - STARTADDRESS3_INPUTREGISTER + 1) + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
	else if (startaddress >= 0x12A0 && startaddress <= 0x15C9) /* V1.116 dq2020.4.2 */
		startaddress = startaddress - STARTADDRESS5_INPUTREGISTER + (ENDADDRESS4_INPUTREGISTER - STARTADDRESS4_INPUTREGISTER + 1) + (ENDADDRESS3_INPUTREGISTER - STARTADDRESS3_INPUTREGISTER + 1) + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);

	range = startaddress + registeramount;

	for (i = startaddress, j = 0; i < range; i++, j++)
	{
		registervalue[j] = DSM_InputRegisterArray[i];
	}

	return true;
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

	if (IsHoldingRegisterZeroSegment(startaddress, registeramount))
	{
		return 0;
	}

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
		return 0;
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

	if (IsHoldingRegisterZeroSegment(startaddress, registeramount))
	{
		for (j = 0; j < (int)registeramount; j++)
		{
			registervalue[j] = 0;
		}
		return true;
	}

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
		registervalue[j] = DSM_HoldingRegisterArray[i];
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
		/* return false; */
	}

	if (startaddress >= 0x1000 && startaddress <= 0x129F) /* V1.116 dq2020.4.2 */
		startaddress = startaddress - STARTADDRESS4_INPUTREGISTER + (ENDADDRESS3_INPUTREGISTER - STARTADDRESS3_INPUTREGISTER + 1) + (ENDADDRESS2_INPUTREGISTER - STARTADDRESS2_INPUTREGISTER + 1) + (ENDADDRESS1_INPUTREGISTER - STARTADDRESS1_INPUTREGISTER + 1);
	else if (startaddress >= 0x12A0 && startaddress <= 0x15C9) /* V1.116 dq2020.4.2 */
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

	if (IsHoldingRegisterZeroSegment(startaddress, registeramount))
	{
		return true;
	}

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

	if (IsHoldingRegisterZeroSegment(startaddress, registeramount))
	{
		return true;
	}

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
/* / ****************************************************** */
/* 函数功能： 写线圈 */
/* */
/* 函 数 名： PresetCoil */
/* 参 数： unsigned int startaddress 起始地址 */
/* unsigned int coilamount 数量 */
/* unsigned int *coilvalue 线圈值 [0x0000 0xff00] */
/* */
/* 返 回 值： */
/* true 成功 */
/* false 失败,返回参数无效 */
/* ****************************************************** / */
/* bool PresetCoil(unsigned int startaddress, int coilvalue) */
/* { */
/* int coilarrayaddress; */
/* int tempvalue; */
/* int i; */
/* */
/* if (startaddress < 0x0100) */
/* { */
/* coilarrayaddress = startaddress - STARTADDRESS1_COM; */
/* } */
/* else if (startaddress < 0x0200) */
/* { */
/* coilarrayaddress = startaddress - STARTADDRESS2_COM + (ENDADDRESS1_COM - STARTADDRESS1_COM + 1); */
/* } */
/* else */
/* { */
/* coilarrayaddress = startaddress - STARTADDRESS3_COM + (ENDADDRESS2_COM - STARTADDRESS2_COM + 1) + (ENDADDRESS1_COM - STARTADDRESS1_COM + 1); */
/* } */
/* */
/* if (coilvalue == 0x0000) */
/* { */
/* tempvalue = 0; */
/* CoilArray[coilarrayaddress] = tempvalue; */
/* } */
/* else if (coilvalue == 0xff00) */
/* { */
/* tempvalue = 1; */
/* */
/* for (i = 0; i < coilarrayaddress; i++) */
/* { */
/* CoilArray[i] = 0; */
/* } */
/* */
/* CoilArray[coilarrayaddress] = tempvalue; */
/* */
/* for (i = coilarrayaddress + 1; i < COILAMOUNT; i++) */
/* { */
/* CoilArray[i] = 0; */
/* } */
/* } */
/* return true; */
/* } */
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
	if ((registeramount <= 0) || (registeramount > 125))
	{
		return ResponseException(functioncode, EXCEPTIONCODE_ERRORDATA, sendframe);
	}
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

	case 0x300:
		if (!IsHoldingRegisterZeroSegment((unsigned int)startaddress, (unsigned int)registeramount))
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
		sendframe[2] = 0x02; /* 超出自定义范围 */
		framelen = 3;
	}
	else if (!IsHoldingRegisterZeroSegment((unsigned int)startaddress, (unsigned int)registeramount) &&
			 !CPU2_CommIsAvailable())
	{
		/* CPU2 参数快照无效时禁止返回未确认值，第6段全零占位不受影响。 */
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + readholdingregisterfuncode;
		sendframe[2] = EXCEPTIONCODE_ERRORDEVIVEBUSY;
		framelen = 3;
	}
	else
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = functioncode;
		sendframe[2] = registeramount * 2;
		/* 读保持寄存器 */
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

	/* free(registervalue); */

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
	/* int *registervalue; */
	unsigned short crc;
	int i;
	int j;

	functioncode = revframe[1];
	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	registeramount = ((revframe[4] & 0x00ff) << 8) + revframe[5];
	if ((registeramount <= 0) || (registeramount > 125))
	{
		return ResponseException(functioncode, EXCEPTIONCODE_ERRORDATA, sendframe);
	}
	endaddress = startaddress + registeramount - 1;
	/* Input_Write();/ /提到前面 */
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

	if (startaddress >= 0x1000 && startaddress <= 0x129F && endaddress <= ENDADDRESS4_INPUTREGISTER) /* V1.116 dq2020.4.2 */
		flagofaddress = false;
	else if (startaddress >= 0x12A0 && startaddress <= 0x15C9 && endaddress <= ENDADDRESS5_INPUTREGISTER) /* V1.116 dq2020.4.2 */
		flagofaddress = false;

	memset(TempBuffer, 0, 1676);

	if (flagofaddress)
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + readinputregisterfuncode;
		sendframe[2] = 0x02; /* 超出自定义范围 */
		framelen = 3;
	}
	else
	{
		uint8_t needs_runtime = CPU3_ExternalDsmInputRangeNeedsRuntime(
			(uint16_t)startaddress,
			(uint16_t)registeramount);

		if ((needs_runtime != 0U) && !CPU2_CommHasRuntimeSnapshot())
		{
			/* CPU2 派生字段和混合范围在首次失联后整帧返回设备忙。 */
			sendframe[0] = SlaveAddress;
			sendframe[1] = 0x80 + readinputregisterfuncode;
			sendframe[2] = EXCEPTIONCODE_ERRORDEVIVEBUSY;
			framelen = 3;
		}
		else
		{
			/* 先刷新静态白名单；只有运行态门禁通过后才允许消费 CPU2 与自检影子。 */
			DSM_RefreshLocalInputRegisters();
			if (needs_runtime != 0U)
			{
				Input_Write();
			}

			sendframe[0] = SlaveAddress;
			sendframe[1] = functioncode;
			sendframe[2] = registeramount * 2;
			/* 读输入寄存器 */
			ReadInputRegister(startaddress, registeramount, TempBuffer);

			for (i = 0, j = 0; i < registeramount; i++, j = j + 2)
			{
				sendframe[j + 3] = (TempBuffer[i] >> 8) & 0xff;
				sendframe[j + 4] = TempBuffer[i] & 0xff;
			}

			framelen = 3 + registeramount * 2;
		}
	}

	crc = CRC16_Calculate(sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;

	/* free(registervalue); */

	return (framelen + 2);
}
typedef enum {
	COIL_ACTION_SEND_CMD,
	COIL_ACTION_NOOP_OK,
	COIL_ACTION_INVALID_CMD,
	COIL_ACTION_SELF_CHECK_PLACEHOLDER,
} CoilActionType;

typedef struct {
    uint16_t coil;          /* 线圈起始地址（COM_xxx） */
    uint8_t  cmd;           /* 内部命令（CMD_xxx） */
	CoilActionType action; /* DSM确认口径：下发、占位成功或返回无效指令 */
} CoilCmdMap;

/* 线圈地址 -> 内部命令映射表 */
static const CoilCmdMap g_coil_cmd_map[] = {
    /* ========= 工作模式区 (0x000A ~ 0x0017) ========= */
    { COM_SET_WORKPATTER,      CMD_UNKNOWN, COIL_ACTION_NOOP_OK },                 /* 只写工作模式，不直接触发动作 */

    { COM_BACK_ZERO,           CMD_BACK_ZERO, COIL_ACTION_SEND_CMD },               /* 回零点 */
    { COM_FIND_ZERO,           CMD_BACK_ZERO, COIL_ACTION_SEND_CMD },               /* 确认口径：0x000C 也执行回零点 */

    { COM_SINGLE_POINT,        CMD_MEASURE_SINGLE, COIL_ACTION_SEND_CMD },          /* 单点测量 */
    { COM_SP_TEST,             CMD_MONITOR_SINGLE, COIL_ACTION_SEND_CMD },          /* 单点监测 */

    { COM_SPREADPOINTS,        CMD_MEASURE_DISTRIBUTED, COIL_ACTION_SEND_CMD },     /* 分布测量（带高度） */
    { COM_SPREADPOINTS_AI,     CMD_MEASURE_DISTRIBUTED, COIL_ACTION_SEND_CMD },     /* 自动分布测量（同一类命令） */

    { COM_FIND_OIL,            CMD_FIND_OIL, COIL_ACTION_SEND_CMD },                /* 寻找液位 */
    { COM_FIND_WATER,          CMD_FIND_WATER, COIL_ACTION_SEND_CMD },              /* 寻找水位 */
    { COM_FIND_BOTTOM,         CMD_FIND_BOTTOM, COIL_ACTION_SEND_CMD },             /* 寻找罐底 */

    { COM_SYNTHETIC,           CMD_SYNTHETIC, COIL_ACTION_SEND_CMD },               /* 综合指令测量 */

    { COM_METER_DENSITY,       CMD_MEASURE_DENSITY_METER, COIL_ACTION_SEND_CMD },   /* 密度每米测量 */
    { COM_INTERVAL_DENSITY,    CMD_MEASURE_DENSITY_RANGE, COIL_ACTION_SEND_CMD },   /* 区间密度测量 */
    { COM_WATER_FOLLOW,        CMD_FOLLOW_WATER, COIL_ACTION_SEND_CMD },            /* 水位跟随兼容 */

    /* ========= 调试模式区 (0x0100 ~ 0x0107) ========= */
    { COM_CAL_OIL,             CMD_CALIBRATE_OIL, COIL_ACTION_SEND_CMD },           /* 液位标定 */
    { COM_READPARAMETER,       CMD_READ_PART_PARAMS, COIL_ACTION_SEND_CMD },         /* 读取当前参数 */

    { COM_RUNUP,               CMD_MOVE_UP, COIL_ACTION_SEND_CMD },                 /* 向上运行 */
    { COM_RUNDOWN,             CMD_MOVE_DOWN, COIL_ACTION_SEND_CMD },               /* 向下运行 */

    { COM_SET_ZEROCIRCLE,      CMD_UNKNOWN, COIL_ACTION_INVALID_CMD },              /* DSM V1.228 按无效指令处理 */
    { COM_SET_ZEROANGLE,       CMD_UNKNOWN, COIL_ACTION_INVALID_CMD },              /* DSM V1.228 按无效指令处理 */
    { COM_CORRECTION_OIL,      CMD_CORRECT_OIL, COIL_ACTION_SEND_CMD },             /* 修正液位 */
    { COM_FORCE_ZERO,          CMD_CALIBRATE_ZERO, COIL_ACTION_SEND_CMD },          /* 确认口径：保持原有映射 */
    { COM_CAL_WATER,           CMD_CALIBRATE_WATER, COIL_ACTION_SEND_CMD },         /* 水位标定 */
    { COM_SELF_CHECK,          CMD_UNKNOWN, COIL_ACTION_SELF_CHECK_PLACEHOLDER },    /* 自检占位，不扩展CPU2协议 */
    { COM_CALIBRATE_TANKHEIGHT, CMD_CALIBRATE_TANKHEIGHT, COIL_ACTION_SEND_CMD },   /* 罐高标定 */

    /* ========= 解锁模式区 (0x0200 ~ 0x0202) ========= */
    { COM_RESTOR_EFACTORYSETTING, CMD_RESTORE_FACTORY, COIL_ACTION_SEND_CMD },      /* 恢复出厂设置 */
    { COM_BACKUP_FILE,         CMD_UNKNOWN, COIL_ACTION_NOOP_OK },                  /* 备份文件占位，不恢复出厂 */
    { COM_RESTORY_FILE,        CMD_UNKNOWN, COIL_ACTION_INVALID_CMD },              /* DSM V1.228 按无效指令处理 */
};

/* 简单的数组长度宏 */
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/**
 * @brief 根据 DSM 线圈地址查找对应的 CPU2 命令映射。
 * @param coil_addr DSM 线圈地址。
 * @return 命令映射指针，NULL 表示该线圈未定义命令。
 */
static const CoilCmdMap *FindCoilCmdMap(uint16_t coil_addr)
{
    for (size_t i = 0; i < ARRAY_SIZE(g_coil_cmd_map); i++) {
        if (g_coil_cmd_map[i].coil == coil_addr) {
            return &g_coil_cmd_map[i];
        }
    }
    return NULL;
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
	uint32_t cmd = CMD_UNKNOWN;
	int startaddress;
	int coilvalue;
	int framelen = 0;
	bool should_send_cmd = false;
	const CoilCmdMap *coil_map;
	unsigned short crc;
	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	coilvalue = ((revframe[4] & 0x00ff) << 8) + revframe[5];
	coil_map = FindCoilCmdMap((uint16_t)startaddress);

	if (coil_map == NULL)
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + presetsinglecoilfuncode;
		sendframe[2] = 0x02; /* 非法数据地址 */
		framelen = 3;
	}
	else if ((coilvalue != 0x0000) && (coilvalue != 0xff00))
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + presetsinglecoilfuncode;
		sendframe[2] = 0x03; /* 超出自定义范围 */
		framelen = 3;
	}
	else if (coil_map->action == COIL_ACTION_INVALID_CMD)
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + presetsinglecoilfuncode;
		sendframe[2] = EXCEPTIONCODE_ERRORDATA; /* 已定义地址，但按确认口径属于无效命令 */
		framelen = 3;
	}
	else
	{
		/* 写线圈 */
/* PresetCoil(startaddress, coilvalue); */
		sendframe[0] = SlaveAddress;
		sendframe[1] = presetsinglecoilfuncode;
		sendframe[2] = revframe[2];
		sendframe[3] = revframe[3];
		sendframe[4] = revframe[4];
		sendframe[5] = revframe[5];
		framelen = 6;

		if (coilvalue == 0xff00)
		{
			if (coil_map->action == COIL_ACTION_SEND_CMD)
			{
				cmd = coil_map->cmd;
				should_send_cmd = (cmd != CMD_UNKNOWN);
			}
			else if (coil_map->action == COIL_ACTION_SELF_CHECK_PLACEHOLDER)
			{
				DSM_RequestSelfCheckPlaceholder();
			}
		}
	}

	CPU3_LOG_DEBUG("DSM",
				   "写线圈 起始地址=0x%04X 值=0x%04X",
				   startaddress,
				   coilvalue);

	if (should_send_cmd)
	{
		/* 只有合法动作线圈写入 0xFF00 时才下发 CPU2，避免异常帧或 0x0000 误动作。 */
		CPU3_LOG_INFO("DSM",
				  "收到动作线圈并准备下发CPU2 命令=%lu",
				  (unsigned long)cmd);
		if (!CPU2_CommCanSendCommand((CommandType)cmd) ||
			!CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
									  HOLDREGISTER_DEVICEPARAM_COMMAND,
									  2,
									  &cmd))
		{
			sendframe[0] = SlaveAddress;
			sendframe[1] = 0x80 + presetsinglecoilfuncode;
			sendframe[2] = EXCEPTIONCODE_ERRORDEVIVEBUSY;
			framelen = 3;
		}
	}
	crc = CRC16_Calculate(sendframe, framelen);
	sendframe[framelen] = crc & 0xff;
	sendframe[framelen + 1] = (crc >> 8) & 0xff;
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
	bool parameter_snapshot_taken = false;

	functioncode = revframe[1];
	startaddress = ((revframe[2] & 0x00ff) << 8) + revframe[3];
	registeramount = ((revframe[4] & 0x00ff) << 8) + revframe[5];
	if ((registeramount <= 0) || (registeramount > 123) || (revframe[6] != (unsigned char)(registeramount * 2)))
	{
		return ResponseException(functioncode, EXCEPTIONCODE_ERRORDATA, sendframe);
	}
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

	case 0x300:
		if (!IsHoldingRegisterZeroSegment((unsigned int)startaddress, (unsigned int)registeramount))
			flagofaddress = true;

		break;

	default:
		flagofaddress = true;
		break;
	}

	/* 状态识别 */

	/* printf("flagofaddress=%d\r\n",flagofaddress); */
	/* registervalue = malloc(registeramount); */
	memset(TempBuffer, 0, sizeof(TempBuffer));
	for (i = 0, j = 0; i < registeramount; i++, j = j + 2)
	{
		TempBuffer[i] = ((revframe[j + 7] & 0x00ff) << 8) + revframe[j + 8];
	}

	if (flagofaddress)
	{
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + presetmultipleregisterfuncode;
		sendframe[2] = EXCEPTIONCODE_ERRORADDRESS; /* 超出地址范围 */
		framelen = 3;
	}
	else
	{
		if (!IsHoldingRegisterZeroSegment((unsigned int)startaddress, (unsigned int)registeramount) &&
			!CPU2_CommIsAvailable())
		{
			/* 已知链路不可用时先返回设备忙，不提交 DSM 保持寄存器影子。 */
			ret = PARAMETER_WRITE_FAIL;
		}
		else if (IsHoldingRegisterZeroSegment((unsigned int)startaddress, (unsigned int)registeramount))
		{
			/* 第6段当前只做协议占位：写请求回成功，但不落参数、不下发CPU2。 */
			ret = 0;
		}
		else if (!ReadHoldingRegister(startaddress, registeramount, s_dsm_holding_register_snapshot))
		{
			ret = PARAMETER_ERROR;
		}
		else
		{
			/* 写前保存最后一次确认值，避免 CPU2 ACK 失败后暴露请求影子。 */
			memcpy(&s_dsm_parameter_snapshot,
				   (const void *)&g_deviceParams,
				   sizeof(s_dsm_parameter_snapshot));
			parameter_snapshot_taken = true;

			if (!WriteHoldingRegister(startaddress, registeramount, TempBuffer))
			{
				ret = PARAMETER_ERROR;
			}
			else
			{
				/* 解析保持寄存器到设备参数，并下发到CPU2。 */
				ret = UpdateDeviceParamsFromLegacyRegs(startaddress, registeramount);
			}
		}

		if ((ret != 0) && parameter_snapshot_taken)
		{
			/* 同步结果不确定时先回滚本地影子，CPU2 实值由强制补读重新确认。 */
			memcpy((void *)&g_deviceParams,
				   &s_dsm_parameter_snapshot,
				   sizeof(g_deviceParams));
			(void)WriteHoldingRegister(startaddress,
								   registeramount,
								   s_dsm_holding_register_snapshot);
			SystemParameterSet();
		}

		/* 先处理异常边界，避免Modbus 协议状态机带故障继续运行。 */
		if (ret == PARAMETER_ERROR) /* 设置的数据错误 */
		{
			sendframe[0] = SlaveAddress;
			sendframe[1] = 0x80 + presetmultipleregisterfuncode;
			sendframe[2] = EXCEPTIONCODE_ERRORDATA; /* 设置数据超出范围 */
			framelen = 3;
		}
		else if (ret == PARAMETER_WRITE_FAIL) /* V1.106待机和故障状态允许设置 */
		{
			sendframe[0] = SlaveAddress;
			sendframe[1] = 0x80 + presetmultipleregisterfuncode;
			sendframe[2] = EXCEPTIONCODE_ERRORDEVIVEBUSY; /* 设备忙 */
			framelen = 3;
		}
		else /* 正常情况 */
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
