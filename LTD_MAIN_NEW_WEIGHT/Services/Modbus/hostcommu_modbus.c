#include "hostcommu_modbus.h"
#include "crc.h"
#include "usart.h"
#include "dataanalysis_modbus.h"
#include "stateformodbus.h"
#include "hostcommu.h"
#include <string.h>
#include <ctype.h>

/*从机地址*/
int SlaveAddress = 1;
/*功能码*/
static const int readholdingregisterfuncode = 0x03; //读保持寄存器功能码
static const int readinputregisterfuncode = 0x04; //读输入寄存器功能码
static const int presetmultipleregisterfuncode = 0x10; //写多个寄存器功能码
/*异常码*/
static const int illegalfunction = 0x01; //非法功能
static const int illegaldataaddress = 0x02; //非法数据地址
//static const int illegaldatavalue = 0x03; //非法数据值
//static const int slavedevicefailure = 0x04; //从设备故障
//static const int slavedevicebusy = 0x05; //从设备忙
/*保持寄存器*/
static const int HoldingregisterAddress = 0x00; //保持寄存器起始地址
static const int HoldingregisterAmount = HOLEREGISTER_STOP + 1; //保持寄存器总数
static int HoldingRegisterArray[HOLEREGISTER_STOP] = { 0 }; //保持寄存器数组
/*输入寄存器*/
static const int InputregisterAddress = 0x00; //输入寄存器起始地址
static const int InputRegisterAmount = INPUTREGISTER_AMOUNT; //输入寄存器总数
static uint16_t InputRegisterArray[INPUTREGISTER_AMOUNT] = { 0 };    //输入寄存器数组
/*发送区暂存数组*/
static int SlaveTempBuffer[HOSTCOMMU_SENDLENGTH];
/*接收到的命令包数据暂存变量*/
static int RCV_functioncode = 0;
static int RCV_startaddress = 0;
static int RCV_registercnt = 0;
/*静态函数*/
static bool JudgeFunctioncode(void);
static bool JudgeStartAddress(void);
static void ReadRegister(bool registertype, int *registervalue); //读寄存器
static void PresetRegister(bool registertype, int const *registervalue); //写寄存器
static int ResponseException(int exception, uint8_t  *sendframe);
static int Compose03Package(uint8_t  *revframe, uint8_t  *sendframe);
static int Compose04Package(uint8_t  *revframe, uint8_t  *sendframe);
static int Compose10Package(uint8_t  const *revframe, uint8_t  *sendframe);

/*接收到的数据包进行地址检查*/
bool SlaveCheckAddress(uint8_t  const *revframe, int framelen) {
	if (revframe[0] != SlaveAddress && revframe[0] != 0) {
		return false;
	} else {
		return true;
	}
}
/*设置从机地址对照量 作为数据包地址是否正确的判断依据*/
void SetSlaveaddress(int address) {
	SlaveAddress = address;
#if DEBUG_HOSTCOMMU_MODBUS
    printf("SlaveAddress = %d\n",SlaveAddress);
    #endif
}

/*判断功能码是否正确*/
static bool JudgeFunctioncode(void) {
	if ((RCV_functioncode != readholdingregisterfuncode) && (RCV_functioncode != readinputregisterfuncode)
			&& (RCV_functioncode != presetmultipleregisterfuncode)) {
		return false;
	} else {
		return true;
	}
}
/*根据不同功能码判断相应的地址是否正确*/
static bool JudgeStartAddress(void) {
	bool ret = true;
	switch (RCV_functioncode) {
	case FUNCTIONCODE_READ_HOLDREGISTER:
	case FUNCTIONCODE_WRITE_MULREGISTER: {
		if ((RCV_startaddress < HoldingregisterAddress) || (RCV_registercnt > HoldingregisterAmount)
				|| ((RCV_startaddress + RCV_registercnt - 1) >= HoldingregisterAmount + HoldingregisterAddress)) {
			ret = false;
		}
		break;
	}
	case FUNCTIONCODE_READ_INPUTREGISTER: {
		if ((RCV_startaddress < InputregisterAddress) || (RCV_registercnt > InputRegisterAmount)
				|| ((RCV_startaddress + RCV_registercnt - 1) >= (InputRegisterAmount + InputregisterAddress))) {
			ret = false;
		}
		break;
	}

	}
	return ret;
}
/*
 读寄存器
 registertype --> true - 输入寄存器
 --> false - 保持寄存器
 */
static void ReadRegister(bool registertype, int *registervalue) {
	int range;
	int i;
	int j;

	range = RCV_startaddress + RCV_registercnt;
	if (registertype) {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			registervalue[j] = InputRegisterArray[i];
		}
	} else {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			registervalue[j] = HoldingRegisterArray[i];
		}
	}
}

/*检查功能码，若错误则组织违法功能码响应包*/
bool FunctionCheckIllPack(uint8_t  *sendframe, int *framelength) {
	if (JudgeFunctioncode() == false) {
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + RCV_functioncode;
		sendframe[2] = illegalfunction;
		*framelength = 3;
		return false;
	} else {
		return true;
	}
}
/*检查数据起始地址和寄存器数量,若错误则组织违法数据响应包*/
bool IllegalDataAddressPack(uint8_t  *sendframe, int *framelength) {
	if (JudgeStartAddress() == false) {
		sendframe[0] = SlaveAddress;
		sendframe[1] = 0x80 + RCV_functioncode;
		sendframe[2] = illegaldataaddress;
		*framelength = 3;
		return false;
	} else {
		return true;
	}
}

/*更新功能码\起始地址\寄存器数量*/
void UpdateRcvPara(int funccode, int startadd, int registercnt) {
	RCV_functioncode = funccode;
	RCV_startaddress = startadd;
	RCV_registercnt = registercnt;
#if DEBUG_HOSTCOMMU
    printf("功能码\t0x%02x\r\n",RCV_functioncode);
    printf("起始地址\t0x%02x\r\n",RCV_startaddress);
    printf("寄存器数量\t%d\r\n",RCV_registercnt);
    #endif
}

/*处理03功能码命令包 并组织响应包*/
int Response03Process(uint8_t  *revframe, uint8_t  *sendframe) {
	int length;
	/*重置保持寄存器*/
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	/*组包*/
	length = Compose03Package(revframe, sendframe);
	return length;
}

/*在命令包格式正确的情况下,组织03响应包*/
static int Compose03Package(uint8_t  *revframe, uint8_t  *sendframe) {
	int i;
	int j;
	int sendlength;
	sendframe[0] = SlaveAddress;
	sendframe[1] = RCV_functioncode;
	sendframe[2] = RCV_registercnt * 2;
	/*读保持寄存器*/
	ReadRegister(false, SlaveTempBuffer);
	for (i = 0, j = 3; i < RCV_registercnt; i++, j = j + 2) {
		sendframe[j] = (SlaveTempBuffer[i] >> 8) & 0xff;
		sendframe[j + 1] = SlaveTempBuffer[i] & 0xff;
	}
	sendlength = 3 + RCV_registercnt * 2;
	return sendlength;
}
/*处理04功能码命令包 并组织响应包*/
int Response04Process(uint8_t  *revframe, uint8_t  *sendframe) {
	int length;
	/*重置输入寄存器*/
	write_measurement_result_to_IputerRegisters(InputRegisterArray);
	/*组包*/
	length = Compose04Package(revframe, sendframe);
	return length;
}
/*在命令包格式正确的情况下,组织04响应包*/
static int Compose04Package(uint8_t  *revframe, uint8_t  *sendframe) {
	int i;
	int j;
	int sendlength;
	sendframe[0] = SlaveAddress;
	sendframe[1] = RCV_functioncode;
	sendframe[2] = RCV_registercnt * 2;
	/*读输入寄存器*/
	ReadRegister(true, SlaveTempBuffer);
	for (i = 0, j = 3; i < RCV_registercnt; i++, j = j + 2) {
		sendframe[j] = (SlaveTempBuffer[i] >> 8) & 0xff;
		sendframe[j + 1] = SlaveTempBuffer[i] & 0xff;
	}
	sendlength = 3 + RCV_registercnt * 2;
	return sendlength;
}

static int ResponseException(int exception, uint8_t *sendframe) {
	int framelen;
	sendframe[0] = SlaveAddress;
	sendframe[1] = RCV_functioncode + 0x80;
	sendframe[2] = exception;
	framelen = 3;
	return framelen;
}

int Response10Process(uint8_t  const *revframe, uint8_t  *sendframe) {
	int length;
//	int ret;
	/*重置保持寄存器*/
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	/*更新保持寄存器并组包*/
	length = Compose10Package(revframe, sendframe);
	printf("command: %u\n", g_deviceParams.command);
	/*更新对应参数,若需存储则写入铁电*/
	ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
//	ret = ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);

//	if (ret != RETURN_OK) {
//		memset(sendframe, 0, HOSTCOMMU_SENDLENGTH);
//		if (ret == RETURN_EXEFAIL) {
//			length = ResponseException(slavedevicefailure, sendframe);
//		} else if (ret == RETURN_UNSUPPORTED) {
//			length = ResponseException(illegaldatavalue, sendframe);
//		}
//	}
	/* 存储参数 	*/
	save_device_params(); // 保存设备参数到铁电
	return length;
}

/*更新保持寄存器,组织10响应包*/
static int Compose10Package(uint8_t  const *revframe, uint8_t  *sendframe) {
	int i, j;
	int length;
	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));
	for (i = 0, j = 0; i < RCV_registercnt; i++, j = j + 2) {
		SlaveTempBuffer[i] = (revframe[j + 7] << 8) + revframe[j + 8];
	}
	/*写保持寄存器*/
	PresetRegister(false, SlaveTempBuffer);
	sendframe[0] = SlaveAddress;
	sendframe[1] = RCV_functioncode;
	sendframe[2] = revframe[2];
	sendframe[3] = revframe[3];
	sendframe[4] = revframe[4];
	sendframe[5] = revframe[5];
	length = 6;
	return length;
}

/*
 写寄存器
 registertype --> false - 保持寄存器
 --> true - 输入寄存器
 */
static void PresetRegister(bool registertype, int const *registervalue) {
	int range;
	int i;
	int j;

	range = RCV_startaddress + RCV_registercnt;
	if (registertype) {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			InputRegisterArray[i] = registervalue[j];
		}
	} else {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			HoldingRegisterArray[i] = registervalue[j];
			printf("HoldingRegisterArray[%d] = %d\n", i, HoldingRegisterArray[i]);
		}
	}
}

