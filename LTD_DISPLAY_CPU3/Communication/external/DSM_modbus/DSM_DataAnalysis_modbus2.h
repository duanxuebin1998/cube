#ifndef _DSM_DATAANALYSIS_MODBUS2_H
#define _DSM_DATAANALYSIS_MODBUS2_H
#include "DSM_SlaveModbus_modbus2.h"
#include "main.h"
#include "system_parameter.h"

//extern char SoftOfversion6;
//#define SOFTOFVERSION_0 1213 // 程序版本
//#define SOFTOFVERSION_1 'P'	 // 程序版本
//#define SOFTOFVERSION_2 'B'	 // 程序版本
//#define SOFTOFVERSION_3 'C'	 // 程序版本
//#define SOFTOFVERSION_4 'D'	 // 程序版本
//#define SOFTOFVERSION_5 'E'	 // 程序版本
// 第六位是数据定义
#define PARAMETER_ERROR -1
#define PARAMETER_WRITE_FAIL -2

#define RETURN_OK 			0
#define RETURN_SLAVEFAIL 		-1
#define RETURN_UNSUPPORTED 	-2
#define RETURN_UNDEFADDRESS -3
#define RETURN_UNDEFDATEORDER -4
#define RETURN_SLAVEBUSY -5

void SystemParameterSet(void);
int UpdateDeviceParamsFromLegacyRegs(int startadd, int reamount);
void Input_Write(void);
#endif
