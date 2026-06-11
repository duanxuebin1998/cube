#ifndef _DSM_DATAANALYSIS_MODBUS2_H
#define _DSM_DATAANALYSIS_MODBUS2_H
#include "DSM_SlaveModbus_modbus2.h"
#include "main.h"
#include "system_parameter.h"

/* #define SOFTOFVERSION_0 1213 / / 程序版本 */
/* #define SOFTOFVERSION_1 'P' / / 程序版本 */
/* #define SOFTOFVERSION_2 'B' / / 程序版本 */
/* #define SOFTOFVERSION_3 'C' / / 程序版本 */
/* #define SOFTOFVERSION_4 'D' / / 程序版本 */
/* #define SOFTOFVERSION_5 'E' / / 程序版本 */
/* 第六位是数据定义 */
#define PARAMETER_ERROR -1
#define PARAMETER_WRITE_FAIL -2

#define RETURN_OK 			0
#define RETURN_SLAVEFAIL 		-1
#define RETURN_UNSUPPORTED 	-2
#define RETURN_UNDEFADDRESS -3
#define RETURN_UNDEFDATEORDER -4
#define RETURN_SLAVEBUSY -5

/**
 * @brief 执行Modbus 协议中的 SystemParameterSet 逻辑。
 */
void SystemParameterSet(void);
/**
 * @brief 更新Modbus 协议中的 UpdateDeviceParamsFromLegacyRegs 逻辑。
 *
 * @param startadd 业务参数。
 * @param reamount 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int UpdateDeviceParamsFromLegacyRegs(int startadd, int reamount);
/**
 * @brief 写入或设置Modbus 协议中的 Input_Write 逻辑。
 */
void Input_Write(void);
#endif
