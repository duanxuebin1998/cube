#ifndef _DATAANALYSIS_MODBUS_H
#define _DATAANALYSIS_MODBUS_H

#include <stdint.h>
#include <stdbool.h>

#define RETURN_OK 0//运行正常
#define RETURN_EXEFAIL -1//发生故障
#define RETURN_UNSUPPORTED -2//不支持的操作
#define RETURN_UNDEFADDRESS -3//非法数据地址



void WriteDeviceParamsToHoldingRegisters(int *p_holdregister);//写入保持寄存器
void ReadDeviceParamsFromHoldingRegisters(int *p_inputregister);//从输入寄存器读取设备参数
int ProcessWriteCoil(int const *p_coilarray);
void ResetCoil(int *p_coilarray,int coilamount);
int UpdateHoldRegisterParameter(int const *p_holdingregister,bool iswrite);













#endif



