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
void write_measurement_result_to_IputerRegisters(uint16_t *regs); // 将测量结果写入输入寄存器数组













#endif



