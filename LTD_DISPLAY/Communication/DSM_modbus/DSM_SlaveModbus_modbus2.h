/*
 * @FilePath     : \KEILe:\03CodeRepository\DSM_MCB\MEASURE\SlaveModbus_modbus2.h
 * @Description  : 
 * @Author       : Aubon
 * @Date         : 2023-10-10 17:16:52
 * @LastEditors  : Duan
 * @LastEditTime : 2024-01-31 08:31:52
 * Copyright 2024 Aubon, All Rights Reserved. 
 * 2023-10-10 17:16:52
 */
#ifndef _DSM_SLAVEMODBUS_MODBUS2_H
#define _DSM_SLAVEMODBUS_MODBUS2_H
/*数组的定义，功能码的响应*/

#include "stdbool.h"
#include "main.h"

#define HOLDREGISTERAMOUNT 200    //
#define INPUTREGISTERAMOUNT 1700 //


extern int SlaveAddress;
extern int HoldingRegisterArray1[HOLDREGISTERAMOUNT]; //保持寄存器数组
extern int InputRegisterArray[INPUTREGISTERAMOUNT];   //输入寄存器数组

extern int MaxNum_Coil;            			//线圈最大有效值
extern int MaxNum_HoldingRegister; 			//保持寄存器最大有效值
extern int MaxNum_InputRegister;   			//输入寄存器最大有效值


bool SetSlaveaddress(int address);
bool GetFunctioncode(unsigned char *revframe, int *funcode);

int Response03(unsigned char *revframe, unsigned char* sendframe);
int Response04(unsigned char *revframe, unsigned char* sendframe);
int Response05(unsigned char *revframe, unsigned char* sendframe);
int Response16(unsigned char *revframe, unsigned char* sendframe);

bool ReadInputRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);
bool ReadHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);
bool WriteHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);
bool WriteInputRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);

bool WriteOneInputRegister(unsigned int startaddress, unsigned int registeramount, int registervalue);
bool WriteOneHoldingRegister(unsigned int startaddress, unsigned int registeramount, int registervalue);
int ReadOneHoldingRegister(unsigned int startaddress, unsigned int registeramount);
int ReadOneInputRegister(unsigned int startaddress, u8 registeramount);

int ResponseException(unsigned int functioncode, unsigned int exception, unsigned char *sendframe);
#endif




