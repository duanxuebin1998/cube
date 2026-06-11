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
/* 数组的定义，功能码的响应 */

#include "stdbool.h"
#include "main.h"

#define HOLDREGISTERAMOUNT 200    /* */
#define INPUTREGISTERAMOUNT 1700 /* */


extern int SlaveAddress;
extern uint16_t DSM_HoldingRegisterArray[HOLDREGISTERAMOUNT]; /* 保持寄存器数组 */
extern int DSM_InputRegisterArray[INPUTREGISTERAMOUNT];   /* 输入寄存器数组 */

extern int MaxNum_Coil;            			/* 线圈最大有效值 */
extern int MaxNum_HoldingRegister; 			/* 保持寄存器最大有效值 */
extern int MaxNum_InputRegister;   			/* 输入寄存器最大有效值 */


/**
 * @brief 写入或设置Modbus 协议中的 SetSlaveaddress 逻辑。
 *
 * @param address 地址参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool SetSlaveaddress(int address);
/**
 * @brief 读取Modbus 协议中的 GetFunctioncode 逻辑。
 *
 * @param revframe 业务参数。
 * @param funcode 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool GetFunctioncode(unsigned char *revframe, int *funcode);

/**
 * @brief 执行Modbus 协议中的 Response01 逻辑。
 *
 * @param revframe 业务参数。
 * @param sendframe 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int Response01(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 执行Modbus 协议中的 Response03 逻辑。
 *
 * @param revframe 业务参数。
 * @param sendframe 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int Response03(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 执行Modbus 协议中的 Response04 逻辑。
 *
 * @param revframe 业务参数。
 * @param sendframe 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int Response04(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 执行Modbus 协议中的 Response05 逻辑。
 *
 * @param revframe 业务参数。
 * @param sendframe 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int Response05(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 执行Modbus 协议中的 Response16 逻辑。
 *
 * @param revframe 业务参数。
 * @param sendframe 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int Response16(unsigned char *revframe, unsigned char* sendframe);

/**
 * @brief 读取Modbus 协议中的 ReadInputRegister 逻辑。
 *
 * @param startaddress 地址参数。
 * @param registeramount 业务参数。
 * @param registervalue 待处理数值。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool ReadInputRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);
/**
 * @brief 读取Modbus 协议中的 ReadHoldingRegister 逻辑。
 *
 * @param startaddress 地址参数。
 * @param registeramount 业务参数。
 * @param registervalue 待处理数值。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool ReadHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);
/**
 * @brief 写入或设置Modbus 协议中的 WriteHoldingRegister 逻辑。
 *
 * @param startaddress 地址参数。
 * @param registeramount 业务参数。
 * @param registervalue 待处理数值。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool WriteHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);

/**
 * @brief 写入或设置Modbus 协议中的 WriteOneInputRegister 逻辑。
 *
 * @param startaddress 地址参数。
 * @param registeramount 业务参数。
 * @param registervalue 待处理数值。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool WriteOneInputRegister(unsigned int startaddress, unsigned int registeramount, int registervalue);
/**
 * @brief 写入或设置Modbus 协议中的 WriteOneHoldingRegister 逻辑。
 *
 * @param startaddress 地址参数。
 * @param registeramount 业务参数。
 * @param registervalue 待处理数值。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool WriteOneHoldingRegister(unsigned int startaddress, unsigned int registeramount, int registervalue);
/**
 * @brief 读取Modbus 协议中的 ReadOneHoldingRegister 逻辑。
 *
 * @param startaddress 地址参数。
 * @param registeramount 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int ReadOneHoldingRegister(unsigned int startaddress, unsigned int registeramount);

/**
 * @brief 检查Modbus 协议中的 DSM_RequestSelfCheckPlaceholder 逻辑。
 */
void DSM_RequestSelfCheckPlaceholder(void);
/**
 * @brief 检查Modbus 协议中的 DSM_ConsumeSelfCheckPlaceholderState 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint16_t DSM_ConsumeSelfCheckPlaceholderState(void);
/**
 * @brief 执行Modbus 协议中的 ResponseException 逻辑。
 *
 * @param functioncode 业务参数。
 * @param exception 业务参数。
 * @param sendframe 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int ResponseException(unsigned int functioncode, unsigned int exception, unsigned char *sendframe);
#endif




