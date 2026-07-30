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
/* _DSM_SLAVEMODBUS_MODBUS2_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define _DSM_SLAVEMODBUS_MODBUS2_H
/* 数组的定义，功能码的响应 */

#include "stdbool.h"
#include "main.h"

/* DSM 保持寄存器镜像数组容量 200；用于分配内部连续存储，不等同于分段协议地址的最大值。 */
#define HOLDREGISTERAMOUNT 200    /* */
/* DSM 输入寄存器镜像数组容量 1700；分段地址映射到该连续数组前必须完成边界换算。 */
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
 * @brief 从 DSM Modbus 请求帧提取功能码，并校验其是否属于当前支持列表。
 *
 * @param revframe 至少包含地址和功能码字段的 DSM Modbus 请求帧；函数读取 revframe[1]。
 * @param funcode 功能码输出指针；无论功能码是否受支持，均写入请求帧中的原始功能码。
 * @return true 表示功能码属于 FC01、FC03、FC04、FC05 或 FC10；false 表示功能码不受支持。
 * @note 函数不检查 revframe 和 funcode 空指针；返回 false 时 funcode 仍保留请求帧中的原始功能码，供调用方构造非法功能码响应。
 */
bool GetFunctioncode(unsigned char *revframe, int *funcode);

/**
 * @brief 兼容上位机读线圈请求：按协议格式正常响应，线圈值统一返回 0。
 *
 * @param revframe DSM Modbus RTU 请求帧缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe DSM Modbus RTU 响应输出缓冲区；函数写入从站地址、功能码、寄存器数据或异常字段，CRC 由统一流程追加。
 * @return 返回已构造的 DSM FC01 正常或异常响应帧总长度，单位字节。
 */
int Response01(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 下位机响应0x03请求。
 *
 * 函 数 名： Response03。
 * 参 数： char *revframe 请求帧。
 * char* sendframe 响应帧。
 * 返 回 值：组帧的长度。
 *
 * @param revframe DSM Modbus RTU 请求帧缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe DSM Modbus RTU 响应输出缓冲区；函数写入从站地址、功能码、寄存器数据或异常字段，CRC 由统一流程追加。
 * @return 返回已构造的 FC03 正常或异常响应帧总长度，单位字节。
 */
int Response03(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 下位机响应0x04请求。
 *
 * 函 数 名： Response04。
 * 参 数： char *revframe 请求帧。
 * char* sendframe 响应帧。
 * 返 回 值：组帧的长度。
 *
 * @param revframe DSM Modbus RTU 请求帧缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe DSM Modbus RTU 响应输出缓冲区；函数写入从站地址、功能码、寄存器数据或异常字段，CRC 由统一流程追加。
 * @return 返回已构造的 FC04 正常或异常响应帧总长度，单位字节。
 */
int Response04(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 下位机响应0x05请求。
 *
 * 函 数 名： Response05。
 * 参 数： char *revframe 请求帧。
 * char* sendframe 响应帧。
 * 返 回 值：组帧的长度。
 *
 * @param revframe DSM Modbus RTU 请求帧缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe DSM Modbus RTU 响应输出缓冲区；函数写入从站地址、功能码、寄存器数据或异常字段，CRC 由统一流程追加。
 * @return 返回已构造的 FC05 回显或异常响应帧总长度，单位字节。
 */
int Response05(unsigned char *revframe, unsigned char* sendframe);
/**
 * @brief 下位机响应0x10请求。
 *
 * 函 数 名： Response16。
 * 参 数： char *revframe 请求帧。
 * char* sendframe 响应帧。
 * 返 回 值：组帧的长度。
 *
 * @param revframe DSM Modbus RTU 请求帧缓冲区；函数按固定字段位置解析从站地址、功能码、寄存器地址和数据。
 * @param sendframe DSM Modbus RTU 响应输出缓冲区；函数写入从站地址、功能码、寄存器数据或异常字段，CRC 由统一流程追加。
 * @return 返回已构造的 FC10 回显或异常响应帧总长度，单位字节。
 */
int Response16(unsigned char *revframe, unsigned char* sendframe);

/**
 * @brief 读输入寄存器。
 *
 * 函 数 名： ReadInputRegister。
 * 参 数： bool registertype 寄存器类型。
 * true： 输入寄存器。
 * false：保持寄存器。
 * unsigned int startaddress 起始地址。
 * unsigned int registeramount 数量。
 * int *registervalue 寄存器值。
 * 返 回 值：true 成功。
 *
 * @param startaddress 本次 Modbus 访问的起始寄存器地址。
 * @param registeramount 本次连续访问的寄存器数量。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 * @return 当前实现固定返回 true，表示已按 DSM 分段地址换算并复制请求数量的输入寄存器；该函数自身不返回 false，地址合法性必须由上层先行校验。
 */
bool ReadInputRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);
/**
 * @brief 读保持寄存器。
 *
 * 函 数 名： ReadHoldingRegister。
 * 参 数： unsigned int startaddress 起始地址。
 * unsigned int registeramount 数量。
 * int *registervalue 寄存器值。
 * 返 回 值：true 成功。
 *
 * @param startaddress 本次 Modbus 访问的起始寄存器地址。
 * @param registeramount 本次连续访问的寄存器数量。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 * @return true 表示零填充兼容段已处理，或请求已映射并复制保持寄存器；false 表示起始地址不属于任一 DSM 保持寄存器分段。
 */
bool ReadHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);
/**
 * @brief 写保持寄存器。
 *
 * 函 数 名： WriteHoldingRegister。
 * 参 数： unsigned int startaddress 起始地址。
 * unsigned int registeramount 数量。
 * int *registervalue 寄存器值。
 * 返 回 值：true 成功。
 *
 * @param startaddress 本次 Modbus 访问的起始寄存器地址。
 * @param registeramount 本次连续访问的寄存器数量。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 * @return true 表示兼容零段无需落地，或连续值已写入 DSM 保持寄存器数组；false 表示起始地址无法映射到任何保持寄存器分段。
 */
bool WriteHoldingRegister(unsigned int startaddress, unsigned int registeramount, int *registervalue);

/**
 * @brief 写单或双输入寄存器。
 *
 * 函 数 名： WriteOneInputRegister。
 * 参 数： unsigned int startaddress 起始地址。
 * unsigned int registeramount 数量。
 * 返 回 值：true 成功。
 *
 * @param startaddress 本次 Modbus 访问的起始寄存器地址。
 * @param registeramount 本次连续访问的寄存器数量。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 * @return 当前实现固定返回 true，表示单值已按请求宽度拆分并写入 DSM 输入寄存器数组；该函数自身不拒绝地址，调用前必须完成区间校验。
 */
bool WriteOneInputRegister(unsigned int startaddress, unsigned int registeramount, int registervalue);
/**
 * @brief 写单或双保持寄存器。
 *
 * 函 数 名： WriteOneHoldingRegister。
 * 参 数： unsigned int startaddress 起始地址。
 * unsigned int registeramount 数量。
 * 返 回 值。
 *
 * @param startaddress 本次 Modbus 访问的起始寄存器地址。
 * @param registeramount 本次连续访问的寄存器数量。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 * @return true 表示兼容零段无需落地，或单值已按 1～2 个寄存器写入保持区；false 表示起始地址不属于任一 DSM 保持寄存器分段。
 */
bool WriteOneHoldingRegister(unsigned int startaddress, unsigned int registeramount, int registervalue);
/**
 * @brief 读单或双保持寄存器。
 *
 * 函 数 名： ReadOneHoldingRegister。
 * 参 数： unsigned int startaddress 起始地址。
 * unsigned int registeramount 数量。
 * 返 回 值。
 *
 * @param startaddress 本次 Modbus 访问的起始寄存器地址。
 * @param registeramount 本次连续访问的寄存器数量。
 * @return 返回起始地址处一个或两个保持寄存器按高字在前组合的整数；零段或非法地址返回 0。
 */
int ReadOneHoldingRegister(unsigned int startaddress, unsigned int registeramount);

/**
 * @brief 检查Modbus 协议中的 DSM_RequestSelfCheckPlaceholder 逻辑。
 */
void DSM_RequestSelfCheckPlaceholder(void);
/**
 * @brief 消费 DSM 自检占位状态。
 *
 * 第一次返回自检中，第二次返回自检完成，再恢复为真实内部状态翻译。
 *
 * @return 返回本次消费的 DSM 自检占位状态码 0x0027 或 0x8027；当前状态不是占位值时返回 0。
 */
uint16_t DSM_ConsumeSelfCheckPlaceholderState(void);
/**
 * @brief 下位机响应异常请求。
 *
 * 函数名称: ResponseException。
 * 函数参数：unsigned int functioncode 功能码。
 * unsigned int exception 异常码。
 * char* sendframe 响应帧。
 * 原返回说明：组帧长度。
 * 1组帧失败。
 *
 * @param functioncode 本次 Modbus 或 HART 请求的功能码。
 * @param exception 异常码。
 * @param sendframe DSM Modbus RTU 响应输出缓冲区；函数写入从站地址、功能码、寄存器数据或异常字段，CRC 由统一流程追加。
 * @return 返回包含从站地址、异常功能码、异常码和 CRC16 的固定响应帧长度 5 字节。
 */
int ResponseException(unsigned int functioncode, unsigned int exception, unsigned char *sendframe);
#endif




