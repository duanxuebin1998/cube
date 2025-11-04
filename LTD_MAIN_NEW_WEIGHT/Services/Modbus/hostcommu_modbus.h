#ifndef __HOSTCOMMU_MODBUS_H
#define __HOSTCOMMU_MODBUS_H

#include <stdbool.h>  // 提供布尔类型支持
#include <stdint.h>   // 提供固定宽度整数类型支持

// 调试模式开关：设置为1启用调试输出，0禁用
#define DEBUG_HOSTCOMMU_MODBUS 0

// 全局变量：当前Modbus从站地址
extern int SlaveAddress;

/**
 * @brief 检查接收帧中的地址是否匹配本机地址
 *
 * @param revframe 接收到的Modbus帧数据
 * @param framelen 帧长度
 * @return true 地址匹配（本机地址）
 * @return false 地址不匹配
 */
bool SlaveCheckAddress(uint8_t  const *revframe, int framelen);

/**
 * @brief 设置Modbus从站地址
 *
 * @param address 新的从站地址
 */
void SetSlaveaddress(int address);

/**
 * @brief 处理Modbus功能码03（读保持寄存器）请求
 *
 * @param revframe 接收到的Modbus帧数据
 * @param sendframe 待发送的响应帧缓冲区
 * @return int 响应帧长度
 */
int Response03Process(uint8_t  *revframe, uint8_t  *sendframe);

/**
 * @brief 处理Modbus功能码04（读输入寄存器）请求
 *
 * @param revframe 接收到的Modbus帧数据
 * @param sendframe 待发送的响应帧缓冲区
 * @return int 响应帧长度
 */
int Response04Process(uint8_t  *revframe, uint8_t  *sendframe);

/**
 * @brief 处理Modbus功能码05（写单个线圈）请求
 *
 * @param revframe 接收到的Modbus帧数据
 * @param sendframe 待发送的响应帧缓冲区
 * @return int 响应帧长度
 */
int Response05Process(uint8_t  const *revframe, uint8_t  *sendframe);

/**
 * @brief 处理Modbus功能码16（写多个寄存器）请求
 *
 * @param revframe 接收到的Modbus帧数据
 * @param sendframe 待发送的响应帧缓冲区
 * @return int 响应帧长度
 */
int Response10Process(uint8_t  const *revframe, uint8_t  *sendframe);

/**
 * @brief 生成功能码错误响应（非法功能码）
 *
 * @param sendframe 待发送的错误响应帧缓冲区
 * @param framelength 返回的错误帧长度
 * @return true 成功生成错误响应
 */
bool FunctionCheckIllPack(uint8_t  *sendframe, int *framelength);

/**
 * @brief 生成数据地址错误响应（非法数据地址）
 *
 * @param sendframe 待发送的错误响应帧缓冲区
 * @param framelength 返回的错误帧长度
 * @return true 成功生成错误响应
 */
bool IllegalDataAddressPack(uint8_t  *sendframe, int  *framelength);

/**
 * @brief 更新接收参数状态（用于调试或状态跟踪）
 *
 * @param funccode 接收到的功能码
 * @param startadd 起始地址
 * @param registercnt 寄存器数量
 */
void UpdateRcvPara(int funccode, int startadd, int registercnt);

#endif
