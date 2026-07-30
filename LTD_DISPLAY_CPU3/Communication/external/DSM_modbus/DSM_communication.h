#ifndef _DSM_COMMUNICATION_H
/* _DSM_COMMUNICATION_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define _DSM_COMMUNICATION_H
#include "main.h"

/* DSM 外部帧校验结果；调用方据此区分成功、长度错误、CRC 错误和站号不匹配。 */
typedef enum {
	/* DSM 外部通信帧的校验结果。 */
	DSM_COMM_OK = 0, /* 处理成功。 */
	DSM_COMM_ERR_BAD_LENGTH, /* 帧长度与协议要求不符。 */
	DSM_COMM_ERR_CRC, /* 帧 CRC 校验失败。 */
	DSM_COMM_ERR_ADDRESS_MISMATCH /* 帧地址不是本机或目标设备地址。 */
} DsmCommResult;

/**
 * @brief 初始化 DSM Modbus 地址、寄存器访问上限和系统状态寄存器。
 *
 * 函数读取当前设备地址并调用 SetSlaveaddress，随后设置线圈、保持寄存器和输入寄存器的最大有效数量，最后把系统状态输入寄存器写为 STATE_INIT。
 *
 * @return 固定返回 0，表示既有初始化入口已执行完毕；当前返回值不反映 SetSlaveaddress 或输入寄存器写入是否成功。
 * @note 当前实现不初始化 UART4、DMA 或定时器，也不检查地址设置及寄存器写入结果；固定返回 0 仅保留既有初始化接口约定。
 */
int  DSM_CommunicationInit(void);
/**
 * @brief 校验并处理一帧 DSM Modbus RTU 请求，生成正常或标准异常响应。
 *
 * 处理顺序为最小长度、从站地址、CRC 和功能码对应帧长校验，随后分发 FC01、FC03、FC04、FC05 或 FC10 响应。
 *
 * @param rcvbuff 接收到的完整 DSM Modbus RTU 请求帧。
 * @param rcvcount 接收缓冲区中的有效帧长度，单位字节。
 * @param tx DSM Modbus RTU 响应帧输出缓冲区；正常响应和标准异常响应均写入此处。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 DSM_COMM_OK 表示请求已处理；其他值分别表示帧长、从机地址或 CRC 校验失败。
 * @note 当前实现同时接受本机地址和广播地址 0；非法功能码及数据形状由响应缓冲区返回 Modbus 异常帧。
 */
int DSM_CommunicationProcess(unsigned char *rcvbuff, int rcvcount, uint8_t* tx, uint16_t* tx_len);
#endif
