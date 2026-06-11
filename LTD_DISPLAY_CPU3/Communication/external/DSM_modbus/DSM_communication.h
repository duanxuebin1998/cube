#ifndef _DSM_COMMUNICATION_H
#define _DSM_COMMUNICATION_H
#include "main.h"



/**
 * @brief 执行Modbus 协议中的 DSM_CommunicationInit 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int  DSM_CommunicationInit(void);
/**
 * @brief 处理Modbus 协议中的 DSM_CommunicationProcess 逻辑。
 *
 * @param rcvbuff 数据缓冲区。
 * @param rcvcount 业务参数。
 * @param tx 业务参数。
 * @param tx_len 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int DSM_CommunicationProcess(unsigned char *rcvbuff, int rcvcount, uint8_t* tx, uint16_t* tx_len);
#endif
