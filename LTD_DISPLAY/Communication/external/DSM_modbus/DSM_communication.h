#ifndef _DSM_COMMUNICATION_H
#define _DSM_COMMUNICATION_H
#include "main.h"


#define COM1_SET_RECV_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_SET)
#define COM1_SET_SEND_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_RESET)
#define COM2_SET_RECV_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_SET)
#define COM2_SET_SEND_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_RESET)
#define COM3_SET_RECV_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_SET)
#define COM3_SET_SEND_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_RESET)

int  DSM_CommunicationInit(void);
int DSM_CommunicationProcess(unsigned char *rcvbuff, int rcvcount, uint8_t* tx, uint16_t* tx_len);
void SendPacketToHost(uint8_t*arr,uint16_t len);
#endif
