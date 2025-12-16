#ifndef __HOSTCOMMU_H
#define __HOSTCOMMU_H
#include "main.h"

#define RS485_SET_RECV_MODE()  HAL_GPIO_WritePin(CPU2_485_SEL_GPIO_Port, CPU2_485_SEL_Pin, GPIO_PIN_SET)
#define RS485_SET_SEND_MODE()  HAL_GPIO_WritePin(CPU2_485_SEL_GPIO_Port, CPU2_485_SEL_Pin, GPIO_PIN_RESET)

#define HOSTCOMMU_SENDLENGTH 1000

int HostCommuInit(void);
void HostCommuProcess(uint8_t *rcvbuff, int rcvcount);

#endif	  

