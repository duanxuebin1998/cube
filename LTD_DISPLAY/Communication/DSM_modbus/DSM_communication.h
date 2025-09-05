#ifndef _DSM_COMMUNICATION_H
#define _DSM_COMMUNICATION_H
#include "main.h"

int  DSM_CommunicationInit(void);
void DSM_CommunicationProcess(unsigned char* rcvbuff, int rcvcount);
void SendPacketToHost(uint8_t*arr,uint16_t len);
#endif
