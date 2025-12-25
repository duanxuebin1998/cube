#ifndef _DSM_COMMUNICATION_H
#define _DSM_COMMUNICATION_H
#include "main.h"



int  DSM_CommunicationInit(void);
int DSM_CommunicationProcess(unsigned char *rcvbuff, int rcvcount, uint8_t* tx, uint16_t* tx_len);
void SendPacketToHost(uint8_t*arr,uint16_t len);
#endif
