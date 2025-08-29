#ifndef _DSM_COMMUNICATION_H
#define _DSM_COMMUNICATION_H
#include "main.h"

int  DSM_CommunicationInit(void);
void DSM_CommunicationProcess(unsigned char* rcvbuff, int rcvcount);
#endif
