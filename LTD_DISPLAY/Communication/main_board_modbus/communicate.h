#ifndef __COMMU_H
#define __COMMU_H
#include "stateformodbus.h"



extern int count_com_CPU2;
extern volatile bool wait_response;




void CommuToCPU2Init(void);
void HostCommuProcess(uint8_t*rcv,int len);
void PollingInputData(void);
void CPU2_CombinatePackage_Send(uint8_t f_code,uint16_t startadd,uint16_t registercnt,uint32_t* holddata);
void sendToCPU2(uint8_t*arr,uint16_t len,bool flag_fromhost);








#endif

