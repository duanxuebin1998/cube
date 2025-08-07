#ifndef _CRC_H
#define _CRC_H
#include "main.h"





uint16_t CRC16(const uint8_t* puchMSG, uint32_t usDataLen);
bool SlaveCheckCRC(const uint8_t *revframe,int framelen); 






#endif
