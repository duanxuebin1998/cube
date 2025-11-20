/*
 * address.c
 *
 *  Created on: 2025年11月19日
 *      Author: admin
 */

#include "address.h"

int SlaveAddress = 0x01;							// 下位机地址
uint8_t Get_Device_Address(void)
{
    uint8_t addr = 0;

    addr |= (READ_DIP_BIT(ADDRESS0_GPIO_Port, ADDRESS0_Pin) << 0);
    addr |= (READ_DIP_BIT(ADDRESS1_GPIO_Port, ADDRESS1_Pin)             << 1);
    addr |= (READ_DIP_BIT(ADDRESS2_GPIO_Port, ADDRESS2_Pin)             << 2);
    addr |= (READ_DIP_BIT(ADDRESS3_GPIO_Port, ADDRESS3_Pin)             << 3);
    addr |= (READ_DIP_BIT(ADDRESS4_GPIO_Port, ADDRESS4_Pin)             << 4);
    addr |= (READ_DIP_BIT(ADDRESS5_GPIO_Port, ADDRESS5_Pin)             << 5);
    addr |= (READ_DIP_BIT(ADDRESS6_GPIO_Port, ADDRESS6_Pin)             << 6);
    addr |= (READ_DIP_BIT(ADDRESS7_GPIO_Port, ADDRESS7_Pin)             << 7);

    return addr; // 返回 0~255
}
bool SetSlaveaddress(int address)
{
	if ((address < 1) || (address > 247))
	{
		SlaveAddress = -1;
		return false;
	}
	else
	{
		SlaveAddress = address;
		return true;
	}
}
