/*
 * address.c
 *
 *  Created on: 2025年11月19日
 *      Author: Duan Xuebin
 */

#include "address.h"

int SlaveAddress = 0x01;							/* 下位机地址 */
/**
 * @brief 读取八路地址拨码，ADDRESS0 为最低位，组合为 0～255 的原始地址。
 * @return 返回按当前映射得到的八路地址拨码，ADDRESS0 为最低位，组合为 0～255 的原始地址；非法输入使用 @brief 说明的兜底地址或无效值。
 */
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

    return addr; /* 返回 0~255 */
}
/**
 * @brief 校验并设置 CPU3 对外 Modbus 从站地址。
 *
 * @param address 准备设置的 Modbus 从站地址；合法范围为 1 至 247。
 * @return true 表示地址位于 1 至 247 且已保存；false 表示地址非法并已把内部地址标记为无效。
 */
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
