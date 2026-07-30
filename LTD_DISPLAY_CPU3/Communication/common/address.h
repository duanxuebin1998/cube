/*
 * address.h
 *
 *  Created on: 2025年11月19日
 *      Author: Duan Xuebin
 */

#ifndef DSM_MODBUS_ADDRESS_H_
/* DSM_MODBUS_ADDRESS_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define DSM_MODBUS_ADDRESS_H_

#include "main.h"

/* 读取指定拨码 GPIO 并归一化为 0/1；GPIO_PIN_SET 返回 1，GPIO_PIN_RESET 返回 0，参数会各求值一次。 */
#define READ_DIP_BIT(GPIOx, PIN)   ((HAL_GPIO_ReadPin(GPIOx, PIN) == GPIO_PIN_SET) ? 1 : 0)

extern int SlaveAddress;
/**
 * @brief 读取八路地址拨码，ADDRESS0 为最低位，组合为 0～255 的原始地址。
 * @return 返回按当前映射得到的八路地址拨码，ADDRESS0 为最低位，组合为 0～255 的原始地址；非法输入使用 @brief 说明的兜底地址或无效值。
 */
uint8_t Get_Device_Address(void);

#endif /* DSM_MODBUS_ADDRESS_H_ */
