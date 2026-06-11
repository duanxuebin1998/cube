/*
 * address.h
 *
 *  Created on: 2025年11月19日
 *      Author: Duan Xuebin
 */

#ifndef DSM_MODBUS_ADDRESS_H_
#define DSM_MODBUS_ADDRESS_H_

#include "main.h"

#define READ_DIP_BIT(GPIOx, PIN)   ((HAL_GPIO_ReadPin(GPIOx, PIN) == GPIO_PIN_SET) ? 1 : 0)

extern int SlaveAddress;
/**
 * @brief 读取屏幕显示中的 Get_Device_Address 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint8_t Get_Device_Address(void);

#endif /* DSM_MODBUS_ADDRESS_H_ */
