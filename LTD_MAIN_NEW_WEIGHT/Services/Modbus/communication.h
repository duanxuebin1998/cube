/*
 * communication.h
 *
 *  Created on: Apr 10, 2025
 *      Author: 1
 */

#ifndef MODBUS_COMMUNICATION_H_
#define MODBUS_COMMUNICATION_H_
#include "main.h"
#include "TMC5130.h"
#include "measure.h"
void process_command(uint8_t *command); //串口1电机控制函数

#endif /* MODBUS_COMMUNICATION_H_ */
