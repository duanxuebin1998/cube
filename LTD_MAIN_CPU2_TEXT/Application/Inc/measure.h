/*
 * measure.h
 *
 *  Created on: Mar 20, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_H_
#define INC_MEASURE_H_

#include "app_main.h"
int MeasureStart(void);

void ProcessMeasureCmd(CommandType command);
void process_command(uint8_t *command); // 处理接收到的命令

#endif /* INC_MEASURE_H_ */
