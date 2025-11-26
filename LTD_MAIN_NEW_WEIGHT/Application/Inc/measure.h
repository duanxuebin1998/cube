/*
 * measure.h
 *
 *  Created on: Mar 20, 2025
 *      Author: 1
 */

#ifndef INC_MEASURE_H_
#define INC_MEASURE_H_

#include "app_main.h"

void ProcessMeasureCmd(CommandType command);
void process_command(uint8_t *command); // 处理接收到的命令
void MeasureZero(void);
void MeasureBottom(void);
void MeasureAndFollowOilLevel(void);
void CalibrateZeroPoint(void);
void CalibrateOilLevel(void);
#endif /* INC_MEASURE_H_ */
