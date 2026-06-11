/*
 * measure.h
 *
 *  Created on: Mar 20, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_H_
#define INC_MEASURE_H_

#include "app_main.h"
/**
 * @brief 执行测量流程中的 MeasureStart 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int MeasureStart(void);

/**
 * @brief 处理测量流程中的 ProcessMeasureCmd 逻辑。
 *
 * @param command 命令值。
 */
void ProcessMeasureCmd(CommandType command);
/**
 * @brief 处理测量流程中的 process_command 逻辑。
 *
 * @param command 命令值。
 */
void process_command(uint8_t *command); /* 处理接收到的命令 */

#endif /* INC_MEASURE_H_ */
