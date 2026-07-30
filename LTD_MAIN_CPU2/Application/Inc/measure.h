/*
 * measure.h
 *
 *  Created on: Mar 20, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_H_
/* INC_MEASURE_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_MEASURE_H_

#include "app_main.h"
/**
 * @brief 初始化故障、驱动和扭力模块，并清除新测量流程的协议辅助运行态。
 * @return NO_ERROR 表示初始化故障、驱动和扭力模块，并清除新测量流程的协议辅助运行态已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
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
