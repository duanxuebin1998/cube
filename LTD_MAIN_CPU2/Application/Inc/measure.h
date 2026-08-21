/**
 * @file measure.h
 * @brief 测量子系统初始化、传感器门禁和命令分发入口。
 *
 * measure.h
 *
 *  Created on: Mar 20, 2025
 *      Author: Duan Xuebin
 */

#ifndef INC_MEASURE_H_
/* INC_MEASURE_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_MEASURE_H_

#include <stdint.h>
#include "system_parameter.h"
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
 * @brief 判断正式命令是否依赖本次运行期确认的传感器身份。
 * @param command 待执行的正式命令。
 * @return 1 表示必须先识别传感器，0 表示该命令可在传感器离线时执行。
 */
uint8_t Measure_CommandRequiresDetectedSensor(CommandType command);

#endif /* INC_MEASURE_H_ */
