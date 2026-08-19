/*
 * 模块职责：声明传感器通信超时后的无线链路归因接口。
 * 调用场景：SensorService或维护诊断取得协议层超时结果之后。
 * 边界约束：本接口不发送传感器命令，不修改当前驱动和识别结果。
 */
#ifndef SENSOR_COMM_DIAGNOSTICS_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_COMM_DIAGNOSTICS_H_

#include <stdint.h>

/*
 * 函数用途：传感器通信超时后统一检查无线滑环链路。
 * 调用场景：SensorService和部件诊断收到通信超时时调用。
 * 关键约束：非超时错误原样返回；命令切换不记为故障。
 */
uint32_t SensorComm_DiagnoseTimeout(uint32_t result, const char *context);

#endif /* SENSOR_COMM_DIAGNOSTICS_H_ */