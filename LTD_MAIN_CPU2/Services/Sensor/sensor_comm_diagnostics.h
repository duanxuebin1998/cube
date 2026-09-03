/*
 * 模块职责：声明DM4 V4失败包打印和通信超时后的无线链路归因接口。
 * 调用场景：测量层取得异常结果，或SensorService事务通信超时之后。
 * 边界约束：本接口不发送传感器命令，不修改当前驱动、识别结果或错误码。
 */
#ifndef SENSOR_COMM_DIAGNOSTICS_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_COMM_DIAGNOSTICS_H_

#include <stdint.h>

/*
 * 函数用途：在传感器业务发现异常后打印本次可用的原始通信包证据。
 * 调用场景：测量层需要把协议返回值与DM4 V4原始收发包关联时调用。
 * 关键约束：只在线程态打印；非DM4 V4、正常结果和命令切换均不输出。
 */
void SensorComm_PrintFailurePackets(uint32_t result, const char *context);

/*
 * 函数用途：传感器通信超时后统一检查无线滑环链路。
 * 调用场景：SensorService和部件诊断收到通信超时时调用。
 * 关键约束：非超时错误原样返回；命令切换不记为故障。
 */
uint32_t SensorComm_DiagnoseTimeout(uint32_t result, const char *context);

#endif /* SENSOR_COMM_DIAGNOSTICS_H_ */