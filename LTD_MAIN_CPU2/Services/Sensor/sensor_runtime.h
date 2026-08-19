/*
 * 模块职责：声明本次上电识别结果、当前驱动和DM4协议状态的运行态访问接口。
 * 数据边界：运行态识别结果与FRAM持久化物理类型分开，调用者不能相互替代。
 * DM4约束：Safe会话只改变协议分派，不产生第二种物理传感器类型。
 */
#ifndef SENSOR_RUNTIME_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_RUNTIME_H_

#include <stdint.h>

#include "sensor_driver.h"
#include "sensor_types.h"

/*
 * 函数用途：开始新一轮识别并撤销本次启动的测量授权。
 * 调用场景：上电或维护重新探测之前。
 * 关键约束：不覆盖FRAM中的历史物理身份。
 */
void SensorRuntime_BeginDetection(void);

/*
 * 函数用途：提交识别成功的物理类型和编号。
 * 调用场景：候选协议完成身份校验之后。
 * 关键约束：编号0合法；仅身份变化时保存设备参数。
 */
void SensorRuntime_CommitIdentity(uint32_t sensor_type, uint32_t sensor_id);

/*
 * 函数用途：记录本次识别失败或命令切换结果。
 * 调用场景：识别流程未成功提交身份时。
 * 关键约束：不覆盖上次持久化身份。
 */
void SensorRuntime_SetDetectionResult(uint32_t result);

/*
 * 函数用途：判断本次启动是否成功识别传感器。
 * 调用场景：测量和维护入口门禁。
 * 关键约束：只读运行态，不使用历史FRAM身份代替本次结果。
 */
uint8_t SensorRuntime_IsDetectionValid(void);

/*
 * 函数用途：读取本次传感器识别结果码。
 * 调用场景：上层错误传播和诊断显示。
 * 关键约束：读取不清零、不转换错误码。
 */
uint32_t SensorRuntime_GetDetectionResult(void);

/*
 * 函数用途：解析当前应使用的传感器驱动操作表。
 * 调用场景：SensorService分派业务操作之前。
 * 关键约束：DM4物理类型固定为14，Safe只由运行会话选择。
 */
const SensorDriverOps *SensorRuntime_GetDriver(void);


/*
 * 函数用途：查询DM4当前V4或Safe协议模式。
 * 调用场景：串口状态和未来Safe调试入口。
 * 关键约束：非DM4返回NOT_APPLICABLE。
 */
SensorDm4ProtocolMode SensorRuntime_GetDm4ProtocolMode(void);

#endif /* SENSOR_RUNTIME_H_ */
