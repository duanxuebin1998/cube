/*
 * 模块职责：声明物理传感器类型和DM4会话状态到唯一驱动操作表的解析接口。
 * 类型边界：DSM、V3和DM4互斥；V4与Safe只是同一DM4的协议状态。
 * 生命周期：返回只读静态操作表，调用方不得修改或释放。
 */
#ifndef SENSOR_DRIVER_REGISTRY_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_DRIVER_REGISTRY_H_

#include <stdint.h>

#include "sensor_driver.h"

/*
 * 函数用途：根据物理传感器类型和DM4会话状态返回唯一驱动操作表。
 * 调用场景：SensorRuntime刷新当前运行驱动时调用。
 * 关键约束：DM4始终使用类型14；Safe只作为显式会话模式，不参与上电探测。
 */
const SensorDriverOps *SensorDriverRegistry_Resolve(uint32_t sensor_type,
                                                    uint8_t dm4_safe_session_active);

#endif /* SENSOR_DRIVER_REGISTRY_H_ */