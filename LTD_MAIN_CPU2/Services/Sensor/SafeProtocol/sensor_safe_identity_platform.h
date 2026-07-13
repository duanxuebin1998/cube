#ifndef SENSOR_SAFE_IDENTITY_PLATFORM_H_
#define SENSOR_SAFE_IDENTITY_PLATFORM_H_

#include "sensor_safe_identity.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数用途：提供当前 STM32F429、SPI4 FRAM 和硬件 RNG 的身份模块平台回调。
 * 调用场景：CPU2 安全传感器服务初始化身份上下文之前。
 * 关键约束：FRAM 写接口本身无状态返回，最终成功与否由身份模块回读 CRC 和逐字节比较决定。
 */
void SensorSafeIdentityPlatform_GetOps(SensorSafeIdentityOps *ops);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_IDENTITY_PLATFORM_H_ */
