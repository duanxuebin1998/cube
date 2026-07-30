#ifndef SENSOR_SAFE_IDENTITY_PLATFORM_H_
/* SENSOR_SAFE_IDENTITY_PLATFORM_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_IDENTITY_PLATFORM_H_

#include "sensor_safe_identity.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 提供当前 STM32F429、SPI4 FRAM 和硬件 RNG 的身份模块平台回调。
 *
 * @details 调用场景：CPU2 安全传感器服务初始化身份上下文之前。
 * @note 关键约束：FRAM 写接口本身无状态返回，最终成功与否由身份模块回读 CRC 和逐字节比较决定。
 */
void SensorSafeIdentityPlatform_GetOps(SensorSafeIdentityOps *ops);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_IDENTITY_PLATFORM_H_ */
