#ifndef SENSOR_SAFE_CRC32C_H_
/* SENSOR_SAFE_CRC32C_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_CRC32C_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按协议固定参数计算 CRC32C Castagnoli。
 *
 * @details 调用场景：控制帧、快报帧和规范化参数字节流校验。
 * @note 关键约束：输入为空时返回 0；非零长度输入不得传入空指针。
 */
uint32_t SensorSafeCrc32c_Calculate(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_CRC32C_H_ */
