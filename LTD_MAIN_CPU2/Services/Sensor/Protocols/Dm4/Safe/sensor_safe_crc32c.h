/*
 * 模块职责：公开Safe控制帧、快报帧和身份记录共用的CRC32C计算接口。
 * 输入约束：非零长度必须提供有效缓冲区；调用方负责选择协议规定的计算范围。
 * 协议边界：本接口只返回校验值，不负责帧长度、字节序或业务状态校验。
 */
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
