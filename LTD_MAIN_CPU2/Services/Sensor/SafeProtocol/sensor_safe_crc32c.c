#include "sensor_safe_crc32c.h"

#define SENSOR_SAFE_CRC32C_INIT            0xFFFFFFFFUL
#define SENSOR_SAFE_CRC32C_XOR_OUT         0xFFFFFFFFUL
#define SENSOR_SAFE_CRC32C_REFLECTED_POLY  0x82F63B78UL

/*
 * 函数用途：按 Castagnoli 反射多项式计算安全协议帧的 CRC32C。
 * 调用场景：控制帧、快报帧和身份记录在编码或验收前调用。
 * 关键约束：空数据只允许长度为 0；本函数不分配内存且不访问硬件。
 */
uint32_t SensorSafeCrc32c_Calculate(const uint8_t *data, size_t length)
{
    uint32_t crc = SENSOR_SAFE_CRC32C_INIT;
    size_t index;

    /* 非零长度却没有数据地址属于调用契约错误，固定返回 0 使上层校验失败。 */
    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }

    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= (uint32_t)data[index];
        for (bit = 0U; bit < 8U; bit++) {
            uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (SENSOR_SAFE_CRC32C_REFLECTED_POLY & mask);
        }
    }

    return crc ^ SENSOR_SAFE_CRC32C_XOR_OUT;
}
