#include "sensor_safe_crc32c.h"

/* 安全协议 CRC32C 初始余数 0xFFFFFFFF。 */
#define SENSOR_SAFE_CRC32C_INIT            0xFFFFFFFFUL
/* 安全协议 CRC32C 最终异或值 0xFFFFFFFF。 */
#define SENSOR_SAFE_CRC32C_XOR_OUT         0xFFFFFFFFUL
/* 安全协议 CRC32C 反射多项式 0x82F63B78；与逐字节低位优先算法配套，不能替换为普通 CRC32 多项式。 */
#define SENSOR_SAFE_CRC32C_REFLECTED_POLY  0x82F63B78UL

/**
 * @brief 按 Castagnoli 反射多项式计算安全协议帧的 CRC32C。
 *
 * @details 调用场景：控制帧、快报帧和身份记录在编码或验收前调用。
 * @note 关键约束：空数据只允许长度为 0；本函数不分配内存且不访问硬件。
 *
 * @param data 参与 CRC32C 计算的连续只读字节序列；有效范围为 data[0..length-1]，length 可为 0。
 * @param length 输入数据的有效长度，单位字节。
 * @return 返回输入数据按 Castagnoli 反射多项式计算并完成末异或的 CRC32C；空输入返回 0。
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
