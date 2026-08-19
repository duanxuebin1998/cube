/*
 * 模块职责：把Safe身份模块接入CPU2的SPI4 FRAM事务入口、硬件RNG和复位信息。
 * 硬件边界：本文件只提供平台回调，不解释身份双副本选择规则和会话语义。
 * 并发约束：FRAM访问必须经过统一仲裁；RNG等待必须有界，禁止在身份路径无限阻塞。
 */
#include "sensor_safe_identity_platform.h"

#include <string.h>

#include "mb85rs2m.h"
#include "stm32f4xx_hal.h"

/* 等待硬件随机数就绪的最大轮询次数 100000；达到上限必须返回失败，禁止在身份生成路径无限忙等。 */
#define SENSOR_SAFE_RNG_WAIT_LOOPS  100000UL
/**
 * @brief 通过统一FRAM事务入口读取身份记录字节。
 *
 * @details 调用场景：身份双副本初始化和写后回读校验使用。
 * @note 关键约束：本函数为任务上下文调用，不得绕过FRAM仲裁直接控制SPI4和片选。
 *
 * @param address 安全身份平台存储区的 FRAM 绝对字节地址；读写长度由 length 指定并接受底层范围校验。
 * @param data 安全身份平台存储的连续数据缓冲区；读取时作为输出区，写入时作为只读输入区，有效长度由 length 指定。
 * @param length 输入数据的有效长度，单位字节。
 * @return 1 表示指定身份记录字节已从 FRAM 完整读出，0 表示参数非法或 FRAM 事务失败。
 */
static uint8_t SensorSafeIdentityPlatform_Read(uint32_t address,
                                               uint8_t *data,
                                               size_t length)
{
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX)) {
        return 0U;
    }
    return (uint8_t)((FRAM_Read(data, address, (uint32_t)length) ==
                      FRAM_STATUS_OK) ? 1U : 0U);
}

/**
 * @brief 通过统一FRAM事务入口写入身份记录字节。
 *
 * @details 调用场景：身份模块推进 A/B 双副本时调用。
 * @note 关键约束：本函数为任务上下文调用，不得绕过FRAM仲裁直接控制SPI4和片选。
 *
 * @param address 安全身份平台存储区的 FRAM 绝对字节地址；读写长度由 length 指定并接受底层范围校验。
 * @param data 安全身份平台存储的连续数据缓冲区；读取时作为输出区，写入时作为只读输入区，有效长度由 length 指定。
 * @param length 输入数据的有效长度，单位字节。
 * @return 1 表示身份记录字节已通过统一 FRAM 事务完整写入；参数非法或 FRAM 失败时返回 0。
 */
static uint8_t SensorSafeIdentityPlatform_Write(uint32_t address,
                                                const uint8_t *data,
                                                size_t length)
{
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX)) {
        return 0U;
    }
    return (uint8_t)((FRAM_Write(data, address, (uint32_t)length) ==
                      FRAM_STATUS_OK) ? 1U : 0U);
}

/**
 * @brief 直接使用 F429 RNG 寄存器，避免启用当前工程未配置的 HAL RNG 模块。
 *
 * @param value 用于返回 STM32 硬件 RNG 生成的非零 32 位随机数。
 * @return 1 表示 F429 RNG 在限定轮询内生成有效随机数并写入输出参数；参数非法、时钟错误或超时返回 0。
 */
static uint8_t SensorSafeIdentityPlatform_Random(uint32_t *value)
{
    uint32_t saved_rng_cr;
    uint32_t clock_was_enabled;
    uint32_t loop;
    uint32_t status;
    uint8_t success = 0U;

    if (value == NULL) {
        return 0U;
    }
    /* 临时保存时钟和 RNG 控制态，调用完成后恢复，避免改变其他模块的外设所有权。 */
    clock_was_enabled = RCC->AHB2ENR & RCC_AHB2ENR_RNGEN;
    saved_rng_cr = RNG->CR;
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    (void)RCC->AHB2ENR;
    RNG->CR |= RNG_CR_RNGEN;
    for (loop = 0U; loop < SENSOR_SAFE_RNG_WAIT_LOOPS; loop++) {
        status = RNG->SR;
        if ((status & (RNG_SR_CECS | RNG_SR_SECS)) != 0U) {
            break;
        }
        if ((status & RNG_SR_DRDY) != 0U) {
            *value = RNG->DR;
            success = (*value != 0U) ? 1U : 0U;
            break;
        }
    }
    RNG->CR = saved_rng_cr;
    if (clock_was_enabled == 0U) {
        RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
    }
    return success;
}

/**
 * @brief 只暴露 STM32 唯一标识的三个有效字，越界索引固定返回 0。
 *
 * @param word_index STM32 96 位唯一标识的零基字索引；0、1、2 分别读取 HAL_GetUIDw0、HAL_GetUIDw1、HAL_GetUIDw2，其他值返回 0。
 * @return 返回只暴露 STM32 唯一标识的三个有效字，越界索引固定返回 0的零基索引；无法映射时使用函数约定的无效值或默认项。
 */
static uint32_t SensorSafeIdentityPlatform_Uid(uint8_t word_index)
{
    if (word_index == 0U) {
        return HAL_GetUIDw0();
    }
    if (word_index == 1U) {
        return HAL_GetUIDw1();
    }
    if (word_index == 2U) {
        return HAL_GetUIDw2();
    }
    return 0U;
}

/**
 * @brief 提供 HAL 单调毫秒节拍，用于确定性 nonce 降级输入。
 *
 * @return 返回 HAL_GetTick 提供的 32 位单调毫秒节拍；结果按 uint32_t 自然回绕。
 */
static uint32_t SensorSafeIdentityPlatform_NowMs(void)
{
    return HAL_GetTick();
}

/**
 * @brief 组装身份模块所需的 FRAM、随机数、UID 和时钟平台接口。
 *
 * @details 调用场景：安全协议服务初始化前由设备适配层调用。
 * @note 关键约束：只填写函数表，不访问 FRAM 或推进启动计数。
 *
 * @param ops 用于接收 CPU2 身份持久化平台函数表的输出对象。
 */
void SensorSafeIdentityPlatform_GetOps(SensorSafeIdentityOps *ops)
{
    if (ops == NULL) {
        return;
    }
    (void)memset(ops, 0, sizeof(*ops));
    ops->read = SensorSafeIdentityPlatform_Read;
    ops->write = SensorSafeIdentityPlatform_Write;
    ops->random_u32 = SensorSafeIdentityPlatform_Random;
    ops->uid_word = SensorSafeIdentityPlatform_Uid;
    ops->now_ms = SensorSafeIdentityPlatform_NowMs;
}
