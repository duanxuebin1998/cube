#include "sensor_safe_identity_platform.h"

#include <string.h>

#include "mb85rs2m.h"
#include "stm32f4xx_hal.h"

#define SENSOR_SAFE_RNG_WAIT_LOOPS  100000UL
/*
 * 函数用途：通过统一FRAM事务入口读取身份记录字节。
 * 调用场景：身份双副本初始化和写后回读校验使用。
 * 关键约束：本函数为任务上下文调用，不得绕过FRAM仲裁直接控制SPI4和片选。
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

/*
 * 函数用途：通过统一FRAM事务入口写入身份记录字节。
 * 调用场景：身份模块推进 A/B 双副本时调用。
 * 关键约束：本函数为任务上下文调用，不得绕过FRAM仲裁直接控制SPI4和片选。
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

/* 直接使用 F429 RNG 寄存器，避免启用当前工程未配置的 HAL RNG 模块。 */
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

/* 只暴露 STM32 唯一标识的三个有效字，越界索引固定返回 0。 */
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

/* 提供 HAL 单调毫秒节拍，用于确定性 nonce 降级输入。 */
static uint32_t SensorSafeIdentityPlatform_NowMs(void)
{
    return HAL_GetTick();
}

/*
 * 函数用途：组装身份模块所需的 FRAM、随机数、UID 和时钟平台接口。
 * 调用场景：安全协议服务初始化前由设备适配层调用。
 * 关键约束：只填写函数表，不访问 FRAM 或推进启动计数。
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
