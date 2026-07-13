#include "sensor_safe_identity_platform.h"

#include <string.h>

#include "mb85rs2m.h"
#include "stm32f4xx_hal.h"

#define SENSOR_SAFE_RNG_WAIT_LOOPS  100000UL
#define SENSOR_SAFE_FRAM_CAPACITY_BYTES  0x40000UL
#define SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS  100U

/* 校验 FRAM 地址加长度不越界，并限制为 HAL SPI 可表达的 16 位传输长度。 */
static uint8_t SensorSafeIdentityPlatform_IsRangeValid(uint32_t address,
                                                       size_t length)
{
    if ((length == 0U) || (length > UINT16_MAX) ||
        (address >= SENSOR_SAFE_FRAM_CAPACITY_BYTES)) {
        return 0U;
    }
    return (uint8_t)((length <=
                      (size_t)(SENSOR_SAFE_FRAM_CAPACITY_BYTES - address))
                         ? 1U
                         : 0U);
}

/* 无论 SPI 成功或失败都释放 FRAM 片选，避免阻塞同总线后续设备。 */
static void SensorSafeIdentityPlatform_Deselect(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/* 发送 FRAM 写使能命令；调用结束前必定恢复片选为非选中。 */
static uint8_t SensorSafeIdentityPlatform_WriteEnable(void)
{
    uint8_t command = MB_WRITEENABLE;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&FRAM_SPI,
                              &command,
                              1U,
                              SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS);
    SensorSafeIdentityPlatform_Deselect();
    return (uint8_t)((status == HAL_OK) ? 1U : 0U);
}

/*
 * 函数用途：通过 SPI 从指定 FRAM 地址读取身份记录字节。
 * 调用场景：身份双副本初始化和写后回读校验使用。
 * 关键约束：本函数为任务上下文阻塞调用，不得在中断中执行；所有出口释放片选。
 */
static uint8_t SensorSafeIdentityPlatform_Read(uint32_t address,
                                               uint8_t *data,
                                               size_t length)
{
    uint8_t command = MB_READDATA;
    uint8_t address_bytes[3];
    HAL_StatusTypeDef status;

    if ((data == NULL) ||
        (SensorSafeIdentityPlatform_IsRangeValid(address, length) == 0U)) {
        return 0U;
    }
    address_bytes[0] = (uint8_t)((address >> 16U) & 0xFFU);
    address_bytes[1] = (uint8_t)((address >> 8U) & 0xFFU);
    address_bytes[2] = (uint8_t)(address & 0xFFU);

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&FRAM_SPI,
                              &command,
                              1U,
                              SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Transmit(&FRAM_SPI,
                                  address_bytes,
                                  (uint16_t)sizeof(address_bytes),
                                  SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS);
    }
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&FRAM_SPI,
                                 data,
                                 (uint16_t)length,
                                 SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS);
    }
    SensorSafeIdentityPlatform_Deselect();
    return (uint8_t)((status == HAL_OK) ? 1U : 0U);
}

/*
 * 函数用途：写使能后通过 SPI 向指定 FRAM 地址写入身份记录字节。
 * 调用场景：身份模块推进 A/B 双副本时调用。
 * 关键约束：本函数为任务上下文阻塞调用，不得在中断中执行；范围非法时不触碰硬件。
 */
static uint8_t SensorSafeIdentityPlatform_Write(uint32_t address,
                                                const uint8_t *data,
                                                size_t length)
{
    uint8_t command = MB_WRITEDATA;
    uint8_t address_bytes[3];
    HAL_StatusTypeDef status;

    if ((data == NULL) ||
        (SensorSafeIdentityPlatform_IsRangeValid(address, length) == 0U)) {
        return 0U;
    }
    if (SensorSafeIdentityPlatform_WriteEnable() == 0U) {
        return 0U;
    }
    address_bytes[0] = (uint8_t)((address >> 16U) & 0xFFU);
    address_bytes[1] = (uint8_t)((address >> 8U) & 0xFFU);
    address_bytes[2] = (uint8_t)(address & 0xFFU);

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&FRAM_SPI,
                              &command,
                              1U,
                              SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Transmit(&FRAM_SPI,
                                  address_bytes,
                                  (uint16_t)sizeof(address_bytes),
                                  SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS);
    }
    if (status == HAL_OK) {
        status = HAL_SPI_Transmit(&FRAM_SPI,
                                  (uint8_t *)data,
                                  (uint16_t)length,
                                  SENSOR_SAFE_FRAM_SPI_TIMEOUT_MS);
    }
    SensorSafeIdentityPlatform_Deselect();
    return (uint8_t)((status == HAL_OK) ? 1U : 0U);
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
