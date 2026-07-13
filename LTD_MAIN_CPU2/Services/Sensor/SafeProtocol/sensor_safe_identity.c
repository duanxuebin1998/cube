#include "sensor_safe_identity.h"

#include <limits.h>
#include <string.h>

#include "sensor_safe_crc32c.h"

#define SENSOR_SAFE_IDENTITY_MAGIC          0x53424944UL
#define SENSOR_SAFE_IDENTITY_VERSION        1U
#define SENSOR_SAFE_IDENTITY_CRC_OFFSET     16U
#define SENSOR_SAFE_IDENTITY_NONCE_BYTES    28U

_Static_assert(SENSOR_SAFE_IDENTITY_RECORD_SIZE <= SENSOR_SAFE_IDENTITY_SLOT_SIZE,
               "identity record exceeds FRAM slot");
_Static_assert((SENSOR_SAFE_IDENTITY_SLOT_A_ADDRESS + SENSOR_SAFE_IDENTITY_SLOT_SIZE) <=
                   SENSOR_SAFE_IDENTITY_SLOT_B_ADDRESS,
               "identity FRAM slots overlap");

/* FRAM 身份副本的可信摘要；valid 只在魔数、版本、长度、CRC 和填充均通过后置位。 */
typedef struct {
    uint32_t generation;
    uint32_t boot_counter;
    uint8_t valid;
} SensorSafeIdentityRecord;

/* 从身份记录的小端线格式读取 16 位字段。 */
static uint16_t SensorSafeIdentity_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/* 从身份记录的小端线格式读取 32 位字段。 */
static uint32_t SensorSafeIdentity_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/* 按小端线格式写入 16 位字段，避免结构体填充影响持久化布局。 */
static void SensorSafeIdentity_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/* 按小端线格式写入 32 位字段，确保跨编译器记录一致。 */
static void SensorSafeIdentity_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/* 逐字段解码持久化记录，禁止依赖编译器结构体布局。 */
static SensorSafeIdentityRecord SensorSafeIdentity_DecodeRecord(const uint8_t *raw)
{
    SensorSafeIdentityRecord record;
    uint32_t expected_crc;
    size_t index;

    (void)memset(&record, 0, sizeof(record));
    if ((SensorSafeIdentity_ReadU32(&raw[0]) != SENSOR_SAFE_IDENTITY_MAGIC) ||
        (SensorSafeIdentity_ReadU16(&raw[4]) != SENSOR_SAFE_IDENTITY_VERSION) ||
        (SensorSafeIdentity_ReadU16(&raw[6]) != SENSOR_SAFE_IDENTITY_RECORD_SIZE)) {
        return record;
    }
    expected_crc = SensorSafeCrc32c_Calculate(raw, SENSOR_SAFE_IDENTITY_CRC_OFFSET);
    if (SensorSafeIdentity_ReadU32(&raw[SENSOR_SAFE_IDENTITY_CRC_OFFSET]) != expected_crc) {
        return record;
    }
    for (index = SENSOR_SAFE_IDENTITY_RECORD_SIZE;
         index < (size_t)SENSOR_SAFE_IDENTITY_SLOT_SIZE;
         index++) {
        if (raw[index] != 0xFFU) {
            return record;
        }
    }
    record.generation = SensorSafeIdentity_ReadU32(&raw[8]);
    record.boot_counter = SensorSafeIdentity_ReadU32(&raw[12]);
    if ((record.generation == 0U) || (record.boot_counter == 0U)) {
        return record;
    }
    record.valid = 1U;
    return record;
}

/* 按冻结的小端字段顺序编码记录并计算 CRC32C。 */
static void SensorSafeIdentity_EncodeRecord(uint32_t generation,
                                            uint32_t boot_counter,
                                            uint8_t *raw)
{
    (void)memset(raw, 0xFF, SENSOR_SAFE_IDENTITY_SLOT_SIZE);
    SensorSafeIdentity_WriteU32(&raw[0], SENSOR_SAFE_IDENTITY_MAGIC);
    SensorSafeIdentity_WriteU16(&raw[4], SENSOR_SAFE_IDENTITY_VERSION);
    SensorSafeIdentity_WriteU16(&raw[6], SENSOR_SAFE_IDENTITY_RECORD_SIZE);
    SensorSafeIdentity_WriteU32(&raw[8], generation);
    SensorSafeIdentity_WriteU32(&raw[12], boot_counter);
    SensorSafeIdentity_WriteU32(&raw[SENSOR_SAFE_IDENTITY_CRC_OFFSET],
                                SensorSafeCrc32c_Calculate(raw,
                                                          SENSOR_SAFE_IDENTITY_CRC_OFFSET));
}

/* 只有全 0 或全 0xFF 才视为从未初始化，损坏记录不得静默重置计数。 */
static uint8_t SensorSafeIdentity_IsBlank(const uint8_t *raw)
{
    size_t index;
    uint8_t all_zero = 1U;
    uint8_t all_erased = 1U;

    for (index = 0U; index < (size_t)SENSOR_SAFE_IDENTITY_SLOT_SIZE; index++) {
        if (raw[index] != 0U) {
            all_zero = 0U;
        }
        if (raw[index] != 0xFFU) {
            all_erased = 0U;
        }
    }
    return (uint8_t)(((all_zero != 0U) || (all_erased != 0U)) ? 1U : 0U);
}

/* 写入一个副本并立即回读逐字节核对，底层无错误返回值时仍能发现失败。 */
static uint8_t SensorSafeIdentity_WriteAndVerify(const SensorSafeIdentityOps *ops,
                                                 uint32_t address,
                                                 const uint8_t *raw)
{
    uint8_t verify[SENSOR_SAFE_IDENTITY_SLOT_SIZE];
    SensorSafeIdentityRecord record;

    if (ops->write(address, raw, SENSOR_SAFE_IDENTITY_SLOT_SIZE) == 0U) {
        return 0U;
    }
    (void)memset(verify, 0, sizeof(verify));
    if (ops->read(address, verify, sizeof(verify)) == 0U) {
        return 0U;
    }
    record = SensorSafeIdentity_DecodeRecord(verify);
    if ((record.valid == 0U) || (memcmp(raw, verify, sizeof(verify)) != 0)) {
        return 0U;
    }
    return 1U;
}

/*
 * 函数用途：验收 FRAM A/B 身份副本，并为本次上电原子推进代次和启动计数。
 * 调用场景：安全协议服务首次初始化时调用，同一上电周期不得重复执行。
 * 关键约束：损坏或相互矛盾的非空副本必须报错；计数耗尽时禁止回绕重用身份。
 */
SensorSafeIdentityResult SensorSafeIdentity_Initialize(SensorSafeIdentityContext *context,
                                                       const SensorSafeIdentityOps *ops)
{
    uint8_t raw_a[SENSOR_SAFE_IDENTITY_SLOT_SIZE];
    uint8_t raw_b[SENSOR_SAFE_IDENTITY_SLOT_SIZE];
    uint8_t new_raw[SENSOR_SAFE_IDENTITY_SLOT_SIZE];
    SensorSafeIdentityRecord record_a;
    SensorSafeIdentityRecord record_b;
    SensorSafeIdentityRecord newest;
    uint32_t primary_address;
    uint32_t secondary_address;
    uint32_t next_generation;
    uint32_t next_boot_counter;

    if ((context == NULL) || (ops == NULL) || (ops->read == NULL) || (ops->write == NULL)) {
        return SENSOR_SAFE_IDENTITY_INVALID_ARGUMENT;
    }
    (void)memset(context, 0, sizeof(*context));
    context->ops = *ops;
    (void)memset(raw_a, 0, sizeof(raw_a));
    (void)memset(raw_b, 0, sizeof(raw_b));
    if (ops->read(SENSOR_SAFE_IDENTITY_SLOT_A_ADDRESS, raw_a, sizeof(raw_a)) == 0U) {
        return SENSOR_SAFE_IDENTITY_STORAGE_ERROR;
    }
    if (ops->read(SENSOR_SAFE_IDENTITY_SLOT_B_ADDRESS, raw_b, sizeof(raw_b)) == 0U) {
        return SENSOR_SAFE_IDENTITY_STORAGE_ERROR;
    }
    record_a = SensorSafeIdentity_DecodeRecord(raw_a);
    record_b = SensorSafeIdentity_DecodeRecord(raw_b);
    (void)memset(&newest, 0, sizeof(newest));

    /* 双副本必须同时保持代次与启动计数单调，否则无法证明最新身份。 */
    if ((record_a.valid != 0U) && (record_b.valid != 0U)) {
        if ((record_a.generation == record_b.generation) &&
            (record_a.boot_counter != record_b.boot_counter)) {
            return SENSOR_SAFE_IDENTITY_RECORD_CONFLICT;
        }
        if (((record_a.generation > record_b.generation) &&
             (record_a.boot_counter <= record_b.boot_counter)) ||
            ((record_b.generation > record_a.generation) &&
             (record_b.boot_counter <= record_a.boot_counter))) {
            return SENSOR_SAFE_IDENTITY_RECORD_CONFLICT;
        }
        newest = (record_a.generation >= record_b.generation) ? record_a : record_b;
    } else if (record_a.valid != 0U) {
        newest = record_a;
    } else if (record_b.valid != 0U) {
        newest = record_b;
    }

    if ((newest.valid == 0U) &&
        ((SensorSafeIdentity_IsBlank(raw_a) == 0U) ||
         (SensorSafeIdentity_IsBlank(raw_b) == 0U))) {
        return SENSOR_SAFE_IDENTITY_RECORD_CONFLICT;
    }
    if (newest.valid == 0U) {
        next_generation = 1U;
        next_boot_counter = 1U;
    } else {
        if ((newest.generation == UINT32_MAX) || (newest.boot_counter == UINT32_MAX)) {
            return SENSOR_SAFE_IDENTITY_COUNTER_EXHAUSTED;
        }
        next_generation = newest.generation + 1U;
        next_boot_counter = newest.boot_counter + 1U;
    }

    SensorSafeIdentity_EncodeRecord(next_generation, next_boot_counter, new_raw);
    if ((next_generation & 1U) != 0U) {
        primary_address = SENSOR_SAFE_IDENTITY_SLOT_A_ADDRESS;
        secondary_address = SENSOR_SAFE_IDENTITY_SLOT_B_ADDRESS;
    } else {
        primary_address = SENSOR_SAFE_IDENTITY_SLOT_B_ADDRESS;
        secondary_address = SENSOR_SAFE_IDENTITY_SLOT_A_ADDRESS;
    }
    /* 先写本代主槽再镜像到另一槽；任一回读失败都不发布内存态身份。 */
    if (SensorSafeIdentity_WriteAndVerify(ops, primary_address, new_raw) == 0U) {
        return SENSOR_SAFE_IDENTITY_STORAGE_ERROR;
    }
    if (SensorSafeIdentity_WriteAndVerify(ops, secondary_address, new_raw) == 0U) {
        return SENSOR_SAFE_IDENTITY_STORAGE_ERROR;
    }

    context->boot_counter = next_boot_counter;
    context->generation = next_generation;
    context->initialized = 1U;
    return SENSOR_SAFE_IDENTITY_OK;
}

/*
 * 函数用途：读取已持久化并在本次上电推进后的启动计数。
 * 调用场景：HELLO 请求组装控制器身份时调用。
 * 关键约束：身份模块未初始化时不得返回默认值冒充有效计数。
 */
SensorSafeIdentityResult SensorSafeIdentity_GetBootCounter(const SensorSafeIdentityContext *context,
                                                           uint32_t *boot_counter)
{
    if ((context == NULL) || (boot_counter == NULL)) {
        return SENSOR_SAFE_IDENTITY_INVALID_ARGUMENT;
    }
    if (context->initialized == 0U) {
        return SENSOR_SAFE_IDENTITY_NOT_INITIALIZED;
    }
    *boot_counter = context->boot_counter;
    return SENSOR_SAFE_IDENTITY_OK;
}

/*
 * 函数用途：生成 HELLO 挑战随机数，并显式报告硬件随机或确定性降级来源。
 * 调用场景：每次建立或恢复安全会话前调用。
 * 关键约束：禁止返回 0 或重复上次值；硬件 RNG 不可用时混合 UID、启动计数和单调计数。
 */
SensorSafeIdentityResult SensorSafeIdentity_GenerateNonce(SensorSafeIdentityContext *context,
                                                          uint32_t *nonce,
                                                          SensorSafeNonceSource *source)
{
    uint8_t normalized[SENSOR_SAFE_IDENTITY_NONCE_BYTES];
    uint32_t candidate = 0U;
    uint32_t now_ms = 0U;
    uint8_t index;

    if ((context == NULL) || (nonce == NULL) || (source == NULL)) {
        return SENSOR_SAFE_IDENTITY_INVALID_ARGUMENT;
    }
    if (context->initialized == 0U) {
        return SENSOR_SAFE_IDENTITY_NOT_INITIALIZED;
    }
    /* 优先使用硬件随机数，但 0 和与上次相同的值均视为不可接受。 */
    if ((context->ops.random_u32 != NULL) &&
        (context->ops.random_u32(&candidate) != 0U) &&
        (candidate != 0U) && (candidate != context->last_nonce)) {
        context->last_nonce = candidate;
        *nonce = candidate;
        *source = SENSOR_SAFE_NONCE_HARDWARE_RANDOM;
        return SENSOR_SAFE_IDENTITY_OK;
    }
    if (context->nonce_counter == UINT32_MAX) {
        return SENSOR_SAFE_IDENTITY_COUNTER_EXHAUSTED;
    }
    /* 降级路径仍依赖本次上电内不可回绕的计数，避免同一设备快速重复挑战。 */
    context->nonce_counter++;
    (void)memset(normalized, 0, sizeof(normalized));
    for (index = 0U; index < 3U; index++) {
        uint32_t uid = (context->ops.uid_word != NULL) ? context->ops.uid_word(index) : 0U;
        SensorSafeIdentity_WriteU32(&normalized[(size_t)index * 4U], uid);
    }
    if (context->ops.now_ms != NULL) {
        now_ms = context->ops.now_ms();
    }
    SensorSafeIdentity_WriteU32(&normalized[12], context->boot_counter);
    SensorSafeIdentity_WriteU32(&normalized[16], context->nonce_counter);
    SensorSafeIdentity_WriteU32(&normalized[20], now_ms);
    SensorSafeIdentity_WriteU32(&normalized[24], context->last_nonce);
    candidate = SensorSafeCrc32c_Calculate(normalized, sizeof(normalized));
    if ((candidate == 0U) || (candidate == context->last_nonce)) {
        candidate ^= 0xA5C35A3CUL;
        if (candidate == 0U) {
            candidate = context->nonce_counter;
        }
    }
    context->last_nonce = candidate;
    *nonce = candidate;
    *source = SENSOR_SAFE_NONCE_DETERMINISTIC_FALLBACK;
    return SENSOR_SAFE_IDENTITY_OK_FALLBACK_NONCE;
}
