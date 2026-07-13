#ifndef SENSOR_SAFE_IDENTITY_H_
#define SENSOR_SAFE_IDENTITY_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_SAFE_IDENTITY_SLOT_A_ADDRESS  0x1100UL
#define SENSOR_SAFE_IDENTITY_SLOT_B_ADDRESS  0x1140UL
#define SENSOR_SAFE_IDENTITY_SLOT_SIZE       0x0040UL
#define SENSOR_SAFE_IDENTITY_RECORD_SIZE     20U

typedef uint8_t (*SensorSafeIdentityReadFn)(uint32_t address,
                                            uint8_t *data,
                                            size_t length);
typedef uint8_t (*SensorSafeIdentityWriteFn)(uint32_t address,
                                             const uint8_t *data,
                                             size_t length);
typedef uint8_t (*SensorSafeIdentityRandomFn)(uint32_t *value);
typedef uint32_t (*SensorSafeIdentityUidFn)(uint8_t word_index);
typedef uint32_t (*SensorSafeIdentityNowMsFn)(void);

typedef struct {
    SensorSafeIdentityReadFn read;
    SensorSafeIdentityWriteFn write;
    SensorSafeIdentityRandomFn random_u32;
    SensorSafeIdentityUidFn uid_word;
    SensorSafeIdentityNowMsFn now_ms;
} SensorSafeIdentityOps;

typedef enum {
    SENSOR_SAFE_IDENTITY_OK = 0,
    SENSOR_SAFE_IDENTITY_OK_FALLBACK_NONCE,
    SENSOR_SAFE_IDENTITY_INVALID_ARGUMENT,
    SENSOR_SAFE_IDENTITY_STORAGE_ERROR,
    SENSOR_SAFE_IDENTITY_RECORD_CONFLICT,
    SENSOR_SAFE_IDENTITY_COUNTER_EXHAUSTED,
    SENSOR_SAFE_IDENTITY_NOT_INITIALIZED
} SensorSafeIdentityResult;

typedef enum {
    SENSOR_SAFE_NONCE_HARDWARE_RANDOM = 0,
    SENSOR_SAFE_NONCE_DETERMINISTIC_FALLBACK
} SensorSafeNonceSource;

typedef struct {
    SensorSafeIdentityOps ops;
    uint32_t boot_counter;
    uint32_t generation;
    uint32_t nonce_counter;
    uint32_t last_nonce;
    uint8_t initialized;
} SensorSafeIdentityContext;

/*
 * 函数用途：从 FRAM 双副本恢复并原子递增 CPU2 启动计数。
 * 调用场景：SPI4 FRAM 初始化完成后、安全协议首次 HELLO 之前。
 * 关键约束：主副本按代次奇偶交替写入并逐次回读，随后镜像到另一副本；任一写后校验失败都禁止建立安全会话。
 */
SensorSafeIdentityResult SensorSafeIdentity_Initialize(SensorSafeIdentityContext *context,
                                                       const SensorSafeIdentityOps *ops);

/*
 * 函数用途：读取本次启动已经持久化确认的单调启动计数。
 * 调用场景：配置 SensorSafeClientConfig.cpu_boot_counter。
 * 关键约束：初始化失败时不得返回可用于 HELLO 的计数。
 */
SensorSafeIdentityResult SensorSafeIdentity_GetBootCounter(const SensorSafeIdentityContext *context,
                                                           uint32_t *boot_counter);

/*
 * 函数用途：为新的 HELLO 事务生成一次性挑战。
 * 调用场景：每次发起全新 HELLO，而不是同一事务超时重发时。
 * 关键约束：优先硬件 RNG；失败时混合 UID、启动计数、本地计数和时钟生成确定性后备值，后备值不得宣称具备密码学安全性。
 */
SensorSafeIdentityResult SensorSafeIdentity_GenerateNonce(SensorSafeIdentityContext *context,
                                                          uint32_t *nonce,
                                                          SensorSafeNonceSource *source);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_IDENTITY_H_ */
