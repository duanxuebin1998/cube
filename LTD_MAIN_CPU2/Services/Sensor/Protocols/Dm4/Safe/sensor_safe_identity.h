/*
 * 模块职责：声明Safe身份记录格式、双槽位置、平台依赖和初始化结果。
 * 持久化边界：槽地址、记录长度、版本和启动计数属于兼容性契约，修改时必须整体迁移。
 * 调用约束：上层只消费校验后的身份上下文，不直接解释FRAM原始字节。
 */
#ifndef SENSOR_SAFE_IDENTITY_H_
/* SENSOR_SAFE_IDENTITY_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_IDENTITY_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 安全身份 A 槽在 FRAM 中的起始地址 0x1100。 */
#define SENSOR_SAFE_IDENTITY_SLOT_A_ADDRESS  0x1100UL
/* 安全身份 B 槽在 FRAM 中的起始地址 0x1140；与 A 槽分离，用于双槽冗余和掉电恢复。 */
#define SENSOR_SAFE_IDENTITY_SLOT_B_ADDRESS  0x1140UL
/* 每个安全身份 FRAM 槽预留大小 0x40 字节；槽间地址步长和边界校验均以此值为准。 */
#define SENSOR_SAFE_IDENTITY_SLOT_SIZE       0x0040UL
/* 当前安全身份有效记录长度 20 字节；小于槽容量，剩余区域保留给后续兼容扩展。 */
#define SENSOR_SAFE_IDENTITY_RECORD_SIZE     20U

/* 传感器安全协议身份管理的平台存储回调类型组；读写回调必须按请求长度完整访问指定 FRAM 地址，随机数和 UID 回调用于构造启动身份。 */
typedef uint8_t (*SensorSafeIdentityReadFn)(uint32_t address,
                                            uint8_t *data,
                                            size_t length);
typedef uint8_t (*SensorSafeIdentityWriteFn)(uint32_t address,
                                             const uint8_t *data,
                                             size_t length);
typedef uint8_t (*SensorSafeIdentityRandomFn)(uint32_t *value);
typedef uint32_t (*SensorSafeIdentityUidFn)(uint8_t word_index);
/* 传感器安全协议身份管理的单调毫秒时钟回调；用于生成回退 nonce，不能使用会倒退的日历时间。 */
typedef uint32_t (*SensorSafeIdentityNowMsFn)(void);

typedef struct {
    /* 身份管理依赖的持久化、随机数、芯片 UID 和单调时钟回调。 */
    SensorSafeIdentityReadFn read; /* 从身份持久化区完整读取指定字节范围的回调。 */
    SensorSafeIdentityWriteFn write; /* 向身份持久化区完整写入指定字节范围的回调。 */
    SensorSafeIdentityRandomFn random_u32; /* 取得硬件随机 32 位值的回调；失败时允许身份层改用确定性回退。 */
    SensorSafeIdentityUidFn uid_word; /* 按字索引读取 MCU 唯一标识的回调。 */
    SensorSafeIdentityNowMsFn now_ms; /* 返回单调毫秒节拍的回调，用于回退 nonce 混合。 */
} SensorSafeIdentityOps;

typedef enum {
    /* 启动身份与持久化启动计数处理结果。 */
    SENSOR_SAFE_IDENTITY_OK = 0, /* 启动计数和 nonce 均通过首选路径生成。 */
    SENSOR_SAFE_IDENTITY_OK_FALLBACK_NONCE, /* 启动身份有效，但 nonce 使用确定性回退源。 */
    SENSOR_SAFE_IDENTITY_INVALID_ARGUMENT, /* 身份上下文或平台回调配置非法。 */
    SENSOR_SAFE_IDENTITY_STORAGE_ERROR, /* FRAM 身份双副本读取或写回失败。 */
    SENSOR_SAFE_IDENTITY_RECORD_CONFLICT, /* 两个有效身份副本的代际或启动计数相互冲突。 */
    SENSOR_SAFE_IDENTITY_COUNTER_EXHAUSTED, /* 持久化启动计数已经达到无符号上限。 */
    SENSOR_SAFE_IDENTITY_NOT_INITIALIZED /* 身份上下文尚未完成启动初始化。 */
} SensorSafeIdentityResult;

typedef enum {
    /* 本次会话 nonce 的随机源。 */
    SENSOR_SAFE_NONCE_HARDWARE_RANDOM = 0, /* 本次 nonce 来自硬件随机数源。 */
    SENSOR_SAFE_NONCE_DETERMINISTIC_FALLBACK /* 硬件随机数不可用，本次 nonce 使用 UID、启动计数和节拍确定性混合。 */
} SensorSafeNonceSource;

/* CPU2 安全协议启动身份上下文；保存平台操作、双副本代际、启动计数和 nonce 状态，初始化成功前不得建立会话。 */
typedef struct {
    /* 身份管理运行态，包括启动代际、nonce 计数和初始化状态。 */
    SensorSafeIdentityOps ops; /* 身份管理使用的持久化、随机数、UID 和单调时钟平台回调。 */
    uint32_t boot_counter; /* CPU2 持久化启动计数；成功写回双副本后才对外用于 HELLO。 */
    uint32_t generation; /* 身份双副本记录代际；装载时选择校验有效且代际更新的副本。 */
    uint32_t nonce_counter; /* 本次启动期间生成 nonce 的本地递增计数。 */
    uint32_t last_nonce; /* 最近一次成功生成的会话 nonce，用于避免同一启动周期内重复。 */
    uint8_t initialized; /* 启动计数已从 FRAM 恢复、递增并安全写回的标志。 */
} SensorSafeIdentityContext;

/**
 * @brief 从 FRAM 双副本恢复并原子递增 CPU2 启动计数。
 *
 * @details 调用场景：SPI4 FRAM 初始化完成后、安全协议首次 HELLO 之前。
 * @note 关键约束：主副本按代次奇偶交替写入并逐次回读，随后镜像到另一副本；任一写后校验失败都禁止建立安全会话。
 */
SensorSafeIdentityResult SensorSafeIdentity_Initialize(SensorSafeIdentityContext *context,
                                                       const SensorSafeIdentityOps *ops);

/**
 * @brief 读取本次启动已经持久化确认的单调启动计数。
 *
 * @details 调用场景：配置 SensorSafeClientConfig.cpu_boot_counter。
 * @note 关键约束：初始化失败时不得返回可用于 HELLO 的计数。
 */
SensorSafeIdentityResult SensorSafeIdentity_GetBootCounter(const SensorSafeIdentityContext *context,
                                                           uint32_t *boot_counter);

/**
 * @brief 为新的 HELLO 事务生成一次性挑战。
 *
 * @details 调用场景：每次发起全新 HELLO，而不是同一事务超时重发时。
 * @note 关键约束：优先硬件 RNG；失败时混合 UID、启动计数、本地计数和时钟生成确定性后备值，后备值不得宣称具备密码学安全性。
 */
SensorSafeIdentityResult SensorSafeIdentity_GenerateNonce(SensorSafeIdentityContext *context,
                                                          uint32_t *nonce,
                                                          SensorSafeNonceSource *source);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_IDENTITY_H_ */
