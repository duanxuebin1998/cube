/*
 * 模块职责：声明Safe协议的UART6传输结果、诊断状态和同步交换/被动接收接口。
 * 所有权边界：每次收发必须申请DM4-Safe所有权，退出前完成DMA停止和线路清理。
 * 调用约束：接口用于任务上下文；UART回调只更新传输状态，不执行Safe业务解析。
 */
#ifndef SENSOR_SAFE_TRANSPORT_UART6_H_
/* SENSOR_SAFE_TRANSPORT_UART6_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_TRANSPORT_UART6_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 安全协议 UART6 DMA 接收缓冲容量 256 字节；大于最大控制帧，并为异步残留数据留出清理空间。 */
#define SENSOR_SAFE_UART6_RX_BUFFER_SIZE 256U

/* UART6 安全协议单次传输结果；区分参数、忙、主动中止、收发失败、超时、溢出和硬件错误。 */
typedef enum {
    /* UART6 传输层单次操作结果。 */
    SENSOR_SAFE_TRANSPORT_OK = 0, /* UART6 传输操作成功。 */
    SENSOR_SAFE_TRANSPORT_INVALID_ARGUMENT, /* 传输缓冲区、长度或配置非法。 */
    SENSOR_SAFE_TRANSPORT_BUSY, /* UART6 正被另一安全协议事务占用。 */
    SENSOR_SAFE_TRANSPORT_ABORTED, /* 上层中止回调要求立即结束传输。 */
    SENSOR_SAFE_TRANSPORT_TX_FAILED, /* UART6 发送未能完成。 */
    SENSOR_SAFE_TRANSPORT_RX_FAILED, /* UART6 接收未能完成。 */
    SENSOR_SAFE_TRANSPORT_TIMEOUT, /* 在配置时限内未完成收发。 */
    SENSOR_SAFE_TRANSPORT_OVERFLOW, /* 接收数据超过本地缓冲容量。 */
    SENSOR_SAFE_TRANSPORT_HARDWARE_ERROR /* UART6 报告奇偶、噪声、帧、溢出或 DMA 硬件错误。 */
} SensorSafeTransportResult;

typedef void (*SensorSafeTransportDirectionHook)(void);
typedef uint8_t (*SensorSafeTransportAbortHook)(void);

typedef struct {
    /* UART6 半双工方向回调、中止回调和发送/排空超时配置。 */
    SensorSafeTransportDirectionHook set_transmit_mode; /* 发送前切换半双工收发器到发送方向的回调。 */
    SensorSafeTransportDirectionHook set_receive_mode; /* 发送完成后恢复半双工收发器到接收方向的回调。 */
    SensorSafeTransportAbortHook should_abort; /* 等待收发期间查询上层是否要求立即中止的回调。 */
    uint32_t transmit_timeout_ms; /* UART6 单帧发送允许的最大等待时间，单位为 ms。 */
    uint32_t drain_idle_ms; /* 开始收帧前清理旧字节时要求连续空闲的时间，单位为 ms。 */
    uint32_t drain_total_ms; /* 开始事务前清理旧字节允许的总时长上限，单位为 ms。 */
} SensorSafeTransportUart6Config;

/* UART6 安全协议传输诊断快照；累计收发、超时、硬件错误、重同步和溢出信息供故障归因。 */
typedef struct {
    /* UART6 收发及各类硬件、帧同步、溢出错误的累计诊断。 */
    uint32_t exchange_count; /* UART6 主动交换累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t receive_count; /* UART6 被动接收累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t timeout_count; /* 通信超时累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t hardware_error_count; /* UART6 硬件错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t parity_error_count; /* UART6 奇偶校验错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t noise_error_count; /* UART6 噪声错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t framing_error_count; /* UART6 帧格式错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t overrun_error_count; /* UART6 接收溢出错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t dma_error_count; /* UART6 DMA 错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t break_detect_count; /* UART6 BREAK 检测累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t short_frame_count; /* 短于协议最小长度的帧累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t malformed_prefix_count; /* 帧头前缀非法累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t resync_count; /* 接收流成功重同步累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t overflow_count; /* 安全协议接收缓冲区溢出累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t last_hal_error; /* 最近一次 UART6 硬件错误位快照。 */
    uint16_t last_received_bytes; /* 最近一次 UART6 接收返回的实际字节数。 */
} SensorSafeTransportDiagnostics;

/**
 * @brief 初始化安全协议 UART6 传输配置和诊断计数。
 *
 * @details 调用场景：App_Init 完成 UART6 和 CH9141 透传准备之后。
 * @note 关键约束：当前板卡 UART6 接 CH9141 时方向回调传 NULL；直连 RS485 板卡必须提供独立 DE/RE 回调。
 */
void SensorSafeTransportUart6_Init(const SensorSafeTransportUart6Config *config);

/**
 * @brief 在 UART6 上完成一次请求发送和单帧响应接收。
 *
 * @details 调用场景：安全协议请求响应模式下由 client 层调用。
 * @note 关键约束：接收 DMA 先于发送启动；函数仅能在主循环任务上下文调用，不得在 ISR 中调用。
 */
SensorSafeTransportResult SensorSafeTransportUart6_Exchange(const uint8_t *request,
                                                            uint16_t request_len,
                                                            uint8_t *response,
                                                            size_t response_capacity,
                                                            uint16_t *response_len,
                                                            uint32_t *rx_timestamp_ms,
                                                            uint32_t response_timeout_ms);

/**
 * @brief 等待并提取一个安全协议控制帧或 44 字节快报帧。
 *
 * @details 调用场景：周期上报接收、控制窗口响应或恢复流程。
 * @note 关键约束：调用前 UART6 必须处于 CH9141 透传或直连 RS485 接收状态。
 */
SensorSafeTransportResult SensorSafeTransportUart6_ReceiveFrame(uint8_t *frame,
                                                                size_t frame_capacity,
                                                                uint16_t *frame_len,
                                                                uint32_t *rx_timestamp_ms,
                                                                uint32_t timeout_ms);

/**
 * @brief 停止本模块当前 UART6 DMA 并恢复接收方向。
 *
 * @details 调用场景：命令切换、协议回退、AT 模式切换或故障恢复入口。
 * @note 关键约束：只清理本次同步传输状态，不打印日志、不修改业务错误码。
 */
void SensorSafeTransportUart6_Abort(void);

/**
 * @brief 返回安全传输是否正在占用 UART6；只用于状态观察，不替代互斥保护。
 *
 * @return 1 表示安全传输当前持有 UART6；0 表示空闲。该值仅供观察，不替代互斥。
 */
uint8_t SensorSafeTransportUart6_IsBusy(void);
/**
 * @brief 复制累计传输诊断快照；空指针时不写输出。
 *
 * @param diagnostics 用于接收 UART6 超时、中止、溢出和硬件错误累计值的诊断快照。
 */
void SensorSafeTransportUart6_GetDiagnostics(SensorSafeTransportDiagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_TRANSPORT_UART6_H_ */
