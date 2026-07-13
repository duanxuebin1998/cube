#ifndef SENSOR_SAFE_TRANSPORT_UART6_H_
#define SENSOR_SAFE_TRANSPORT_UART6_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_SAFE_UART6_RX_BUFFER_SIZE 256U

typedef enum {
    SENSOR_SAFE_TRANSPORT_OK = 0,
    SENSOR_SAFE_TRANSPORT_INVALID_ARGUMENT,
    SENSOR_SAFE_TRANSPORT_BUSY,
    SENSOR_SAFE_TRANSPORT_ABORTED,
    SENSOR_SAFE_TRANSPORT_TX_FAILED,
    SENSOR_SAFE_TRANSPORT_RX_FAILED,
    SENSOR_SAFE_TRANSPORT_TIMEOUT,
    SENSOR_SAFE_TRANSPORT_OVERFLOW,
    SENSOR_SAFE_TRANSPORT_HARDWARE_ERROR
} SensorSafeTransportResult;

typedef void (*SensorSafeTransportDirectionHook)(void);
typedef uint8_t (*SensorSafeTransportAbortHook)(void);

typedef struct {
    SensorSafeTransportDirectionHook set_transmit_mode;
    SensorSafeTransportDirectionHook set_receive_mode;
    SensorSafeTransportAbortHook should_abort;
    uint32_t transmit_timeout_ms;
    uint32_t drain_idle_ms;
    uint32_t drain_total_ms;
} SensorSafeTransportUart6Config;

typedef struct {
    uint32_t exchange_count;
    uint32_t receive_count;
    uint32_t timeout_count;
    uint32_t hardware_error_count;
    uint32_t parity_error_count;
    uint32_t noise_error_count;
    uint32_t framing_error_count;
    uint32_t overrun_error_count;
    uint32_t dma_error_count;
    uint32_t break_detect_count;
    uint32_t short_frame_count;
    uint32_t malformed_prefix_count;
    uint32_t resync_count;
    uint32_t overflow_count;
    uint32_t last_hal_error;
    uint16_t last_received_bytes;
} SensorSafeTransportDiagnostics;

/*
 * 函数用途：初始化安全协议 UART6 传输配置和诊断计数。
 * 调用场景：App_Init 完成 UART6 和 CH9141 透传准备之后。
 * 关键约束：当前板卡 UART6 接 CH9141 时方向回调传 NULL；直连 RS485 板卡必须提供独立 DE/RE 回调。
 */
void SensorSafeTransportUart6_Init(const SensorSafeTransportUart6Config *config);

/*
 * 函数用途：在 UART6 上完成一次请求发送和单帧响应接收。
 * 调用场景：安全协议请求响应模式下由 client 层调用。
 * 关键约束：接收 DMA 先于发送启动；函数仅能在主循环任务上下文调用，不得在 ISR 中调用。
 */
SensorSafeTransportResult SensorSafeTransportUart6_Exchange(const uint8_t *request,
                                                            uint16_t request_len,
                                                            uint8_t *response,
                                                            size_t response_capacity,
                                                            uint16_t *response_len,
                                                            uint32_t *rx_timestamp_ms,
                                                            uint32_t response_timeout_ms);

/*
 * 函数用途：等待并提取一个安全协议控制帧或 44 字节快报帧。
 * 调用场景：周期上报接收、控制窗口响应或恢复流程。
 * 关键约束：调用前 UART6 必须处于 CH9141 透传或直连 RS485 接收状态。
 */
SensorSafeTransportResult SensorSafeTransportUart6_ReceiveFrame(uint8_t *frame,
                                                                size_t frame_capacity,
                                                                uint16_t *frame_len,
                                                                uint32_t *rx_timestamp_ms,
                                                                uint32_t timeout_ms);

/*
 * 函数用途：停止本模块当前 UART6 DMA 并恢复接收方向。
 * 调用场景：命令切换、协议回退、AT 模式切换或故障恢复入口。
 * 关键约束：只清理本次同步传输状态，不打印日志、不修改业务错误码。
 */
void SensorSafeTransportUart6_Abort(void);

uint8_t SensorSafeTransportUart6_IsBusy(void);
void SensorSafeTransportUart6_GetDiagnostics(SensorSafeTransportDiagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_TRANSPORT_UART6_H_ */
