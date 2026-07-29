/*
 * ch9141_at.h
 *
 * UART6 上的 CH9141K AT 指令辅助接口。
 */

#ifndef SENSOR_CH9141_AT_H_
#define SENSOR_CH9141_AT_H_

#include <stdint.h>

#define CH9141_AT_RESPONSE_TEXT_SIZE 768U /* CH9141K AT 指令参数：响应 文本 大小。 */

typedef enum {
    CH9141_AT_WAIT_ACK = 0,
    CH9141_AT_WAIT_SCAN_END,
    CH9141_AT_WAIT_LINK,
    CH9141_AT_WAIT_RSSI,
} CH9141AtWaitMode;

typedef struct {
    char text[CH9141_AT_RESPONSE_TEXT_SIZE];
    uint16_t len;
    uint8_t has_ok;
    uint8_t has_err;
    uint8_t has_link_ok;
    uint8_t has_pair_err;
    uint8_t has_scan_end;
    uint8_t has_rssi;
} CH9141AtResponse;

/**
 * @brief 清空 CH9141K AT 响应缓存。
 *
 * AT 命令返回内容需要保留给扫描解析、名称匹配和失败诊断使用。
 */
void CH9141_AT_ResetResponse(CH9141AtResponse *response);

/*
 * 标记传感器重新上电，下一次进入 AT 失败且只收到百分号时允许重试一次。
 */
void CH9141_AT_NotifySensorPowerOn(void);

/**
 * @brief 抢占 UART6 并等待串口空闲。
 *
 * 该函数会停止 UART6 DMA、清除硬件错误和残留数据，只能在主循环任务上下文调用。
 * 持续收包时达到固定总时限后返回通信超时，避免主循环无限等待。
 */
uint32_t CH9141_AT_PrepareUart6(uint32_t idle_ms);

/**
 * @brief 在 RSSI 查询清理不确定时，强制关闭异步上报并退出 AT 模式。
 *
 * 该函数绕过命令切换检查发送清理命令，只能在已进入 AT 模式的主循环任务上下文调用。
 */
void CH9141_AT_RecoverRssiQuery(void);

/**
 * @brief 通过 UART6 软件方式进入 CH9141K AT 配置。
 *
 * 硬件 AT 引脚未确认接入 CPU2 时，先保证 UART6 空闲，再发送 AT... 进入软件 AT 配置。
 */
uint32_t CH9141_AT_EnterSoftwareMode(CH9141AtResponse *response);

/**
 * @brief 不发送新命令，只等待 CH9141K 后续异步上报。
 *
 * RSSI 读取等命令会先 ACK，再按周期异步输出数据；这类数据不能按直接命令响应处理。
 */
uint32_t CH9141_AT_WaitAsync(CH9141AtWaitMode wait_mode,
                             uint32_t timeout_ms,
                             CH9141AtResponse *response);

/**
 * @brief 发送一条 AT 指令并等待指定结束条件。
 *
 * 每条指令自动追加 CRLF，调用方必须等待本函数返回后再发送下一条指令。
 */
uint32_t CH9141_AT_SendCommand(const char *cmd,
                               CH9141AtWaitMode wait_mode,
                               uint32_t timeout_ms,
                               CH9141AtResponse *response);

#endif /* SENSOR_CH9141_AT_H_ */
