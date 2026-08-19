/*
 * 模块职责：声明CH9141K AT响应快照、等待模式、链路交接状态和收发接口。
 * 调用边界：无线匹配与链路诊断使用本层；传感器协议不得直接发送AT文本。
 * 状态约束：只有透明传输就绪状态允许DSM、V3或DM4启动新的UART6事务。
 */

#ifndef SENSOR_CH9141_AT_H_
/* SENSOR_CH9141_AT_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_CH9141_AT_H_

#include <stdint.h>

#define CH9141_AT_RESPONSE_TEXT_SIZE 768U /* CH9141K AT 指令参数：响应 文本 大小。 */

typedef enum {
    /* CH9141K AT 命令等待结束的判定模式。 */
    CH9141_AT_WAIT_ACK = 0, /* 等待普通 AT 命令的 OK 或 ERR 结束标志。 */
    CH9141_AT_WAIT_SCAN_END, /* 等待无线扫描结束标志，并持续收集候选行。 */
    CH9141_AT_WAIT_LINK, /* 等待连接成功或配对失败标志。 */
    CH9141_AT_WAIT_RSSI, /* 等待完整 RSSI 文本字段；半 ACK 时仍由恢复流程继续清理。 */
} CH9141AtWaitMode;

typedef enum {
    /* UART6 在 DSM 透传与 CH9141K AT 操作之间的公开交接状态。 */
    CH9141_UART6_TRANSPARENT_READY = 0, /* 透明传输链路已确认就绪，允许传感器协议申请UART6。 */
    CH9141_UART6_ENTERING_AT, /* 正在进入软件AT模式，透明传感器事务必须等待。 */
    CH9141_UART6_AT_ACTIVE, /* 软件AT模式已激活，UART6用于模块维护命令。 */
    CH9141_UART6_RECOVERING, /* 正在退出AT并清理残留异步文本。 */
    CH9141_UART6_NOT_READY /* 透明链路未确认就绪，禁止开始传感器事务。 */
} CH9141Uart6LinkState;

typedef struct {
    /* CH9141K AT 响应解析结果；保留原始文本长度及 OK、ERR、连接、扫描和 RSSI 关键标志。 */
    char text[CH9141_AT_RESPONSE_TEXT_SIZE]; /* 本次 AT 响应的原始文本副本，固定容量并保证以 NUL 结束。 */
    uint16_t len; /* text 中不含终止符的有效字节数。 */
    uint8_t has_ok; /* 响应文本中已经识别到独立 OK 结束标志。 */
    uint8_t has_err; /* 响应文本中已经识别到通用 ERR 标志。 */
    uint8_t has_link_ok; /* 响应文本中已经识别到连接成功标志。 */
    uint8_t has_pair_err; /* 响应文本中已经识别到配对失败标志。 */
    uint8_t has_scan_end; /* 响应文本中已经识别到扫描结束标志。 */
    uint8_t has_rssi; /* 响应文本中已经解析出完整 RSSI 字段的标志。 */
} CH9141AtResponse;

/**
 * @brief 清空 CH9141K AT 响应缓存。
 *
 * AT 命令返回内容需要保留给扫描解析、名称匹配和失败诊断使用。
 */
void CH9141_AT_ResetResponse(CH9141AtResponse *response);

/**
 * @brief 标记传感器重新上电后的首次百分号透传过滤机会。
 *
 * 设备启动或明确重新给传感器供电后，由主循环任务调用。
 *
 * 标记传感器重新上电，下一次进入 AT 失败且只收到百分号时允许重试一次。
 *
 * @note 只允许首次进入 AT 失败且响应仅为百分号时重试一次。
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
 * @brief 在退出 AT 或模块复位后完成不可被命令切换打断的透明传输交接。
 *
 * 该函数只允许在已经发出 AT+EXIT/AT+RESET 的收尾路径调用；返回前必须完成最终空闲确认。
 */
uint32_t CH9141_AT_CompleteTransparentHandoff(uint32_t idle_ms);

/**
 * @brief 读取 UART6 当前是否已经完成透明传输交接。
 *
 * DSM 发送入口只允许在 CH9141_UART6_TRANSPARENT_READY 状态启动新事务。
 */
CH9141Uart6LinkState CH9141_AT_GetUart6LinkState(void);

/**
 * @brief 在 RSSI 查询清理不确定时，强制关闭异步上报并退出 AT 模式。
 *
 * 该函数绕过命令切换检查发送清理命令，只能在已进入 AT 模式的主循环任务上下文调用。
 * 返回前会执行最终 UART6 空闲确认；只有 NO_ERROR 才表示透明传输重新就绪。
 */
uint32_t CH9141_AT_RecoverRssiQuery(void);

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
