#ifndef SENSOR_SAFE_MONITOR_H_
/* SENSOR_SAFE_MONITOR_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_MONITOR_H_

#include "sensor_safe_session.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 周期快报的双缓冲发布快照；保存同一帧的测量值、序号、数据年龄、状态位和信号质量。 */
typedef struct {
    /* 周期快报经校验后发布的测量、序号、年龄、状态和质量快照。 */
    int16_t temperature_c_x100; /* 温度百倍定点值，单位为 0.01 ℃；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    int32_t density_kg_m3_x100; /* 密度百倍定点值，单位为 0.01 kg/m3；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t frequency_hz_x1000; /* 频率千倍定点值，单位为 0.001 Hz；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint32_t stream_seq; /* 该周期快报的流内序号，用于消费者识别丢帧和新旧。 */
    uint32_t effective_age_ms; /* 折算后的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
    uint16_t status_flags16; /* 16 位状态标志位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint16_t diag_flags16; /* 16 位诊断标志位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint8_t signal_quality; /* 传感器报告的信号质量指标；仍需结合状态位和数据年龄判定可用性。 */
} SensorSafePeriodicSnapshot;

typedef struct {
    /* 周期快报双缓冲监测上下文；发布者写非活动槽并原子切换索引，消费者只读取活动槽。 */
    SensorSafePeriodicSnapshot slots[2]; /* 周期快报双缓冲槽；发布者只能写入当前非活动槽。 */
    uint32_t rx_timestamp_ms[2]; /* 本地接收时间戳，使用单调毫秒节拍；仅可通过无符号差值比较超时。 */
    uint16_t wire_data_age_ms[2]; /* 双缓冲槽对应的远端数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
    SensorSafeResult invalid_reason; /* 最近一次周期快照被置无效时的具体本地校验结果。 */
    uint32_t publish_count; /* 成功发布快照累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t invalidate_count; /* 主动置无效累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint8_t active_slot; /* 消费者当前应读取的双缓冲槽索引；发布者只写另一个槽。 */
    uint8_t valid; /* active_slot 指向的周期快照当前通过完整性和新鲜度校验的标志。 */
} SensorSafeMonitorContext;

/**
 * @brief 初始化周期快照双缓冲及其失效原因。
 *
 * @details 调用场景：client 初始化和每次新周期流建立前调用。
 *
 * 初始化或重新进入上报流时清除旧快照，禁止跨 stream_id 复用。
 *
 * @note 关键约束：初始化后快照默认无效，必须接收并验收首帧后才能读取。
 *
 * @param context 周期快照监测上下文；保存双缓冲快照、接收时间、线端数据年龄、失效原因、发布与失效计数以及当前有效槽，更新时由临界区保护切换有效槽。
 */
void SensorSafeMonitor_Init(SensorSafeMonitorContext *context);

/**
 * @brief 把已通过 client 全部校验的快报发布到双缓冲快照。
 *
 * @details 调用场景：SensorSafeClient_PollPeriodic() 返回成功之后。
 * @note 关键约束：先完整写入非活动槽，再以单字节切换活动槽；不得发布未经会话层校验的帧；仅允许主循环单生产者调用，不得从 ISR 并发发布。
 */
SensorSafeResult SensorSafeMonitor_Publish(SensorSafeMonitorContext *context,
                                           const SensorSafeFastReport *report,
                                           uint32_t rx_timestamp_ms,
                                           uint32_t now_ms,
                                           uint32_t max_data_age_ms);

/**
 * @brief 读取当前周期上报安全快照并重新计算本地有效年龄。
 *
 * @details 调用场景：显式允许周期上报作为输入的主循环消费点。
 * @note 关键约束：过期或已失效会清零输出并撤销 valid；当前实现只保证同一主循环调度域内的发布/读取一致性，不作为跨 ISR 的无锁队列使用。
 */
SensorSafeResult SensorSafeMonitor_Read(SensorSafeMonitorContext *context,
                                        uint32_t now_ms,
                                        uint32_t max_data_age_ms,
                                        SensorSafePeriodicSnapshot *snapshot);

/**
 * @brief 撤销当前周期快照并保存首个可追溯失效原因。
 *
 * @details 调用场景：退出周期流、帧校验失败、数据过期或并发一致性失败时调用。
 *
 * 任一快报或退出异常都立即撤销快照，并保存首个可追溯失效原因。
 *
 * @note 关键约束：成功码不得成为失效原因；本函数不打印、不阻塞，可由接收链路调用。
 *
 * @param context 周期快照监测上下文；保存双缓冲快照、接收时间、线端数据年龄、失效原因、发布与失效计数以及当前有效槽，更新时由临界区保护切换有效槽。
 * @param reason 导致周期快照失效的安全协议结果码；保留 CRC、会话、序号、能力和数据有效性等具体协议原因。
 */
void SensorSafeMonitor_Invalidate(SensorSafeMonitorContext *context,
                                  SensorSafeResult reason);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_MONITOR_H_ */
