#ifndef SENSOR_SAFE_MONITOR_H_
#define SENSOR_SAFE_MONITOR_H_

#include "sensor_safe_session.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t temperature_c_x100;
    int32_t density_kg_m3_x100;
    uint32_t frequency_hz_x1000;
    uint32_t sample_counter;
    uint32_t stream_seq;
    uint32_t effective_age_ms;
    uint16_t status_flags16;
    uint16_t diag_flags16;
    uint8_t signal_quality;
} SensorSafePeriodicSnapshot;

typedef struct {
    SensorSafePeriodicSnapshot slots[2];
    uint32_t rx_timestamp_ms[2];
    uint16_t wire_data_age_ms[2];
    SensorSafeResult invalid_reason;
    uint32_t publish_count;
    uint32_t invalidate_count;
    uint8_t active_slot;
    uint8_t valid;
} SensorSafeMonitorContext;

/* 初始化或重新进入上报流时清除旧快照，禁止跨 stream_id 复用。 */
void SensorSafeMonitor_Init(SensorSafeMonitorContext *context);

/*
 * 函数用途：把已通过 client 全部校验的快报发布到双缓冲快照。
 * 调用场景：SensorSafeClient_PollPeriodic() 返回成功之后。
 * 关键约束：先完整写入非活动槽，再以单字节切换活动槽；不得发布未经会话层校验的帧；仅允许主循环单生产者调用，不得从 ISR 并发发布。
 */
SensorSafeResult SensorSafeMonitor_Publish(SensorSafeMonitorContext *context,
                                           const SensorSafeFastReport *report,
                                           uint32_t rx_timestamp_ms,
                                           uint32_t now_ms,
                                           uint32_t max_data_age_ms);

/*
 * 函数用途：读取当前周期上报安全快照并重新计算本地有效年龄。
 * 调用场景：显式允许周期上报作为输入的主循环消费点。
 * 关键约束：过期或已失效会清零输出并撤销 valid；当前实现只保证同一主循环调度域内的发布/读取一致性，不作为跨 ISR 的无锁队列使用。
 */
SensorSafeResult SensorSafeMonitor_Read(SensorSafeMonitorContext *context,
                                        uint32_t now_ms,
                                        uint32_t max_data_age_ms,
                                        SensorSafePeriodicSnapshot *snapshot);

/* 任一快报或退出异常都立即撤销快照，并保存首个可追溯失效原因。 */
void SensorSafeMonitor_Invalidate(SensorSafeMonitorContext *context,
                                  SensorSafeResult reason);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_MONITOR_H_ */
