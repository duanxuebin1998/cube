#include "sensor_safe_monitor.h"

#include <string.h>

/**
 * @brief 诊断计数饱和保持，避免长稳运行后回绕造成错误次数倒退。
 *
 * @param counter 本次节拍、重试或统计使用的计数值。指针非 NULL 且当前值未达到 UINT32_MAX 时原地加一，达到上限后饱和保持。
 */
static void SensorSafeMonitor_IncrementCounter(uint32_t *counter)
{
    if ((counter != NULL) && (*counter != UINT32_MAX)) {
        (*counter)++;
    }
}

/**
 * @brief 安全快照再次核对协议内在状态，避免绕过会话层直接发布无效快报。
 *
 * @param report 用于接收本次诊断或测量结果的报告对象。
 * @return 1 表示测量模式、周期上报模式、信号质量、状态位和诊断位均合法，且数据有效、新鲜、模式匹配、数据流活动并且对应通道无故障；0 表示任一字段越界或含保留位、数据无效或陈旧、配置或模式不匹配，或存在快速帧或通道诊断故障。
 */
static uint8_t SensorSafeMonitor_IsReportDataValid(const SensorSafeFastReport *report)
{
    uint16_t channel_diag_mask = 0U;

    switch (report->measure_mode) {
    case (uint8_t)SENSOR_SAFE_MEASURE_DENSITY:
        channel_diag_mask = (uint16_t)SENSOR_SAFE_DIAG_DENSITY_CHANNEL_ERROR;
        break;
    case (uint8_t)SENSOR_SAFE_MEASURE_LEVEL:
        channel_diag_mask = (uint16_t)SENSOR_SAFE_DIAG_LEVEL_CHANNEL_ERROR;
        break;
    case (uint8_t)SENSOR_SAFE_MEASURE_WATER_CAP:
        channel_diag_mask = (uint16_t)SENSOR_SAFE_DIAG_WATER_CAP_ERROR;
        break;
    case (uint8_t)SENSOR_SAFE_MEASURE_GYRO:
        channel_diag_mask = (uint16_t)SENSOR_SAFE_DIAG_GYRO_CHANNEL_ERROR;
        break;
    default:
        break;
    }

    if ((report->measure_mode > (uint8_t)SENSOR_SAFE_MEASURE_BOTTOM) ||
        (report->comm_mode != (uint8_t)SENSOR_SAFE_COMM_PERIODIC_UPLINK) ||
        (report->signal_quality > 100U) ||
        ((report->status_flags16 & (uint16_t)(~SENSOR_SAFE_ALLOWED_FAST_STATUS16)) != 0U) ||
        ((report->diag_flags16 & (uint16_t)(~SENSOR_SAFE_ALLOWED_FAST_DIAG16)) != 0U)) {
        return 0U;
    }
    if (((report->status_flags16 & SENSOR_SAFE_STATUS_DATA_VALID) == 0U) ||
        ((report->status_flags16 & SENSOR_SAFE_STATUS_DATA_STALE) != 0U) ||
        ((report->status_flags16 & SENSOR_SAFE_STATUS_MODE_MATCH) == 0U) ||
        ((report->status_flags16 & SENSOR_SAFE_STATUS_MODE_SETTLING) != 0U) ||
        ((report->status_flags16 & SENSOR_SAFE_STATUS_CONFIG_MISMATCH) != 0U) ||
        ((report->status_flags16 & SENSOR_SAFE_STATUS_FAST_INVALID_MASK) != 0U) ||
        ((report->status_flags16 & SENSOR_SAFE_STATUS_STREAM_ACTIVE) == 0U) ||
        ((report->diag_flags16 &
          (uint16_t)((uint16_t)SENSOR_SAFE_DIAG_FAST_INVALID_MASK | channel_diag_mask)) != 0U)) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 初始化周期快照双缓冲及其失效原因。
 *
 * @details 调用场景：client 初始化和每次新周期流建立前调用。
 * @note 关键约束：初始化后快照默认无效，必须接收并验收首帧后才能读取。
 *
 * @param context 周期快照监测上下文；保存双缓冲快照、接收时间、线端数据年龄、失效原因、发布与失效计数以及当前有效槽，更新时由临界区保护切换有效槽。
 */
void SensorSafeMonitor_Init(SensorSafeMonitorContext *context)
{
    if (context == NULL) {
        return;
    }
    (void)memset(context, 0, sizeof(*context));
    context->invalid_reason = SENSOR_SAFE_NEEDS_HELLO;
}

/**
 * @brief 复核快报数据、新鲜度并原子式发布到非活动快照槽。
 *
 * @details 调用场景：周期帧通过线格式和会话校验后调用。
 * @note 关键约束：任何状态、诊断或年龄异常都撤销旧快照，禁止沿用上一有效值。
 *
 * @param context 周期快照监测上下文；保存双缓冲快照、接收时间、线端数据年龄、失效原因、发布与失效计数以及当前有效槽，更新时由临界区保护切换有效槽。
 * @param report 用于接收本次诊断或测量结果的报告对象。
 * @param rx_timestamp_ms 完整接收本帧时记录的本机 HAL 毫秒节拍；作为指针传入时由函数写回。
 * @param now_ms 当前系统节拍，单位 ms。
 * @param max_data_age_ms 允许样本保持有效的最大数据年龄，单位 ms。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeMonitor_Publish(SensorSafeMonitorContext *context,
                                           const SensorSafeFastReport *report,
                                           uint32_t rx_timestamp_ms,
                                           uint32_t now_ms,
                                           uint32_t max_data_age_ms)
{
    SensorSafePeriodicSnapshot *target;
    uint32_t effective_age_ms;
    uint8_t next_slot;

    if ((context == NULL) || (report == NULL) || (max_data_age_ms == 0U)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (SensorSafeMonitor_IsReportDataValid(report) == 0U) {
        SensorSafeMonitor_Invalidate(context, SENSOR_SAFE_DATA_INVALID);
        return SENSOR_SAFE_DATA_INVALID;
    }
    effective_age_ms = SensorSafeSession_EffectiveAgeMs(report->data_age_ms,
                                                        rx_timestamp_ms,
                                                        now_ms);
    if (effective_age_ms > max_data_age_ms) {
        SensorSafeMonitor_Invalidate(context, SENSOR_SAFE_DATA_STALE);
        return SENSOR_SAFE_DATA_STALE;
    }
    /* 先完整写非活动槽，最后一次写 active_slot 才向读取侧发布新快照。 */
    next_slot = (uint8_t)(context->active_slot ^ 1U);
    target = &context->slots[next_slot];
    target->temperature_c_x100 = report->temperature_c_x100;
    target->density_kg_m3_x100 = report->density_kg_m3_x100;
    target->frequency_hz_x1000 = report->frequency_hz_x1000;
    target->sample_counter = report->sample_counter;
    target->stream_seq = report->stream_seq;
    target->effective_age_ms = effective_age_ms;
    target->status_flags16 = report->status_flags16;
    target->diag_flags16 = report->diag_flags16;
    target->signal_quality = report->signal_quality;
    context->rx_timestamp_ms[next_slot] = rx_timestamp_ms;
    context->wire_data_age_ms[next_slot] = report->data_age_ms;
    context->active_slot = next_slot;
    SensorSafeMonitor_IncrementCounter(&context->publish_count);
    context->invalid_reason = SENSOR_SAFE_OK;
    context->valid = 1U;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 读取一致的周期快照，并按当前时刻重新计算有效数据年龄。
 *
 * @details 调用场景：业务层获取最新周期测量值时调用。
 * @note 关键约束：检测到并发翻转两次或数据过期时清零输出并使快照失效。
 *
 * @param context 周期快照监测上下文；保存双缓冲快照、接收时间、线端数据年龄、失效原因、发布与失效计数以及当前有效槽，更新时由临界区保护切换有效槽。
 * @param now_ms 当前系统节拍，单位 ms。
 * @param max_data_age_ms 允许样本保持有效的最大数据年龄，单位 ms。
 * @param snapshot 安全协议周期数据快照输出对象；成功时写入流标识、样本序号、测量数据、传感器数据年龄和信号质量。
 * @return 返回安全协议校验结果；SENSOR_SAFE_OK 表示帧校验或状态转换成功，其他值保留参数、长度、帧头、CRC、会话、序号、能力及数据有效性等具体失败原因。
 */
SensorSafeResult SensorSafeMonitor_Read(SensorSafeMonitorContext *context,
                                        uint32_t now_ms,
                                        uint32_t max_data_age_ms,
                                        SensorSafePeriodicSnapshot *snapshot)
{
    uint32_t effective_age_ms;
    uint8_t slot_before;
    uint8_t slot_after;

    if ((context == NULL) || (snapshot == NULL) || (max_data_age_ms == 0U)) {
        return SENSOR_SAFE_INVALID_ARGUMENT;
    }
    if (context->valid == 0U) {
        (void)memset(snapshot, 0, sizeof(*snapshot));
        return context->invalid_reason;
    }
    /* 双读活动槽；一次翻转允许重取，连续翻转说明无法获得一致快照。 */
    slot_before = context->active_slot;
    *snapshot = context->slots[slot_before];
    slot_after = context->active_slot;
    if (slot_before != slot_after) {
        slot_before = slot_after;
        *snapshot = context->slots[slot_before];
        if (slot_before != context->active_slot) {
            SensorSafeMonitor_Invalidate(context, SENSOR_SAFE_DATA_INVALID);
            (void)memset(snapshot, 0, sizeof(*snapshot));
            return SENSOR_SAFE_DATA_INVALID;
        }
    }
    effective_age_ms = SensorSafeSession_EffectiveAgeMs(context->wire_data_age_ms[slot_before],
                                                        context->rx_timestamp_ms[slot_before],
                                                        now_ms);
    if (effective_age_ms > max_data_age_ms) {
        SensorSafeMonitor_Invalidate(context, SENSOR_SAFE_DATA_STALE);
        (void)memset(snapshot, 0, sizeof(*snapshot));
        return SENSOR_SAFE_DATA_STALE;
    }
    snapshot->effective_age_ms = effective_age_ms;
    return SENSOR_SAFE_OK;
}

/**
 * @brief 撤销当前周期快照并保存首个可追溯失效原因。
 *
 * @details 调用场景：退出周期流、帧校验失败、数据过期或并发一致性失败时调用。
 * @note 关键约束：成功码不得成为失效原因；本函数不打印、不阻塞，可由接收链路调用。
 *
 * @param context 周期快照监测上下文；保存双缓冲快照、接收时间、线端数据年龄、失效原因、发布与失效计数以及当前有效槽，更新时由临界区保护切换有效槽。
 * @param reason 导致周期快照失效的安全协议结果码；保留 CRC、会话、序号、能力和数据有效性等具体协议原因。
 */
void SensorSafeMonitor_Invalidate(SensorSafeMonitorContext *context,
                                  SensorSafeResult reason)
{
    if (context == NULL) {
        return;
    }
    if ((context->valid != 0U) || (context->invalid_reason == SENSOR_SAFE_NEEDS_HELLO)) {
        context->invalid_reason = (reason == SENSOR_SAFE_OK) ? SENSOR_SAFE_DATA_INVALID : reason;
    }
    context->valid = 0U;
    SensorSafeMonitor_IncrementCounter(&context->invalidate_count);
}
