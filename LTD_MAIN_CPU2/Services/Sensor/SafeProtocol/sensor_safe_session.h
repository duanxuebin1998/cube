#ifndef SENSOR_SAFE_SESSION_H_
#define SENSOR_SAFE_SESSION_H_

#include "sensor_safe_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSOR_SAFE_SESSION_OFFLINE = 0,
    SENSOR_SAFE_SESSION_READY,
    SENSOR_SAFE_SESSION_MEASURE_READY,
    SENSOR_SAFE_SESSION_PERIODIC_ACTIVE,
    SENSOR_SAFE_SESSION_RECOVERING,
    SENSOR_SAFE_SESSION_FAULT
} SensorSafeSessionState;

typedef struct {
    uint8_t selected_protocol;
    uint8_t sensor_node_id;
    uint16_t sensor_type;
    uint32_t sensor_id;
    uint32_t sensor_boot_counter;
    uint32_t sensor_session_counter;
    uint32_t echo_cpu_boot_counter;
    uint32_t echo_cpu_nonce;
    uint32_t new_session_id;
    uint32_t capability_flags;
    uint16_t scale_version;
    uint16_t calibration_version;
    uint16_t param_table_version;
    uint16_t digest_schema_version;
    uint32_t safety_param_crc;
} SensorSafeHelloAcceptance;

typedef struct {
    SensorSafeSessionState state;
    uint8_t local_node_id;
    uint8_t remote_node_id;
    uint32_t session_id;
    uint32_t sensor_id;
    uint32_t safety_param_crc;
    uint32_t capability_flags;
    uint32_t next_seq;
    uint32_t pending_seq;
    uint8_t pending_cmd;
    uint8_t pending_active;
    uint32_t cpu_boot_counter;
    uint32_t cpu_nonce;
    uint8_t hello_active;
    uint32_t last_sensor_boot_counter;
    uint32_t last_sensor_session_counter;
    uint32_t last_session_id;
    uint8_t sensor_counter_valid;
    uint16_t stream_id;
    uint16_t config_epoch;
    uint32_t expected_stream_seq;
    uint32_t last_sample_counter;
    uint8_t sample_counter_valid;
    uint8_t expected_measure_mode;
    uint8_t stream_valid;
    uint32_t last_valid_rx_ms;
} SensorSafeSessionContext;

/*
 * 函数用途：初始化 CPU2 安全协议会话上下文。
 * 调用场景：上电、传感器协议切换或最终通信恢复入口。
 * 关键约束：初始化后只能发送 HELLO，旧会话和旧上报流全部失效。
 */
void SensorSafeSession_Init(SensorSafeSessionContext *context,
                            uint8_t local_node_id,
                            uint8_t remote_node_id);

/*
 * 函数用途：停用当前安全会话并清除事务、流和测量运行态。
 * 调用场景：协议回退、传感器类型切换或安全适配器主动停用。
 * 关键约束：保留本次 CPU2 启动内最近接受的传感器计数和会话号，后续 HELLO 仍必须执行跨停用边界防重放校验。
 */
void SensorSafeSession_Deactivate(SensorSafeSessionContext *context);

/*
 * 函数用途：登记新的 HELLO 挑战并分配事务号。
 * 调用场景：CPU2 准备编码 HELLO 请求之前。
 * 关键约束：新的 HELLO 必须使用新的 nonce；超时重发使用 RetryPending 保留原挑战。
 */
SensorSafeResult SensorSafeSession_BeginHello(SensorSafeSessionContext *context,
                                              uint32_t cpu_boot_counter,
                                              uint32_t cpu_nonce,
                                              uint32_t *seq_out);

/*
 * 函数用途：登记一个已建立会话的普通控制请求。
 * 调用场景：编码 SET_MEASURE_MODE、读数、参数或状态请求之前。
 * 关键约束：同一时刻只允许一个未完成请求，事务号预计回绕前要求重新 HELLO。
 */
SensorSafeResult SensorSafeSession_BeginRequest(SensorSafeSessionContext *context,
                                                uint8_t cmd,
                                                uint32_t *seq_out);

/*
 * 函数用途：取得当前未完成事务用于原序号重发。
 * 调用场景：控制请求超时后的有限次数重试。
 * 关键约束：重试只能恢复可用性，不延长旧安全数据有效期。
 */
SensorSafeResult SensorSafeSession_RetryPending(const SensorSafeSessionContext *context,
                                                uint8_t *cmd_out,
                                                uint32_t *seq_out);

/*
 * 函数用途：校验控制响应与当前唯一未完成事务是否精确匹配。
 * 调用场景：帧层 CRC 校验通过后、解析和使用响应负载之前。
 * 关键约束：HELLO 只做事务和地址预检，挑战回显由 AcceptHello 完成。
 */
SensorSafeResult SensorSafeSession_CheckControlResponse(const SensorSafeSessionContext *context,
                                                        const SensorSafeControlFrame *frame);

/*
 * 函数用途：接受通过挑战回显和计数防重放校验的新会话。
 * 调用场景：HELLO 响应字段全部解码完成后。
 * 关键约束：传感器启动计数不得回退，同一次启动的会话计数必须严格前进。
 */
SensorSafeResult SensorSafeSession_AcceptHello(SensorSafeSessionContext *context,
                                               const SensorSafeControlFrame *frame,
                                               const SensorSafeHelloAcceptance *hello);

/*
 * 函数用途：在业务响应字段全部校验完成后结束当前控制事务。
 * 调用场景：响应 result_code、状态、诊断和负载均已处理。
 * 关键约束：不得在负载校验前清除 pending，否则错误响应可能被后续帧掩盖。
 */
SensorSafeResult SensorSafeSession_CompleteRequest(SensorSafeSessionContext *context);

/*
 * 函数用途：记录传感器已明确确认的稳定测量模式。
 * 调用场景：SET_MEASURE_MODE 响应或有效测量响应通过模式与状态校验后。
 * 关键约束：只有 SESSION_READY/MEASURE_READY 可进入，不能绕过周期流或恢复状态。
 */
SensorSafeResult SensorSafeSession_SetMeasureReady(SensorSafeSessionContext *context,
                                                   uint8_t measure_mode);

/*
 * 函数用途：根据 SET_COMM_MODE 成功响应登记新上报流。
 * 调用场景：accepted_stream_id、accepted_config_epoch 和测量模式已确认后。
 * 关键约束：新流从 stream_seq=1 开始，旧流立即失效。
 */
SensorSafeResult SensorSafeSession_StartStream(SensorSafeSessionContext *context,
                                               uint16_t stream_id,
                                               uint16_t config_epoch,
                                               uint8_t measure_mode);

/*
 * 函数用途：在退出上报 ACK 通过后正常关闭当前流并保留会话。
 * 调用场景：SET_COMM_MODE(REQUEST_RESPONSE) 响应字段全部确认后。
 * 关键约束：旧 stream_id 立即失效，后续旧快报不得更新任何测量缓存。
 */
SensorSafeResult SensorSafeSession_StopStream(SensorSafeSessionContext *context);

/*
 * 函数用途：校验快报的会话、流、序号、样本、状态和年龄并推进连续性状态。
 * 调用场景：固定 44 字节快报已通过 CRC 和保留位校验后。
 * 关键约束：重复、回退、跳号或错配置会永久失效当前流，后续帧不得自行恢复。
 */
SensorSafeResult SensorSafeSession_AcceptFastReport(SensorSafeSessionContext *context,
                                                    const SensorSafeFastReport *report,
                                                    uint32_t rx_timestamp_ms,
                                                    uint32_t max_data_age_ms);

/*
 * 函数用途：立即撤销当前安全上报流并进入恢复状态。
 * 调用场景：快报 CRC、长度、静默、地址或其它关键校验失败。
 * 关键约束：恢复前必须重新 SET_COMM_MODE 或 HELLO，不能靠后续连续帧恢复。
 */
void SensorSafeSession_InvalidateStream(SensorSafeSessionContext *context);

/*
 * 函数用途：使用本地单调时钟计算饱和有效年龄。
 * 调用场景：快照被业务消费时，把线上年龄和本地排队时间相加。
 * 关键约束：加法饱和到 UINT32_MAX，禁止整数回绕生成伪造的新鲜数据。
 */
uint32_t SensorSafeSession_EffectiveAgeMs(uint16_t data_age_ms,
                                          uint32_t rx_timestamp_ms,
                                          uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_SESSION_H_ */
