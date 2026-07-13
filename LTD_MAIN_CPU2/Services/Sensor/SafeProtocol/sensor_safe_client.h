#ifndef SENSOR_SAFE_CLIENT_H_
#define SENSOR_SAFE_CLIENT_H_

#include "sensor_safe_session.h"
#include "sensor_safe_transport_uart6.h"

#ifdef __cplusplus
extern "C" {
#endif

/* client 默认时序和周期边界；实际周期值还必须由传感器响应回显确认。 */
#define SENSOR_SAFE_CLIENT_DEFAULT_TIMEOUT_MS       300U
#define SENSOR_SAFE_CLIENT_MODE_TIMEOUT_MS          1000U
#define SENSOR_SAFE_CLIENT_DEFAULT_MAX_ATTEMPTS     3U
#define SENSOR_SAFE_CLIENT_MIN_PERIOD_MS            500U
#define SENSOR_SAFE_CLIENT_MAX_PERIOD_MS            5000U
#define SENSOR_SAFE_CLIENT_MIN_CONTROL_GUARD_MS     100U
#define SENSOR_SAFE_CLIENT_EXIT_REQUEST_BUDGET_MS   60U

/* client 对服务层暴露的稳定结果分类，具体协议/传输原因保存在 context 中。 */
typedef enum {
    SENSOR_SAFE_CLIENT_OK = 0,
    SENSOR_SAFE_CLIENT_INVALID_ARGUMENT,
    SENSOR_SAFE_CLIENT_TRANSPORT_ERROR,
    SENSOR_SAFE_CLIENT_PROTOCOL_ERROR,
    SENSOR_SAFE_CLIENT_REMOTE_ERROR,
    SENSOR_SAFE_CLIENT_DATA_INVALID,
    SENSOR_SAFE_CLIENT_DATA_STALE,
    SENSOR_SAFE_CLIENT_NEEDS_HELLO,
    SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY,
    SENSOR_SAFE_CLIENT_CONTROL_WINDOW_CLOSED,
    SENSOR_SAFE_CLIENT_BUFFER_TOO_SMALL
} SensorSafeClientResult;

/* 平台传输函数表：exchange 用于控制事务，receive 仅用于周期主动上报。 */
typedef SensorSafeTransportResult (*SensorSafeClientExchangeFn)(const uint8_t *request,
                                                                uint16_t request_len,
                                                                uint8_t *response,
                                                                size_t response_capacity,
                                                                uint16_t *response_len,
                                                                uint32_t *rx_timestamp_ms,
                                                                uint32_t response_timeout_ms);
typedef SensorSafeTransportResult (*SensorSafeClientReceiveFn)(uint8_t *frame,
                                                               size_t frame_capacity,
                                                               uint16_t *frame_len,
                                                               uint32_t *rx_timestamp_ms,
                                                               uint32_t timeout_ms);
typedef uint32_t (*SensorSafeClientNowMsFn)(void);

/* client 依赖注入接口；三个回调均在任务上下文同步调用。 */
typedef struct {
    SensorSafeClientExchangeFn exchange;
    SensorSafeClientReceiveFn receive;
    SensorSafeClientNowMsFn now_ms;
} SensorSafeClientTransportOps;

/* client 静态配置；cpu_boot_counter 来自身份双副本初始化结果。 */
typedef struct {
    uint8_t cpu_node_id;
    uint8_t sensor_node_id;
    uint32_t cpu_boot_counter;
    uint32_t max_data_age_ms;
    uint32_t request_timeout_ms;
    uint8_t max_attempts;
} SensorSafeClientConfig;

/* HELLO 验收后的传感器身份、版本、能力和配置摘要。 */
typedef struct {
    uint16_t sensor_type;
    uint32_t sensor_id;
    uint32_t sensor_sw_version;
    uint32_t sensor_hw_version;
    uint32_t sensor_boot_counter;
    uint32_t sensor_session_counter;
    uint32_t capability_flags;
    uint16_t scale_version;
    uint16_t calibration_version;
    uint16_t param_table_version;
    uint16_t digest_schema_version;
    uint32_t safety_param_crc;
} SensorSafeIdentity;

/* GET_STATUS 的公共状态及实际接收时间。 */
typedef struct {
    SensorSafeCommonResponse common;
    uint32_t rx_timestamp_ms;
} SensorSafeStatus;

/* 配置摘要用于把会话绑定到参数 CRC、版本和配置代次。 */
typedef struct {
    SensorSafeCommonResponse common;
    uint32_t safety_param_crc;
    uint32_t capability_flags;
    uint16_t scale_version;
    uint16_t calibration_version;
    uint16_t param_table_version;
    uint16_t digest_schema_version;
    uint16_t config_epoch;
} SensorSafeConfigDigest;

/* 密度组合测量，所有数值保持协议定点单位以避免 client 内浮点误差。 */
typedef struct {
    int32_t temperature_c_x100;
    int32_t density_kg_m3_x100;
    uint32_t frequency_hz_x1000;
    uint32_t freq_45_hz_x1000;
    uint32_t freq_225_hz_x1000;
    uint16_t signal_quality;
    uint32_t sample_counter;
    uint32_t effective_age_ms;
} SensorSafeMeasurementCombo;

/* 液位通道测量及传感器声明的有效频率范围。 */
typedef struct {
    uint32_t level_frequency_hz_x1000;
    uint16_t signal_quality;
    uint16_t valid_min_hz;
    uint16_t valid_max_hz;
    uint32_t sample_counter;
    uint32_t effective_age_ms;
} SensorSafeLevelMeasurement;

/* 水位电容测量，单位为 0.1 pF。 */
typedef struct {
    int32_t capacitance_pf_x10;
    uint16_t signal_quality;
    uint32_t sample_counter;
    uint32_t effective_age_ms;
} SensorSafeWaterCapMeasurement;

/* 双轴姿态测量，角度单位为 0.01 度。 */
typedef struct {
    int32_t angle_x_deg_x100;
    int32_t angle_y_deg_x100;
    uint16_t gyro_status;
    uint32_t sample_counter;
    uint32_t effective_age_ms;
} SensorSafeGyroMeasurement;

/* 电源诊断测量；不直接驱动整机安全状态。 */
typedef struct {
    uint32_t voltage_mv;
    uint16_t power_status;
    uint32_t sample_counter;
    uint32_t effective_age_ms;
} SensorSafePowerMeasurement;

/* 参数线格式的规范化表示，param_crc 覆盖版本和全部解释字段。 */
typedef struct {
    uint16_t param_id;
    uint16_t param_index;
    uint8_t param_type;
    uint8_t access;
    int16_t scale;
    uint16_t unit_id;
    int64_t value_i64;
    uint32_t param_crc;
} SensorSafeParameterValue;

/* 自检任务结果；test_id 用于防止读取到其他任务的历史结果。 */
typedef struct {
    uint32_t test_id;
    uint16_t test_result;
    uint16_t failed_item;
    uint32_t detail_code;
} SensorSafeSelfTestResult;

/* client 累计诊断计数，全部采用饱和计数而不自动清零。 */
typedef struct {
    uint32_t control_request_count;
    uint32_t control_attempt_count;
    uint32_t control_retry_count;
    uint32_t control_response_count;
    uint32_t transport_failure_count;
    uint32_t protocol_reject_count;
    uint32_t crc_reject_count;
    uint32_t address_reject_count;
    uint32_t remote_error_count;
    uint32_t fast_report_receive_count;
    uint32_t fast_report_accept_count;
} SensorSafeClientCounters;

/*
 * client 完整运行态。
 * 会话、防重放、周期协商、诊断计数和固定缓冲区均由单一任务串行访问，接口不可重入。
 */
typedef struct {
    SensorSafeSessionContext session;
    SensorSafeClientConfig config;
    SensorSafeClientTransportOps transport;
    SensorSafeIdentity identity;
    SensorSafeTransportResult last_transport_result;
    SensorSafeResult last_protocol_result;
    uint16_t last_wire_result;
    uint32_t last_rx_timestamp_ms;
    uint16_t accepted_period_ms;
    uint16_t accepted_max_silent_ms;
    uint16_t accepted_control_guard_ms;
    uint16_t accepted_start_after_ms;
    uint32_t periodic_start_ms;
    SensorSafeClientCounters counters;
    uint32_t last_control_sample[6];
    uint8_t last_control_sample_valid[6];
    uint8_t tx_buffer[SENSOR_SAFE_CTRL_MAX_FRAME_LEN];
    uint8_t rx_buffer[SENSOR_SAFE_UART6_RX_BUFFER_SIZE];
} SensorSafeClientContext;

/* 初始化与下线只管理本地运行态，不发送总线命令。 */
void SensorSafeClient_Init(SensorSafeClientContext *context,
                           const SensorSafeClientConfig *config,
                           const SensorSafeClientTransportOps *transport);
void SensorSafeClient_Deactivate(SensorSafeClientContext *context);

/* 会话与基础诊断接口。 */
SensorSafeClientResult SensorSafeClient_Hello(SensorSafeClientContext *context,
                                              uint32_t cpu_nonce,
                                              SensorSafeIdentity *identity_out);
SensorSafeClientResult SensorSafeClient_Ping(SensorSafeClientContext *context,
                                             uint32_t token,
                                             uint32_t *sensor_uptime_ms);
SensorSafeClientResult SensorSafeClient_GetStatus(SensorSafeClientContext *context,
                                                  SensorSafeStatus *status_out);
SensorSafeClientResult SensorSafeClient_GetConfigDigest(SensorSafeClientContext *context,
                                                        SensorSafeConfigDigest *digest_out);
SensorSafeClientResult SensorSafeClient_ResetSession(SensorSafeClientContext *context,
                                                     uint16_t reset_reason);
/* 请求响应测量接口；输出仅在返回 SENSOR_SAFE_CLIENT_OK 时有效。 */
SensorSafeClientResult SensorSafeClient_SetMeasureMode(SensorSafeClientContext *context,
                                                       uint8_t target_mode,
                                                       uint16_t *settle_time_ms);
SensorSafeClientResult SensorSafeClient_ReadMeasurementCombo(SensorSafeClientContext *context,
                                                             SensorSafeMeasurementCombo *measurement);
SensorSafeClientResult SensorSafeClient_ReadLevelFrequency(SensorSafeClientContext *context,
                                                          SensorSafeLevelMeasurement *measurement);
SensorSafeClientResult SensorSafeClient_ReadWaterCap(SensorSafeClientContext *context,
                                                     SensorSafeWaterCapMeasurement *measurement);
SensorSafeClientResult SensorSafeClient_ReadGyro(SensorSafeClientContext *context,
                                                 SensorSafeGyroMeasurement *measurement);
SensorSafeClientResult SensorSafeClient_ReadPower(SensorSafeClientContext *context,
                                                  SensorSafePowerMeasurement *measurement);
/* 周期主动上报接口；进入后只允许受控停流和会话维护命令。 */
SensorSafeClientResult SensorSafeClient_StartPeriodic(SensorSafeClientContext *context,
                                                      uint16_t period_ms,
                                                      uint16_t max_silent_ms,
                                                      uint16_t stream_id,
                                                      uint16_t control_guard_ms,
                                                      uint8_t measure_mode,
                                                      uint16_t *start_after_ms);
SensorSafeClientResult SensorSafeClient_PollPeriodic(SensorSafeClientContext *context,
                                                     SensorSafeFastReport *report,
                                                     uint32_t timeout_ms);
SensorSafeClientResult SensorSafeClient_StopPeriodic(SensorSafeClientContext *context);
/* 参数读写为受控维护能力；COMMIT 成功后必须重新 HELLO。 */
SensorSafeClientResult SensorSafeClient_ReadParam(SensorSafeClientContext *context,
                                                  uint16_t param_id,
                                                  uint16_t param_index,
                                                  SensorSafeParameterValue *value_out);
/* 一次逻辑调用自动分页读取完整参数表；只有返回 OK 时输出数组整体有效。 */
SensorSafeClientResult SensorSafeClient_ReadAllParams(SensorSafeClientContext *context,
                                                      SensorSafeParameterValue *values,
                                                      uint16_t value_capacity,
                                                      uint16_t *value_count_out);
SensorSafeClientResult SensorSafeClient_WriteParam(SensorSafeClientContext *context,
                                                   const SensorSafeParameterValue *value,
                                                   uint8_t access_guard,
                                                   uint32_t expected_param_crc,
                                                   uint32_t *new_param_crc);
SensorSafeClientResult SensorSafeClient_CommitParam(SensorSafeClientContext *context,
                                                    uint16_t commit_scope,
                                                    uint32_t expected_safety_param_crc,
                                                    uint32_t *new_safety_param_crc);
/* 自检接口通过 test_id 关联启动与结果读取。 */
SensorSafeClientResult SensorSafeClient_RunSelfTest(SensorSafeClientContext *context,
                                                    uint32_t test_mask,
                                                    uint16_t timeout_ms,
                                                    uint32_t *test_id,
                                                    uint16_t *estimated_time_ms);
SensorSafeClientResult SensorSafeClient_ReadSelfTestResult(SensorSafeClientContext *context,
                                                           uint32_t test_id,
                                                           SensorSafeSelfTestResult *result_out);
/* 读取累计诊断计数，不修改 client 状态。 */
void SensorSafeClient_GetCounters(const SensorSafeClientContext *context,
                                  SensorSafeClientCounters *counters_out);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_CLIENT_H_ */
