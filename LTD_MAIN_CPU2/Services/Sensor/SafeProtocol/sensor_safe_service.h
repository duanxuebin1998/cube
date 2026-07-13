#ifndef SENSOR_SAFE_SERVICE_H_
#define SENSOR_SAFE_SERVICE_H_

#include "sensor_safe_client.h"
#include "sensor_safe_identity.h"
#include "sensor_safe_monitor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_SAFE_SERVICE_CPU_NODE_ID          0x01U
#define SENSOR_SAFE_SERVICE_SENSOR_NODE_ID       0x10U
#define SENSOR_SAFE_SERVICE_MAX_DATA_AGE_MS      500U
#define SENSOR_SAFE_SERVICE_REQUIRED_CAPABILITIES \
    (SENSOR_SAFE_CAP_DENSITY | SENSOR_SAFE_CAP_LEVEL | SENSOR_SAFE_CAP_MEAS_COMBO)

typedef enum {
    SENSOR_SAFE_SERVICE_OK = 0,
    SENSOR_SAFE_SERVICE_INVALID_ARGUMENT,
    SENSOR_SAFE_SERVICE_IDENTITY_ERROR,
    SENSOR_SAFE_SERVICE_TRANSPORT_ERROR,
    SENSOR_SAFE_SERVICE_PROTOCOL_ERROR,
    SENSOR_SAFE_SERVICE_REMOTE_ERROR,
    SENSOR_SAFE_SERVICE_DATA_INVALID,
    SENSOR_SAFE_SERVICE_DATA_STALE,
    SENSOR_SAFE_SERVICE_UNSUPPORTED,
    SENSOR_SAFE_SERVICE_SESSION_ERROR,
    SENSOR_SAFE_SERVICE_BUFFER_TOO_SMALL
} SensorSafeServiceResult;

typedef struct {
    SensorSafeIdentityContext startup_identity;
    SensorSafeClientContext client;
    SensorSafeIdentity identity;
    SensorSafeConfigDigest config_digest;
    SensorSafeMonitorContext monitor;
    SensorSafeIdentityResult last_identity_result;
    SensorSafeClientResult last_client_result;
    SensorSafeNonceSource last_nonce_source;
    uint32_t hello_count;
    uint32_t recovery_attempt_count;
    uint32_t recovery_count;
    uint32_t recovery_blocked_count;
    SensorSafeClientResult last_recovery_trigger;
    SensorSafeTransportResult last_recovery_transport_result;
    SensorSafeResult last_recovery_protocol_result;
    uint16_t last_recovery_wire_result;
    uint8_t initialized;
    uint8_t active;
} SensorSafeServiceContext;

/*
 * 函数用途：初始化安全传感器服务、持久化启动计数并绑定传输回调。
 * 调用场景：FRAM、UART6 和 CH9141 完成基础初始化后，首次协议探测之前。
 * 关键约束：同一 CPU2 启动周期只允许初始化一次，重复调用不会再次递增启动计数。
 */
SensorSafeServiceResult SensorSafeService_Init(SensorSafeServiceContext *context,
                                               const SensorSafeIdentityOps *identity_ops,
                                               const SensorSafeClientTransportOps *transport_ops);

/*
 * 函数用途：通过新挑战执行 HELLO 并激活安全协议身份。
 * 调用场景：传感器识别时优先于 LTD/DSM 探测调用。
 * 关键约束：失败只撤销安全协议活动状态，调用方仍可继续探测旧协议。
 */
SensorSafeServiceResult SensorSafeService_Probe(SensorSafeServiceContext *context,
                                                uint32_t *sensor_id);

/* 撤销安全协议运行态，不修改 FRAM 启动计数和旧协议实现。 */
void SensorSafeService_Deactivate(SensorSafeServiceContext *context);
uint8_t SensorSafeService_IsActive(const SensorSafeServiceContext *context);

/* 切换安全传感器测量模式并返回传感器声明的稳定等待时间。 */
SensorSafeServiceResult SensorSafeService_SetMeasureMode(SensorSafeServiceContext *context,
                                                         uint8_t mode,
                                                         uint16_t *settle_time_ms);

/* 读取并换算密度组合数据，输出单位分别为 Hz、kg/m3 和 degC。 */
SensorSafeServiceResult SensorSafeService_ReadDensity(SensorSafeServiceContext *context,
                                                      float *frequency_hz,
                                                      float *density_kg_m3,
                                                      float *temperature_c);

/* 读取液位频率并按最接近整数 Hz 输出。 */
SensorSafeServiceResult SensorSafeService_ReadLevelFrequency(SensorSafeServiceContext *context,
                                                             uint32_t *frequency_hz);

/* 读取水位电容和姿态角，保持现有业务接口单位。 */
SensorSafeServiceResult SensorSafeService_ReadWaterCapacitance(SensorSafeServiceContext *context,
                                                               float *capacitance_pf);
SensorSafeServiceResult SensorSafeService_ReadGyroAngle(SensorSafeServiceContext *context,
                                                        float *angle_x_deg,
                                                        float *angle_y_deg);

/*
 * 对上提供一次调用读取固定 115 项 LTD 参数表，底层分页过程不暴露给业务层。
 * value_capacity 小于 115 时不发起通信，并通过 value_count_out 返回所需容量。
 */
SensorSafeServiceResult SensorSafeService_ReadAllParams(SensorSafeServiceContext *context,
                                                        SensorSafeParameterValue *values,
                                                        uint16_t value_capacity,
                                                        uint16_t *value_count_out);

/* 周期上报只在显式调用后启用，默认请求响应业务不会自动进入该模式。 */
SensorSafeServiceResult SensorSafeService_StartPeriodic(SensorSafeServiceContext *context,
                                                        uint16_t period_ms,
                                                        uint16_t max_silent_ms,
                                                        uint16_t stream_id,
                                                        uint16_t control_guard_ms,
                                                        uint8_t measure_mode,
                                                        uint16_t *start_after_ms);
SensorSafeServiceResult SensorSafeService_PollPeriodic(SensorSafeServiceContext *context,
                                                       uint32_t timeout_ms);
SensorSafeServiceResult SensorSafeService_ReadPeriodicSnapshot(SensorSafeServiceContext *context,
                                                               SensorSafePeriodicSnapshot *snapshot);
SensorSafeServiceResult SensorSafeService_StopPeriodic(SensorSafeServiceContext *context);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_SERVICE_H_ */
