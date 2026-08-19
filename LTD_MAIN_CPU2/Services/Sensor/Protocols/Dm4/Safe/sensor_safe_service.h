/*
 * 模块职责：声明Safe服务配置、运行上下文、诊断和密度/液位等高层事务接口。
 * 调用边界：旧业务适配器只依赖本层，不直接组合client、session或monitor内部步骤。
 * 运行约束：服务上下文由调用方长期持有；初始化失败时不得继续发起安全测量。
 */
#ifndef SENSOR_SAFE_SERVICE_H_
/* SENSOR_SAFE_SERVICE_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_SERVICE_H_

#include "sensor_safe_client.h"
#include "sensor_safe_identity.h"
#include "sensor_safe_monitor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 安全协议中 CPU2 主节点 ID 0x01；用于控制帧源/目的地址校验。 */
#define SENSOR_SAFE_SERVICE_CPU_NODE_ID          0x01U
/* 安全协议中传感器节点 ID 0x10；不匹配的目标帧必须丢弃。 */
#define SENSOR_SAFE_SERVICE_SENSOR_NODE_ID       0x10U
/* CPU2 接受安全传感器快报的最大数据年龄 500 ms；超过后即使 CRC 正确也必须按过期数据处理。 */
#define SENSOR_SAFE_SERVICE_MAX_DATA_AGE_MS      500U
/* 当前 CPU2 安全业务启动所必需的能力位合集：密度、液位和组合测量；HELLO 缺少任一位都不能进入正常周期服务。 */
#define SENSOR_SAFE_SERVICE_REQUIRED_CAPABILITIES \
    (SENSOR_SAFE_CAP_DENSITY | SENSOR_SAFE_CAP_LEVEL | SENSOR_SAFE_CAP_MEAS_COMBO)

/* 安全传感器服务层结果；把身份、会话、传输、协议、远端和数据质量错误统一返回给业务适配层。 */
typedef enum {
    /* 安全传感器服务层对业务调用返回的结果。 */
    SENSOR_SAFE_SERVICE_OK = 0, /* 服务层调用成功。 */
    SENSOR_SAFE_SERVICE_INVALID_ARGUMENT, /* 服务上下文或调用参数非法。 */
    SENSOR_SAFE_SERVICE_IDENTITY_ERROR, /* 启动身份或 nonce 处理失败。 */
    SENSOR_SAFE_SERVICE_TRANSPORT_ERROR, /* 底层 UART6 传输失败。 */
    SENSOR_SAFE_SERVICE_PROTOCOL_ERROR, /* 本地安全协议校验失败。 */
    SENSOR_SAFE_SERVICE_REMOTE_ERROR, /* 远端返回错误结果码。 */
    SENSOR_SAFE_SERVICE_DATA_INVALID, /* 远端数据状态表明数据无效。 */
    SENSOR_SAFE_SERVICE_DATA_STALE, /* 远端数据超过允许年龄。 */
    SENSOR_SAFE_SERVICE_UNSUPPORTED, /* 远端能力不支持所请求服务。 */
    SENSOR_SAFE_SERVICE_SESSION_ERROR, /* 当前会话状态不允许该服务。 */
    SENSOR_SAFE_SERVICE_BUFFER_TOO_SMALL /* 调用方输出缓冲区不足。 */
} SensorSafeServiceResult;

typedef struct {
    /* 安全传感器服务总上下文；聚合启动身份、客户端、配置摘要、周期监测和恢复诊断。 */
    SensorSafeIdentityContext startup_identity; /* CPU2 启动计数和 nonce 生成使用的身份管理上下文。 */
    SensorSafeClientContext client; /* 安全协议控制事务和周期快报客户端上下文。 */
    SensorSafeIdentity identity; /* HELLO 接受并锁存的远端传感器身份。 */
    SensorSafeConfigDigest config_digest; /* HELLO 后确认的远端配置摘要基线。 */
    SensorSafeMonitorContext monitor; /* 周期快报双缓冲监测上下文。 */
    SensorSafeIdentityResult last_identity_result; /* 最近一次启动身份初始化或更新结果。 */
    SensorSafeClientResult last_client_result; /* 最近一次服务层调用对应的客户端结果。 */
    SensorSafeNonceSource last_nonce_source; /* 最近一次 HELLO 使用的 nonce 来源。 */
    uint32_t hello_count; /* 成功完成 HELLO 的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t recovery_attempt_count; /* 会话恢复尝试累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t recovery_count; /* 会话恢复成功累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t recovery_blocked_count; /* 因策略禁止副作用而阻止恢复的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    SensorSafeClientResult last_recovery_trigger; /* 触发最近一次会话恢复的客户端错误原因。 */
    SensorSafeTransportResult last_recovery_transport_result; /* 最近一次恢复失败时保留的传输层结果。 */
    SensorSafeResult last_recovery_protocol_result; /* 最近一次恢复失败时保留的本地协议结果。 */
    uint16_t last_recovery_wire_result; /* 最近一次恢复失败时保留的远端线协议结果码。 */
    uint8_t initialized; /* 安全传感器服务的身份、客户端和监测上下文均已初始化的标志。 */
    uint8_t active; /* 安全传感器服务已经完成 HELLO 和配置摘要确认、可提供业务数据的标志。 */
} SensorSafeServiceContext;

/**
 * @brief 初始化安全传感器服务、持久化启动计数并绑定传输回调。
 *
 * @details 调用场景：FRAM、UART6 和 CH9141 完成基础初始化后，首次协议探测之前。
 * @note 关键约束：同一 CPU2 启动周期只允许初始化一次，重复调用不会再次递增启动计数。
 */
SensorSafeServiceResult SensorSafeService_Init(SensorSafeServiceContext *context,
                                               const SensorSafeIdentityOps *identity_ops,
                                               const SensorSafeClientTransportOps *transport_ops);

/**
 * @brief 通过新挑战执行 HELLO 并激活安全协议身份。
 *
 * @details 调用场景：传感器识别时优先于 LTD/DSM 探测调用。
 * @note 关键约束：失败只撤销安全协议活动状态，调用方仍可继续探测旧协议。
 */
SensorSafeServiceResult SensorSafeService_Probe(SensorSafeServiceContext *context,
                                                uint32_t *sensor_id);

/**
 * @brief 撤销服务活动态、client 会话及周期快照，不重复初始化持久化身份。
 *
 * 撤销安全协议运行态，不修改 FRAM 启动计数和旧协议实现。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 */
void SensorSafeService_Deactivate(SensorSafeServiceContext *context);
/**
 * @brief 查询服务是否已通过探测并保持活动态。
 *
 * @param context 安全传感器服务只读上下文；用于读取服务激活状态、客户端最近错误、周期流状态和恢复诊断信息，不修改服务运行态。
 * @return 1 表示 context 非空且 active 标志已置位；0 表示上下文为空或服务尚未激活。
 */
uint8_t SensorSafeService_IsActive(const SensorSafeServiceContext *context);

/**
 * @brief 切换测量模式并返回传感器声明的稳定等待时间。
 *
 * @details 调用场景：旧测量流程进入密度或液位阶段时由适配层调用。
 * @note 关键约束：模式切换属于副作用事务，响应不确定时禁止自动重发。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param mode 准备下发给安全传感器的测量模式码，必须在已协商能力范围内。
 * @param settle_time_ms 传感器切换测量模式后要求的稳定等待时间，单位 ms。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_SetMeasureMode(SensorSafeServiceContext *context,
                                                         uint8_t mode,
                                                         uint16_t *settle_time_ms);

/**
 * @brief 读取密度组合原始定点数并转换为旧业务层浮点单位。
 *
 * @details 调用场景：密度测量流程读取频率、密度和温度时调用。
 *
 * 读取并换算密度组合数据，输出单位分别为 Hz、kg/m3 和 degC。
 *
 * @note 关键约束：只读事务最多允许一次重新 HELLO；失败时不修改三个输出值。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param frequency_hz 传感器频率，单位 Hz。
 * @param density_kg_m3 用于返回实时密度的输出参数，单位 kg/m3。
 * @param temperature_c 温度值，单位 ℃。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_ReadDensity(SensorSafeServiceContext *context,
                                                      float *frequency_hz,
                                                      float *density_kg_m3,
                                                      float *temperature_c);

/**
 * @brief 读取液位频率并按四舍五入转换为整数 Hz。
 *
 * @details 调用场景：液位测量流程读取探头频率时调用。
 *
 * 读取液位频率并按最接近整数 Hz 输出。
 *
 * @note 关键约束：加 500 前检查 UINT32 上界，避免溢出后产生虚假低频值。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param frequency_hz 传感器频率，单位 Hz。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_ReadLevelFrequency(SensorSafeServiceContext *context,
                                                             uint32_t *frequency_hz);

/**
 * @brief 读取水位电容定点数并转换为 pF；能力缺失由 client 明确返回不支持。
 *
 * 读取水位电容和姿态角，保持现有业务接口单位。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param capacitance_pf 用于返回水位通道电容值的输出参数，单位 pF。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_ReadWaterCapacitance(SensorSafeServiceContext *context,
                                                               float *capacitance_pf);
/**
 * @brief 读取双轴姿态定点数并转换为度；失败时不覆盖调用方输出。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param angle_x_deg 用于返回陀螺仪 X 轴角度的输出参数，单位度。
 * @param angle_y_deg 用于返回陀螺仪 Y 轴角度的输出参数，单位度。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_ReadGyroAngle(SensorSafeServiceContext *context,
                                                        float *angle_x_deg,
                                                        float *angle_y_deg);

/**
 * @brief 在服务层以一次调用读取完整 LTD 参数表并隐藏底层分页事务。
 *
 * @details 调用场景：参数同步、维护导出和诊断快照需要取得完整参数集合时调用。
 *
 * 对上提供一次调用读取固定 115 项 LTD 参数表，底层分页过程不暴露给业务层。
 * value_capacity 小于 115 时不发起通信，并通过 value_count_out 返回所需容量。
 *
 * @note 关键约束：固定校验 115 项映射；属于只读事务，允许会话失效后恢复一次。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param values 用于保存连续参数值或测量值的数组。
 * @param value_capacity 调用方结果数组可容纳的元素数量。
 * @param value_count_out 用于返回实际写入结果数组的元素数量。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_ReadAllParams(SensorSafeServiceContext *context,
                                                        SensorSafeParameterValue *values,
                                                        uint16_t value_capacity,
                                                        uint16_t *value_count_out);

/**
 * @brief 协商周期、静默预算、流标识和控制保护窗并启动主动上报。
 *
 * @details 调用场景：显式启用安全协议周期模式时调用，默认测量流程不会自动进入。
 *
 * 周期上报只在显式调用后启用，默认请求响应业务不会自动进入该模式。
 *
 * @note 关键约束：启动属于副作用事务；成功后清空旧快照，必须等待本流首帧重新发布。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param period_ms 传感器周期主动上报的目标周期，单位 ms。
 * @param max_silent_ms 周期主动上报允许连续未收到有效快报的最长静默时间，单位 ms。
 * @param stream_id 数据流编号。
 * @param control_guard_ms 周期主动上报期间预留给控制请求的保护窗口，单位 ms。
 * @param measure_mode 测量模式。
 * @param start_after_ms 传感器接受周期主动上报请求后延迟开始快报的时间，单位 ms。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_StartPeriodic(SensorSafeServiceContext *context,
                                                        uint16_t period_ms,
                                                        uint16_t max_silent_ms,
                                                        uint16_t stream_id,
                                                        uint16_t control_guard_ms,
                                                        uint8_t measure_mode,
                                                        uint16_t *start_after_ms);
/**
 * @brief 接收并验收一帧周期快报，再发布为一致性监控快照。
 *
 * @details 调用场景：周期模式任务轮询入口。
 * @note 关键约束：任一 client 或 monitor 错误立即撤销旧快照，禁止陈旧值继续被读取。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_PollPeriodic(SensorSafeServiceContext *context,
                                                       uint32_t timeout_ms);
/**
 * @brief 按当前时刻读取仍在新鲜度预算内的周期快照。
 *
 * @details 调用场景：业务层消费最近一次已验收快报时调用。
 * @note 关键约束：读取失败时由 monitor 清零输出，服务层只映射失效类别。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @param snapshot 安全协议周期数据快照输出对象；成功时写入流标识、样本序号、测量数据、传感器数据年龄和信号质量。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_ReadPeriodicSnapshot(SensorSafeServiceContext *context,
                                                               SensorSafePeriodicSnapshot *snapshot);
/**
 * @brief 请求退出周期模式，并无条件撤销当前快照。
 *
 * @details 调用场景：回到请求响应模式、命令切换或业务结束时调用。
 * @note 关键约束：即使停流响应失败也不得继续使用旧周期数据。
 *
 * @param context 安全传感器服务上下文；聚合启动身份、客户端会话、远端身份、配置摘要、周期快照监测器以及恢复次数和最近恢复失败现场，本函数按职责读取或更新服务状态。
 * @return 返回服务层结果码；SENSOR_SAFE_SERVICE_OK 表示服务操作完成，其他值区分参数、身份、会话、传输、远端、协议、容量、能力和数据有效性故障。
 */
SensorSafeServiceResult SensorSafeService_StopPeriodic(SensorSafeServiceContext *context);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_SERVICE_H_ */
