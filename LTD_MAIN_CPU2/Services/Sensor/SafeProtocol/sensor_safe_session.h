#ifndef SENSOR_SAFE_SESSION_H_
/* SENSOR_SAFE_SESSION_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_SESSION_H_

#include "sensor_safe_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 安全协议会话状态机状态；限制控制事务、周期快报和恢复动作只能在相应阶段执行。 */
typedef enum {
    /* 安全协议会话状态机状态。 */
    SENSOR_SAFE_SESSION_OFFLINE = 0, /* 尚未建立有效会话。 */
    SENSOR_SAFE_SESSION_READY, /* 会话已建立，可执行状态和配置类命令。 */
    SENSOR_SAFE_SESSION_MEASURE_READY, /* 测量模式已确认，可读取控制测量结果。 */
    SENSOR_SAFE_SESSION_PERIODIC_ACTIVE, /* 周期主动上报流已经建立。 */
    SENSOR_SAFE_SESSION_RECOVERING, /* 会话正在执行有界恢复。 */
    SENSOR_SAFE_SESSION_FAULT /* 会话进入故障状态，需重新握手或上层处理。 */
} SensorSafeSessionState;

typedef struct {
    /* HELLO 响应通过校验后可提交到会话的远端身份和协商结果。 */
    uint8_t selected_protocol; /* HELLO 协商接受的安全协议版本。 */
    uint8_t sensor_node_id; /* 目标传感器在安全协议中的节点地址。 */
    uint16_t sensor_type; /* HELLO 返回的传感器类型编号。 */
    uint32_t sensor_id; /* HELLO 返回的传感器唯一标识；会话内必须保持不变。 */
    uint32_t sensor_boot_counter; /* 传感器报告的启动计数；倒退或异常复用会使会话失效。 */
    uint32_t sensor_session_counter; /* 传感器报告的会话计数；用于拒绝旧会话响应。 */
    uint32_t echo_cpu_boot_counter; /* HELLO 响应回显的 CPU2 启动计数，必须与请求完全一致。 */
    uint32_t echo_cpu_nonce; /* HELLO 响应回显的 CPU2 nonce，必须与请求完全一致。 */
    uint32_t new_session_id; /* 传感器为本次 HELLO 分配的新会话标识。 */
    uint32_t capability_flags; /* 远端能力位位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint16_t scale_version; /* 传感器数据缩放规则版本；不匹配时不得直接解释测量值。 */
    uint16_t calibration_version; /* 传感器标定数据版本。 */
    uint16_t param_table_version; /* 传感器安全参数表版本。 */
    uint16_t digest_schema_version; /* 配置摘要载荷结构版本。 */
    uint32_t safety_param_crc; /* 传感器安全参数集合的 CRC 摘要；会话及测量响应必须与接受基线一致。 */
} SensorSafeHelloAcceptance;

typedef struct {
    /* 安全协议会话状态；跟踪节点、会话、事务序号、远端启动计数、周期流和数据新鲜度基线。 */
    SensorSafeSessionState state; /* 当前安全协议会话状态，取值遵循 SensorSafeSessionState。 */
    uint8_t local_node_id; /* 会话中的 CPU2 本地节点地址。 */
    uint8_t remote_node_id; /* 会话中的传感器远端节点地址。 */
    uint32_t session_id; /* HELLO 协商得到的会话标识；控制帧和快报必须一致。 */
    uint32_t sensor_id; /* HELLO 返回的传感器唯一标识；会话内必须保持不变。 */
    uint32_t safety_param_crc; /* 传感器安全参数集合的 CRC 摘要；会话及测量响应必须与接受基线一致。 */
    uint32_t capability_flags; /* 远端能力位位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint32_t next_seq; /* 下一个控制请求使用的事务序号。 */
    uint32_t pending_seq; /* 当前等待响应的控制请求序号。 */
    uint8_t pending_cmd; /* 当前等待响应的控制命令码。 */
    uint8_t pending_active; /* 当前存在尚未配对响应的控制事务标志。 */
    uint32_t cpu_boot_counter; /* CPU2 从持久化身份区取得的启动计数；每次冷启动原子递增。 */
    uint32_t cpu_nonce; /* 本次 HELLO 由 CPU2 生成并要求远端回显的 nonce。 */
    uint8_t hello_active; /* 当前事务是 HELLO 握手、允许使用零会话标识的标志。 */
    uint32_t last_sensor_boot_counter; /* 最近一次已接受身份的传感器启动计数基线。 */
    uint32_t last_sensor_session_counter; /* 最近一次已接受身份的传感器会话计数基线。 */
    uint32_t last_session_id; /* 上一次接受的会话标识，用于检测异常复用。 */
    uint8_t sensor_counter_valid; /* 远端启动计数和会话计数基线已经建立的标志。 */
    uint16_t stream_id; /* 当前周期快报流标识。 */
    uint16_t config_epoch; /* 传感器配置提交代际；周期流中变化时需要重新确认配置摘要。 */
    uint32_t expected_stream_seq; /* 下一帧期望的周期流序号，用于检测丢帧、乱序和回放。 */
    uint32_t last_sample_counter; /* 最近一次接受的远端采样计数，用于校验后续数据单调性。 */
    uint8_t sample_counter_valid; /* 最近接受的采样计数基线已经建立的标志。 */
    uint8_t expected_measure_mode; /* 会话当前期望的测量模式；响应模式不一致时拒绝数据。 */
    uint8_t stream_valid; /* 周期流标识、序号和样本基线已经建立并可校验后续快报的标志。 */
    uint32_t last_valid_rx_ms; /* 最近一次接受有效会话数据的本地单调毫秒节拍，用于静默超时判断。 */
} SensorSafeSessionContext;

/**
 * @brief 初始化 CPU2 安全协议会话上下文。
 *
 * @details 调用场景：上电、传感器协议切换或最终通信恢复入口。
 * @note 关键约束：初始化后只能发送 HELLO，旧会话和旧上报流全部失效。
 */
void SensorSafeSession_Init(SensorSafeSessionContext *context,
                            uint8_t local_node_id,
                            uint8_t remote_node_id);

/**
 * @brief 停用当前安全会话并清除事务、流和测量运行态。
 *
 * @details 调用场景：协议回退、传感器类型切换或安全适配器主动停用。
 * @note 关键约束：保留本次 CPU2 启动内最近接受的传感器计数和会话号，后续 HELLO 仍必须执行跨停用边界防重放校验。
 */
void SensorSafeSession_Deactivate(SensorSafeSessionContext *context);

/**
 * @brief 登记新的 HELLO 挑战并分配事务号。
 *
 * @details 调用场景：CPU2 准备编码 HELLO 请求之前。
 * @note 关键约束：新的 HELLO 必须使用新的 nonce；超时重发使用 RetryPending 保留原挑战。
 */
SensorSafeResult SensorSafeSession_BeginHello(SensorSafeSessionContext *context,
                                              uint32_t cpu_boot_counter,
                                              uint32_t cpu_nonce,
                                              uint32_t *seq_out);

/**
 * @brief 登记一个已建立会话的普通控制请求。
 *
 * @details 调用场景：编码 SET_MEASURE_MODE、读数、参数或状态请求之前。
 * @note 关键约束：同一时刻只允许一个未完成请求，事务号预计回绕前要求重新 HELLO。
 */
SensorSafeResult SensorSafeSession_BeginRequest(SensorSafeSessionContext *context,
                                                uint8_t cmd,
                                                uint32_t *seq_out);

/**
 * @brief 取得当前未完成事务用于原序号重发。
 *
 * @details 调用场景：控制请求超时后的有限次数重试。
 * @note 关键约束：重试只能恢复可用性，不延长旧安全数据有效期。
 */
SensorSafeResult SensorSafeSession_RetryPending(const SensorSafeSessionContext *context,
                                                uint8_t *cmd_out,
                                                uint32_t *seq_out);

/**
 * @brief 校验控制响应与当前唯一未完成事务是否精确匹配。
 *
 * @details 调用场景：帧层 CRC 校验通过后、解析和使用响应负载之前。
 * @note 关键约束：HELLO 只做事务和地址预检，挑战回显由 AcceptHello 完成。
 */
SensorSafeResult SensorSafeSession_CheckControlResponse(const SensorSafeSessionContext *context,
                                                        const SensorSafeControlFrame *frame);

/**
 * @brief 接受通过挑战回显和计数防重放校验的新会话。
 *
 * @details 调用场景：HELLO 响应字段全部解码完成后。
 * @note 关键约束：传感器启动计数不得回退，同一次启动的会话计数必须严格前进。
 */
SensorSafeResult SensorSafeSession_AcceptHello(SensorSafeSessionContext *context,
                                               const SensorSafeControlFrame *frame,
                                               const SensorSafeHelloAcceptance *hello);

/**
 * @brief 在业务响应字段全部校验完成后结束当前控制事务。
 *
 * @details 调用场景：响应 result_code、状态、诊断和负载均已处理。
 * @note 关键约束：不得在负载校验前清除 pending，否则错误响应可能被后续帧掩盖。
 */
SensorSafeResult SensorSafeSession_CompleteRequest(SensorSafeSessionContext *context);

/**
 * @brief 记录传感器已明确确认的稳定测量模式。
 *
 * @details 调用场景：SET_MEASURE_MODE 响应或有效测量响应通过模式与状态校验后。
 * @note 关键约束：只有 SESSION_READY/MEASURE_READY 可进入，不能绕过周期流或恢复状态。
 */
SensorSafeResult SensorSafeSession_SetMeasureReady(SensorSafeSessionContext *context,
                                                   uint8_t measure_mode);

/**
 * @brief 根据 SET_COMM_MODE 成功响应登记新上报流。
 *
 * @details 调用场景：accepted_stream_id、accepted_config_epoch 和测量模式已确认后。
 * @note 关键约束：新流从 stream_seq=1 开始，旧流立即失效。
 */
SensorSafeResult SensorSafeSession_StartStream(SensorSafeSessionContext *context,
                                               uint16_t stream_id,
                                               uint16_t config_epoch,
                                               uint8_t measure_mode);

/**
 * @brief 在退出上报 ACK 通过后正常关闭当前流并保留会话。
 *
 * @details 调用场景：SET_COMM_MODE(REQUEST_RESPONSE) 响应字段全部确认后。
 * @note 关键约束：旧 stream_id 立即失效，后续旧快报不得更新任何测量缓存。
 */
SensorSafeResult SensorSafeSession_StopStream(SensorSafeSessionContext *context);

/**
 * @brief 校验快报的会话、流、序号、样本、状态和年龄并推进连续性状态。
 *
 * @details 调用场景：固定 44 字节快报已通过 CRC 和保留位校验后。
 * @note 关键约束：重复、回退、跳号或错配置会永久失效当前流，后续帧不得自行恢复。
 */
SensorSafeResult SensorSafeSession_AcceptFastReport(SensorSafeSessionContext *context,
                                                    const SensorSafeFastReport *report,
                                                    uint32_t rx_timestamp_ms,
                                                    uint32_t max_data_age_ms);

/**
 * @brief 立即撤销当前安全上报流并进入恢复状态。
 *
 * @details 调用场景：快报 CRC、长度、静默、地址或其它关键校验失败。
 * @note 关键约束：恢复前必须重新 SET_COMM_MODE 或 HELLO，不能靠后续连续帧恢复。
 */
void SensorSafeSession_InvalidateStream(SensorSafeSessionContext *context);

/**
 * @brief 使用本地单调时钟计算饱和有效年龄。
 *
 * @details 调用场景：快照被业务消费时，把线上年龄和本地排队时间相加。
 * @note 关键约束：加法饱和到 UINT32_MAX，禁止整数回绕生成伪造的新鲜数据。
 */
uint32_t SensorSafeSession_EffectiveAgeMs(uint16_t data_age_ms,
                                          uint32_t rx_timestamp_ms,
                                          uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_SESSION_H_ */
