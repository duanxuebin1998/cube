#ifndef SENSOR_SAFE_CLIENT_H_
/* SENSOR_SAFE_CLIENT_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_CLIENT_H_

#include "sensor_safe_session.h"
#include "sensor_safe_transport_uart6.h"

#ifdef __cplusplus
extern "C" {
#endif

/* client 默认时序和周期边界；实际周期值还必须由传感器响应回显确认。 */
#define SENSOR_SAFE_CLIENT_DEFAULT_TIMEOUT_MS       300U
/* 安全传感器切换测量模式后的确认超时 1000 ms；超时前必须持续允许接收应答，不能阻塞 UART 服务。 */
#define SENSOR_SAFE_CLIENT_MODE_TIMEOUT_MS          1000U
/* 安全协议控制事务默认最多尝试 3 次。 */
#define SENSOR_SAFE_CLIENT_DEFAULT_MAX_ATTEMPTS     3U
/* 周期快报配置允许的最短周期 500 ms；更快请求会增加串口负载并被参数校验拒绝。 */
#define SENSOR_SAFE_CLIENT_MIN_PERIOD_MS            500U
/* 周期快报配置允许的最长周期 5000 ms。 */
#define SENSOR_SAFE_CLIENT_MAX_PERIOD_MS            5000U
/* 连续安全控制事务之间的最小保护间隔 100 ms；避免请求与迟到响应交叠。 */
#define SENSOR_SAFE_CLIENT_MIN_CONTROL_GUARD_MS     100U
/* 流程退出时发送停止/收尾请求可占用的时间预算 60 ms；超过预算必须让出控制，不能拖延命令切换。 */
#define SENSOR_SAFE_CLIENT_EXIT_REQUEST_BUDGET_MS   60U

/* client 对服务层暴露的稳定结果分类，具体协议/传输原因保存在 context 中。 */
typedef enum {
    /* 安全协议客户端对一次业务请求给出的归一化结果。 */
    SENSOR_SAFE_CLIENT_OK = 0, /* 客户端请求成功并通过全部本地校验。 */
    SENSOR_SAFE_CLIENT_INVALID_ARGUMENT, /* 调用参数、缓冲区或命令组合非法。 */
    SENSOR_SAFE_CLIENT_TRANSPORT_ERROR, /* UART6 传输层未能完成收发。 */
    SENSOR_SAFE_CLIENT_PROTOCOL_ERROR, /* 收到帧但本地协议校验失败。 */
    SENSOR_SAFE_CLIENT_REMOTE_ERROR, /* 远端返回非成功线协议结果码。 */
    SENSOR_SAFE_CLIENT_DATA_INVALID, /* 响应状态位表明数据无效。 */
    SENSOR_SAFE_CLIENT_DATA_STALE, /* 响应数据年龄超过客户端允许值。 */
    SENSOR_SAFE_CLIENT_NEEDS_HELLO, /* 当前没有可用会话，需要重新执行 HELLO。 */
    SENSOR_SAFE_CLIENT_UNSUPPORTED_CAPABILITY, /* 远端能力位未声明支持所请求功能。 */
    SENSOR_SAFE_CLIENT_CONTROL_WINDOW_CLOSED, /* 周期流控制保护窗口尚未开放。 */
    SENSOR_SAFE_CLIENT_BUFFER_TOO_SMALL /* 调用方结果缓冲区容量不足。 */
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
    /* 安全协议客户端依赖的交换、被动接收和单调时钟回调。 */
    SensorSafeClientExchangeFn exchange; /* 发送控制请求并等待对应响应的同步交换回调。 */
    SensorSafeClientReceiveFn receive; /* 在不发送请求时接收周期快报的回调。 */
    SensorSafeClientNowMsFn now_ms; /* 返回单调毫秒节拍的回调，用于超时和数据年龄计算。 */
} SensorSafeClientTransportOps;

/* client 静态配置；cpu_boot_counter 来自身份双副本初始化结果。 */
typedef struct {
    /* 安全协议客户端节点身份、数据最大年龄、请求超时和最大尝试次数配置。 */
    uint8_t cpu_node_id; /* CPU2 在安全协议中的本地节点地址。 */
    uint8_t sensor_node_id; /* 目标传感器在安全协议中的节点地址。 */
    uint32_t cpu_boot_counter; /* CPU2 从持久化身份区取得的启动计数；每次冷启动原子递增。 */
    uint32_t max_data_age_ms; /* 客户端允许接受的数据最大年龄，单位为 ms；超过后返回数据陈旧。 */
    uint32_t request_timeout_ms; /* 单次安全协议控制请求等待响应的超时时间，单位为 ms。 */
    uint8_t max_attempts; /* 一次控制事务允许的最大发送尝试次数，包含首次发送。 */
} SensorSafeClientConfig;

/* HELLO 验收后的传感器身份、版本、能力和配置摘要。 */
typedef struct {
    /* HELLO 接受后锁存的远端传感器身份、版本、能力和安全参数摘要。 */
    uint16_t sensor_type; /* HELLO 返回的传感器类型编号。 */
    uint32_t sensor_id; /* HELLO 返回的传感器唯一标识；会话内必须保持不变。 */
    uint32_t sensor_sw_version; /* 传感器软件版本编码。 */
    uint32_t sensor_hw_version; /* 传感器硬件版本编码。 */
    uint32_t sensor_boot_counter; /* 传感器报告的启动计数；倒退或异常复用会使会话失效。 */
    uint32_t sensor_session_counter; /* 传感器报告的会话计数；用于拒绝旧会话响应。 */
    uint32_t capability_flags; /* 远端能力位位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint16_t scale_version; /* 传感器数据缩放规则版本；不匹配时不得直接解释测量值。 */
    uint16_t calibration_version; /* 传感器标定数据版本。 */
    uint16_t param_table_version; /* 传感器安全参数表版本。 */
    uint16_t digest_schema_version; /* 配置摘要载荷结构版本。 */
    uint32_t safety_param_crc; /* 传感器安全参数集合的 CRC 摘要；会话及测量响应必须与接受基线一致。 */
} SensorSafeIdentity;

/* GET_STATUS 的公共状态及实际接收时间。 */
typedef struct {
    /* GET_STATUS 的公共状态响应及本地接收时间戳。 */
    SensorSafeCommonResponse common; /* 从同一响应载荷解析出的公共结果、模式、状态和数据质量字段。 */
    uint32_t rx_timestamp_ms; /* 本地接收时间戳，使用单调毫秒节拍；仅可通过无符号差值比较超时。 */
} SensorSafeStatus;

/* 配置摘要用于把会话绑定到参数 CRC、版本和配置代次。 */
typedef struct {
    /* 配置摘要响应；把安全参数 CRC、能力、版本和配置代际作为一致性校验基线。 */
    SensorSafeCommonResponse common; /* 从同一响应载荷解析出的公共结果、模式、状态和数据质量字段。 */
    uint32_t safety_param_crc; /* 传感器安全参数集合的 CRC 摘要；会话及测量响应必须与接受基线一致。 */
    uint32_t capability_flags; /* 远端能力位位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint16_t scale_version; /* 传感器数据缩放规则版本；不匹配时不得直接解释测量值。 */
    uint16_t calibration_version; /* 传感器标定数据版本。 */
    uint16_t param_table_version; /* 传感器安全参数表版本。 */
    uint16_t digest_schema_version; /* 配置摘要载荷结构版本。 */
    uint16_t config_epoch; /* 传感器配置提交代际；周期流中变化时需要重新确认配置摘要。 */
} SensorSafeConfigDigest;

/* 密度组合测量，所有数值保持协议定点单位以避免 client 内浮点误差。 */
typedef struct {
    /* 组合测量响应；温度、密度和三路频率与同一采样计数、数据年龄及信号质量绑定。 */
    int32_t temperature_c_x100; /* 温度百倍定点值，单位为 0.01 ℃；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    int32_t density_kg_m3_x100; /* 密度百倍定点值，单位为 0.01 kg/m3；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t frequency_hz_x1000; /* 频率千倍定点值，单位为 0.001 Hz；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t freq_45_hz_x1000; /* 45 kHz 通道测量频率的千倍定点值，单位为 0.001 Hz。 */
    uint32_t freq_225_hz_x1000; /* 225 kHz 通道测量频率的千倍定点值，单位为 0.001 Hz。 */
    uint16_t signal_quality; /* 传感器报告的信号质量指标；仍需结合状态位和数据年龄判定可用性。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint32_t effective_age_ms; /* 折算后的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
} SensorSafeMeasurementCombo;

/* 液位通道测量及传感器声明的有效频率范围。 */
typedef struct {
    /* 液位频率响应；除测量值外携带传感器声明的有效频率范围、采样计数和数据年龄。 */
    uint32_t level_frequency_hz_x1000; /* 液位频率千倍定点值，单位为 0.001 Hz；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint16_t signal_quality; /* 传感器报告的信号质量指标；仍需结合状态位和数据年龄判定可用性。 */
    uint16_t valid_min_hz; /* 传感器声明的液位频率有效下限，单位为 Hz。 */
    uint16_t valid_max_hz; /* 传感器声明的液位频率有效上限，单位为 Hz。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint32_t effective_age_ms; /* 折算后的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
} SensorSafeLevelMeasurement;

/* 水位电容测量，单位为 0.1 pF。 */
typedef struct {
    /* 水位电容响应及其信号质量、采样计数和数据年龄。 */
    int32_t capacitance_pf_x10; /* 电容十倍定点值，单位为 0.1 pF；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint16_t signal_quality; /* 传感器报告的信号质量指标；仍需结合状态位和数据年龄判定可用性。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint32_t effective_age_ms; /* 折算后的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
} SensorSafeWaterCapMeasurement;

/* 双轴姿态测量，角度单位为 0.01 度。 */
typedef struct {
    /* 陀螺仪双轴角度、状态、采样计数和数据年龄。 */
    int32_t angle_x_deg_x100; /* X 轴角度百倍定点值，单位为 0.01°；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    int32_t angle_y_deg_x100; /* Y 轴角度百倍定点值，单位为 0.01°；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint16_t gyro_status; /* 陀螺仪测量状态字；非零位按传感器协议定义解释。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint32_t effective_age_ms; /* 折算后的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
} SensorSafeGyroMeasurement;

/* 电源诊断测量；不直接驱动整机安全状态。 */
typedef struct {
    /* 传感器供电电压、供电状态、采样计数和数据年龄。 */
    uint32_t voltage_mv; /* 传感器测得的供电电压，单位为 mV。 */
    uint16_t power_status; /* 传感器供电状态字；必须与电压和诊断位一起判断。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint32_t effective_age_ms; /* 折算后的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
} SensorSafePowerMeasurement;

/* 参数线格式的规范化表示，param_crc 覆盖版本和全部解释字段。 */
typedef struct {
    /* 单个安全参数值；携带编号、索引、类型、权限、缩放、单位、64 位值和参数级 CRC。 */
    uint16_t param_id; /* 安全参数的协议编号。 */
    uint16_t param_index; /* 该参数在分页参数表中的稳定索引。 */
    uint8_t param_type; /* 参数值的线格式类型，取值遵循 SensorSafeParamType。 */
    uint8_t access; /* 参数访问属性位；至少说明可读、可写及是否需要提交。 */
    int16_t scale; /* 参数十进制缩放指数，用于把 value_i64 换算为物理量。 */
    uint16_t unit_id; /* 参数单位编号，由协议单位表解释。 */
    int64_t value_i64; /* 参数的有符号 64 位原始值；物理值需结合 scale 和 unit_id。 */
    uint32_t param_crc; /* 覆盖本参数元数据和值的参数级 CRC，用于分页传输完整性校验。 */
} SensorSafeParameterValue;

/* 自检任务结果；test_id 用于防止读取到其他任务的历史结果。 */
typedef struct {
    /* 远端自检编号、结果、首个失败项和详细诊断码。 */
    uint32_t test_id; /* 远端本次自检任务编号。 */
    uint16_t test_result; /* 远端自检结果码。 */
    uint16_t failed_item; /* 远端报告的首个自检失败项目编号。 */
    uint32_t detail_code; /* 远端自检失败的详细诊断码。 */
} SensorSafeSelfTestResult;

/* client 累计诊断计数，全部采用饱和计数而不自动清零。 */
typedef struct {
    /* 安全协议客户端控制事务、重试、拒绝和周期快报的累计计数。 */
    uint32_t control_request_count; /* 控制请求逻辑次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t control_attempt_count; /* 控制帧实际发送尝试次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t control_retry_count; /* 控制事务重试次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t control_response_count; /* 有效控制响应次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t transport_failure_count; /* 传输层失败次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t protocol_reject_count; /* 本地协议校验拒绝次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t crc_reject_count; /* CRC 校验拒绝次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t address_reject_count; /* 地址校验拒绝次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t remote_error_count; /* 远端错误响应次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t fast_report_receive_count; /* 收到周期快报次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t fast_report_accept_count; /* 通过校验并接受的周期快报次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
} SensorSafeClientCounters;

/*
 * client 完整运行态。
 * 会话、防重放、周期协商、诊断计数和固定缓冲区均由单一任务串行访问，接口不可重入。
 */
typedef struct {
    /* 安全协议客户端完整上下文；会话、配置、平台回调、身份、诊断、周期协商值和收发缓冲区由此统一管理。 */
    SensorSafeSessionContext session; /* 客户端持有的安全协议会话状态机上下文。 */
    SensorSafeClientConfig config; /* 客户端节点、超时、新鲜度和重试配置。 */
    SensorSafeClientTransportOps transport; /* 客户端使用的传输和单调时钟回调集合。 */
    SensorSafeIdentity identity; /* HELLO 接受并锁存的远端传感器身份。 */
    SensorSafeTransportResult last_transport_result; /* 最近一次客户端收发操作的传输层结果。 */
    SensorSafeResult last_protocol_result; /* 最近一次响应或快报的本地协议校验结果。 */
    uint16_t last_wire_result; /* 最近一次远端响应携带的线协议结果码。 */
    uint32_t last_rx_timestamp_ms; /* 最近一次接受有效控制响应的本地单调毫秒时间戳。 */
    uint16_t accepted_period_ms; /* 协商接受的快报周期，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
    uint16_t accepted_max_silent_ms; /* 协商接受的最大静默时间，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
    uint16_t accepted_control_guard_ms; /* 协商接受的控制窗口保护时间，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
    uint16_t accepted_start_after_ms; /* 协商接受的快报延迟启动时间，使用单调毫秒节拍；仅可通过无符号差值比较超时。 */
    uint32_t periodic_start_ms; /* 本地周期流启动时刻，使用单调毫秒节拍；仅可通过无符号差值比较超时。 */
    SensorSafeClientCounters counters; /* 客户端累计事务、重试、拒绝和周期快报统计。 */
    uint32_t last_control_sample[6]; /* 按控制命令类别保存的最近采样计数基线，用于拒绝回放或倒退样本。 */
    uint8_t last_control_sample_valid[6]; /* 六类控制命令各自的采样计数基线有效标志数组。 */
    uint8_t tx_buffer[SENSOR_SAFE_CTRL_MAX_FRAME_LEN]; /* 发送缓冲区；容量由声明中的编译期常量限定，写入前必须检查边界。 */
    uint8_t rx_buffer[SENSOR_SAFE_UART6_RX_BUFFER_SIZE]; /* 接收缓冲区；容量由声明中的编译期常量限定，写入前必须检查边界。 */
} SensorSafeClientContext;

/**
 * @brief 保存 client 配置和传输接口并初始化离线会话。
 *
 * @details 调用场景：服务层完成持久化身份初始化后调用。
 *
 * 初始化与下线只管理本地运行态，不发送总线命令。
 *
 * @note 关键约束：零超时或零尝试次数替换为受控默认值；本函数不访问总线。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param config 安全协议客户端静态配置；包含 CPU2 与传感器节点号、CPU2 单调启动计数、最大数据年龄、请求超时和最大尝试次数。
 * @param transport 安全协议客户端使用的请求响应传输接口表。
 */
void SensorSafeClient_Init(SensorSafeClientContext *context,
                           const SensorSafeClientConfig *config,
                           const SensorSafeClientTransportOps *transport);
/**
 * @brief 撤销 client 会话、周期协商值和本会话样本历史，但保留累计诊断计数。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 */
void SensorSafeClient_Deactivate(SensorSafeClientContext *context);

/**
 * @brief 发送 HELLO 挑战并验收传感器回显、身份、能力及单调会话计数。
 *
 * @details 调用场景：首次探测和服务层允许的故障恢复。
 *
 * 会话与基础诊断接口。
 *
 * @note 关键约束：同一 HELLO 的重试复用 nonce 和 seq；任何未绑定响应不得激活会话。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param cpu_nonce CPU随机数。
 * @param identity_out 用于接收已经完成身份、能力和单调计数验收的 HELLO 响应。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_Hello(SensorSafeClientContext *context,
                                              uint32_t cpu_nonce,
                                              SensorSafeIdentity *identity_out);
/**
 * @brief 执行无副作用在线检查并返回传感器运行时间。
 *
 * @details 调用场景：安全会话健康检查或周期流控制保护窗内诊断。
 * @note 关键约束：响应长度和公共状态必须通过统一校验后才写输出。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param token 用于串行化访问共享资源的控制标记。
 * @param sensor_uptime_ms 用于返回传感器报告的本次启动运行时长，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_Ping(SensorSafeClientContext *context,
                                             uint32_t token,
                                             uint32_t *sensor_uptime_ms);
/**
 * @brief 读取传感器公共状态及复位原因，供诊断流程判断当前运行态。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param status_out 用于接收本次查询得到的状态快照。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_GetStatus(SensorSafeClientContext *context,
                                                  SensorSafeStatus *status_out);
/**
 * @brief 读取并核对安全参数摘要、参数表版本和配置代次。
 *
 * @details 调用场景：HELLO 成功后由服务层立即调用，建立配置身份基线。
 * @note 关键约束：响应中的 safety_param_crc 必须与控制帧身份字段一致且非 0。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param digest_out 用于接收通过核对的安全参数摘要、参数表版本和配置代次。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_GetConfigDigest(SensorSafeClientContext *context,
                                                        SensorSafeConfigDigest *digest_out);
/**
 * @brief 请求传感器撤销当前会话并返回复位接受状态。
 *
 * @details 调用场景：显式维护或不可继续使用当前会话时调用。
 * @note 关键约束：命令成功后本地也立即下线，后续业务必须重新 HELLO。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param reset_reason 复位。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ResetSession(SensorSafeClientContext *context,
                                                     uint16_t reset_reason);
/**
 * @brief 切换测量模式并验收模式回显、能力位和稳定等待时间。
 *
 * @details 调用场景：密度、液位、水位电容、陀螺仪等测量前调用。
 *
 * 请求响应测量接口；输出仅在返回 SENSOR_SAFE_CLIENT_OK 时有效。
 *
 * @note 关键约束：未声明能力的模式在发送前拒绝；模式未匹配时不得进入 MEASURE_READY。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param target_mode 目标模式。
 * @param settle_time_ms 传感器切换测量模式后要求的稳定等待时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_SetMeasureMode(SensorSafeClientContext *context,
                                                       uint8_t target_mode,
                                                       uint16_t *settle_time_ms);
/**
 * @brief 读取密度模式的频率、密度、温度组合值，并执行样本单调性与新鲜度检查。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议组合测量中的密度、温度、频率、位置、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ReadMeasurementCombo(SensorSafeClientContext *context,
                                                             SensorSafeMeasurementCombo *measurement);
/**
 * @brief 读取液位频率定点值，只有液位通道诊断和样本检查全部通过才写输出。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议液位频率、位置、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ReadLevelFrequency(SensorSafeClientContext *context,
                                                          SensorSafeLevelMeasurement *measurement);
/**
 * @brief 读取水位电容定点值；未声明 WATER_CAP 能力时不发送总线请求。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议水位电容、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ReadWaterCap(SensorSafeClientContext *context,
                                                     SensorSafeWaterCapMeasurement *measurement);
/**
 * @brief 读取双轴姿态定点值；未声明 GYRO 能力时不发送总线请求。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议陀螺角度、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ReadGyro(SensorSafeClientContext *context,
                                                 SensorSafeGyroMeasurement *measurement);
/**
 * @brief 读取电源电压、电流及电源状态，作为诊断数据而非直接控制依据。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param measurement 本次处理使用的测量结果对象。成功时写入安全协议电源电压、电流、状态、数据年龄和有效性字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ReadPower(SensorSafeClientContext *context,
                                                  SensorSafePowerMeasurement *measurement);
/**
 * @brief 请求进入周期主动上报并锁存传感器实际接受的时序参数。
 *
 * @details 调用场景：上层显式选择周期模式且已处于对应测量就绪态时调用。
 *
 * 周期主动上报接口；进入后只允许受控停流和会话维护命令。
 *
 * @note 关键约束：周期、静默预算、保护窗、stream_id、配置代次和模式必须逐项回显一致。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param period_ms 传感器周期主动上报的目标周期，单位 ms。
 * @param max_silent_ms 周期主动上报允许连续未收到有效快报的最长静默时间，单位 ms。
 * @param stream_id 数据流编号。
 * @param control_guard_ms 周期主动上报期间预留给控制请求的保护窗口，单位 ms。
 * @param measure_mode 测量模式。
 * @param start_after_ms 传感器接受周期主动上报请求后延迟开始快报的时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_StartPeriodic(SensorSafeClientContext *context,
                                                      uint16_t period_ms,
                                                      uint16_t max_silent_ms,
                                                      uint16_t stream_id,
                                                      uint16_t control_guard_ms,
                                                      uint8_t measure_mode,
                                                      uint16_t *start_after_ms);
/**
 * @brief 在协商静默预算内接收并验收一帧周期快报。
 *
 * @details 调用场景：周期模式轮询任务调用，每次最多交付一帧。
 * @note 关键约束：接收等待不超过剩余静默预算；迟到、跳号或身份异常立即使流失效。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param report 用于接收本次诊断或测量结果的报告对象。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_PollPeriodic(SensorSafeClientContext *context,
                                                     SensorSafeFastReport *report,
                                                     uint32_t timeout_ms);
/**
 * @brief 只在最近快报后的控制保护窗内请求退出周期模式。
 *
 * @details 调用场景：业务结束或需要返回请求响应模式时调用。
 * @note 关键约束：预留完整发送预算；窗口关闭或响应异常时流立即失效，禁止假定已停流。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_StopPeriodic(SensorSafeClientContext *context);
/**
 * @brief 按参数 ID 和索引读取单项参数并核对参数级 CRC。
 *
 * @details 调用场景：维护诊断明确请求参数读取时调用，基础测量流程不使用。
 *
 * 参数读写为受控维护能力；COMMIT 成功后必须重新 HELLO。
 *
 * @note 关键约束：能力位、回显标识和 CRC 任一不符时不接受返回值。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param param_id 参数编号。
 * @param param_index 参数索引。
 * @param value_out 用于接收通过类型、范围和参数 CRC 校验的参数值。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ReadParam(SensorSafeClientContext *context,
                                                  uint16_t param_id,
                                                  uint16_t param_index,
                                                  SensorSafeParameterValue *value_out);
/**
 * @brief 通过自动分页读取完整 LTD 参数表，对上保持一次逻辑调用。
 *
 * @details 调用场景：设备识别完成后的参数同步、维护导出或诊断快照。
 *
 * 一次逻辑调用自动分页读取完整参数表；只有返回 OK 时输出数组整体有效。
 *
 * @note 关键约束：首帧确认总数后才写调用方数组；任一页失败时 value_count_out 保持 0。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param values 调用方提供的参数值数组；全部分页成功后才写入。
 * @param value_capacity 参数值数组容量，单位为 64 位参数值个数。
 * @param value_count_out 用于返回完整参数表的实际元素数量；任一页失败时保持为 0。
 * @return 返回安全协议结果码；SENSOR_SAFE_RESULT_OK 表示完整参数表读取成功，其他值表示分页、容量或通信失败。
 */
SensorSafeClientResult SensorSafeClient_ReadAllParams(SensorSafeClientContext *context,
                                                      SensorSafeParameterValue *values,
                                                      uint16_t value_capacity,
                                                      uint16_t *value_count_out);
/**
 * @brief 携带访问保护和期望参数 CRC 写入暂存参数值。
 *
 * @details 调用场景：受控维护流程修改单项参数时调用。
 * @note 关键约束：这是副作用事务；调用方必须使用返回的新参数 CRC 继续提交闭环。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param value 待原地读取或更新的数值对象。该参数实际是只读安全参数记录，包含参数 ID、类型、访问属性、长度和值字节，用于 CRC、语义校验或写入请求。
 * @param access_guard 参数写入使用的访问保护值，必须与当前协议授权条件一致。
 * @param expected_param_crc 期望参数CRC。
 * @param new_param_crc 用于返回远端暂存参数表更新后的 CRC32C。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_WriteParam(SensorSafeClientContext *context,
                                                   const SensorSafeParameterValue *value,
                                                   uint8_t access_guard,
                                                   uint32_t expected_param_crc,
                                                   uint32_t *new_param_crc);
/**
 * @brief 以当前安全参数 CRC 为前置条件提交参数事务。
 *
 * @details 调用场景：一组 WRITE_PARAM 完成后由受控维护流程调用。
 * @note 关键约束：提交成功会改变配置身份，本地立即下线并要求重新 HELLO 获取新摘要。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param commit_scope 安全参数提交范围，决定传感器应用哪些暂存修改。
 * @param expected_safety_param_crc 期望参数CRC。
 * @param new_safety_param_crc 用于返回远端提交后生效的安全参数 CRC32C。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_CommitParam(SensorSafeClientContext *context,
                                                    uint16_t commit_scope,
                                                    uint32_t expected_safety_param_crc,
                                                    uint32_t *new_safety_param_crc);
/**
 * @brief 启动指定掩码的传感器自检并返回 test_id 与预计完成时间。
 *
 * @details 调用场景：受控诊断流程显式调用。
 *
 * 自检接口通过 test_id 关联启动与结果读取。
 *
 * @note 关键约束：必须声明 SELF_TEST 能力；timeout_ms 同时约束远端和本地等待。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param test_mask 测试。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param test_id 用于返回远端本次自检事务的标识号。
 * @param estimated_time_ms 用于返回传感器自检预计持续时间，单位 ms。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_RunSelfTest(SensorSafeClientContext *context,
                                                    uint32_t test_mask,
                                                    uint16_t timeout_ms,
                                                    uint32_t *test_id,
                                                    uint16_t *estimated_time_ms);
/**
 * @brief 读取指定自检任务结果，并核对非零 test_id 回显防止串用旧任务结果。
 *
 * @param context 安全协议客户端运行上下文；保存会话与防重放状态、周期流协商结果、最近传输和协议错误、诊断计数以及固定收发缓冲区，本函数按职责读取或更新其中字段。
 * @param test_id 测试编号。
 * @param result_out 用于接收与请求 test_id 一致的自检结果。
 * @return 返回客户端结果码；SENSOR_SAFE_CLIENT_OK 表示本次客户端操作完成，其他值区分参数非法、能力不支持、会话未建立、传输失败、远端错误、协议错误以及数据无效或过期。
 */
SensorSafeClientResult SensorSafeClient_ReadSelfTestResult(SensorSafeClientContext *context,
                                                           uint32_t test_id,
                                                           SensorSafeSelfTestResult *result_out);
/**
 * @brief 复制 client 累计诊断计数快照，不清零计数且不访问传输硬件。
 *
 * 读取累计诊断计数，不修改 client 状态。
 *
 * @param context 安全协议客户端只读运行上下文；用于读取当前会话、周期流协商结果、最近错误和诊断计数，函数不会修改上下文字段。
 * @param counters_out 用于接收客户端累计诊断计数快照；读取不会清零计数。
 */
void SensorSafeClient_GetCounters(const SensorSafeClientContext *context,
                                  SensorSafeClientCounters *counters_out);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_CLIENT_H_ */
