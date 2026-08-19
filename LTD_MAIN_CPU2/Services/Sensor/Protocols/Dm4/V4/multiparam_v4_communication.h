/*
 * 模块职责：公开DM4-V4固定交互帧、主动快照、通信质量诊断和模式控制接口。
 * 分层边界：业务层只依赖本头文件；流解析、事务状态和UART6共享变量留在internal头文件。
 * 兼容约束：主动帧标准长度为72字节，无线滑环插入50字节零填充由内部codec兼容处理。
 */
#ifndef MULTIPARAM_V4_COMMUNICATION_H
/* 本头文件的包含保护标记。 */
#define MULTIPARAM_V4_COMMUNICATION_H

#include <stdint.h>
/* V4交互请求和应答固定长度，单位字节。 */
#define MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE       8U
/* V4标准主动上报帧固定长度，单位字节。 */
#define MULTIPARAM_V4_ACTIVE_FRAME_SIZE            72U
/* V4主动帧携带的32位参数数量。 */
#define MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT       15U
/* V4主动帧参数数据区长度，单位字节。 */
#define MULTIPARAM_V4_ACTIVE_DATA_SIZE             60U
/* 判定V4主动数据流超时的最大无帧时间，单位ms。 */
#define MULTIPARAM_V4_ACTIVE_TIMEOUT_MS            1500U
/* 主动帧结束后允许发送半双工切换命令前的最小保护时间，单位ms。 */
#define MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS      15U
/* 保存最近异常主动包的最大容量，兼容无线补零帧，单位字节。 */
#define MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE      122U
/* 连续主动包异常触发通信质量故障的门限，单位包。 */
#define MULTIPARAM_V4_CONSECUTIVE_ERROR_LIMIT      3U
/* 未锁定目标时使用的任意地址过滤标记，不是线协议节点地址。 */
#define MULTIPARAM_V4_ANY_ADDRESS                  0xFFU
/* CPU2当前接受的DM4-V4协议版本数值。 */
#define MULTIPARAM_V4_PROTOCOL_VERSION_VALUE       4.0f

typedef enum {
    MULTIPARAM_V4_COMMUNICATION_UNKNOWN = 0, /* 通信方式尚未确认或协议栈已退出。 */
    MULTIPARAM_V4_COMMUNICATION_ACTIVE = 1, /* 传感器周期主动上报，CPU2保持接收。 */
    MULTIPARAM_V4_COMMUNICATION_INTERACTIVE = 2 /* CPU2按请求发起8字节交互事务。 */
} multiparam_v4_communication_mode_t;

typedef enum {
    MULTIPARAM_V4_MEASUREMENT_INVALID = 0, /* R02低位不能映射为已知测量模式。 */
    MULTIPARAM_V4_MEASUREMENT_DENSITY = 1, /* 传感器处于密度测量模式。 */
    MULTIPARAM_V4_MEASUREMENT_LEVEL = 2 /* 传感器处于液位测量模式。 */
} multiparam_v4_measurement_mode_t;

typedef enum {
    MULTIPARAM_V4_FEATURE_NONE = 0, /* 密度模式下水位与零点霍尔均未开启。 */
    MULTIPARAM_V4_FEATURE_WATER = 1, /* 密度模式下水位电容功能已开启。 */
    MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO = 2, /* 密度模式下零点霍尔功能已开启。 */
    MULTIPARAM_V4_FEATURE_INVALID = 3 /* 功能位组合非法或与当前测量模式冲突。 */
} multiparam_v4_feature_state_t;

/* 最近一次有效主动帧的规范化快照；所有字段来自同一帧和同一发布代次。 */
typedef struct {
    uint8_t valid; /* 快照已通过完整帧校验并可供业务读取的标志。 */
    uint8_t address; /* 主动帧中的传感器节点地址。 */
    uint16_t sequence; /* 主动帧16位序号，用于丢包、重复和乱序统计。 */
    uint32_t received_tick; /* 完整主动帧接收结束的本地毫秒节拍。 */
    uint32_t generation; /* 每发布一帧递增的本地代次，用于避免重复发布同一快照。 */
    uint32_t raw_parameter[MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT]; /* 主动数据区15个参数的原始32位位型。 */
    float software_version; /* 传感器软件版本参数。 */
    float protocol_version; /* 传感器报告的协议版本参数。 */
    uint32_t status_word; /* R02运行状态原始位集合。 */
    float magnetic_zero_voltage; /* 零点霍尔电压，单位V；是否有效由magnetic_zero_valid指示。 */
    int32_t measurement_frequency_hz; /* 当前测量频率，单位Hz。 */
    float water_capacitance_pf; /* 水位电容，单位pF；是否有效由water_capacitance_valid指示。 */
    float temperature_c; /* 介质温度，单位摄氏度。 */
    float density_kg_m3; /* 密度，单位kg/m3。 */
    float dynamic_viscosity_cp; /* 动力黏度，单位cP。 */
    float kinematic_viscosity_cst; /* 运动黏度，单位cSt。 */
    float supply_voltage_v; /* 传感器供电电压，单位V。 */
    float angle_x_deg; /* X轴姿态角，单位度。 */
    float angle_y_deg; /* Y轴姿态角，单位度。 */
    float sweep_period_square_mean_45; /* 45度扫频周期平方均值，单位沿传感器协议定义。 */
    float sweep_period_square_mean_22_5; /* 22.5度扫频周期平方均值，单位沿传感器协议定义。 */
    multiparam_v4_measurement_mode_t measurement_mode; /* 从R02低位解码的密度或液位模式。 */
    multiparam_v4_feature_state_t feature_state; /* 从R02低位解码的水位、零点霍尔或关闭状态。 */
    uint8_t magnetic_zero_valid; /* 零点霍尔电压当前具有业务语义的标志。 */
    uint8_t water_capacitance_valid; /* 水位电容当前具有业务语义的标志。 */
} multiparam_v4_snapshot_t;

/* V4主动流和交互事务的饱和累计诊断；读取不会自动清零。 */
typedef struct {
    uint32_t received_bytes; /* UART6主动接收累计字节数。 */
    uint32_t receive_events; /* Receive-to-IDLE回调累计次数。 */
    uint32_t frames_seen; /* 流解析器遇到的完整候选帧数量。 */
    uint32_t valid_frames; /* 完成全部校验并发布的主动帧数量。 */
    uint32_t wireless_zero_padding_frames; /* 成功兼容中插50字节零填充的主动帧数量。 */
    uint32_t crc_errors; /* 主动候选帧CRC校验失败次数。 */
    uint32_t fixed_field_errors; /* 帧头、版本、类型或固定长度字段错误次数。 */
    uint32_t address_errors; /* 地址不符合当前过滤条件的帧数量。 */
    uint32_t value_errors; /* 帧结构正确但参数值无法按协议接受的次数。 */
    uint32_t short_receive_events; /* 单次接收不足完整候选帧的事件数量。 */
    uint32_t resync_events; /* 丢弃无效前缀并重新寻找帧头的次数。 */
    uint32_t duplicate_frames; /* 序号与上一有效帧相同的重复帧数量。 */
    uint32_t out_of_order_frames; /* 序号回退且不属于正常回绕的乱序帧数量。 */
    uint32_t lost_frames; /* 按有效序号跳变推算的累计丢包数量。 */
    uint32_t sequence_resets; /* 判定传感器序号重新建立基线的次数。 */
    uint32_t stream_overflows; /* ISR环形流缓冲空间不足的次数。 */
    uint32_t uart_errors; /* UART6硬件错误回调次数。 */
    uint32_t receive_restart_errors; /* 主动接收DMA重启失败次数。 */
    uint32_t timeout_events; /* 主动数据流超过超时门限的次数。 */
    uint32_t abnormal_frames; /* 计入通信质量的异常主动包总数。 */
    uint32_t consecutive_abnormal_frames; /* 当前连续异常主动包数量。 */
    uint32_t max_consecutive_abnormal_frames; /* 本统计窗口内最大连续异常包数量。 */
    uint32_t quality_alarm_events; /* 达到连续异常门限并生成待处理错误的次数。 */
    uint32_t interactive_transactions; /* 已发起的V4交互事务数量。 */
    uint32_t interactive_errors; /* V4交互事务失败数量。 */
} multiparam_v4_diagnostics_t;

/* 最近一次主动流异常的原始证据；data保留标准帧或无线补零候选包。 */
typedef struct {
    uint8_t valid; /* 已保存至少一份异常包的标志。 */
    uint8_t reserved; /* 保留字节，固定为0，供后续结构兼容。 */
    uint16_t length; /* data中的有效异常包长度，单位字节。 */
    uint32_t error_code; /* 记录该包时得到的CPU2错误码。 */
    uint32_t received_tick; /* 保存异常包的本地毫秒节拍。 */
    uint8_t data[MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE]; /* 最近异常包的原始字节。 */
} multiparam_v4_abnormal_frame_t;

/*
 * 函数用途：初始化 V4.0 协议运行态和地址过滤条件。
 * 调用场景：自动识别开始前或确认 V4.0 设备后重新建立接收基线。
 * 关键约束：本函数不启动 UART6 DMA；主动接收需另行调用 StartActiveReceive。
 */
void MULTIPARAM_V4_Init(uint8_t expected_address);

/*
 * 函数用途：停止 V4.0 主动接收并清除当前通信方式。
 * 调用场景：传感器类型切换、UART6 所有权交接或重新识别前。
 * 关键约束：不修改已发布快照，便于故障诊断读取最后一帧。
 */
void MULTIPARAM_V4_Deinit(void);

/* Identification probes may expect a non-V4 device to stay silent. */
void MULTIPARAM_V4_SetIdentificationProbeMode(uint8_t enabled);

/*
 * 函数用途：启动 UART6 的 V4.0 主动上报常驻接收。
 * 调用场景：识别到主动模式或写参数65为0并收到应答后。
 * 关键约束：调用前必须保证 UART6 没有被其它协议占用。
 */
uint32_t MULTIPARAM_V4_StartActiveReceive(void);

/*
 * 函数用途：停止 V4.0 主动上报 DMA 接收。
 * 调用场景：取得交互事务窗口或交还 UART6 所有权前。
 * 关键约束：先停止DMA并清解析队列，最后释放UART6主动所有权。
 */
void MULTIPARAM_V4_StopActiveReceive(void);

/*
 * 函数用途：为既有同步等待流程保留一次主动流超时复核。
 * 调用场景：进入交互/主动模式的等待过程。
 * 关键约束：实时超时和接收恢复由SysTick/PendSV负责，本入口不是时序依赖点。
 */
void MULTIPARAM_V4_Service(void);

/*
 * 函数用途：挂起UART6主动帧的PendSV延后处理。
 * 调用场景：UART6接收、错误中断或测试字节注入完成后。
 * 关键约束：只置位请求，不做CRC、浮点解析、打印或等待。
 */
void MULTIPARAM_V4_RequestDeferredFromISR(void);

/*
 * 函数用途：在PendSV中有界搬运、重同步、校验并发布主动帧。
 * 调用场景：统一PendSV_Handler每轮调用一次。
 * 关键约束：每轮最多处理128字节和2个候选帧；有积压时重新挂起。
 * 返回值：非0表示本轮发布了至少一个新快照。
 */
uint8_t MULTIPARAM_V4_ProcessDeferredPendSV(void);

/*
 * 函数用途：记录 UART6 Receive-to-IDLE 事件中的新字节并切换接收缓冲。
 * 调用场景：HAL_UARTEx_RxEventCallback 确认来源为 USART6 后调用。
 * 关键约束：仅复制字节、更新计数和重启DMA，不解析浮点、不打印、不阻塞。
 */
void MULTIPARAM_V4_OnUartRxEventISR(uint16_t received_length);

/*
 * 函数用途：记录 UART6 硬件错误并请求线程态有界恢复。
 * 调用场景：HAL_UART_ErrorCallback 确认来源为 USART6 后调用。
 * 关键约束：中断中不打印、不延时、不执行协议解析。
 */
void MULTIPARAM_V4_OnUartErrorISR(uint32_t uart_error);
/*
 * 函数用途：从1ms系统节拍触发V4主动流超时确认和接收恢复。
 * 调用场景：SysTick在HAL_IncTick之后调用。
 * 关键约束：中断中只检查到期条件并挂起PendSV，不停止DMA、不解析、不打印。
 */
void MULTIPARAM_V4_TickFromISR(void);

/*
 * 函数用途：处理UART6异步终止接收完成事件并重新安排V4接收恢复。
 * 调用场景：HAL UART6 AbortReceive完成回调。
 * 关键约束：中断中只更新状态并挂起PendSV，不直接重启DMA或打印。
 */
void MULTIPARAM_V4_OnUartAbortReceiveCompleteISR(void);

/*
 * 函数用途：查询V4主动DMA是否正在运行。
 * 调用场景：协议状态检查和调试输出。
 * 关键约束：只返回软件状态，不代表快照新鲜或UART6可以被其它协议使用。
 */
uint8_t MULTIPARAM_V4_IsActiveReceiverRunning(void);

/*
 * 函数用途：向主动流注入一段受控字节并请求PendSV解析。
 * 调用场景：主机协议测试和不经过HAL回调的测试输入。
 * 关键约束：不直接解析、不打印；生产UART数据应由接收回调入口提供。
 */
void MULTIPARAM_V4_FeedBytes(const uint8_t *data, uint16_t length);

/*
 * 函数用途：校验并解析一帧标准72字节V4主动数据。
 * 调用场景：内部候选帧还原完成后或主机协议测试。
 * 关键约束：只处理标准帧；无线122字节补零帧必须先由内部候选解码器还原。
 */
uint32_t MULTIPARAM_V4_ParseActiveFrame(const uint8_t frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE],
                                        multiparam_v4_snapshot_t *snapshot);

/*
 * 函数用途：原子复制最近一次有效V4主动快照。
 * 调用场景：测量读取、状态确认和串口调试。
 * 关键约束：没有有效快照返回通信超时，不返回半更新数据。
 */
uint32_t MULTIPARAM_V4_CopyLatestSnapshot(multiparam_v4_snapshot_t *snapshot);

/*
 * 函数用途：打印识别窗口最近主动原帧或本次空接收结果。
 * 调用场景：自动识别主动监听阶段结束时。
 * 关键约束：只在线程态调用；不得从UART ISR或PendSV打印。
 */
void MULTIPARAM_V4_PrintActiveProbePacket(const char *operation, uint32_t result);

/*
 * 函数用途：开启或关闭V4交互探测原包日志。
 * 调用场景：自动识别R01/R67探测窗口。
 * 关键约束：正常测量保持关闭；开启时重置探测事务序号。
 */
void MULTIPARAM_V4_SetProbeTraceEnabled(uint8_t enabled);

/*
 * 函数用途：打印最近一次V4交互事务的TX、RX和失败阶段。
 * 调用场景：部件参数交互读取失败后。
 * 关键约束：只在线程态调用；主动上报读取失败不得打印可能属于旧事务的数据。
 */
void MULTIPARAM_V4_PrintLastTransactionPackets(const char *operation,
                                               uint32_t result);

/*
 * 函数用途：复制V4累计通信质量和事务计数。
 * 调用场景：串口通信质量查询。
 * 关键约束：读取不清零，输出必须为有效可写对象。
 */
void MULTIPARAM_V4_GetDiagnostics(multiparam_v4_diagnostics_t *diagnostics);

/*
 * 函数用途：原子复制最近一次主动流异常及其原始包。
 * 调用场景：串口异常包查询。
 * 关键约束：无记录时valid为0；读取不消费异常记录。
 */
void MULTIPARAM_V4_GetLastAbnormalFrame(multiparam_v4_abnormal_frame_t *frame);

/*
 * 函数用途：取走连续三个主动包异常产生的待锁存错误码。
 * 调用场景：主循环V4 Service之后。
 * 关键约束：只在线程态调用；读取后清零，单个事件只上抛一次。
 */
uint32_t MULTIPARAM_V4_TakeQualityError(void);

/*
 * 函数用途：清零V4累计诊断、连续异常和最近异常包。
 * 调用场景：串口维护命令开始新的质量观察窗口时。
 * 关键约束：保留通信方式、主动接收和最近有效快照，不改变业务运行状态。
 */
void MULTIPARAM_V4_ClearDiagnostics(void);

/*
 * 函数用途：判断最近主动快照是否仍在允许年龄内。
 * 调用场景：业务读取和功能状态确认门禁。
 * 关键约束：无快照返回否；不等待下一帧。
 */
uint8_t MULTIPARAM_V4_IsSnapshotFresh(uint32_t maximum_age_ms);

/*
 * 函数用途：取得最近主动快照年龄。
 * 调用场景：通信质量和状态调试输出。
 * 关键约束：无快照返回UINT32_MAX。
 */
uint32_t MULTIPARAM_V4_GetSnapshotAgeMs(void);

/*
 * 函数用途：计算V4八字节交互帧校验值。
 * 调用场景：请求组帧和应答校验。
 * 关键约束：纯计算，不访问UART6；调用方保证完整帧缓冲。
 */
uint8_t MULTIPARAM_V4_CalculateChecksum(const uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]);

/*
 * 函数用途：生成V4八字节交互请求帧。
 * 调用场景：读写、模式和功能命令发送前。
 * 关键约束：只组帧不发送，32位数据按小端编码。
 */
void MULTIPARAM_V4_BuildRequestFrame(uint8_t address,
                                     uint8_t function,
                                     uint32_t raw_value,
                                     uint8_t parameter,
                                     uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]);
/*
 * 函数用途：按生产严格地址策略校验V4交互应答。
 * 调用场景：普通交互事务收到完整八字节应答后。
 * 关键约束：广播探测地址例外不适用本入口。
 */
uint32_t MULTIPARAM_V4_ValidateReply(const uint8_t request[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
                                    const uint8_t reply[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]);

/*
 * 函数用途：读取一个V4参数的32位原始值。
 * 调用场景：整数、浮点和状态读取的底层入口。
 * 关键约束：仅交互模式可用；可读范围外返回调用条件错误。
 */
uint32_t MULTIPARAM_V4_ReadParamRaw(uint8_t parameter, uint32_t *raw_value);

/*
 * 函数用途：读取一个V4参数并解释为有符号32位整数。
 * 调用场景：编号和状态类寄存器读取。
 * 关键约束：只转换数据表示，具体业务范围由调用者判断。
 */
uint32_t MULTIPARAM_V4_ReadIntParam(uint8_t parameter, int32_t *value);

/*
 * 函数用途：读取一个V4参数并解释为有限浮点值。
 * 调用场景：交互模式读取测量和配置浮点参数。
 * 关键约束：NaN返回数据未就绪，无穷值返回格式异常，不伪造0值。
 */
uint32_t MULTIPARAM_V4_ReadFloatParam(uint8_t parameter, float *value);

/*
 * 函数用途：读取 V4 协议 R67 传感器号。
 * 调用场景：协议版本识别完成且通信已进入交互模式后调用。
 * 关键约束：编号0表示未初始化且仍为合法编号，负数无效；V4不得使用R22读取编号。
 */
uint32_t MULTIPARAM_V4_ReadSensorID(uint32_t *sensor_id);
/*
 * 函数用途：以单次、短超时事务读取一个浮点参数。
 * 调用场景：交互测量完成后的附加调试量刷新。
 * 关键约束：不做协议层重试，避免非关键刷新长时间阻塞核心测量流程。
 */
uint32_t MULTIPARAM_V4_ReadFloatParamOnce(uint8_t parameter, float *value);
/*
 * 函数用途：读取R02并解析当前测量模式和功能使能状态。
 * 调用场景：交互方式读取R04、R05、R11和R12前核对数据语义与异常位。
 * 关键约束：状态低四位无法映射时返回响应格式错误，不输出伪造模式。
 */
uint32_t MULTIPARAM_V4_ReadOperatingState(uint32_t *status_word,
                                          multiparam_v4_measurement_mode_t *mode,
                                          multiparam_v4_feature_state_t *feature);
/*
 * 函数用途：向允许写入的V4参数写入明确32位值。
 * 调用场景：交互维护配置，不包括R65通信方式。
 * 关键约束：写应答丢失时只读回确认，不盲目重发写命令。
 */
uint32_t MULTIPARAM_V4_WriteParamRaw(uint8_t parameter, uint32_t raw_value);

/*
 * 函数用途：广播读取R01并判断是否为多参数V4协议版本。
 * 调用场景：自动识别区分V3和V4。
 * 关键约束：版本不是4.0时仍返回实际有限值并报告不兼容；成功后锁定实际应答地址。
 */
uint32_t MULTIPARAM_V4_ProbeProtocolVersion(float *protocol_version);

/*
 * 函数用途：把V4传感器调整到密度模式。
 * 调用场景：密度业务和L/M功能启用之前。
 * 关键约束：已在密度模式时不重复发送D。
 */
uint32_t MULTIPARAM_V4_SelectDensityMode(void);

/*
 * 函数用途：把V4传感器调整到液位模式。
 * 调用场景：液位搜索、跟随和恢复。
 * 关键约束：最终状态必须无L/M附加功能。
 */
uint32_t MULTIPARAM_V4_SelectLevelMode(void);

/*
 * 函数用途：把V4测水功能调整为明确状态。
 * 调用场景：水位电容读取和串口调试。
 * 关键约束：仅密度模式可开启，并与零点霍尔互斥。
 */
uint32_t MULTIPARAM_V4_EnsureWaterEnabled(uint8_t enabled);

/*
 * 函数用途：把V4零点霍尔功能调整为明确状态。
 * 调用场景：磁零点读取和串口调试。
 * 关键约束：仅密度模式可开启，并与测水功能互斥。
 */
uint32_t MULTIPARAM_V4_EnsureMagneticZeroEnabled(uint8_t enabled);

/*
 * 函数用途：把V4从主动上发切换到交互通信。
 * 调用场景：执行模式、功能或参数事务之前。
 * 关键约束：主动帧结束后15至100ms窗口内发送，抢占时重新等待下一窗口。
 */
uint32_t MULTIPARAM_V4_EnterInteractive(void);

/*
 * 函数用途：把V4从交互通信恢复为主动上发。
 * 调用场景：交互业务完成后的必要收尾。
 * 关键约束：应答丢失时等待首个主动帧确认事实。
 */
uint32_t MULTIPARAM_V4_EnterActive(void);

/*
 * 函数用途：查询V4软件记录的当前通信方式。
 * 调用场景：驱动适配和串口状态输出。
 * 关键约束：只读软件状态，不读取R65。
 */
multiparam_v4_communication_mode_t MULTIPARAM_V4_GetCommunicationMode(void);

#endif /* MULTIPARAM_V4_COMMUNICATION_H */
