/*
 * multiparam_v4_communication.h
 * 多参数传感器通信协议 V4.0 的固定帧、主动上报和交互事务接口。
 */
#ifndef MULTIPARAM_V4_COMMUNICATION_H
#define MULTIPARAM_V4_COMMUNICATION_H

#include <stdint.h>

#define MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE       8U
#define MULTIPARAM_V4_ACTIVE_FRAME_SIZE            72U
#define MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT       15U
#define MULTIPARAM_V4_ACTIVE_DATA_SIZE             60U
#define MULTIPARAM_V4_ACTIVE_TIMEOUT_MS            1500U
#define MULTIPARAM_V4_ACTIVE_COMMAND_GUARD_MS      15U
#define MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE      122U
#define MULTIPARAM_V4_CONSECUTIVE_ERROR_LIMIT      3U
#define MULTIPARAM_V4_ANY_ADDRESS                  0xFFU
#define MULTIPARAM_V4_PROTOCOL_VERSION_VALUE       4.0f

typedef enum {
    MULTIPARAM_V4_COMMUNICATION_UNKNOWN = 0,
    MULTIPARAM_V4_COMMUNICATION_ACTIVE = 1,
    MULTIPARAM_V4_COMMUNICATION_INTERACTIVE = 2
} multiparam_v4_communication_mode_t;

typedef enum {
    MULTIPARAM_V4_MEASUREMENT_INVALID = 0,
    MULTIPARAM_V4_MEASUREMENT_DENSITY = 1,
    MULTIPARAM_V4_MEASUREMENT_LEVEL = 2
} multiparam_v4_measurement_mode_t;

typedef enum {
    MULTIPARAM_V4_FEATURE_NONE = 0,
    MULTIPARAM_V4_FEATURE_WATER = 1,
    MULTIPARAM_V4_FEATURE_MAGNETIC_ZERO = 2,
    MULTIPARAM_V4_FEATURE_INVALID = 3
} multiparam_v4_feature_state_t;

typedef struct {
    uint8_t valid;
    uint8_t address;
    uint16_t sequence;
    uint32_t received_tick;
    uint32_t generation;
    uint32_t raw_parameter[MULTIPARAM_V4_ACTIVE_PARAMETER_COUNT];
    float software_version;
    float protocol_version;
    uint32_t status_word;
    float magnetic_zero_voltage;
    int32_t measurement_frequency_hz;
    float water_capacitance_pf;
    float temperature_c;
    float density_kg_m3;
    float dynamic_viscosity_cp;
    float kinematic_viscosity_cst;
    float supply_voltage_v;
    float angle_x_deg;
    float angle_y_deg;
    float sweep_period_square_mean_45;
    float sweep_period_square_mean_22_5;
    multiparam_v4_measurement_mode_t measurement_mode;
    multiparam_v4_feature_state_t feature_state;
    uint8_t magnetic_zero_valid;
    uint8_t water_capacitance_valid;
} multiparam_v4_snapshot_t;

typedef struct {
    uint32_t received_bytes;
    uint32_t receive_events;
    uint32_t frames_seen;
    uint32_t valid_frames;
    uint32_t wireless_zero_padding_frames;
    uint32_t crc_errors;
    uint32_t fixed_field_errors;
    uint32_t address_errors;
    uint32_t value_errors;
    uint32_t short_receive_events;
    uint32_t resync_events;
    uint32_t duplicate_frames;
    uint32_t out_of_order_frames;
    uint32_t lost_frames;
    uint32_t sequence_resets;
    uint32_t stream_overflows;
    uint32_t uart_errors;
    uint32_t receive_restart_errors;
    uint32_t timeout_events;
    uint32_t abnormal_frames;
    uint32_t consecutive_abnormal_frames;
    uint32_t max_consecutive_abnormal_frames;
    uint32_t quality_alarm_events;
    uint32_t interactive_transactions;
    uint32_t interactive_errors;
} multiparam_v4_diagnostics_t;

typedef struct {
    uint8_t valid;
    uint8_t reserved;
    uint16_t length;
    uint32_t error_code;
    uint32_t received_tick;
    uint8_t data[MULTIPARAM_V4_ABNORMAL_FRAME_MAX_SIZE];
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

/*
 * 函数用途：启动 UART6 的 V4.0 主动上报常驻接收。
 * 调用场景：识别到主动模式或写参数65为0并收到应答后。
 * 关键约束：调用前必须保证 UART6 没有被其它协议占用。
 */
uint32_t MULTIPARAM_V4_StartActiveReceive(void);

/*
 * 函数用途：停止 V4.0 主动上报 DMA 接收。
 * 调用场景：取得交互事务窗口或交还 UART6 所有权前。
 */
void MULTIPARAM_V4_StopActiveReceive(void);

/*
 * 函数用途：在线程态处理UART6强制恢复和主动流超时诊断。
 * 调用场景：CPU2主循环及进入交互/主动模式的等待过程。
 * 关键约束：不再负责主动帧解包；主循环阻塞不影响正常主动帧发布。
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
/* TIM4中只检查恢复条件并挂起PendSV，不执行DMA停止或协议解析。 */
void MULTIPARAM_V4_PollRecoveryFromTimerISR(void);
/* 异步终止接收完成后只更新状态并重新挂起PendSV。 */
void MULTIPARAM_V4_OnUartAbortReceiveCompleteISR(void);

uint8_t MULTIPARAM_V4_IsActiveReceiverRunning(void);
void MULTIPARAM_V4_FeedBytes(const uint8_t *data, uint16_t length);
uint32_t MULTIPARAM_V4_ParseActiveFrame(const uint8_t frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE],
                                        multiparam_v4_snapshot_t *snapshot);
uint32_t MULTIPARAM_V4_CopyLatestSnapshot(multiparam_v4_snapshot_t *snapshot);
/* 仅由自动识别任务调用，输出最近主动原帧或本次监听的空接收结果。 */
void MULTIPARAM_V4_PrintActiveProbePacket(const char *operation, uint32_t result);
/* 自动识别窗口内开启交互原包日志；正常测量阶段必须保持关闭。 */
void MULTIPARAM_V4_SetProbeTraceEnabled(uint8_t enabled);
/* 部件参数读取失败时输出最近一次交互事务的TX/RX原始包和校验阶段。 */
void MULTIPARAM_V4_PrintLastTransactionPackets(const char *operation,
                                               uint32_t result);
void MULTIPARAM_V4_GetDiagnostics(multiparam_v4_diagnostics_t *diagnostics);
/* 原子复制最近一次主动流异常及其原始包；无记录时valid为0。 */
void MULTIPARAM_V4_GetLastAbnormalFrame(multiparam_v4_abnormal_frame_t *frame);
/* 在线程态取走连续3个主动包异常产生的待锁存错误码。 */
uint32_t MULTIPARAM_V4_TakeQualityError(void);
/* 清零V4累计诊断、连续异常和异常包，但保留通信方式、主动接收和最近快照。 */
void MULTIPARAM_V4_ClearDiagnostics(void);
uint8_t MULTIPARAM_V4_IsSnapshotFresh(uint32_t maximum_age_ms);
uint32_t MULTIPARAM_V4_GetSnapshotAgeMs(void);

uint8_t MULTIPARAM_V4_CalculateChecksum(const uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]);
void MULTIPARAM_V4_BuildRequestFrame(uint8_t address,
                                     uint8_t function,
                                     uint32_t raw_value,
                                     uint8_t parameter,
                                     uint8_t frame[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]);
uint32_t MULTIPARAM_V4_ValidateReply(const uint8_t request[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
                                    const uint8_t reply[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]);

uint32_t MULTIPARAM_V4_ReadParamRaw(uint8_t parameter, uint32_t *raw_value);
uint32_t MULTIPARAM_V4_ReadIntParam(uint8_t parameter, int32_t *value);
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
uint32_t MULTIPARAM_V4_WriteParamRaw(uint8_t parameter, uint32_t raw_value);
/**
 * @brief 读取R01并判断是否为多参数V4协议版本。
 * @param protocol_version R01为有限浮点数时返回实际版本；版本不是4.0时仍返回该值并报告版本不兼容。
 * @return NO_ERROR表示R01约等于4.0；其他值为通信、响应或版本错误。
 */
uint32_t MULTIPARAM_V4_ProbeProtocolVersion(float *protocol_version);
uint32_t MULTIPARAM_V4_SelectDensityMode(void);
uint32_t MULTIPARAM_V4_SelectLevelMode(void);
uint32_t MULTIPARAM_V4_EnsureWaterEnabled(uint8_t enabled);
uint32_t MULTIPARAM_V4_EnsureMagneticZeroEnabled(uint8_t enabled);
uint32_t MULTIPARAM_V4_EnterInteractive(void);
uint32_t MULTIPARAM_V4_EnterActive(void);
multiparam_v4_communication_mode_t MULTIPARAM_V4_GetCommunicationMode(void);

#endif /* MULTIPARAM_V4_COMMUNICATION_H */
