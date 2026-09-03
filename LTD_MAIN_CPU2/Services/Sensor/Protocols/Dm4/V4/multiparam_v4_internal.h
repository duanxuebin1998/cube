/*
 * 模块职责：声明DM4-V4各实现文件共享的私有常量、运行状态和内部协作接口。
 * 可见性边界：仅V4 codec、主动流、交互事务和诊断实现可包含，本头文件不是业务API。
 * 所有权约束：共享变量按注释指定单一写入方，跨中断字段必须通过既定临界区访问。
 */
#ifndef MULTIPARAM_V4_INTERNAL_H_
/* 本头文件的包含保护标记。 */
#define MULTIPARAM_V4_INTERNAL_H_

#include "multiparam_v4_communication.h"

/* 主动帧格式、无线补零兼容和流解析容量。 */
#define MULTIPARAM_V4_ACTIVE_HEADER0                 0x55U
/* V4主动帧第二个同步字节0xAA。 */
#define MULTIPARAM_V4_ACTIVE_HEADER1                 0xAAU
/* V4主动帧格式版本字段期望值。 */
#define MULTIPARAM_V4_ACTIVE_FORMAT_VERSION          0x01U
/* V4主动测量帧类型字段期望值。 */
#define MULTIPARAM_V4_ACTIVE_FRAME_TYPE              0xA1U
/* Receive-to-IDLE单个DMA接收缓冲容量，单位字节。 */
#define MULTIPARAM_V4_ACTIVE_DMA_BUFFER_SIZE         128U
/* 无线滑环插入50字节零填充的标准帧偏移，单位字节。 */
#define MULTIPARAM_V4_WIRELESS_ZERO_PADDING_OFFSET    50U
/* 无线滑环异常插入的连续零填充长度，单位字节。 */
#define MULTIPARAM_V4_WIRELESS_ZERO_PADDING_SIZE      50U
/* 无线滑环补零后的候选主动帧总长度，单位字节。 */
#define MULTIPARAM_V4_WIRELESS_PADDED_FRAME_SIZE      \
    (MULTIPARAM_V4_ACTIVE_FRAME_SIZE + MULTIPARAM_V4_WIRELESS_ZERO_PADDING_SIZE)
/* ISR到PendSV之间的主动字节环形缓冲容量，单位字节。 */
#define MULTIPARAM_V4_STREAM_BUFFER_SIZE             256U
/* PendSV流重组使用的线性解析缓冲容量，单位字节。 */
#define MULTIPARAM_V4_PARSE_BUFFER_SIZE              256U
/* 单次PendSV最多搬运和检查的主动流字节数。 */
#define MULTIPARAM_V4_DEFERRED_BYTE_BUDGET           128U
/* 单次PendSV最多发布的主动帧数量。 */
#define MULTIPARAM_V4_DEFERRED_FRAME_BUDGET          2U

/* 交互事务超时、主动转交互窗口及接收恢复时序。 */
#define MULTIPARAM_V4_TRANSACTION_TIMEOUT_MS         DSM_CMD_TIMEOUT
/* 上一笔V4事务结束到下一次CPU2请求发送的最小间隔，单位ms。 */
#define MULTIPARAM_V4_INTER_FRAME_GAP_MS              30U
/* 切换为主动通信后等待首帧确认的超时，单位ms。 */
#define MULTIPARAM_V4_ACTIVE_RESPONSE_TIMEOUT_MS     300U
/* 主动帧结束后可发送切换命令的最晚窗口，单位ms。 */
#define MULTIPARAM_V4_ACTIVE_WINDOW_LATEST_MS        100U
/* 停止主动流并取得交互窗口的总超时，单位ms。 */
#define MULTIPARAM_V4_STOP_ACTIVE_TOTAL_TIMEOUT_MS    3000U
/* 接收停止未完成时请求强制终止前的等待时间，单位ms。 */
#define MULTIPARAM_V4_RX_FORCE_ABORT_DELAY_MS         100U
/* V4幂等交互事务最大尝试次数。 */
#define MULTIPARAM_V4_MAX_RETRY                       UART6_COMM_MAX_RETRY

/* V4寄存器、功能码和当前临时广播发包策略。 */
#define MULTIPARAM_V4_PARAM_PROTOCOL_VERSION          0x01U
/* V4线协议广播地址0，仅用于允许广播的请求。 */
#define MULTIPARAM_V4_BROADCAST_ADDRESS               0x00U
/* 尚未从有效应答锁定地址时使用的默认节点地址。 */
#define MULTIPARAM_V4_DEFAULT_ADDRESS                 0x01U
/* 现场调试期间强制V4请求使用广播地址的临时开关。 */
#define MULTIPARAM_V4_TEMP_FORCE_BROADCAST_TX          1U
/* V4运行状态寄存器R02参数号。 */
#define MULTIPARAM_V4_PARAM_STATUS                    0x02U
/* V4通信方式寄存器R65参数号0x41。 */
#define MULTIPARAM_V4_PARAM_COMMUNICATION_MODE        0x41U
/* V4传感器编号寄存器R67参数号0x43。 */
#define MULTIPARAM_V4_PARAM_SENSOR_ID                 0x43U
/* V4当前允许访问的最大参数号。 */
#define MULTIPARAM_V4_PARAM_MAX                       0x9FU
/* V4读取参数功能码R。 */
#define MULTIPARAM_V4_FUNCTION_READ                   ((uint8_t)'R')
/* V4写入参数功能码W。 */
#define MULTIPARAM_V4_FUNCTION_WRITE                  ((uint8_t)'W')
/* V4切换密度模式功能码D。 */
#define MULTIPARAM_V4_FUNCTION_DENSITY                ((uint8_t)'D')
/* V4切换水位电容功能码L。 */
#define MULTIPARAM_V4_FUNCTION_WATER                  ((uint8_t)'L')
/* V4切换零点霍尔功能码M。 */
#define MULTIPARAM_V4_FUNCTION_MAGNETIC_ZERO          ((uint8_t)'M')
/* V4切换液位模式功能码S。 */
#define MULTIPARAM_V4_FUNCTION_LEVEL                  ((uint8_t)'S')

/*
 * 应答地址策略只在R01广播探测时放宽：广播请求允许设备用自身地址应答，其余事务严格匹配。
 * 放宽策略不能用于普通寄存器读写，否则无线链路上的其它传感器应答可能被误接收。
 */
typedef enum {
    MULTIPARAM_V4_REPLY_ADDRESS_STRICT = 0, /* 应答地址必须与非广播请求地址完全一致。 */
    MULTIPARAM_V4_REPLY_ADDRESS_PROTOCOL_PROBE = 1 /* R01广播探测允许设备用自身地址应答。 */
} multiparam_v4_reply_address_policy_t;

/*
 * 主动流候选结果区分“等待更多字节”和“当前前缀已无效”，避免分包到达时过早丢弃帧头。
 * VALID表示同时完成标准帧或无线中插50字节零填充兼容解析，并给出实际消费长度。
 */
typedef enum {
    MULTIPARAM_V4_ACTIVE_CANDIDATE_NEED_MORE = 0, /* 当前前缀可能有效，需要等待更多字节。 */
    MULTIPARAM_V4_ACTIVE_CANDIDATE_INVALID = 1, /* 当前前缀不可能组成有效主动帧。 */
    MULTIPARAM_V4_ACTIVE_CANDIDATE_VALID = 2 /* 已还原并校验一帧标准或无线补零主动帧。 */
} multiparam_v4_active_candidate_state_t;

/*
 * 保存最近一次交互事务的原始包和UART现场，供业务校验失败时打印完整通信证据。
 * stage指向静态字符串，不拥有内存；该快照只用于诊断，不参与下一次协议判定。
 */
typedef struct {
    uint8_t request[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]; /* 最近交互事务实际发送的8字节请求。 */
    uint8_t reply[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE]; /* 最近交互事务收到的前8字节应答。 */
    uint16_t received_length; /* 最近事务实际接收长度，单位字节。 */
    uint8_t valid; /* 本快照是否对应一次真正启动的UART事务。 */
    uint8_t failure_packets_printed; /* 当前事务的失败原包是否已经输出，供上层去重。 */
    uint32_t elapsed_ms; /* 从发送开始到事务结束的耗时，单位ms。 */
    uint32_t uart_error; /* 事务结束时锁存的HAL UART硬件错误位。 */
    uint32_t uart_state; /* 事务结束时锁存的HAL UART状态位。 */
    const char *stage; /* 指向静态阶段文本，描述失败发生在传输或哪一项协议校验。 */
} multiparam_v4_transaction_diagnostic_t;

/*
 * 跨V4子模块共享状态按“单一写入者”维护：codec只解析，active_stream维护DMA/流状态，
 * diagnostics维护计数和异常包，interactive维护通信方式及事务快照。其它模块只能通过接口读取。
 */
extern uint8_t s_expected_address;
extern uint8_t s_active_dma_buffer[MULTIPARAM_V4_ACTIVE_DMA_BUFFER_SIZE];
extern volatile uint16_t s_active_dma_consumed;
extern volatile uint8_t s_active_receive_requested;
extern volatile uint8_t s_active_receive_running;
extern volatile uint8_t s_receive_restart_pending;
extern volatile uint8_t s_receive_restart_in_progress;
extern volatile uint8_t s_receive_restart_wait_started;
extern volatile uint32_t s_receive_restart_wait_tick;
extern volatile uint8_t s_receive_abort_requested;
extern volatile uint8_t s_receive_abort_in_progress;
extern uint8_t s_stream_buffer[MULTIPARAM_V4_STREAM_BUFFER_SIZE];
extern volatile uint16_t s_stream_head;
extern volatile uint16_t s_stream_tail;
extern uint8_t s_deferred_bytes[MULTIPARAM_V4_DEFERRED_BYTE_BUDGET];
extern volatile uint8_t s_deferred_parse_requested;
extern uint8_t s_parse_buffer[MULTIPARAM_V4_PARSE_BUFFER_SIZE];
extern uint16_t s_parse_length;
extern uint8_t s_latest_active_frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE];
extern volatile uint8_t s_latest_active_frame_valid;
extern multiparam_v4_abnormal_frame_t s_last_abnormal_frame;
extern multiparam_v4_snapshot_t s_latest_snapshot;
extern multiparam_v4_diagnostics_t s_diagnostics;
extern uint8_t s_sequence_valid;
extern uint16_t s_last_sequence;
extern uint8_t s_timeout_latched;
extern uint8_t s_quality_alarm_latched;
extern uint32_t s_active_receive_start_tick;
extern volatile uint32_t s_pending_quality_error;
extern multiparam_v4_communication_mode_t s_communication_mode;
extern volatile uint32_t s_last_active_frame_end_tick;
extern volatile uint8_t s_last_active_frame_end_valid;
extern multiparam_v4_transaction_diagnostic_t s_last_transaction_diagnostic;
extern uint8_t s_probe_trace_enabled;
extern uint32_t s_probe_trace_transaction;

/* 编解码内部接口：不访问UART6，也不修改主动接收状态。 */
uint32_t MULTIPARAM_V4_DecodeU32Le(const uint8_t *data);
float MULTIPARAM_V4_RawToFloat(uint32_t raw);
uint8_t MULTIPARAM_V4_IsReadableParameter(uint8_t parameter);
uint8_t MULTIPARAM_V4_IsZeroRange(const uint8_t *data, uint16_t length);
void MULTIPARAM_V4_DecodeOperatingState(uint32_t status_word,
                                        multiparam_v4_operating_state_t *operating_state);
multiparam_v4_active_candidate_state_t MULTIPARAM_V4_DecodeActiveCandidate(
    const uint8_t *data,
    uint16_t available_length,
    multiparam_v4_snapshot_t *candidate,
    uint8_t canonical_frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE],
    uint16_t *consumed_length,
    uint8_t *wireless_padding_used,
    uint32_t *decode_result);
uint32_t MULTIPARAM_V4_ValidateReplyInternal(
    const uint8_t request[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
    const uint8_t reply[MULTIPARAM_V4_INTERACTIVE_FRAME_SIZE],
    multiparam_v4_reply_address_policy_t address_policy,
    const char **stage);

/* 诊断内部接口：记录探测、质量和最近交互事务，不改变协议重试结果。 */
void MULTIPARAM_V4_PrintProbePacket(const uint8_t request[8],
                                    const char *direction,
                                    const uint8_t *bytes,
                                    uint16_t length,
                                    uint32_t result,
                                    const char *stage);
void MULTIPARAM_V4_RecordQualityAnomaly(uint32_t error_code,
                                        uint32_t counts_for_alarm,
                                        const uint8_t *frame,
                                        uint16_t frame_length);
void MULTIPARAM_V4_RecordQualitySuccess(void);
uint32_t MULTIPARAM_V4_SaturatingAdd(uint32_t value, uint32_t increment);
void MULTIPARAM_V4_BeginTransactionDiagnostic(const uint8_t request[8]);
void MULTIPARAM_V4_LogProtocolProbeFailure(uint32_t error_code);
void MULTIPARAM_V4_RecordTransactionDiagnostic(const uint8_t reply[8],
                                                uint16_t received_length,
                                                uint32_t start_tick,
                                                const char *stage);
void MULTIPARAM_V4_PrintTransactionFailureAttempt(uint32_t result,
                                                  uint32_t attempt,
                                                  uint32_t max_attempts);

/*
 * 主动接收内部接口：调用方必须已取得V4对应UART6所有权。
 * ISR相关入口只更新定长状态并请求PendSV，不允许打印、阻塞或执行协议事务。
 */
uint32_t MULTIPARAM_V4_StartReceiveDmaInternal(void);
void MULTIPARAM_V4_MarkActiveFrameEnd(void);
void MULTIPARAM_V4_WaitActiveFrameGuard(void);
uint32_t MULTIPARAM_V4_ResumeActiveReceivePreservingSnapshot(void);

#endif /* MULTIPARAM_V4_INTERNAL_H_ */
