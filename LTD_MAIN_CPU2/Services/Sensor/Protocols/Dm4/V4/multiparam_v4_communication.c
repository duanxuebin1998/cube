/*
 * 模块职责：定义DM4-V4通信栈共享状态，并管理整套协议栈的初始化与反初始化。
 * 分层关系：codec只处理字节格式，active_stream负责主动帧DMA与解析，interactive负责
 * 8字节交互事务，diagnostics只累计并输出诊断信息。
 * 所有权约束：所有UART6实际收发必须通过active_stream或interactive取得对应所有权。
 */
#include "multiparam_v4_internal.h"

#include "error_log.h"
#include "main.h"
#include "my_crc.h"
#include "sensor_transport_config.h"
#include "sensor_uart6_owner.h"
#include "system_parameter.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* 当前目标地址以及active_stream独占写入的Receive-to-IDLE DMA运行状态。 */
uint8_t s_expected_address = MULTIPARAM_V4_ANY_ADDRESS;
uint8_t s_active_dma_buffer[MULTIPARAM_V4_ACTIVE_DMA_BUFFER_SIZE];
volatile uint16_t s_active_dma_consumed = 0U;
volatile uint8_t s_active_receive_requested = 0U;
volatile uint8_t s_active_receive_running = 0U;
volatile uint8_t s_receive_restart_pending = 0U;
volatile uint8_t s_receive_restart_in_progress = 0U;
volatile uint8_t s_receive_restart_wait_started = 0U;
volatile uint32_t s_receive_restart_wait_tick = 0U;
volatile uint8_t s_receive_abort_requested = 0U;
volatile uint8_t s_receive_abort_in_progress = 0U;

/* ISR生产、PendSV消费的主动字节流，以及单次有界延后解析使用的工作缓冲。 */
uint8_t s_stream_buffer[MULTIPARAM_V4_STREAM_BUFFER_SIZE];
volatile uint16_t s_stream_head = 0U;
volatile uint16_t s_stream_tail = 0U;
uint8_t s_deferred_bytes[MULTIPARAM_V4_DEFERRED_BYTE_BUDGET];
volatile uint8_t s_deferred_parse_requested = 0U;
uint8_t s_parse_buffer[MULTIPARAM_V4_PARSE_BUFFER_SIZE];
uint16_t s_parse_length = 0U;

/* 最近有效原帧、业务快照、异常证据和累计诊断；发布时必须保持同一代数据一致。 */
uint8_t s_latest_active_frame[MULTIPARAM_V4_ACTIVE_FRAME_SIZE];
volatile uint8_t s_latest_active_frame_valid = 0U;
multiparam_v4_abnormal_frame_t s_last_abnormal_frame;
multiparam_v4_snapshot_t s_latest_snapshot;
multiparam_v4_diagnostics_t s_diagnostics;

/* 主动序号质量监测、超时去重和待主循环锁存的通信质量错误。 */
uint8_t s_sequence_valid = 0U;
uint16_t s_last_sequence = 0U;
uint8_t s_timeout_latched = 0U;
uint8_t s_quality_alarm_latched = 0U;
uint32_t s_active_receive_start_tick = 0U;
volatile uint32_t s_pending_quality_error = NO_ERROR;

/* 当前通信方式及最近主动帧结束时间，用于15ms至100ms半双工交互窗口。 */
multiparam_v4_communication_mode_t s_communication_mode = MULTIPARAM_V4_COMMUNICATION_UNKNOWN;
volatile uint32_t s_last_active_frame_end_tick = 0U;
volatile uint8_t s_last_active_frame_end_valid = 0U;

/* 最近交互事务原包和自动识别探测序号，只供线程态诊断打印。 */
multiparam_v4_transaction_diagnostic_t s_last_transaction_diagnostic;
uint8_t s_probe_trace_enabled = 0U;
uint32_t s_probe_trace_transaction = 0U;

/*
 * 函数用途：初始化V4共享运行态、地址过滤和诊断基线。
 * 调用场景：上电探测开始或确认DM4-V4身份后重建协议栈。
 * 关键约束：先停止旧主动DMA；初始化本身不启动接收，调用方需显式StartActiveReceive。
 */
void MULTIPARAM_V4_Init(uint8_t expected_address)
{
    /* 初始化前先结束旧主动接收，防止复用残留DMA、序号和半双工时间戳。 */
    MULTIPARAM_V4_StopActiveReceive();
    s_expected_address = expected_address;
    s_stream_head = 0U;
    s_stream_tail = 0U;
    s_parse_length = 0U;
    s_sequence_valid = 0U;
    s_last_sequence = 0U;
    s_timeout_latched = 0U;
    s_quality_alarm_latched = 0U;
    s_active_receive_start_tick = 0U;
    s_pending_quality_error = NO_ERROR;
    s_deferred_parse_requested = 0U;
    s_receive_restart_in_progress = 0U;
    s_receive_restart_wait_started = 0U;
    s_receive_restart_wait_tick = 0U;
    s_receive_abort_requested = 0U;
    s_receive_abort_in_progress = 0U;
    s_last_active_frame_end_tick = 0U;
    s_last_active_frame_end_valid = 0U;
    s_communication_mode = MULTIPARAM_V4_COMMUNICATION_UNKNOWN;
    memset(&s_latest_snapshot, 0, sizeof(s_latest_snapshot));
    memset(s_latest_active_frame, 0, sizeof(s_latest_active_frame));
    s_latest_active_frame_valid = 0U;
    memset(&s_last_abnormal_frame, 0, sizeof(s_last_abnormal_frame));
    memset(&s_diagnostics, 0, sizeof(s_diagnostics));
}

/*
 * 函数用途：停止V4主动接收并把通信方式恢复为未知。
 * 调用场景：重新识别、传感器类型切换或协议栈退出。
 * 关键约束：先释放UART6主动所有权；只清待处理质量错误，不伪造新的错误上报。
 */
void MULTIPARAM_V4_Deinit(void)
{
    uint32_t primask;

    /* 先停止DMA并释放UART6，再清除待上抛错误和通信方式。 */
    MULTIPARAM_V4_StopActiveReceive();
    primask = __get_PRIMASK();
    __disable_irq();
    s_pending_quality_error = NO_ERROR;
    if (primask == 0U) {
        __enable_irq();
    }
    s_communication_mode = MULTIPARAM_V4_COMMUNICATION_UNKNOWN;
}

/*
 * 函数用途：查询V4协议栈当前记录的主动、交互或未知通信方式。
 * 调用场景：驱动适配、部件诊断和串口调试状态判断。
 * 关键约束：只返回软件状态，不主动读取R65，也不代表UART6一定空闲。
 */
multiparam_v4_communication_mode_t MULTIPARAM_V4_GetCommunicationMode(void)
{
    return s_communication_mode;
}
