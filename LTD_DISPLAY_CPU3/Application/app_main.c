/*
 * app_main.c
 *
 *  Created on: Nov 18, 2025
 *      Author: duan xuebin
 */
#include "cpu2_communicate.h"
#include "display.h"
#include "system_parameter.h"
#include "DSM_communication.h"
#include "wartsila_modbus_communication.h"
#include "cpu3_comm_display_params.h"
#include "cpu3_clock.h"
#include "ltd_modbus_slave.h"
#include "lh_modbus_slave.h"
#include "si_modbus_slave.h"
#include "protocol_switch_frame.h"
#include "address.h"
#include "app_version.h"
#include "cpu3_debug_log.h"
#include <string.h>

#define CPU2_POLL_PERIOD_MS 100u /* CPU2 轮询调度门限；主循环阻塞时实际请求间隔可大于 100 ms。 */
#define CPU3_COMM_STATUS_LOG_INTERVAL_MS 5000U /* 外部COM口通信摘要周期。 */
#define CPU3_COMM_EVENT_LOG_INTERVAL_MS 1000U /* 同一端口异常事件的最短打印间隔。 */
#define CPU3_UART_RECOVERY_RETRY_MS 100U /* UART异常或收满后由主循环执行单次恢复的退避时间。 */
#define CPU3_UART_COUNT 4U /* 三路外部COM口加CPU2板间UART5。 */
#define CPU3_EXTERNAL_PORT_STARTUP_GUARD_MS 1000U /* 外部COM初始化后的非阻塞硬件稳定保护时间。 */

/* CPU3 外部端口配置无效时返回的保留结果码 0xFFFFFFFE；用于区分未配置协议与协议处理器自身返回的 Modbus 错误。 */
#define CPU3_PORT_RESULT_INVALID_CONFIG 0xFFFFFFFEUL
/* CPU3 外部端口找不到对应协议处理器时返回的保留结果码 0xFFFFFFFD；该值不得与正常协议结果码复用。 */
#define CPU3_PORT_RESULT_INVALID_HANDLER 0xFFFFFFFDUL


/**
 * @brief 将 UART5 板间 RS485 收发器切回接收方向。
 */
static inline void RS485_RecvMode(void)
{
    RS485_SET_RECV_MODE();
}

/* ====== 每口 TX busy + 单帧 pending 队列（忙时缓存 1 帧）====== */
volatile uint8_t  g_tx_busy_com1 = 0; /* COM1（UART6）发送 DMA 占用标志；主循环启动发送时置位，最后一帧发送完成或 UART 恢复路径释放。 */
volatile uint8_t  g_tx_busy_com2 = 0; /* COM2（UART2）发送 DMA 占用标志；主循环启动发送时置位，最后一帧发送完成或 UART 恢复路径释放。 */
volatile uint8_t  g_tx_busy_com3 = 0; /* COM3（UART3）发送 DMA 占用标志；主循环启动发送时置位，最后一帧发送完成或 UART 恢复路径释放。 */

static uint16_t g_tx_pending_len_com1 = 0; /* COM1 当前单帧待发缓存的有效字节数；0 表示没有排队帧，发送完成中断取走后立即清零。 */
static uint16_t g_tx_pending_len_com2 = 0; /* COM2 当前单帧待发缓存的有效字节数；0 表示没有排队帧，发送完成中断取走后立即清零。 */
static uint16_t g_tx_pending_len_com3 = 0; /* COM3 当前单帧待发缓存的有效字节数；0 表示没有排队帧，发送完成中断取走后立即清零。 */

static uint8_t  g_tx_pending_buf_com1[256]; /* COM1 忙碌时保存下一帧响应的 256 字节单帧缓存；仅前 g_tx_pending_len_com1 字节有效，当前 DMA 完成后续发。 */
static uint8_t  g_tx_pending_buf_com2[256]; /* COM2 忙碌时保存下一帧响应的 256 字节单帧缓存；仅前 g_tx_pending_len_com2 字节有效，当前 DMA 完成后续发。 */
static uint8_t  g_tx_pending_buf_com3[256]; /* COM3 忙碌时保存下一帧响应的 256 字节单帧缓存；仅前 g_tx_pending_len_com3 字节有效，当前 DMA 完成后续发。 */

/* 统计：如果 pending 已有帧又来新帧，会覆盖旧帧（可观察是否需要更大队列） */
static uint32_t g_tx_pending_overwrite_com1 = 0; /* COM1 单帧待发缓存尚未发送时又被新响应覆盖的累计次数。 */
static uint32_t g_tx_pending_overwrite_com2 = 0; /* COM2 单帧待发缓存尚未发送时又被新响应覆盖的累计次数。 */
static uint32_t g_tx_pending_overwrite_com3 = 0; /* COM3 单帧待发缓存尚未发送时又被新响应覆盖的累计次数。 */


/* UI/参数修改后置位，主循环应用 */
volatile uint8_t g_cpu3_uart_reinit_pending = 0;

typedef enum
{
    /* 每个外部串口的协议切换发送状态；先等待切换响应发送完成，再在前台安全应用新的串口配置。 */
    PROTOCOL_SWITCH_PENDING_NONE = 0, /* 当前串口没有待完成的协议切换。 */
    PROTOCOL_SWITCH_PENDING_TX, /* 切换响应帧正在发送，尚不能重配串口。 */
    PROTOCOL_SWITCH_PENDING_APPLY /* 响应已发送完成，等待前台应用目标协议和串口参数。 */
} ProtocolSwitchPendingState;

/* 三路外部 COM 口各自的协议切换发送状态；UART 发送完成回调推进状态，前台主循环消费并清零。 */
static volatile uint8_t g_protocol_switch_pending_state[3] = {0U, 0U, 0U};
/* 三路外部 COM 口待应用的目标协议；只有对应切换响应发送完成后才写入持久化配置。 */
static ComProtocolType g_protocol_switch_target[3] = {
    COM_PROTO_DSM,
    COM_PROTO_DSM,
    COM_PROTO_DSM
};
static uint32_t s_cpu3_boot_start_tick = 0U; /* CPU3进入应用初始化的时刻，用于启动阶段耗时日志。 */
static uint32_t s_external_ports_ready_tick = 0U; /* 外部COM允许处理协议帧的启动保护截止时刻。 */
static bool s_external_ports_ready_logged = false; /* 外部COM启动保护结束日志是否已输出。 */
static bool s_cpu2_startup_ready_logged = false; /* CPU2首次完整启动快照日志是否已输出。 */

typedef struct {
    /* 单个外部串口的累计通信健康快照；前台与中断回调共享的计数使用 volatile，统计值只用于诊断，不作为协议状态机控制条件。 */
    uint32_t rx_frame_count; /* 接收到完整协议帧的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t tx_accept_count; /* 接受发送请求的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t tx_queued_count; /* 因发送忙而进入单帧等待队列的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t tx_complete_count; /* DMA 发送完成的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t process_failure_count; /* 协议处理失败的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t ignored_frame_count; /* 协议判定为无需响应帧的累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t uart_error_count; /* UART 错误事件累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t uart_ore_count; /* UART 接收溢出错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t uart_fe_count; /* UART 帧格式错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t uart_ne_count; /* UART 噪声错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t uart_pe_count; /* UART 奇偶校验错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t rx_overflow_count; /* 接收缓冲区溢出累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t rx_recovery_count; /* 接收路径成功恢复累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t rx_recovery_fail_count; /* 接收路径恢复失败累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t tx_dma_start_fail_count; /* 发送 DMA 启动失败累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    volatile uint32_t tx_dma_continue_fail_count; /* 等待队列 DMA 续发失败累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t last_process_result; /* 最近一次协议帧处理返回码，供串口健康诊断保留具体失败上下文。 */
} Cpu3PortCommStats;

/* 三路外部 COM 口的通信健康累计统计，供诊断查询读取。 */
static Cpu3PortCommStats s_port_comm_stats[3] = {0};
/* 三路外部 COM 口由中断挂起、等待前台归因和记录的 UART 错误位。 */
static volatile uint32_t s_port_uart_error_pending[3] = {0U, 0U, 0U};
/* 三路外部 COM 口错误是否发生在发送阶段的标志，用于区分收发恢复路径。 */
static volatile uint8_t s_port_uart_error_during_tx[3] = {0U, 0U, 0U};
/* 三路外部 COM 口 DMA 续发失败的挂起计数，由前台并入通信健康统计。 */
static volatile uint32_t s_port_tx_continue_fail_pending[3] = {0U, 0U, 0U};
/* 各 UART 接收异常等待前台执行轻量恢复的标志数组。 */
static volatile uint8_t s_uart_rx_recovery_pending[CPU3_UART_COUNT] = {0U, 0U, 0U, 0U};
/* 各 UART 轻量恢复失败后等待前台完整重初始化的标志数组。 */
static volatile uint8_t s_uart_full_reinit_pending[CPU3_UART_COUNT] = {0U, 0U, 0U, 0U};
/* 各 UART 允许再次执行接收恢复的最早 HAL 毫秒节拍。 */
static volatile uint32_t s_uart_rx_recovery_due_tick[CPU3_UART_COUNT] = {0U, 0U, 0U, 0U};
/* UART5 接收环形缓冲区溢出的累计次数。 */
static volatile uint32_t s_uart5_rx_overflow_count = 0U;
/* UART5 接收路径成功恢复的累计次数。 */
static volatile uint32_t s_uart5_rx_recovery_count = 0U;
/* UART5 接收路径恢复失败的累计次数。 */
static volatile uint32_t s_uart5_rx_recovery_fail_count = 0U;
/* 前台通信健康处理代际；每完成一轮挂起错误和恢复事件消费后递增，供快照一致性检查。 */
static volatile uint32_t s_foreground_health_generation = 0U;

/**
 * @brief 按零基端口索引返回 COM1、COM2 或 COM3 的当前配置。
 *
 * @param port_idx 零基外部串口索引。
 * @return 成功时返回指向按零基端口索引返回 COM1、COM2 或 COM3 的当前配置的指针；输入非法或未找到匹配项时返回 NULL。
 */
static const ComPortConfig* cpu3_get_port_cfg(uint8_t port_idx);
/**
 * @brief 输出三个外部COM口当前生效配置。
 *
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 */
static void cpu3_log_all_port_configs(const char *reason);
/**
 * @brief 在主循环延后输出UART异常，并周期汇总三个外部COM口通信健康状态。
 *
 * 调用场景：App_MainLoop每轮调用，内部按5秒周期限流摘要。
 * 函数由 App_MainLoop 每轮调用，但每个外部 COM 口的异常事件分别按 CPU3_COMM_EVENT_LOG_INTERVAL_MS 限流，避免持续 UART 故障占满调试串口。
 * 读取 ISR 写入的 UART 错误标志、错误发生阶段和续发 DMA 失败计数时，先保存 PRIMASK 并关闭中断，再一次性取走并清零待处理值；临界区结束后只在进入前允许中断时重新使能中断。
 * 线程态根据锁存值输出端口、发送或接收阶段、ORE、FE、NE、PE 以及 DMA 续发失败统计，ISR 本身只累计标志和计数，不执行格式化打印。
 * 达到 CPU3_COMM_STATUS_LOG_INTERVAL_MS 后，逐端口汇总协议、收发、忽略帧、处理失败、UART 异常、接收恢复、DMA 启动或续发失败及待发队列状态，并另行汇总 CPU2
 * UART5 的收满和恢复情况。
 *
 * @note 关键约束：ISR只写pending标志，所有阻塞打印均在本函数执行。
 * @note 静态节拍使用无符号减法，允许 HAL_GetTick 自然回绕；不得把本函数的格式化日志移回 UART 中断上下文。
 */
static void cpu3_comm_debug_task(void);
/**
 * @brief 取消尚未完成应答的协议切换请求。
 *
 * @details 调用场景：DMA 发送启动失败或 UART 错误恢复时调用。
 * @note 关键约束：只清除对应 COM 口，不影响其它端口已经排队的切换请求。
 *
 * @param port_idx 零基外部串口索引。
 */
static void cpu3_cancel_protocol_switch(uint8_t port_idx);
/**
 * @brief 把一个端口切回 RS485 接收方向，并按当前参数只重初始化该端口。
 *
 * @details 调用场景：启动、参数重配置、协议切换和完整恢复队列。
 * @note 关键约束：不得重配其它端口，避免中断无关的外部通信。
 *
 * @param port_idx 零基外部串口索引。
 * @return true 表示端口编号有效，收发方向已切回接收，且对应 UART 已按当前参数重新初始化；false 表示端口编号不在 1～3，或 Cpu3_ReinitPortUart 未能完成该端口重初始化。
 */
static bool cpu3_reinit_external_port(uint8_t port_idx);
/**
 * @brief 逐端口应用当前配置，仅把失败端口投递完整重初始化，避免干扰其它COM口。
 *
 * @return true 表示三路外部 COM 口均按当前配置恢复成功；任一端口失败并进入后续恢复队列时返回 false。
 */
static bool cpu3_reinit_all_external_ports(void);

/**
 * @brief 返回外部端口的统一日志模块名。
 *
 * @param port_idx 零基外部串口索引。
 * @return 返回外部端口的统一日志模块名对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *cpu3_port_module_text(uint8_t port_idx)
{
    switch (port_idx) {
    case 1U:
        return "COM1";
    case 2U:
        return "COM2";
    case 3U:
        return "COM3";
    default:
        return "COM?";
    }
}

/**
 * @brief 返回外部协议的现场可读名称。
 *
 * @param protocol 待判断、显示或写入的协议枚举值。
 * @return 返回外部协议的现场可读名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *cpu3_protocol_text(ComProtocolType protocol)
{
    switch (protocol) {
    case COM_PROTO_DSM:
        return "DSM";
    case COM_PROTO_WARTSILA:
        return "WARTSILA";
    case COM_PROTO_LTD:
        return "LTD";
    case COM_PROTO_LH:
        return "LH";
    case COM_PROTO_SI:
        return "SI";
    case COM_PROTO_2:
        return "预留2";
    default:
        return "未知";
    }
}

/**
 * @brief 返回校验位配置名称。
 *
 * @param parity 串口校验位枚举值。
 * @return 返回校验位配置名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *cpu3_parity_text(ComParityType parity)
{
    switch (parity) {
    case COM_PARITY_EVEN:
        return "偶";
    case COM_PARITY_ODD:
        return "奇";
    case COM_PARITY_NONE:
    default:
        return "无";
    }
}

/**
 * @brief 返回协议分发结果的统一说明。
 *
 * @param protocol 待判断、显示或写入的协议枚举值。
 * @param result 当前协议处理器返回的 CPU3_PORT_RESULT 或协议结果码，用于选择统一诊断文字。
 * @return 返回协议分发结果的统一说明对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *cpu3_port_result_text(ComProtocolType protocol, uint32_t result)
{
    if (result == CPU3_PORT_RESULT_INVALID_CONFIG) {
        return "端口配置不存在";
    }
    if (result == CPU3_PORT_RESULT_INVALID_HANDLER) {
        return "协议未配置处理器";
    }
    if (result == 0U) {
        return "处理成功";
    }
    switch (result) {
    case 1U:
        return "帧长度不匹配";
    case 2U:
        return "CRC校验失败";
    case 3U:
        return "从机地址不匹配";
    case 4U:
        return "功能码不支持";
    default:
        return "未知处理结果";
    }
}

/**
 * @brief 地址不匹配属于多机总线常见流量，其余失败默认提升为警告。
 *
 * @param result 当前端口分发结果码；成功和地址不匹配不告警，其余失败提升为警告。
 * @return true 表示该分发结果需要提升为警告；地址不匹配等正常总线旁路流量返回 false。
 */
static bool cpu3_port_result_needs_warning(uint32_t result)
{
    if ((result == CPU3_PORT_RESULT_INVALID_CONFIG) ||
        (result == CPU3_PORT_RESULT_INVALID_HANDLER)) {
        return true;
    }
    if ((result == 0U) || (result == 3U)) {
        return false;
    }
    return true;
}

/**
 * @brief 把UART句柄映射为恢复状态下标：COM1/2/3为0/1/2，板间UART5为3。
 *
 * @param huart 目标 UART 外设句柄。
 * @return 返回恢复状态数组下标：COM1、COM2、COM3、UART5 分别为 0、1、2、3；未知句柄返回 -1。
 */
static int8_t cpu3_uart_recovery_index(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return -1;
    }
    if (huart->Instance == USART6) {
        return 0;
    }
    if (huart->Instance == USART2) {
        return 1;
    }
    if (huart->Instance == USART3) {
        return 2;
    }
    if (huart->Instance == UART5) {
        return 3;
    }
    return -1;
}

/**
 * @brief 锁存一次DMA恢复请求；中断和主循环均可调用，本函数不打印、不循环重试。
 *
 * @param huart 目标 UART 外设句柄。
 */
void CPU3_UartScheduleRxRecovery(UART_HandleTypeDef *huart)
{
    int8_t index = cpu3_uart_recovery_index(huart);

    if (index < 0) {
        return;
    }
    s_uart_rx_recovery_due_tick[(uint8_t)index] = HAL_GetTick() + CPU3_UART_RECOVERY_RETRY_MS;
    s_uart_rx_recovery_pending[(uint8_t)index] = 1U;
}

/**
 * @brief 完整重初始化只用于三个外部COM口；普通DMA恢复不得降级已经锁存的完整请求。
 *
 * @param huart 目标 UART 外设句柄。
 */
static void cpu3_schedule_uart_full_reinit(UART_HandleTypeDef *huart)
{
    int8_t index = cpu3_uart_recovery_index(huart);

    if ((index < 0) || (index >= 3)) {
        CPU3_UartScheduleRxRecovery(huart);
        return;
    }

    s_uart_full_reinit_pending[(uint8_t)index] = 1U;
    CPU3_UartScheduleRxRecovery(huart);
}

/**
 * @brief 重新启动接收时先关闭IDLEIE，只有DMA真正启动成功后才重新开启。
 *
 * 启动失败时保持端口隔离并投递主循环退避恢复，禁止留下空DMA的IDLE中断。
 *
 * @param huart 目标 UART 外设句柄。
 * @param rx_buf 重新挂载 UART 接收 DMA 的目标缓冲区首地址；可写容量由 rx_buf_size 指定。
 * @param rx_buf_size 目标 UART DMA 接收缓冲区容量，单位字节。
 * @param set_recv_mode DMA 接收启动前用于将对应 RS485 收发器切到接收方向的回调函数。
 * @return true 表示 DMA 接收已启动且 IDLE 中断已重新使能；HAL 启动失败并已投递恢复时返回 false。
 */
static bool uart_restart_rx_dma(UART_HandleTypeDef *huart,
                                uint8_t *rx_buf, uint16_t rx_buf_size,
                                void (*set_recv_mode)(void))
{
    HAL_StatusTypeDef status;

    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    (void)HAL_UART_DMAStop(huart);

    if (set_recv_mode != NULL) {
        set_recv_mode();
    }

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    status = HAL_UART_Receive_DMA(huart, rx_buf, rx_buf_size);
    if (status != HAL_OK) {
        CPU3_UartScheduleRxRecovery(huart);
        return false;
    }

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    return true;
}

/**
 * @brief 准备UART发送DMA
 *
 * 该函数用于在启动DMA发送前进行必要的准备工作，包括禁用UART空闲中断、停止DMA传输、清除空闲标志以及清除各种错误标志。
 *
 * @param huart 指向UART句柄的指针，用于指定要操作的UART外设
 *
 * @note 该函数会清除以下错误标志：
 *       - PE (Parity Error): 奇偶校验错误标志
 *       - ORE (Overrun Error): 过载错误标志
 *       - FE (Framing Error): 帧错误标志
 *       - NE (Noise Error): 噪声错误标志
 *
 * @note 调用此函数后，UART将处于准备启动DMA发送的状态
 */
static void uart_prepare_tx_dma(UART_HandleTypeDef *huart)
{
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_DMAStop(huart);

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE)  != RESET) __HAL_UART_CLEAR_PEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)  != RESET) __HAL_UART_CLEAR_FEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  != RESET) __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
}

/**
 * @brief 恢复 UART 传输状态
 *
 * 该函数用于重置 UART 的传输和接收状态，清除忙标志和待发送长度，
 * 并重新启动 DMA 接收模式。
 *
 * @param huart UART 句柄指针
 * @param tx_busy 发送忙标志指针，将被清零
 * @param pending_len 待发送长度指针，将被清零
 * @param rx_ready 接收就绪标志指针，将被清零
 * @param rx_len 接收数据长度指针，将被清零
 * @param rx_buf 接收缓冲区指针
 * @param rx_buf_size 接收缓冲区大小
 * @param set_recv_mode 设置接收模式的函数指针
 *
 * @note 该函数会将所有状态标志重置为初始状态，并重新启动 DMA 接收
 */
static void cpu3_uart_recover_tx(UART_HandleTypeDef *huart,
                                 volatile uint8_t *tx_busy,
                                 uint16_t *pending_len,
                                 volatile uint8_t *rx_ready,
                                 volatile uint16_t *rx_len,
                                 uint8_t *rx_buf, uint16_t rx_buf_size,
                                 void (*set_recv_mode)(void))
{
    *tx_busy = 0;
    *pending_len = 0;
    *rx_ready = 0;
    *rx_len = 0;
    uart_restart_rx_dma(huart, rx_buf, rx_buf_size, set_recv_mode);
}

/**
 * @brief 应用 UART 重配置（如果挂起）
 *
 * 该函数检查是否有挂起的 UART 重配置请求，并在安全条件下执行重配置操作。
 * 安全条件包括：所有 UART 通道的接收和发送均处于空闲状态。
 *
 * @note 重配置过程包括：
 *       - 停止所有 UART 的 DMA 接收
 *       - 调用 cpu3_reinit_all_external_ports() 逐端口重新初始化 UART
 *       - 成功端口恢复 DMA 接收，失败端口进入完整重初始化队列
 *
 * @note 函数在以下情况下会直接返回，不执行重配置：
 *       - 无挂起的重配置请求
 *       - 任一 UART 通道正在接收数据
 *       - 任一 UART 通道正在发送数据
 *       - 任一 UART 通道有待发送数据
 */
static void cpu3_apply_uart_reinit_if_pending(void)
{
    if (g_cpu3_uart_reinit_pending == 0) return;

    /* 安全点：不能在收发忙时重配 */
    if (com1_rx_ready || com2_rx_ready || com3_rx_ready) return;
    if (g_tx_busy_com1 || g_tx_busy_com2 || g_tx_busy_com3) return;
    if (g_tx_pending_len_com1 || g_tx_pending_len_com2 || g_tx_pending_len_com3) return;

    g_cpu3_uart_reinit_pending = 0;

    /* 逐端口入口会先关闭IDLEIE再停止DMA，禁止在此留下悬空中断窗口。 */
    (void)cpu3_reinit_all_external_ports();
    cpu3_log_all_port_configs("参数重配置完成");
}

/* ====== 统一“尝试发送”：忙则塞进 pending（单帧）====== */
/**
 * @brief 尝试发送 UART 数据或将其加入待发送队列。
 *
 * 该函数用于管理 UART 数据的发送，当发送通道空闲时直接启动 DMA 发送，当发送通道忙碌时将数据缓存到待发送队列（单帧缓冲，新数据覆盖旧数据）。
 *
 * @param port_idx 零基外部串口索引。
 * @param huart UART 句柄指针。
 * @param tx_busy 指向发送忙标志的指针，0 表示空闲，非 0 表示忙碌。
 * @param txbuf 待发送帧的首地址；空闲时该缓冲区直接交给 UART DMA，必须保持有效直至发送完成，忙碌时最多复制 256 字节到 pending_buf。
 * @param txlen 待发送数据长度。
 * @param pending_len 指向待发送队列数据长度的指针。
 * @param pending_buf 待发送队列缓冲区指针。
 * @param pending_overwrite_cnt 指向待发送数据被覆盖次数计数器的指针。
 * @param set_send_mode 设置发送模式的函数指针。
 * @param set_recv_mode 设置接收模式的函数指针。
 * @return true 表示已启动或排队发送，false 表示参数为空或启动发送失败。
 * @note 当发送通道忙碌时，新数据会覆盖待发送队列中的旧数据，且最大缓存长度为 256 字节。
 * @note 发送失败时会立即回退到接收模式，但不负责重启接收 DMA。
 */
static bool uart_try_send_or_queue(uint8_t port_idx,
                                  UART_HandleTypeDef *huart,
                                  volatile uint8_t *tx_busy,
                                  uint8_t *txbuf, uint16_t txlen,
                                  uint16_t *pending_len, uint8_t *pending_buf,
                                  uint32_t *pending_overwrite_cnt,
                                  void (*set_send_mode)(void),
                                  void (*set_recv_mode)(void))
{
    uint8_t port_array_index;

    if ((port_idx < 1U) || (port_idx > 3U) || (txlen == 0U)) {
        return false;
    }
    port_array_index = (uint8_t)(port_idx - 1U);

    if (*tx_busy == 0) {
        *tx_busy = 1;
        uart_prepare_tx_dma(huart);
        set_send_mode();

        if (HAL_UART_Transmit_DMA(huart, txbuf, txlen) != HAL_OK) {
            /* 启动发送失败：立即回退接收并释放 busy */
            *tx_busy = 0;
            set_recv_mode();
            s_port_comm_stats[port_array_index].tx_dma_start_fail_count++;
            CPU3_LOG_WARNING(cpu3_port_module_text(port_idx),
                             "发送DMA启动失败 长度=%u 累计=%lu",
                             (unsigned int)txlen,
                             (unsigned long)s_port_comm_stats[port_array_index].tx_dma_start_fail_count);
            /* 注意：接收 DMA 的重启在上层 handle 里做（保持策略一致） */
            return false;
        }
    } else {
        /* busy：放入 pending（单帧），新来的覆盖旧的 */
        if (*pending_len != 0) {
            (*pending_overwrite_cnt)++;
            CPU3_LOG_WARNING(cpu3_port_module_text(port_idx),
                             "待发送单帧缓存被覆盖 新长度=%u 累计=%lu",
                             (unsigned int)txlen,
                             (unsigned long)(*pending_overwrite_cnt));
        }
        if (txlen > 256) txlen = 256;
        memcpy(pending_buf, txbuf, txlen);
        *pending_len = txlen;
        s_port_comm_stats[port_array_index].tx_queued_count++;
    }

    return true;
}
/* ================== 协议分发 ================== */

typedef uint32_t (*ProtoProcessFn)(const uint8_t* rx, uint16_t rx_len,
                                  uint8_t* tx, uint16_t* tx_len);

typedef void (*ProtoResetFn)(void);

typedef struct {
    /* 外部协议分发表项；process 处理一帧完整接收数据，reset 在协议切换或串口恢复时清理协议私有状态。 */
    ProtoProcessFn process; /* 完整帧处理回调；返回协议处理结果，上层据此累计成功、忽略或失败统计。 */
    ProtoResetFn   reset;   /* 可为 NULL */
} ComProtocolHandler;

/**
 * @brief 处理一帧 DSM Modbus 请求并生成响应。
 *
 * 该函数是 CPU3 外部端口分发表的 DSM 适配入口，直接调用 DSM_CommunicationProcess 并透传处理结果。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx DSM Modbus 正常或异常响应的输出缓冲区；实际响应长度通过 tx_len 返回。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 DSM 请求处理结果；DSM_COMM_OK 表示已生成响应，其他值区分帧长、CRC 和从机地址错误。
 */
static uint32_t proto_dsm_process(const uint8_t* rx, uint16_t rx_len,
                                 uint8_t* tx, uint16_t* tx_len)
{
    return DSM_CommunicationProcess((unsigned char *)rx, rx_len, tx, tx_len);
}

/**
 * @brief 处理一帧瓦锡兰 Modbus 请求并生成响应。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 数据长度。该值是当前瓦锡兰 Modbus RTU 请求帧的有效字节数，函数只解析 rx[0..rx_len-1]。
 * @param tx 瓦锡兰 Modbus 正常或异常响应的输出缓冲区；实际响应长度通过 tx_len 返回。
 * @param tx_len 数据长度。该输出指针用于返回瓦锡兰 Modbus RTU 响应帧的实际字节数。
 * @return 返回瓦锡兰 Modbus 分发结果码；0 表示已生成正常或异常响应，其他值表示帧长、CRC、地址或功能码错误。
 */
static uint32_t proto_wartsila_process(const uint8_t* rx, uint16_t rx_len,
                                      uint8_t* tx, uint16_t* tx_len)
{
    return modbus_rtu_process(rx, rx_len, tx, tx_len);
}

/**
 * @brief 处理 SI协议帧。
 *
 * 主循环只转交完整 RTU 帧，地址表、状态翻译和 CRC 回包都由 SI 模块负责。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx SI Modbus 正常或异常响应的输出缓冲区；实际响应长度通过 tx_len 返回。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 SI Modbus 分发结果码；0 表示已生成正常或异常响应，其他值表示帧长、CRC、地址或功能码错误。
 */
static uint32_t proto_si_process(const uint8_t* rx, uint16_t rx_len,
                                     uint8_t* tx, uint16_t* tx_len)
{
    /* SI 的具体地址映射在独立模块内完成，主分发层只负责转交完整 RTU 帧。 */
    return si_modbus_process_for_dispatch(rx, rx_len, tx, tx_len);
}

/**
 * @brief 处理 LTD 共享 Modbus 协议帧。
 *
 * 读取由 CPU3 已确认快照响应，写入由 LTD 模块等待 CPU2 ACK 后再决定外部响应。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx LTD 共享 Modbus 正常或异常响应的输出缓冲区；实际响应长度通过 tx_len 返回。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 LTD Modbus 分发结果码；0 表示已生成正常或异常响应，其他值表示帧长、CRC、地址或功能码错误。
 */
static uint32_t proto_ltd_process(const uint8_t* rx, uint16_t rx_len,
                                  uint8_t* tx, uint16_t* tx_len)
{
    return ltd_modbus_process_for_dispatch(rx, rx_len, tx, tx_len);
}

/**
 * @brief 处理 LH 现场 Modbus 协议帧。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx LH Modbus 正常或异常响应的输出缓冲区；实际响应长度通过 tx_len 返回。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 LH Modbus 分发结果码；0 表示已生成正常或异常响应，其他值表示帧长、CRC、地址或功能码错误。
 * @note LH 地址重排和参数写入约束全部封装在独立模块；主分发层只转交完整 RTU 帧。
 */
static uint32_t proto_lh_process(const uint8_t* rx, uint16_t rx_len,
                                 uint8_t* tx, uint16_t* tx_len)
{
    return lh_modbus_process_for_dispatch(rx, rx_len, tx, tx_len);
}

/* 关键：用你现有枚举做索引。若枚举不是从 0 连续增长，别用这种表，改 switch（见下） */
static const ComProtocolHandler g_handlers[] = {
    [COM_PROTO_DSM]      = { proto_dsm_process,      NULL },
    [COM_PROTO_WARTSILA] = { proto_wartsila_process, NULL },
    [COM_PROTO_SI]   = { proto_si_process,   NULL },
    [COM_PROTO_LTD]      = { proto_ltd_process,      NULL },
    [COM_PROTO_LH]       = { proto_lh_process,       NULL },
};

/**
 * @brief 按零基端口索引返回 COM1、COM2 或 COM3 的当前配置。
 *
 * @param port_idx 零基外部串口索引。
 * @return 成功时返回指向按零基端口索引返回 COM1、COM2 或 COM3 的当前配置的指针；输入非法或未找到匹配项时返回 NULL。
 */
static const ComPortConfig* cpu3_get_port_cfg(uint8_t port_idx)
{
    switch (port_idx) {
    case 1: return &g_cpu3_comm_display_params.com1;
    case 2: return &g_cpu3_comm_display_params.com2;
    case 3: return &g_cpu3_comm_display_params.com3;
    default: return NULL;
    }
}

/**
 * @brief 输出一个外部COM口当前生效的协议与串口参数。
 *
 * @param port_idx 零基外部串口索引。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 */
static void cpu3_log_port_config(uint8_t port_idx, const char *reason)
{
    const ComPortConfig *cfg = cpu3_get_port_cfg(port_idx);

    if (cfg == NULL) {
        return;
    }

    CPU3_LOG_INFO(cpu3_port_module_text(port_idx),
                  "%s 协议=%s(%u) 波特率=%lu 数据位=%u 校验=%s 停止位=%u 从机地址=%u",
                  (reason != NULL) ? reason : "配置",
                  cpu3_protocol_text(cfg->protocol),
                  (unsigned int)cfg->protocol,
                  (unsigned long)cfg->baudrate,
                  (unsigned int)cfg->databits,
                  cpu3_parity_text(cfg->parity),
                  (cfg->stopbits == COM_STOPBITS_2) ? 2U : 1U,
                  (unsigned int)SlaveAddress);
}

/**
 * @brief 输出三个外部COM口当前生效配置。
 *
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 */
static void cpu3_log_all_port_configs(const char *reason)
{
    uint8_t port_idx;

    for (port_idx = 1U; port_idx <= 3U; port_idx++) {
        cpu3_log_port_config(port_idx, reason);
    }
}

/**
 * @brief 在外部COM口UART中断中累计硬件异常并交给主循环打印。
 *
 * @details 调用场景：HAL_UART_ErrorCallback处理COM1、COM2、COM3时调用。
 * @note 关键约束：中断中只记录计数和标志，禁止直接printf或调用统一日志函数。
 *
 * @param port_idx 零基外部串口索引。
 * @param error_code 待记录、转换或判断的错误码。该值是 HAL UART 错误位掩码，ISR 仅入队，主循环再按端口归并和打印。
 * @param during_tx true 表示错误发生在发送阶段，false 表示发生在接收或空闲阶段。
 */
static void cpu3_record_uart_error_from_isr(uint8_t port_idx,
                                            uint32_t error_code,
                                            uint8_t during_tx)
{
    Cpu3PortCommStats *stats;
    uint8_t index;

    if ((port_idx < 1U) || (port_idx > 3U)) {
        return;
    }

    index = (uint8_t)(port_idx - 1U);
    stats = &s_port_comm_stats[index];
    stats->uart_error_count++;
    if ((error_code & HAL_UART_ERROR_ORE) != 0U) {
        stats->uart_ore_count++;
    }
    if ((error_code & HAL_UART_ERROR_FE) != 0U) {
        stats->uart_fe_count++;
    }
    if ((error_code & HAL_UART_ERROR_NE) != 0U) {
        stats->uart_ne_count++;
    }
    if ((error_code & HAL_UART_ERROR_PE) != 0U) {
        stats->uart_pe_count++;
    }
    s_port_uart_error_pending[index] |= error_code;
    if (during_tx != 0U) {
        s_port_uart_error_during_tx[index] = 1U;
    }
}

/**
 * @brief UART错误或DMA收满后的统一中断出口。
 *
 * 中断只隔离端口、丢弃候选帧并投递恢复事件，DMA重启由主循环执行。
 *
 * @param huart 目标 UART 外设句柄。
 * @param error_code 待记录、转换或判断的错误码。该值是 HAL UART 错误位掩码，ISR 仅入队，主循环再按端口归并和打印。
 * @param overflow true 表示接收长度超过 DMA 缓冲容量，false 表示长度仍在范围内。
 */
void CPU3_UartRxFaultFromISR(UART_HandleTypeDef *huart,
                            uint32_t error_code,
                            uint8_t overflow)
{
    int8_t recovery_index = cpu3_uart_recovery_index(huart);

    if (recovery_index < 0) {
        return;
    }

    if (huart->Instance == USART6) {
        if (error_code != HAL_UART_ERROR_NONE) {
            cpu3_record_uart_error_from_isr(1U, error_code, g_tx_busy_com1);
        }
        if (overflow != 0U) {
            s_port_comm_stats[0].rx_overflow_count++;
        }
        cpu3_cancel_protocol_switch(1U);
        g_tx_busy_com1 = 0U;
        g_tx_pending_len_com1 = 0U;
        com1_rx_ready = 0U;
        UART6_RX_LEN = 0U;
        COM1_RecvMode();
    } else if (huart->Instance == USART2) {
        if (error_code != HAL_UART_ERROR_NONE) {
            cpu3_record_uart_error_from_isr(2U, error_code, g_tx_busy_com2);
        }
        if (overflow != 0U) {
            s_port_comm_stats[1].rx_overflow_count++;
        }
        cpu3_cancel_protocol_switch(2U);
        g_tx_busy_com2 = 0U;
        g_tx_pending_len_com2 = 0U;
        com2_rx_ready = 0U;
        UART2_RX_LEN = 0U;
        COM2_RecvMode();
    } else if (huart->Instance == USART3) {
        if (error_code != HAL_UART_ERROR_NONE) {
            cpu3_record_uart_error_from_isr(3U, error_code, g_tx_busy_com3);
        }
        if (overflow != 0U) {
            s_port_comm_stats[2].rx_overflow_count++;
        }
        cpu3_cancel_protocol_switch(3U);
        g_tx_busy_com3 = 0U;
        g_tx_pending_len_com3 = 0U;
        com3_rx_ready = 0U;
        UART3_RX_LEN = 0U;
        COM3_RecvMode();
    } else {
        uint32_t notify_error = error_code;

        if (overflow != 0U) {
            s_uart5_rx_overflow_count++;
            notify_error |= HAL_UART_ERROR_DMA;
        }
        CPU2_CommNotifyUartErrorFromISR(notify_error);
        UART5_RX_LEN = 0U;
        RS485_RecvMode();
    }

    CPU3_UartScheduleRxRecovery(huart);
}

/**
 * @brief 主循环按端口和固定退避执行DMA恢复或完整UART重初始化。
 *
 * 外部COM保护期只跳过COM1/2/3，板间UART5恢复始终允许执行。
 *
 * @param external_ports_ready true 表示三路外部 COM 口已经完成初始化，可执行恢复；false 表示仍处于启动阶段。
 */
static void cpu3_uart_rx_recovery_task(bool external_ports_ready)
{
    uint32_t now = HAL_GetTick();
    uint8_t index;

    for (index = 0U; index < CPU3_UART_COUNT; index++) {
        bool recovered;
        uint8_t full_reinit;

        if (!external_ports_ready && (index < 3U)) {
            continue;
        }

        if ((s_uart_rx_recovery_pending[index] == 0U) ||
            ((int32_t)(now - s_uart_rx_recovery_due_tick[index]) < 0)) {
            continue;
        }

        s_uart_rx_recovery_pending[index] = 0U;
        full_reinit = s_uart_full_reinit_pending[index];
        if ((full_reinit != 0U) && (index < 3U)) {
            recovered = cpu3_reinit_external_port((uint8_t)(index + 1U));
        } else if (index == 0U) {
            recovered = uart_restart_rx_dma(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE, COM1_RecvMode);
        } else if (index == 1U) {
            recovered = uart_restart_rx_dma(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE, COM2_RecvMode);
        } else if (index == 2U) {
            recovered = uart_restart_rx_dma(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE, COM3_RecvMode);
        } else {
            recovered = uart_restart_rx_dma(&huart5, UART5_RX_BUF, UART5_RX_BUF_SIZE, RS485_RecvMode);
        }

        if (recovered) {
            s_uart_full_reinit_pending[index] = 0U;
            if (index < 3U) {
                s_port_comm_stats[index].rx_recovery_count++;
            } else {
                s_uart5_rx_recovery_count++;
            }
        } else {
            if ((full_reinit != 0U) && (index < 3U)) {
                cpu3_schedule_uart_full_reinit((index == 0U) ? &huart6 :
                                                ((index == 1U) ? &huart2 : &huart3));
            }
            if (index < 3U) {
                s_port_comm_stats[index].rx_recovery_fail_count++;
            } else {
                s_uart5_rx_recovery_fail_count++;
            }
        }
    }
}

/**
 * @brief 前台完成一个有界阶段后推进健康代际；等待循环内部不得调用。
 */
void CPU3_WatchdogReportProgress(void)
{
    s_foreground_health_generation++;
}

/**
 * @brief 看门狗只消费已经完成的前台健康进度，不再由定时器无条件续命。
 *
 * @return true 表示前台健康代次自上次检查后已推进；没有新进度时返回 false。
 */
bool CPU3_WatchdogHealthAdvancedFromISR(void)
{
    static uint32_t last_generation = 0U;
    uint32_t current_generation = s_foreground_health_generation;

    if (current_generation == last_generation) {
        return false;
    }
    last_generation = current_generation;
    return true;
}

/**
 * @brief ISR中记录续发DMA失败，主循环统一输出。
 *
 * @param port_idx 零基外部串口索引。
 */
static void cpu3_record_tx_continue_fail_from_isr(uint8_t port_idx)
{
    uint8_t index;

    if ((port_idx < 1U) || (port_idx > 3U)) {
        return;
    }
    index = (uint8_t)(port_idx - 1U);
    s_port_comm_stats[index].tx_dma_continue_fail_count++;
    s_port_tx_continue_fail_pending[index]++;
}

/**
 * @brief 在主循环延后输出UART异常，并周期汇总三个外部COM口通信健康状态。
 *
 * 调用场景：App_MainLoop每轮调用，内部按5秒周期限流摘要。
 * 函数由 App_MainLoop 每轮调用，但每个外部 COM 口的异常事件分别按 CPU3_COMM_EVENT_LOG_INTERVAL_MS 限流，避免持续 UART 故障占满调试串口。
 * 读取 ISR 写入的 UART 错误标志、错误发生阶段和续发 DMA 失败计数时，先保存 PRIMASK 并关闭中断，再一次性取走并清零待处理值；临界区结束后只在进入前允许中断时重新使能中断。
 * 线程态根据锁存值输出端口、发送或接收阶段、ORE、FE、NE、PE 以及 DMA 续发失败统计，ISR 本身只累计标志和计数，不执行格式化打印。
 * 达到 CPU3_COMM_STATUS_LOG_INTERVAL_MS 后，逐端口汇总协议、收发、忽略帧、处理失败、UART 异常、接收恢复、DMA 启动或续发失败及待发队列状态，并另行汇总 CPU2
 * UART5 的收满和恢复情况。
 *
 * @note 关键约束：ISR只写pending标志，所有阻塞打印均在本函数执行。
 * @note 静态节拍使用无符号减法，允许 HAL_GetTick 自然回绕；不得把本函数的格式化日志移回 UART 中断上下文。
 */
static void cpu3_comm_debug_task(void)
{
    static uint32_t last_status_tick = 0U;
    static uint32_t last_event_log_tick[3] = {0U, 0U, 0U};
    uint32_t now = HAL_GetTick();
    uint8_t port_idx;

    for (port_idx = 1U; port_idx <= 3U; port_idx++) {
        uint8_t index = (uint8_t)(port_idx - 1U);
        uint32_t uart_error_flags;
        uint32_t tx_continue_fail_count;
        uint32_t interrupt_mask;
        uint8_t during_tx;

        if ((now - last_event_log_tick[index]) < CPU3_COMM_EVENT_LOG_INTERVAL_MS) {
            continue;
        }
        last_event_log_tick[index] = now;

        interrupt_mask = __get_PRIMASK();
        __disable_irq();
        uart_error_flags = s_port_uart_error_pending[index];
        during_tx = s_port_uart_error_during_tx[index];
        tx_continue_fail_count = s_port_tx_continue_fail_pending[index];
        s_port_uart_error_pending[index] = 0U;
        s_port_uart_error_during_tx[index] = 0U;
        s_port_tx_continue_fail_pending[index] = 0U;
        if (interrupt_mask == 0U) {
            __enable_irq();
        }

        if (uart_error_flags != HAL_UART_ERROR_NONE) {
            CPU3_LOG_WARNING(cpu3_port_module_text(port_idx),
                             "UART异常已隔离并投递恢复 阶段=%s 标志=0x%08lX ORE=%lu FE=%lu NE=%lu PE=%lu 累计=%lu",
                             (during_tx != 0U) ? "发送" : "接收",
                             (unsigned long)uart_error_flags,
                             (unsigned long)s_port_comm_stats[index].uart_ore_count,
                             (unsigned long)s_port_comm_stats[index].uart_fe_count,
                             (unsigned long)s_port_comm_stats[index].uart_ne_count,
                             (unsigned long)s_port_comm_stats[index].uart_pe_count,
                             (unsigned long)s_port_comm_stats[index].uart_error_count);
        }
        if (tx_continue_fail_count > 0U) {
            CPU3_LOG_WARNING(cpu3_port_module_text(port_idx),
                             "待发帧续发DMA启动失败 本轮=%lu 累计=%lu",
                             (unsigned long)tx_continue_fail_count,
                             (unsigned long)s_port_comm_stats[index].tx_dma_continue_fail_count);
        }
    }

    if ((now - last_status_tick) < CPU3_COMM_STATUS_LOG_INTERVAL_MS) {
        return;
    }
    last_status_tick = now;

    for (port_idx = 1U; port_idx <= 3U; port_idx++) {
        uint8_t index = (uint8_t)(port_idx - 1U);
        const ComPortConfig *cfg = cpu3_get_port_cfg(port_idx);
        ComProtocolType protocol = (cfg != NULL) ? cfg->protocol : (ComProtocolType)0xFFU;
        uint8_t tx_busy;
        uint16_t pending_len;
        uint32_t pending_overwrite;

        if (port_idx == 1U) {
            tx_busy = g_tx_busy_com1;
            pending_len = g_tx_pending_len_com1;
            pending_overwrite = g_tx_pending_overwrite_com1;
        } else if (port_idx == 2U) {
            tx_busy = g_tx_busy_com2;
            pending_len = g_tx_pending_len_com2;
            pending_overwrite = g_tx_pending_overwrite_com2;
        } else {
            tx_busy = g_tx_busy_com3;
            pending_len = g_tx_pending_len_com3;
            pending_overwrite = g_tx_pending_overwrite_com3;
        }

        CPU3_LOG_INFO(cpu3_port_module_text(port_idx),
                      "协议=%s 接收=%lu 发送=完成%lu/受理%lu/排队%lu 忽略=%lu 处理失败=%lu UART异常=%lu 收满=%lu 恢复=%lu/失败%lu DMA失败=启动%lu/续发%lu 队列覆盖=%lu TX忙=%u 待发=%u 最近=%s",
                      cpu3_protocol_text(protocol),
                      (unsigned long)s_port_comm_stats[index].rx_frame_count,
                      (unsigned long)s_port_comm_stats[index].tx_complete_count,
                      (unsigned long)s_port_comm_stats[index].tx_accept_count,
                      (unsigned long)s_port_comm_stats[index].tx_queued_count,
                      (unsigned long)s_port_comm_stats[index].ignored_frame_count,
                      (unsigned long)s_port_comm_stats[index].process_failure_count,
                      (unsigned long)s_port_comm_stats[index].uart_error_count,
                      (unsigned long)s_port_comm_stats[index].rx_overflow_count,
                      (unsigned long)s_port_comm_stats[index].rx_recovery_count,
                      (unsigned long)s_port_comm_stats[index].rx_recovery_fail_count,
                      (unsigned long)s_port_comm_stats[index].tx_dma_start_fail_count,
                      (unsigned long)s_port_comm_stats[index].tx_dma_continue_fail_count,
                      (unsigned long)pending_overwrite,
                      (unsigned int)tx_busy,
                      (unsigned int)pending_len,
                      (s_port_comm_stats[index].process_failure_count > 0U) ?
                          cpu3_port_result_text(protocol, s_port_comm_stats[index].last_process_result) : "无");
    }

    CPU3_LOG_INFO("CPU2链路",
                  "UART5收满=%lu 恢复=%lu/失败%lu",
                  (unsigned long)s_uart5_rx_overflow_count,
                  (unsigned long)s_uart5_rx_recovery_count,
                  (unsigned long)s_uart5_rx_recovery_fail_count);
}

/**
 * @brief 记录某个外部 COM 口已经接受的协议切换目标。
 *
 * @details 调用场景：统一切换帧校验成功时调用，包括目标协议与当前协议相同的请求。
 * @note 关键约束：只暂存 RAM 状态，必须等当前串口参数下的 ACK 发送完成后，才能保存目标协议并恢复接收。
 *
 * @param port_idx 零基外部串口索引。
 * @param target_protocol 目标协议。
 */
static void cpu3_stage_protocol_switch(uint8_t port_idx, ComProtocolType target_protocol)
{
    uint8_t index;

    if ((port_idx < 1U) || (port_idx > 3U)) {
        return;
    }

    index = (uint8_t)(port_idx - 1U);
    g_protocol_switch_target[index] = target_protocol;
    g_protocol_switch_pending_state[index] = PROTOCOL_SWITCH_PENDING_TX;
}

/**
 * @brief 取消尚未完成应答的协议切换请求。
 *
 * @details 调用场景：DMA 发送启动失败或 UART 错误恢复时调用。
 * @note 关键约束：只清除对应 COM 口，不影响其它端口已经排队的切换请求。
 *
 * @param port_idx 零基外部串口索引。
 */
static void cpu3_cancel_protocol_switch(uint8_t port_idx)
{
    if ((port_idx >= 1U) && (port_idx <= 3U) &&
        (g_protocol_switch_pending_state[port_idx - 1U] == PROTOCOL_SWITCH_PENDING_TX))
    {
        g_protocol_switch_pending_state[port_idx - 1U] = PROTOCOL_SWITCH_PENDING_NONE;
    }
}

/**
 * @brief 在发送完成中断中标记切换应答已经使用旧串口参数完整发出。
 *
 * @details 调用场景：COM1、COM2 或 COM3 最后一帧 DMA 发送完成后调用。
 * @note 关键约束：中断内只改状态，不保存 FRAM、不重初始化 UART。
 *
 * @param port_idx 零基外部串口索引。
 * @return true 表示本次发送属于协议切换应答且已锁存发送完成；普通响应或无待决切换时返回 false。
 */
static bool cpu3_mark_protocol_switch_tx_complete(uint8_t port_idx)
{
    uint8_t index;

    if ((port_idx < 1U) || (port_idx > 3U)) {
        return false;
    }

    index = (uint8_t)(port_idx - 1U);
    if (g_protocol_switch_pending_state[index] == PROTOCOL_SWITCH_PENDING_TX) {
        g_protocol_switch_pending_state[index] = PROTOCOL_SWITCH_PENDING_APPLY;
        return true;
    }

    return false;
}

/**
 * @brief 把一个端口切回 RS485 接收方向，并按当前参数只重初始化该端口。
 *
 * @details 调用场景：启动、参数重配置、协议切换和完整恢复队列。
 * @note 关键约束：不得重配其它端口，避免中断无关的外部通信。
 *
 * @param port_idx 零基外部串口索引。
 * @return true 表示端口编号有效，收发方向已切回接收且对应 UART 已按当前参数重新初始化；false 表示端口编号不在 1～3，或 Cpu3_ReinitPortUart 未能完成该端口重初始化。
 */
static bool cpu3_reinit_external_port(uint8_t port_idx)
{
    switch (port_idx)
    {
    case 1U:
        com1_rx_ready = 0U;
        UART6_RX_LEN = 0U;
        COM1_RecvMode();
        break;

    case 2U:
        com2_rx_ready = 0U;
        UART2_RX_LEN = 0U;
        COM2_RecvMode();
        break;

    case 3U:
        com3_rx_ready = 0U;
        UART3_RX_LEN = 0U;
        COM3_RecvMode();
        break;

    default:
        return false;
    }

    return Cpu3_ReinitPortUart(port_idx);
}

/**
 * @brief 逐端口应用当前配置，仅把失败端口投递完整重初始化，避免干扰其它COM口。
 *
 * @return true 表示三路外部 COM 口均按当前配置恢复成功；任一端口失败并进入后续恢复队列时返回 false。
 */
static bool cpu3_reinit_all_external_ports(void)
{
    bool all_ok = true;
    uint8_t port_idx;

    for (port_idx = 1U; port_idx <= 3U; port_idx++) {
        if (!cpu3_reinit_external_port(port_idx)) {
            UART_HandleTypeDef *huart = (port_idx == 1U) ? &huart6 :
                                        ((port_idx == 2U) ? &huart2 : &huart3);
            cpu3_schedule_uart_full_reinit(huart);
            all_ok = false;
        }
    }

    return all_ok;
}

/**
 * @brief 协议切换持久化失败时恢复指定端口的旧配置镜像。
 *
 * @details 调用场景：新协议写入 FRAM 后读回校验失败时调用。
 * @note 关键约束：只恢复目标端口，不修改其它外部 COM 口。
 *
 * @param port_idx 零基外部串口索引。
 * @param config 只读外部串口配置；包含波特率、数据位、校验位、停止位和当前协议类型，用于恢复协议切换前的端口设置。
 * @return true 表示指定端口的协议和串口配置已恢复为切换前镜像并重新生效；false 表示端口编号无效，旧镜像无法持久化，或恢复后的 UART 重初始化失败。
 */
static bool cpu3_restore_protocol_switch_port_config(uint8_t port_idx,
                                                     const ComPortConfig *config)
{
    if (config == NULL) {
        return false;
    }

    switch (port_idx)
    {
    case 1U:
        g_cpu3_comm_display_params.com1 = *config;
        return true;

    case 2U:
        g_cpu3_comm_display_params.com2 = *config;
        return true;

    case 3U:
        g_cpu3_comm_display_params.com3 = *config;
        return true;

    default:
        return false;
    }
}

/**
 * @brief 在主循环中保存已确认发送完成的远程协议切换，并保持当前串口物理参数。
 *
 * @details 调用场景：每轮主循环处理普通通信前调用。
 * @note 关键约束：保存和 HAL 重初始化均在主循环执行，禁止放入 UART 中断。
 */
static void cpu3_apply_ready_protocol_switches(void)
{
    uint8_t port_idx;

    for (port_idx = 1U; port_idx <= 3U; port_idx++) {
        uint8_t index = (uint8_t)(port_idx - 1U);
        OperatingNumber opera;
        ComProtocolType target_protocol;
        const ComPortConfig *current_config;
        ComPortConfig previous_config;

        if (g_protocol_switch_pending_state[index] != PROTOCOL_SWITCH_PENDING_APPLY) {
            continue;
        }

        target_protocol = g_protocol_switch_target[index];
        g_protocol_switch_pending_state[index] = PROTOCOL_SWITCH_PENDING_NONE;
        current_config = cpu3_get_port_cfg(port_idx);
        if (current_config == NULL) {
            continue;
        }
        previous_config = *current_config;

        switch (port_idx)
        {
        case 1U:
            opera = COM_NUM_CPU3_COM1_PROTOCOL;
            break;

        case 2U:
            opera = COM_NUM_CPU3_COM2_PROTOCOL;
            break;

        case 3U:
            opera = COM_NUM_CPU3_COM3_PROTOCOL;
            break;

        default:
            continue;
        }

        if (!Cpu3Local_WriteProtocolPreserveSerialChecked(opera, (int32_t)target_protocol)) {
            (void)cpu3_restore_protocol_switch_port_config(port_idx, &previous_config);
            if (!Cpu3_Params_SaveToFRAM()) {
                CPU3_LOG_CRITICAL(cpu3_port_module_text(port_idx),
                                  "协议切换失败后旧配置回写FRAM失败");
            }
            if (!cpu3_reinit_external_port(port_idx)) {
                UART_HandleTypeDef *huart = (port_idx == 1U) ? &huart6 :
                                            ((port_idx == 2U) ? &huart2 : &huart3);
                cpu3_schedule_uart_full_reinit(huart);
                CPU3_LOG_CRITICAL(cpu3_port_module_text(port_idx),
                                  "协议切换回滚后串口重初始化失败，已进入恢复队列");
            }
            CPU3_LOG_WARNING(cpu3_port_module_text(port_idx),
                             "协议切换持久化失败，已保持原协议=%s(%u)",
                             cpu3_protocol_text(previous_config.protocol),
                             (unsigned int)previous_config.protocol);
            continue;
        }

        if (cpu3_reinit_external_port(port_idx)) {
            CPU3_LOG_INFO(cpu3_port_module_text(port_idx),
                          "协议切换完成，串口参数保持不变 新协议=%s(%u)",
                          cpu3_protocol_text(target_protocol),
                          (unsigned int)target_protocol);
            cpu3_log_port_config(port_idx, "切换后配置");
        } else {
            UART_HandleTypeDef *huart = (port_idx == 1U) ? &huart6 :
                                        ((port_idx == 2U) ? &huart2 : &huart3);
            cpu3_schedule_uart_full_reinit(huart);
            CPU3_LOG_CRITICAL(cpu3_port_module_text(port_idx),
                              "协议切换后串口重初始化失败，已进入恢复队列 目标协议=%s(%u)",
                              cpu3_protocol_text(target_protocol),
                              (unsigned int)target_protocol);
        }
    }
}

/**
 * @brief 按端口配置分派外部协议帧；通过 tx_len 输出响应长度，并返回 CPU3_PORT_RESULT 处理状态。
 *
 * @param port_idx 待处理的零基外部串口索引；用于选择该端口的接收快照、协议分发器、配置和诊断计数。
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 数据长度。该值是当前外部串口接收快照的有效字节数，协议分发器只读取 rx[0..rx_len-1]。
 * @param tx 当前外部端口协议处理器使用的响应帧输出缓冲区；仅在 tx_len 非 0 时由上层启动发送。
 * @param tx_len 数据长度。该输出指针用于返回当前端口协议处理器生成的响应帧实际字节数。
 * @return 返回 CPU3_PORT_RESULT 分发状态；0 表示处理成功，其他值区分端口配置、处理器、帧长、CRC、地址和功能码错误。
 */
static uint32_t cpu3_port_process(uint8_t port_idx,
                                 const uint8_t* rx, uint16_t rx_len,
                                 uint8_t* tx, uint16_t* tx_len)
{
    const ComPortConfig *cfg = cpu3_get_port_cfg(port_idx);
    ComProtocolType target_protocol;
    ProtocolSwitchFrameResult switch_result;

    if (cfg == NULL) {
        *tx_len = 0;
        return CPU3_PORT_RESULT_INVALID_CONFIG;
    }

    target_protocol = cfg->protocol;
    switch_result = ProtocolSwitchFrame_Process((uint8_t)SlaveAddress,
                                                rx,
                                                rx_len,
                                                tx,
                                                tx_len,
                                                &target_protocol);
    if (switch_result != PROTOCOL_SWITCH_FRAME_NOT_MATCHED) {
        if (switch_result == PROTOCOL_SWITCH_FRAME_ACCEPTED) {
            /* 同协议请求保持幂等，只重新保存协议值，不改当前串口物理参数。 */
            cpu3_stage_protocol_switch(port_idx, target_protocol);
        }
        return 0U;
    }

    /* 防御：protocol 越界或 handler 未配置 */
    uint8_t p = cfg->protocol;
    if (p >= (sizeof(g_handlers) / sizeof(g_handlers[0])) || g_handlers[p].process == NULL) {
        *tx_len = 0;
        return CPU3_PORT_RESULT_INVALID_HANDLER;
    }

    return g_handlers[p].process(rx, rx_len, tx, tx_len);
}

/**
 * @brief 判断外部 COM 启动保护时间是否结束。
 *
 * @details 调用场景：主循环处理 COM1、COM2、COM3 接收帧前调用。
 * @note 关键约束：仅限制外部协议帧处理，不阻塞屏幕任务、CPU2 板间轮询和调试日志服务。
 *
 * @return true 表示当前 HAL tick 已达到 s_external_ports_ready_tick，外部 COM 启动保护期结束；false 表示仍处于保护期，暂不初始化外部端口。
 */
static bool cpu3_external_ports_startup_ready(void)
{
    return ((int32_t)(HAL_GetTick() - s_external_ports_ready_tick) >= 0);
}

/**
 * @brief 初始化 CPU3 参数、显示、板间通信和三路外部串口调度。
 *
 * 先初始化异步日志并把 UART5 板间 RS485 收发器切回接收方向，然后初始化 OLED、立即绘制启动页并建立 RTC 时钟。
 * 随后从 FRAM 加载 CPU3 通信与显示参数，按当前配置重建三路外部串口并初始化 DSM 通信模块。
 * 外部串口保留一秒硬件稳定窗口，但只记录非阻塞就绪时刻，使屏幕任务和 CPU2 板间轮询可以立即进入主循环；各阶段持续上报看门狗健康进度。
 */
void App_Init(void) {
    uint32_t phase_start_tick;

    s_cpu3_boot_start_tick = HAL_GetTick();
    s_external_ports_ready_logged = false;
    s_cpu2_startup_ready_logged = false;
	CPU3_WatchdogReportProgress();
	Cpu3Log_Init();
	CPU3_WatchdogReportProgress();
	CPU3_LOG_INFO("系统",
	              "CPU3启动 固件=%s 共享协议=%u 日志级别=%s 调试串口=USART1/115200/8N1",
	              CPU3_APP_VERSION_STRING,
	              (unsigned int)DEVICE_PROTOCOL_VERSION,
	              Cpu3Log_ActiveLevelText());
	/* Force UART5 RS485 direction back to RX after CubeMX init. */
	RS485_SET_RECV_MODE();
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
    phase_start_tick = HAL_GetTick();
	DisplayInit(); /* Initialize the OLED display */
	CPU3_WatchdogReportProgress();
	DisplayAubonLogo(); /* OLED 初始化完成后立即显示启动页。 */
    CPU3_LOG_INFO("启动",
                  "OLED首屏完成 阶段耗时=%lums 总耗时=%lums",
                  (unsigned long)(HAL_GetTick() - phase_start_tick),
                  (unsigned long)(HAL_GetTick() - s_cpu3_boot_start_tick));
	CPU3_WatchdogReportProgress();
    /* RTC 先初始化，保证后续 SI 读当前时间或 profile 时间戳时有合法兜底值。 */
    phase_start_tick = HAL_GetTick();
    Cpu3Clock_Init();
    CPU3_LOG_INFO("启动",
                  "RTC初始化完成 阶段耗时=%lums 时钟源=%u 状态=%u",
                  (unsigned long)(HAL_GetTick() - phase_start_tick),
                  (unsigned int)Cpu3Clock_GetSource(),
                  (unsigned int)Cpu3Clock_GetState());
    phase_start_tick = HAL_GetTick();
    Cpu3_Params_LoadFromFRAM(); /* 从 FRAM 载入 Cpu3 通讯+显示参数（里面会自动回退默认并保存） */
	CPU3_WatchdogReportProgress();
    (void)cpu3_reinit_all_external_ports();
	CPU3_WatchdogReportProgress();
	DSM_CommunicationInit(); /* 初始化通信模块 */
	cpu3_log_all_port_configs("启动配置");
	CPU3_WatchdogReportProgress();
    /*
     * 外部 COM 保留原有 1 秒硬件稳定窗口，但主循环立即运行屏幕和 CPU2 轮询，
     * 避免整机启动被固定延时阻塞。
     */
    s_external_ports_ready_tick = HAL_GetTick() + CPU3_EXTERNAL_PORT_STARTUP_GUARD_MS;
    CPU3_LOG_INFO("启动",
                  "本机参数和外部COM初始化完成 阶段耗时=%lums 外部COM保护=%ums 总耗时=%lums",
                  (unsigned long)(HAL_GetTick() - phase_start_tick),
                  (unsigned int)CPU3_EXTERNAL_PORT_STARTUP_GUARD_MS,
                  (unsigned long)(HAL_GetTick() - s_cpu3_boot_start_tick));
	CPU3_WatchdogReportProgress();
}

/**
 * @brief 调度显示、参数保存、日志、CPU2 板间通信及三路外部协议任务。
 *
 * 每轮先上报看门狗进度，推进外部串口启动保护、接收异常恢复、协议切换和待执行串口重配置，再运行 SI 周期调度与 OLED 显示任务。
 * COM1、COM2、COM3 各自最多消费一帧已完成接收的数据；函数按端口当前协议分发请求，区分需要告警的处理失败和可忽略帧，并累计对应通信统计。
 * 处理成功且存在响应时启动或排队 UART DMA 发送；处理失败、发送启动失败或无需响应时立即恢复该端口 DMA 接收，避免外部串口停收。
 * 三路外部通信之后推进调试任务，并按 100 ms 门限轮询 CPU2，使持续外部流量不能永久饿死板间通信；最后服务异步日志，仅在本轮没有接收或轮询工作时延时 1 ms。
 *
 * @note UART DMA 响应的最后一帧发送完成后，由 HAL_UART_TxCpltCallback 负责切回接收；主循环不得提前重启同一端口接收 DMA。
 */
void App_MainLoop(void)
{
    uint32_t ret;
    uint16_t send_len;
    const ComPortConfig *port_cfg;
    ComProtocolType active_protocol;
    bool external_ports_ready;

    static uint8_t sendbuff1[256] = {0};
    static uint8_t sendbuff2[256] = {0};
    static uint8_t sendbuff3[256] = {0};
    static uint32_t last_cpu2_poll_tick = 0U;

    uint8_t did_work = 0;

    CPU3_WatchdogReportProgress();
    external_ports_ready = cpu3_external_ports_startup_ready();
    cpu3_uart_rx_recovery_task(external_ports_ready);
    if (external_ports_ready) {
        cpu3_apply_ready_protocol_switches();
        cpu3_apply_uart_reinit_if_pending(); /* 如果有待重配的串口，先重配 */
        si_modbus_periodic_task();
    }
    Display_Task();
	CPU3_WatchdogReportProgress();

    if (external_ports_ready && !s_external_ports_ready_logged) {
        s_external_ports_ready_logged = true;
        CPU3_LOG_INFO("启动",
                      "外部COM启动保护结束 总耗时=%lums",
                      (unsigned long)(HAL_GetTick() - s_cpu3_boot_start_tick));
    }

    /* ========= COM1 ========= */
    if (external_ports_ready && (com1_rx_ready == 1)) {
        com1_rx_ready = 0;
        did_work = 1;
        s_port_comm_stats[0].rx_frame_count++;
        Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG, "COM1", "接收", UART6_RX_BUF, UART6_RX_LEN);
        port_cfg = cpu3_get_port_cfg(1U);
        active_protocol = (port_cfg != NULL) ? port_cfg->protocol : (ComProtocolType)0xFFU;
        ret = cpu3_port_process(1, UART6_RX_BUF, UART6_RX_LEN, sendbuff1, &send_len);

        if (ret != 0) {
            if (cpu3_port_result_needs_warning(ret)) {
                s_port_comm_stats[0].process_failure_count++;
                s_port_comm_stats[0].last_process_result = ret;
                CPU3_LOG_WARNING("COM1",
                                 "接收帧处理失败 协议=%s 结果=%lu 原因=%s",
                                 cpu3_protocol_text(active_protocol),
                                 (unsigned long)ret,
                                 cpu3_port_result_text(active_protocol, ret));
                Cpu3Log_Frame(CPU3_LOG_LEVEL_WARNING,
                              "COM1", "接收失败帧", UART6_RX_BUF, UART6_RX_LEN);
            } else {
                s_port_comm_stats[0].ignored_frame_count++;
                CPU3_LOG_DEBUG("COM1",
                               "接收帧未处理 协议=%s 结果=%lu 原因=%s",
                               cpu3_protocol_text(active_protocol),
                               (unsigned long)ret,
                               cpu3_port_result_text(active_protocol, ret));
            }

            (void)uart_restart_rx_dma(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE, COM1_RecvMode);
        } else {
            if (send_len > 0) {
                Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG, "COM1", "准备发送", sendbuff1, send_len);
                if (!uart_try_send_or_queue(1U, &huart6,
                                            &g_tx_busy_com1,
                                            sendbuff1, send_len,
                                            &g_tx_pending_len_com1, g_tx_pending_buf_com1,
                                            &g_tx_pending_overwrite_com1,
                                            COM1_SendMode, COM1_RecvMode))
                {
                    cpu3_cancel_protocol_switch(1U);
                    uart_restart_rx_dma(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE, COM1_RecvMode);
                } else {
                    s_port_comm_stats[0].tx_accept_count++;
                }

                /* 注意：接收 DMA 的重启交给 TxCplt（最后一帧发完才切回接收） */
            } else {
                /* 没有回复也必须恢复接收，否则会“收一帧就停” */
                (void)uart_restart_rx_dma(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE, COM1_RecvMode);
            }
        }
    }
	CPU3_WatchdogReportProgress();

    /* ========= COM2 ========= */
    if (external_ports_ready && (com2_rx_ready == 1)) {
        com2_rx_ready = 0;
        did_work = 1;
        s_port_comm_stats[1].rx_frame_count++;
        Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG, "COM2", "接收", UART2_RX_BUF, UART2_RX_LEN);
        port_cfg = cpu3_get_port_cfg(2U);
        active_protocol = (port_cfg != NULL) ? port_cfg->protocol : (ComProtocolType)0xFFU;
        ret = cpu3_port_process(2, UART2_RX_BUF, UART2_RX_LEN, sendbuff2, &send_len);

        if (ret != 0) {
            if (cpu3_port_result_needs_warning(ret)) {
                s_port_comm_stats[1].process_failure_count++;
                s_port_comm_stats[1].last_process_result = ret;
                CPU3_LOG_WARNING("COM2",
                                 "接收帧处理失败 协议=%s 结果=%lu 原因=%s",
                                 cpu3_protocol_text(active_protocol),
                                 (unsigned long)ret,
                                 cpu3_port_result_text(active_protocol, ret));
                Cpu3Log_Frame(CPU3_LOG_LEVEL_WARNING,
                              "COM2", "接收失败帧", UART2_RX_BUF, UART2_RX_LEN);
            } else {
                s_port_comm_stats[1].ignored_frame_count++;
                CPU3_LOG_DEBUG("COM2",
                               "接收帧未处理 协议=%s 结果=%lu 原因=%s",
                               cpu3_protocol_text(active_protocol),
                               (unsigned long)ret,
                               cpu3_port_result_text(active_protocol, ret));
            }

            (void)uart_restart_rx_dma(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE, COM2_RecvMode);
        } else {
            if (send_len > 0) {
                Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG, "COM2", "准备发送", sendbuff2, send_len);
                if (!uart_try_send_or_queue(2U, &huart2,
                                            &g_tx_busy_com2,
                                            sendbuff2, send_len,
                                            &g_tx_pending_len_com2, g_tx_pending_buf_com2,
                                            &g_tx_pending_overwrite_com2,
                                            COM2_SendMode, COM2_RecvMode))
                {
                    cpu3_cancel_protocol_switch(2U);
                    uart_restart_rx_dma(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE, COM2_RecvMode);
                } else {
                    s_port_comm_stats[1].tx_accept_count++;
                }
            } else {
                (void)uart_restart_rx_dma(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE, COM2_RecvMode);
            }
        }
    }
	CPU3_WatchdogReportProgress();

    /* ========= COM3 ========= */
    if (external_ports_ready && (com3_rx_ready == 1)) {
        com3_rx_ready = 0;
        did_work = 1;
        s_port_comm_stats[2].rx_frame_count++;
        Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG, "COM3", "接收", UART3_RX_BUF, UART3_RX_LEN);
        port_cfg = cpu3_get_port_cfg(3U);
        active_protocol = (port_cfg != NULL) ? port_cfg->protocol : (ComProtocolType)0xFFU;
        ret = cpu3_port_process(3, UART3_RX_BUF, UART3_RX_LEN, sendbuff3, &send_len);

        if (ret != 0) {
            if (cpu3_port_result_needs_warning(ret)) {
                s_port_comm_stats[2].process_failure_count++;
                s_port_comm_stats[2].last_process_result = ret;
                CPU3_LOG_WARNING("COM3",
                                 "接收帧处理失败 协议=%s 结果=%lu 原因=%s",
                                 cpu3_protocol_text(active_protocol),
                                 (unsigned long)ret,
                                 cpu3_port_result_text(active_protocol, ret));
                Cpu3Log_Frame(CPU3_LOG_LEVEL_WARNING,
                              "COM3", "接收失败帧", UART3_RX_BUF, UART3_RX_LEN);
            } else {
                s_port_comm_stats[2].ignored_frame_count++;
                CPU3_LOG_DEBUG("COM3",
                               "接收帧未处理 协议=%s 结果=%lu 原因=%s",
                               cpu3_protocol_text(active_protocol),
                               (unsigned long)ret,
                               cpu3_port_result_text(active_protocol, ret));
            }

            (void)uart_restart_rx_dma(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE, COM3_RecvMode);
        } else {
            if (send_len > 0) {
                Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG, "COM3", "准备发送", sendbuff3, send_len);
                if (!uart_try_send_or_queue(3U, &huart3,
                                            &g_tx_busy_com3,
                                            sendbuff3, send_len,
                                            &g_tx_pending_len_com3, g_tx_pending_buf_com3,
                                            &g_tx_pending_overwrite_com3,
                                            COM3_SendMode, COM3_RecvMode))
                {
                    cpu3_cancel_protocol_switch(3U);
                    uart_restart_rx_dma(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE, COM3_RecvMode);
                } else {
                    s_port_comm_stats[2].tx_accept_count++;
                }
            } else {
                (void)uart_restart_rx_dma(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE, COM3_RecvMode);
            }
        }
    }
	CPU3_WatchdogReportProgress();

    cpu3_comm_debug_task();
    CPU2_CommDebugTask();
	CPU3_WatchdogReportProgress();

    /* 主循环可调度时按 100 ms 门限轮询，避免外部流量永久饿死 CPU2；同步请求阻塞期间不保证实际间隔。 */
    if ((HAL_GetTick() - last_cpu2_poll_tick) >= CPU2_POLL_PERIOD_MS) {
        PollingInputData();
        last_cpu2_poll_tick = HAL_GetTick();
        did_work = 1U;
    }
    if (!s_cpu2_startup_ready_logged && CPU2_CommHasFixedPointSnapshot()) {
        s_cpu2_startup_ready_logged = true;
        CPU3_LOG_INFO("启动",
                      "CPU2启动快照完成 总耗时=%lums",
                      (unsigned long)(HAL_GetTick() - s_cpu3_boot_start_tick));
    }
	CPU3_WatchdogReportProgress();

    /* 业务通信调度完成后再推进调试日志，日志队列忙时立即返回。 */
    Cpu3Log_Service();

    if (!did_work) {
        HAL_Delay(1U);
    }
	CPU3_WatchdogReportProgress();
}
/**
 * @brief 在 UART DMA 发送完成中断中续发排队帧、恢复 RS485 接收或推进协议切换。
 *
 * 该函数在 UART DMA 发送完成时被 HAL 库调用，用于处理多路串口（COM1/COM2/COM3/UART5）的发送完成逻辑。
 * 支持多帧连续发送机制：当有待发送帧时自动续发，最后一帧发送完成后切换回接收模式。
 *
 * @param huart 指向 UART 句柄的指针，用于标识触发的 UART 外设；函数据其实例区分调试串口、三路外部 COM 和 UART5 板间口。
 *
 * @note 支持的串口及其对应实例：
 *       - COM1: USART6
 *       - COM2: USART2
 *       - COM3: USART3
 *       - UART5
 *
 * @note 发送流程：
 *       1. 检查是否有待发送帧（g_tx_pending_len_comX > 0）
 *       2. 如果有待发送帧，直接启动 DMA 发送，不切换到接收模式
 *       3. 普通响应最后一帧完成后恢复 DMA 接收
 *       4. 协议切换 ACK 完成后只标记 APPLY 并保持 RX 停止，由主循环按目标协议默认参数恢复
 *
 * @note 错误处理：
 *       - 如果续发失败，调用 cpu3_uart_recover_tx() 恢复接收模式并释放资源
 *
 * @attention 该函数由 HAL 库在中断上下文中调用，应避免执行耗时操作
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Cpu3Log_TxCompleteFromISR();
        return;
    }

    /*
     * COM1/COM2/COM3 的 TX DMA 均为普通模式，HAL 只会在 UART TC 置位、最后停止位发送完成后进入本回调。
     * 因此最后一帧应立即切回接收，不能在中断中使用空循环或阻塞延时，避免延误高优先级 UART5 收发切换。
     * 如果现场波形证明收发器确实需要 DE 尾保持，应改用 TIM5 微秒级单次事件，并保持 TX busy，
     * 待定时事件到期后再切换接收并启动 RX DMA；如果今后改用循环 TX DMA，必须重新按硬件 TC 判定发送完成。
     */

    /* ========== COM1: USART6 ========== */
    if (huart->Instance == USART6) {

        s_port_comm_stats[0].tx_complete_count++;

        /* 如果还有待发帧：直接续发（保持发送模式） */
        if (g_tx_pending_len_com1 > 0) {
            uint16_t len = g_tx_pending_len_com1;
            g_tx_pending_len_com1 = 0;
            COM1_SET_SEND_MODE();
            /* 注意：这里不做 TC+延时+切接收，因为还要继续发 */
            if (HAL_UART_Transmit_DMA(&huart6, g_tx_pending_buf_com1, len) != HAL_OK) {
                /* 续发失败：回退接收并释放 busy */
                cpu3_record_tx_continue_fail_from_isr(1U);
                cpu3_cancel_protocol_switch(1U);
                cpu3_uart_recover_tx(&huart6,
                                     &g_tx_busy_com1, &g_tx_pending_len_com1,
                                     &com1_rx_ready, &UART6_RX_LEN,
                                     UART6_RX_BUF, UART6_RX_BUF_SIZE,
                                     COM1_RecvMode);
            }
            return;
        }

        /* 最后一帧已达到 UART TC，立即恢复接收。 */
        g_tx_busy_com1 = 0;
        if (cpu3_mark_protocol_switch_tx_complete(1U)) {
            COM1_RecvMode();
            return;
        }
        uart_restart_rx_dma(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE, COM1_RecvMode);
        return;
    }

    /* ========== COM2: USART2 ========== */
    if (huart->Instance == USART2) {

        s_port_comm_stats[1].tx_complete_count++;

        if (g_tx_pending_len_com2 > 0) {
            uint16_t len = g_tx_pending_len_com2;
            g_tx_pending_len_com2 = 0;
            COM2_SET_SEND_MODE();
            if (HAL_UART_Transmit_DMA(&huart2, g_tx_pending_buf_com2, len) != HAL_OK) {
                cpu3_record_tx_continue_fail_from_isr(2U);
                cpu3_cancel_protocol_switch(2U);
                cpu3_uart_recover_tx(&huart2,
                                     &g_tx_busy_com2, &g_tx_pending_len_com2,
                                     &com2_rx_ready, &UART2_RX_LEN,
                                     UART2_RX_BUF, UART2_RX_BUF_SIZE,
                                     COM2_RecvMode);
            }
            return;
        }

        /* 最后一帧已达到 UART TC，立即恢复接收。 */
        g_tx_busy_com2 = 0;
        if (cpu3_mark_protocol_switch_tx_complete(2U)) {
            COM2_RecvMode();
            return;
        }
        uart_restart_rx_dma(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE, COM2_RecvMode);
        return;
    }

    /* ========== COM3: USART3 ========== */
    if (huart->Instance == USART3) {

        s_port_comm_stats[2].tx_complete_count++;

        if (g_tx_pending_len_com3 > 0) {
            uint16_t len = g_tx_pending_len_com3;
            g_tx_pending_len_com3 = 0;
            COM3_SET_SEND_MODE();
            if (HAL_UART_Transmit_DMA(&huart3, g_tx_pending_buf_com3, len) != HAL_OK) {
                cpu3_record_tx_continue_fail_from_isr(3U);
                cpu3_cancel_protocol_switch(3U);
                cpu3_uart_recover_tx(&huart3,
                                     &g_tx_busy_com3, &g_tx_pending_len_com3,
                                     &com3_rx_ready, &UART3_RX_LEN,
                                     UART3_RX_BUF, UART3_RX_BUF_SIZE,
                                     COM3_RecvMode);
            }
            return;
        }

        /* 最后一帧已达到 UART TC，立即恢复接收。 */
        g_tx_busy_com3 = 0;
        if (cpu3_mark_protocol_switch_tx_complete(3U)) {
            COM3_RecvMode();
            return;
        }
        uart_restart_rx_dma(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE, COM3_RecvMode);
        return;
    }
    /* ========== UART5：你工程里也用了（如果你也要 pending，可按 COM1/2/3 同法扩展） ========== */
    if (huart->Instance == UART5) {

        uart_restart_rx_dma(&huart5, UART5_RX_BUF, UART5_RX_BUF_SIZE, RS485_RecvMode);
        return;
    }
}

/**
 * @brief DMA普通模式收满后关闭IDLEIE并丢弃整块数据，等待主循环退避恢复。
 *
 * @param huart 目标 UART 外设句柄。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart->Instance == USART6) ||
        (huart->Instance == USART2) ||
        (huart->Instance == USART3) ||
        (huart->Instance == UART5)) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        CPU3_UartRxFaultFromISR(huart, HAL_UART_ERROR_NONE, 1U);
    }
}

/**
 * @brief UART硬件错误只隔离并投递恢复，禁止在错误回调中立即重启DMA。
 *
 * @param huart 目标 UART 外设句柄。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Cpu3Log_TxErrorFromISR();
        return;
    }

    if ((huart->Instance == USART6) ||
        (huart->Instance == USART2) ||
        (huart->Instance == USART3) ||
        (huart->Instance == UART5)) {
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        CPU3_UartRxFaultFromISR(huart, huart->ErrorCode, 0U);
    }
}
