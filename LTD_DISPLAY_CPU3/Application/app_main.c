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
#include "si_modbus_slave.h"
#include <string.h>

#define DEBUG_APP_MAIN 0

#define CPU2_POLL_IDLE_DELAY_MS 100u /* CPU2 idle polling period, 10Hz */


/* ====== 可调：TX 完成后额外延时（用于 RS485 电平恢复）====== */
#ifndef UART_TX_POST_DELAY_LOOP
#define UART_TX_POST_DELAY_LOOP   (18000u)   /* 你原来的 18000，按主频自行校准 */
#endif

/**
 * @brief UART DMA 发送完成后补充短延时，等待 RS485 电平方向恢复。
 */
static inline void uart_post_tx_delay(void)
{
    for (volatile uint32_t i = 0; i < UART_TX_POST_DELAY_LOOP; i++) {
        __NOP();
    }
}

/**
 * @brief 接收屏幕显示中的 RS485_RecvMode 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static inline void RS485_RecvMode(void)
{
    RS485_SET_RECV_MODE();
}

/* ====== 每口 TX busy + 单帧 pending 队列（忙时缓存 1 帧）====== */
volatile uint8_t  g_tx_busy_com1 = 0; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
volatile uint8_t  g_tx_busy_com2 = 0; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */
volatile uint8_t  g_tx_busy_com3 = 0; /* 屏幕显示模块级变量，保存跨函数共享的业务状态。 */

static uint16_t g_tx_pending_len_com1 = 0; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static uint16_t g_tx_pending_len_com2 = 0; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static uint16_t g_tx_pending_len_com3 = 0; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */

static uint8_t  g_tx_pending_buf_com1[256]; /* 屏幕显示数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */
static uint8_t  g_tx_pending_buf_com2[256]; /* 屏幕显示数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */
static uint8_t  g_tx_pending_buf_com3[256]; /* 屏幕显示数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */

/* 统计：如果 pending 已有帧又来新帧，会覆盖旧帧（可观察是否需要更大队列） */
static uint32_t g_tx_pending_overwrite_com1 = 0; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static uint32_t g_tx_pending_overwrite_com2 = 0; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */
static uint32_t g_tx_pending_overwrite_com3 = 0; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */


/* UI/参数修改后置位，主循环应用 */
volatile uint8_t g_cpu3_uart_reinit_pending = 0; /* 屏幕显示状态标志，通常由主循环或中断回调共同检查。 */

/**
 * @brief 重启UART接收DMA
 *
 * 该函数用于重启UART的DMA接收模式，包括停止当前DMA传输、清除各种错误标志、
 * 重新设置接收模式并启动DMA接收。适用于需要重新初始化UART接收的场景。
 *
 * @param huart UART句柄指针，指定要操作的UART外设
 * @param rx_buf 接收缓冲区指针，用于存储DMA接收到的数据
 * @param rx_buf_size 接收缓冲区大小，指定DMA接收的数据长度
 * @param set_recv_mode 接收模式设置函数指针，可为NULL。如果不为NULL，则在重启前调用该函数设置接收模式
 *
 * @note 函数执行以下操作：
 *       1. 停止当前UART DMA传输
 *       2. 调用set_recv_mode函数设置接收模式（如果提供）
 *       3. 清除UART空闲中断标志
 *       4. 清除可能的错误标志（ORE-溢出错误、FE-帧错误、NE-噪声错误）
 *       5. 使能UART空闲中断
 *       6. 重新启动UART DMA接收
 *
 * @note 该函数为静态函数，仅在当前文件内部可见
 */
static void uart_restart_rx_dma(UART_HandleTypeDef *huart,
                                uint8_t *rx_buf, uint16_t rx_buf_size,
                                void (*set_recv_mode)(void))
{
    HAL_UART_DMAStop(huart);

    if (set_recv_mode != NULL) {
        set_recv_mode();
    }

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)  != RESET) __HAL_UART_CLEAR_FEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  != RESET) __HAL_UART_CLEAR_NEFLAG(huart);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, rx_buf, rx_buf_size);
}

/**
 * @brief 准备UART发送DMA
 *
 * 该函数用于在启动DMA发送前进行必要的准备工作，包括禁用UART空闲中断、停止DMA传输、清除空闲标志以及清除各种错误标志。
 *
 * @param huart 指向UART句柄的指针，用于指定要操作的UART外设
 *
 * @note 该函数会清除以下错误标志：
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

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)  != RESET) __HAL_UART_CLEAR_FEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  != RESET) __HAL_UART_CLEAR_NEFLAG(huart);
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
 *       - 调用 Cpu3_ReinitAllUarts() 重新初始化所有 UART
 *       - 恢复所有 UART 的 DMA 接收
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

    /* 重配前：建议停止 DMA 接收，避免 HAL 状态混乱（根据你现有接收实现调整） */
    HAL_UART_DMAStop(&huart6);
    HAL_UART_DMAStop(&huart2);
    HAL_UART_DMAStop(&huart3);

    Cpu3_ReinitAllUarts();

    /* 重配后：恢复接收 */
    COM1_RecvMode(); HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);
    COM2_RecvMode(); HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);
    COM3_RecvMode(); HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
}

/* ====== 统一“尝试发送”：忙则塞进 pending（单帧）====== */
/**
 * @brief 尝试发送 UART 数据或将其加入待发送队列
 *
 * 该函数用于管理 UART 数据的发送，当发送通道空闲时直接启动 DMA 发送，
 * 当发送通道忙碌时将数据缓存到待发送队列（单帧缓冲，新数据覆盖旧数据）。
 *
 * @param huart UART 句柄指针
 * @param tx_busy 指向发送忙标志的指针，0 表示空闲，非 0 表示忙碌
 * @param txbuf 待发送数据缓冲区指针
 * @param txlen 待发送数据长度
 * @param pending_len 指向待发送队列数据长度的指针
 * @param pending_buf 待发送队列缓冲区指针
 * @param pending_overwrite_cnt 指向待发送数据被覆盖次数计数器的指针
 * @param set_send_mode 设置发送模式的函数指针
 * @param set_recv_mode 设置接收模式的函数指针
 *
 * @note 当发送通道忙碌时，新数据会覆盖待发送队列中的旧数据，且最大缓存长度为 256 字节
 * @note 发送失败时会立即回退到接收模式，但不负责重启接收 DMA
 */
static void uart_try_send_or_queue(UART_HandleTypeDef *huart,
                                  volatile uint8_t *tx_busy,
                                  uint8_t *txbuf, uint16_t txlen,
                                  uint16_t *pending_len, uint8_t *pending_buf,
                                  uint32_t *pending_overwrite_cnt,
                                  void (*set_send_mode)(void),
                                  void (*set_recv_mode)(void))
{
    if (txlen == 0) {
        return;
    }

    if (*tx_busy == 0) {
        *tx_busy = 1;
        uart_prepare_tx_dma(huart);
        set_send_mode();

        if (HAL_UART_Transmit_DMA(huart, txbuf, txlen) != HAL_OK) {
            /* 启动发送失败：立即回退接收并释放 busy */
            *tx_busy = 0;
            set_recv_mode();
            /* 注意：接收 DMA 的重启在上层 handle 里做（保持策略一致） */
        }
    } else {
        /* busy：放入 pending（单帧），新来的覆盖旧的 */
        if (*pending_len != 0) {
            (*pending_overwrite_cnt)++;
        }
        if (txlen > 256) txlen = 256;
        memcpy(pending_buf, txbuf, txlen);
        *pending_len = txlen;
    }
}
/* ================== 协议分发 ================== */

typedef uint32_t (*ProtoProcessFn)(const uint8_t* rx, uint16_t rx_len,
                                  uint8_t* tx, uint16_t* tx_len);

typedef void (*ProtoResetFn)(void);

typedef struct {
    ProtoProcessFn process;
    ProtoResetFn   reset;   /* 可为 NULL */
} ComProtocolHandler;

/* 封装： */
static uint32_t proto_dsm_process(const uint8_t* rx, uint16_t rx_len,
                                 uint8_t* tx, uint16_t* tx_len)
{
    return DSM_CommunicationProcess((unsigned char *)rx, rx_len, tx, tx_len);
}

/**
 * @brief 处理屏幕显示中的 proto_wartsila_process 逻辑。
 *
 * @param rx 业务参数。
 * @param rx_len 数据长度。
 * @param tx 业务参数。
 * @param tx_len 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t proto_wartsila_process(const uint8_t* rx, uint16_t rx_len,
                                      uint8_t* tx, uint16_t* tx_len)
{
    return modbus_rtu_process(rx, rx_len, tx, tx_len);
}

/*
 * 处理 SI协议帧。
 * 主循环只转交完整 RTU 帧，地址表、状态翻译和 CRC 回包都由 SI 模块负责。
 */
static uint32_t proto_si_process(const uint8_t* rx, uint16_t rx_len,
                                     uint8_t* tx, uint16_t* tx_len)
{
    /* SI 的具体地址映射在独立模块内完成，主分发层只负责转交完整 RTU 帧。 */
    return si_modbus_process_for_dispatch(rx, rx_len, tx, tx_len);
}

/* 未实现的协议：安全兜底，不回包 */
static uint32_t proto_no_reply(const uint8_t* rx, uint16_t rx_len,
                               uint8_t* tx, uint16_t* tx_len)
{
    (void)rx; (void)rx_len; (void)tx;
    *tx_len = 0;
    return 0;
}

/* 关键：用你现有枚举做索引。若枚举不是从 0 连续增长，别用这种表，改 switch（见下） */
static const ComProtocolHandler g_handlers[] = {
    [COM_PROTO_DSM]      = { proto_dsm_process,      NULL },
    [COM_PROTO_WARTSILA] = { proto_wartsila_process, NULL },
    [COM_PROTO_SI]   = { proto_si_process,   NULL },
    [COM_PROTO_LTD]      = { proto_no_reply,         NULL },  /* 先占位 */
};

/* 按端口号取配置：你这里的 com1/com2/com3 结构来自 cpu3_comm_display_params.h */
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
 * @brief 处理屏幕显示中的 cpu3_port_process 逻辑。
 *
 * @param port_idx 输入/输出指针。
 * @param rx 业务参数。
 * @param rx_len 数据长度。
 * @param tx 业务参数。
 * @param tx_len 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t cpu3_port_process(uint8_t port_idx,
                                 const uint8_t* rx, uint16_t rx_len,
                                 uint8_t* tx, uint16_t* tx_len)
{
    const ComPortConfig *cfg = cpu3_get_port_cfg(port_idx);
    if (cfg == NULL) {
        *tx_len = 0;
        return 1; /* 参数错误 */
    }

    /* 防御：protocol 越界或 handler 未配置 */
    uint8_t p = cfg->protocol;
    if (p >= (sizeof(g_handlers) / sizeof(g_handlers[0])) || g_handlers[p].process == NULL) {
        *tx_len = 0;
        return 2;
    }

    return g_handlers[p].process(rx, rx_len, tx, tx_len);
}


/**
 * @brief 初始化屏幕显示中的 App_Init 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void App_Init(void) {
	printf("LTD显示端重启！\r\n");
	/* Force UART5 RS485 direction back to RX after CubeMX init. */
	RS485_SET_RECV_MODE();
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	DisplayInit(); /* Initialize the OLED display */
	DisplayAubonLogo(); /* 刚上电显示AUBON LOGO */
    /* RTC 先初始化，保证后续 SI 读当前时间或 profile 时间戳时有合法兜底值。 */
    Cpu3Clock_Init();
    Cpu3_Params_LoadFromFRAM(); /* 从 FRAM 载入 Cpu3 通讯+显示参数（里面会自动回退默认并保存） */
    Cpu3_ReinitAllUarts(); /* 根据参数重配 3 个串口 */
	DSM_CommunicationInit(); /* 初始化通信模块 */
	/* 屏幕显示与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
	HAL_Delay(1000); /* */
}

/**
 * @brief 执行屏幕显示中的 App_MainLoop 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void App_MainLoop(void)
{
    uint32_t ret;
    uint16_t send_len;

    static uint8_t sendbuff1[256] = {0};
    static uint8_t sendbuff2[256] = {0};
    static uint8_t sendbuff3[256] = {0};

    uint8_t did_work = 0;

    cpu3_apply_uart_reinit_if_pending(); /* 如果有待重配的串口，先重配 */
    si_modbus_periodic_task();
    Display_Task();
    /* ========= COM1 ========= */
    if (com1_rx_ready == 1) {
        com1_rx_ready = 0;
        did_work = 1;

#if DEBUG_APP_MAIN
        printf("COM1接收就绪\r\n");
#endif
        ret = cpu3_port_process(1, UART6_RX_BUF, UART6_RX_LEN, sendbuff1, &send_len);

        if (ret != 0) {
            printf("COM1处理结果=%lu\r\n", (unsigned long)ret);

            COM1_RecvMode();
            HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);
        } else {
            if (send_len > 0) {
#if DEBUG_APP_MAIN
                printf("COM1发送(%d): ", send_len);
                for (int i = 0; i < send_len; i++) printf("%02X ", sendbuff1[i]);
                printf("\r\n");
#endif
                uart_try_send_or_queue(&huart6,
                                      &g_tx_busy_com1,
                                      sendbuff1, send_len,
                                      &g_tx_pending_len_com1, g_tx_pending_buf_com1,
                                      &g_tx_pending_overwrite_com1,
                                      COM1_SendMode, COM1_RecvMode);

                if (g_tx_busy_com1 == 0) {
                    uart_restart_rx_dma(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE, COM1_RecvMode);
                }

                /* 注意：接收 DMA 的重启交给 TxCplt（最后一帧发完才切回接收） */
            } else {
                /* 没有回复也必须恢复接收，否则会“收一帧就停” */
                COM1_RecvMode();
                HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);
            }
        }
    }

    /* ========= COM2 ========= */
    if (com2_rx_ready == 1) {
        com2_rx_ready = 0;
        did_work = 1;

#if DEBUG_APP_MAIN
        printf("COM2接收(%d): ", UART2_RX_LEN);
        for (int i = 0; i < UART2_RX_LEN; i++) printf("%02X ", UART2_RX_BUF[i]);
        printf("\r\n");
#endif
        ret = cpu3_port_process(2, UART2_RX_BUF, UART2_RX_LEN, sendbuff2, &send_len);

        if (ret != 0) {
            printf("COM2处理结果=%lu\r\n", (unsigned long)ret);

            COM2_RecvMode();
            HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);
        } else {
            if (send_len > 0) {
#if DEBUG_APP_MAIN
                printf("COM2发送(%d): ", send_len);
                for (int i = 0; i < send_len; i++) printf("%02X ", sendbuff2[i]);
                printf("\r\n");
#endif
                uart_try_send_or_queue(&huart2,
                                      &g_tx_busy_com2,
                                      sendbuff2, send_len,
                                      &g_tx_pending_len_com2, g_tx_pending_buf_com2,
                                      &g_tx_pending_overwrite_com2,
                                      COM2_SendMode, COM2_RecvMode);

                if (g_tx_busy_com2 == 0) {
                    uart_restart_rx_dma(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE, COM2_RecvMode);
                }
            } else {
                COM2_RecvMode();
                HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);
            }
        }
    }

    /* ========= COM3 ========= */
    if (com3_rx_ready == 1) {
        com3_rx_ready = 0;
        did_work = 1;

#if DEBUG_APP_MAIN
        printf("COM3接收就绪\r\n");
#endif
#if DEBUG_APP_MAIN
        printf("COM3接收(%d): ", UART3_RX_LEN);
        for (int i = 0; i < UART3_RX_LEN; i++) printf("%02X ", UART3_RX_BUF[i]);
        printf("\r\n");
#endif
        ret = cpu3_port_process(3, UART3_RX_BUF, UART3_RX_LEN, sendbuff3, &send_len);

        if (ret != 0) {
            printf("COM3处理结果=%lu\r\n", (unsigned long)ret);

            COM3_RecvMode();
            HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
        } else {
            if (send_len > 0) {
#if DEBUG_APP_MAIN
                printf("COM3发送(%d): ", send_len);
                for (int i = 0; i < send_len; i++) printf("%02X ", sendbuff3[i]);
                printf("\r\n");
#endif
                uart_try_send_or_queue(&huart3,
                                      &g_tx_busy_com3,
                                      sendbuff3, send_len,
                                      &g_tx_pending_len_com3, g_tx_pending_buf_com3,
                                      &g_tx_pending_overwrite_com3,
                                      COM3_SendMode, COM3_RecvMode);

                if (g_tx_busy_com3 == 0) {
                    uart_restart_rx_dma(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE, COM3_RecvMode);
                }
            } else {
                COM3_RecvMode();
                HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
            }
        }
    }

    /* ========= 空闲才做轮询任务 ========= */
    if (!did_work) {
        PollingInputData();
/* PrintMeasurementResult(); */
        HAL_Delay(CPU2_POLL_IDLE_DELAY_MS);
    }
}
/**
 * @brief UART 发送完成回调函数
 *
 * 该函数在 UART DMA 发送完成时被 HAL 库调用，用于处理多路串口（COM1/COM2/COM3/UART5）的发送完成逻辑。
 * 支持多帧连续发送机制：当有待发送帧时自动续发，最后一帧发送完成后切换回接收模式。
 *
 * @param huart 指向 UART 句柄的指针，用于标识触发的 UART 外设
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
 *       3. 如果是最后一帧，等待发送完成，延时后切换回接收模式
 *       4. 清除发送忙标志（g_tx_busy_comX），重启 DMA 接收
 *
 * @note 错误处理：
 *       - 如果续发失败，调用 cpu3_uart_recover_tx() 恢复接收模式并释放资源
 *
 * @attention 该函数由 HAL 库在中断上下文中调用，应避免执行耗时操作
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* ========== COM1: USART6 ========== */
    if (huart->Instance == USART6) {

        /* 如果还有待发帧：直接续发（保持发送模式） */
        if (g_tx_pending_len_com1 > 0) {
            uint16_t len = g_tx_pending_len_com1;
            g_tx_pending_len_com1 = 0;
            COM1_SET_SEND_MODE();
            /* 注意：这里不做 TC+延时+切接收，因为还要继续发 */
            if (HAL_UART_Transmit_DMA(&huart6, g_tx_pending_buf_com1, len) != HAL_OK) {
                /* 续发失败：回退接收并释放 busy */
                cpu3_uart_recover_tx(&huart6,
                                     &g_tx_busy_com1, &g_tx_pending_len_com1,
                                     &com1_rx_ready, &UART6_RX_LEN,
                                     UART6_RX_BUF, UART6_RX_BUF_SIZE,
                                     COM1_RecvMode);
            }
            return;
        }

        /* 最后一帧：等待真正发送完，再延时，切回接收 */
        uart_post_tx_delay();

        g_tx_busy_com1 = 0;
        uart_restart_rx_dma(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE, COM1_RecvMode);
        return;
    }

    /* ========== COM2: USART2 ========== */
    if (huart->Instance == USART2) {

        if (g_tx_pending_len_com2 > 0) {
            uint16_t len = g_tx_pending_len_com2;
            g_tx_pending_len_com2 = 0;
            COM2_SET_SEND_MODE();
            if (HAL_UART_Transmit_DMA(&huart2, g_tx_pending_buf_com2, len) != HAL_OK) {
                cpu3_uart_recover_tx(&huart2,
                                     &g_tx_busy_com2, &g_tx_pending_len_com2,
                                     &com2_rx_ready, &UART2_RX_LEN,
                                     UART2_RX_BUF, UART2_RX_BUF_SIZE,
                                     COM2_RecvMode);
            }
            return;
        }

        uart_post_tx_delay();

        g_tx_busy_com2 = 0;
        uart_restart_rx_dma(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE, COM2_RecvMode);
        return;
    }

    /* ========== COM3: USART3 ========== */
    if (huart->Instance == USART3) {

        if (g_tx_pending_len_com3 > 0) {
            uint16_t len = g_tx_pending_len_com3;
            g_tx_pending_len_com3 = 0;
            COM3_SET_SEND_MODE();
            if (HAL_UART_Transmit_DMA(&huart3, g_tx_pending_buf_com3, len) != HAL_OK) {
                cpu3_uart_recover_tx(&huart3,
                                     &g_tx_busy_com3, &g_tx_pending_len_com3,
                                     &com3_rx_ready, &UART3_RX_LEN,
                                     UART3_RX_BUF, UART3_RX_BUF_SIZE,
                                     COM3_RecvMode);
            }
            return;
        }

        uart_post_tx_delay();

        g_tx_busy_com3 = 0;
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
 * @brief UART 错误回调函数
 *
 * 该函数在 UART 发生错误时被 HAL 库调用，用于处理不同 UART 接口的错误恢复。
 * 根据触发错误的 UART 实例，执行相应的错误恢复操作，包括发送恢复和接收重启。
 *
 * @param huart 指向发生错误的 UART 句柄的指针
 *
 * @note 支持的 UART 接口：
 *       - USART6: 调用 cpu3_uart_recover_tx 进行发送恢复
 *       - USART2: 调用 cpu3_uart_recover_tx 进行发送恢复
 *       - USART3: 调用 cpu3_uart_recover_tx 进行发送恢复
 *       - UART5: 重置等待响应标志，清空接收长度，重启 DMA 接收
 *
 * @note 函数内部会调用以下函数：
 *       - cpu3_uart_recover_tx(): UART 发送恢复
 *       - uart_restart_rx_dma(): 重启 UART DMA 接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        cpu3_uart_recover_tx(&huart6,
                             &g_tx_busy_com1, &g_tx_pending_len_com1,
                             &com1_rx_ready, &UART6_RX_LEN,
                             UART6_RX_BUF, UART6_RX_BUF_SIZE,
                             COM1_RecvMode);
        return;
    }

    if (huart->Instance == USART2) {
        cpu3_uart_recover_tx(&huart2,
                             &g_tx_busy_com2, &g_tx_pending_len_com2,
                             &com2_rx_ready, &UART2_RX_LEN,
                             UART2_RX_BUF, UART2_RX_BUF_SIZE,
                             COM2_RecvMode);
        return;
    }

    if (huart->Instance == USART3) {
        cpu3_uart_recover_tx(&huart3,
                             &g_tx_busy_com3, &g_tx_pending_len_com3,
                             &com3_rx_ready, &UART3_RX_LEN,
                             UART3_RX_BUF, UART3_RX_BUF_SIZE,
                             COM3_RecvMode);
        return;
    }

    if (huart->Instance == UART5) {
        wait_response = false;
        UART5_RX_LEN = 0;
        uart_restart_rx_dma(&huart5, UART5_RX_BUF, UART5_RX_BUF_SIZE, RS485_RecvMode);
    }
}
