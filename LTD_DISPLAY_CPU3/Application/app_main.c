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
#include <string.h>

#define DEBUG_APP_MAIN 1


/* ====== 可调：TX 完成后额外延时（用于 RS485 电平恢复）====== */
#ifndef UART_TX_POST_DELAY_LOOP
#define UART_TX_POST_DELAY_LOOP   (18000u)   /* 你原来的 18000，按主频自行校准 */
#endif

static inline void uart_post_tx_delay(void)
{
    for (volatile uint32_t i = 0; i < UART_TX_POST_DELAY_LOOP; i++) {
        __NOP();
    }
}

/* ====== 每口 TX busy + 单帧 pending 队列（忙时缓存 1 帧）====== */
volatile uint8_t  g_tx_busy_com1 = 0;
volatile uint8_t  g_tx_busy_com2 = 0;
volatile uint8_t  g_tx_busy_com3 = 0;

static uint16_t g_tx_pending_len_com1 = 0;
static uint16_t g_tx_pending_len_com2 = 0;
static uint16_t g_tx_pending_len_com3 = 0;

static uint8_t  g_tx_pending_buf_com1[256];
static uint8_t  g_tx_pending_buf_com2[256];
static uint8_t  g_tx_pending_buf_com3[256];

/* 统计：如果 pending 已有帧又来新帧，会覆盖旧帧（可观察是否需要更大队列） */
static uint32_t g_tx_pending_overwrite_com1 = 0;
static uint32_t g_tx_pending_overwrite_com2 = 0;
static uint32_t g_tx_pending_overwrite_com3 = 0;


/* UI/参数修改后置位，主循环应用 */
volatile uint8_t g_cpu3_uart_reinit_pending = 0;

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
    ProtoResetFn   reset;   // 可为 NULL
} ComProtocolHandler;

/* 封装： */
static uint32_t proto_dsm_process(const uint8_t* rx, uint16_t rx_len,
                                 uint8_t* tx, uint16_t* tx_len)
{
    return DSM_CommunicationProcess(rx, rx_len, tx, tx_len);
}

static uint32_t proto_wartsila_process(const uint8_t* rx, uint16_t rx_len,
                                      uint8_t* tx, uint16_t* tx_len)
{
    return modbus_rtu_process(rx, rx_len, tx, tx_len);
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
    [COM_PROTO_LTD]      = { proto_no_reply,         NULL },  // 先占位
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

static uint32_t cpu3_port_process(uint8_t port_idx,
                                 const uint8_t* rx, uint16_t rx_len,
                                 uint8_t* tx, uint16_t* tx_len)
{
    const ComPortConfig *cfg = cpu3_get_port_cfg(port_idx);
    if (cfg == NULL) {
        *tx_len = 0;
        return 1; // 参数错误
    }

    /* 防御：protocol 越界或 handler 未配置 */
    uint8_t p = cfg->protocol;
    if (p >= (sizeof(g_handlers) / sizeof(g_handlers[0])) || g_handlers[p].process == NULL) {
        *tx_len = 0;
        return 2;
    }

    return g_handlers[p].process(rx, rx_len, tx, tx_len);
}


void App_Init(void) {
	printf("LTD demo restart!\r\n");
	DisplayInit(); // Initialize the OLED display
	DisplayAubonLogo(); /* 刚上电显示AUBON LOGO */
    Cpu3_Params_LoadFromFRAM();//从 FRAM 载入 Cpu3 通讯+显示参数（里面会自动回退默认并保存）
    Cpu3_ReinitAllUarts();//根据参数重配 3 个串口
	DSM_CommunicationInit(); // 初始化通信模块
	HAL_Delay(1000); //
}

void App_MainLoop(void)
{
    uint32_t ret;
    uint16_t send_len;

    static uint8_t sendbuff1[256] = {0};
    static uint8_t sendbuff2[256] = {0};
    static uint8_t sendbuff3[256] = {0};

    uint8_t did_work = 0;

    cpu3_apply_uart_reinit_if_pending();// 如果有待重配的串口，先重配
    /* ========= COM1 ========= */
    if (com1_rx_ready == 1) {
        com1_rx_ready = 0;
        did_work = 1;

#if DEBUG_APP_MAIN
        printf("com1_rx_ready == 1\r\n");
#endif
        ret = cpu3_port_process(1, UART6_RX_BUF, UART6_RX_LEN, sendbuff1, &send_len);

        if (ret != 0) {
            printf("com1 process ret=%lu\r\n", (unsigned long)ret);

            COM1_RecvMode();
            HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);
        } else {
            if (send_len > 0) {
#if DEBUG_APP_MAIN
                printf("COM1 TX (%d): ", send_len);
                for (int i = 0; i < send_len; i++) printf("%02X ", sendbuff1[i]);
                printf("\r\n");
#endif
                uart_try_send_or_queue(&huart6,
                                      &g_tx_busy_com1,
                                      sendbuff1, send_len,
                                      &g_tx_pending_len_com1, g_tx_pending_buf_com1,
                                      &g_tx_pending_overwrite_com1,
                                      COM1_SendMode, COM1_RecvMode);

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
        printf("COM2 RX (%d): ", UART2_RX_LEN);
        for (int i = 0; i < UART2_RX_LEN; i++) printf("%02X ", UART2_RX_BUF[i]);
        printf("\r\n");
#endif
        ret = cpu3_port_process(2, UART2_RX_BUF, UART2_RX_LEN, sendbuff2, &send_len);

        if (ret != 0) {
            printf("com2 process ret=%lu\r\n", (unsigned long)ret);

            COM2_RecvMode();
            HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);
        } else {
            if (send_len > 0) {
#if DEBUG_APP_MAIN
                printf("COM2 TX (%d): ", send_len);
                for (int i = 0; i < send_len; i++) printf("%02X ", sendbuff2[i]);
                printf("\r\n");
#endif
                uart_try_send_or_queue(&huart2,
                                      &g_tx_busy_com2,
                                      sendbuff2, send_len,
                                      &g_tx_pending_len_com2, g_tx_pending_buf_com2,
                                      &g_tx_pending_overwrite_com2,
                                      COM2_SendMode, COM2_RecvMode);
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
        printf("com3_rx_ready == 1\r\n");
#endif
        ret = cpu3_port_process(3, UART3_RX_BUF, UART3_RX_LEN, sendbuff3, &send_len);

        if (ret != 0) {
            printf("com3 process ret=%lu\r\n", (unsigned long)ret);

            COM3_RecvMode();
            HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
        } else {
            if (send_len > 0) {
                uart_try_send_or_queue(&huart3,
                                      &g_tx_busy_com3,
                                      sendbuff3, send_len,
                                      &g_tx_pending_len_com3, g_tx_pending_buf_com3,
                                      &g_tx_pending_overwrite_com3,
                                      COM3_SendMode, COM3_RecvMode);
            } else {
                COM3_RecvMode();
                HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
            }
        }
    }

    /* ========= 空闲才做轮询任务 ========= */
    if (!did_work) {
        PollingInputData();
//        PrintMeasurementResult();
        HAL_Delay(10);;
    }
}
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
                g_tx_busy_com1 = 0;
                COM1_SET_RECV_MODE();
                HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);
            }
            return;
        }

        /* 最后一帧：等待真正发送完，再延时，切回接收 */
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET) {;}
        uart_post_tx_delay();

        g_tx_busy_com1 = 0;
        COM1_SET_RECV_MODE();
        HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);
        return;
    }

    /* ========== COM2: USART2 ========== */
    if (huart->Instance == USART2) {

        if (g_tx_pending_len_com2 > 0) {
            uint16_t len = g_tx_pending_len_com2;
            g_tx_pending_len_com2 = 0;
            COM2_SET_SEND_MODE();
            if (HAL_UART_Transmit_DMA(&huart2, g_tx_pending_buf_com2, len) != HAL_OK) {
                g_tx_busy_com2 = 0;
                COM2_SET_RECV_MODE();
                HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);
            }
            return;
        }

        while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {;}
        uart_post_tx_delay();

        g_tx_busy_com2 = 0;
        COM2_SET_RECV_MODE();
        HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);
        return;
    }

    /* ========== COM3: USART3 ========== */
    if (huart->Instance == USART3) {

        if (g_tx_pending_len_com3 > 0) {
            uint16_t len = g_tx_pending_len_com3;
            g_tx_pending_len_com3 = 0;
            COM3_SET_SEND_MODE();
            if (HAL_UART_Transmit_DMA(&huart3, g_tx_pending_buf_com3, len) != HAL_OK) {
                g_tx_busy_com3 = 0;
                COM3_SET_RECV_MODE();
                HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
            }
            return;
        }

        while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) {;}
        uart_post_tx_delay();

        g_tx_busy_com3 = 0;
        COM3_SET_RECV_MODE();
        HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
        return;
    }
    /* ========== UART5：你工程里也用了（如果你也要 pending，可按 COM1/2/3 同法扩展） ========== */
    if (huart->Instance == UART5) {

        while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET) {;}

        RS485_SET_RECV_MODE();

        __HAL_UART_CLEAR_IDLEFLAG(&huart5);
        HAL_UART_Receive_DMA(&huart5, UART5_RX_BUF, UART5_RX_BUF_SIZE);
        return;
    }
}
