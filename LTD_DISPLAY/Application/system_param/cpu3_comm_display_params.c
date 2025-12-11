/*
 * cpu3_comm_display_params.c
 *
 *  Created on: 2025年12月10日
 *      Author: admin
 */


#include "cpu3_comm_display_params.h"
#include "usart.h"
#include <string.h>

/* 这些在 usart.c 里定义 */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern uint8_t UART2_RX_BUF[UART2_RX_BUF_SIZE];
extern uint8_t UART3_RX_BUF[UART3_RX_BUF_SIZE];
extern uint8_t UART6_RX_BUF[UART6_RX_BUF_SIZE];

/* 全局实例 */
Cpu3CommAndDisplayParams g_cpu3_comm_display_params;

/* ================ 内部小工具：用一个 ComPortConfig 初始化一个 UART ================ */
static void Cpu3_ReinitOneUart(UART_HandleTypeDef *huart, const ComPortConfig *cfg)
{
    if (!huart || !cfg) return;

    HAL_UART_DeInit(huart);

    huart->Init.BaudRate = cfg->baudrate;
    huart->Init.WordLength = (cfg->databits == 9) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;

    /* 校验 */
    switch (cfg->parity) {
    case COM_PARITY_EVEN: huart->Init.Parity = UART_PARITY_EVEN; break;
    case COM_PARITY_ODD:  huart->Init.Parity = UART_PARITY_ODD;  break;
    case COM_PARITY_NONE:
    default:              huart->Init.Parity = UART_PARITY_NONE; break;
    }

    /* 停止位 */
    switch (cfg->stopbits) {
    case COM_STOPBITS_2: huart->Init.StopBits = UART_STOPBITS_2; break;
    case COM_STOPBITS_1:
    default:             huart->Init.StopBits = UART_STOPBITS_1; break;
    }

    huart->Init.Mode       = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================== 初始化默认值 ========================== */
void Cpu3_Params_InitDefaults(void)
{
    memset(&g_cpu3_comm_display_params, 0, sizeof(g_cpu3_comm_display_params));

    /* 屏幕基本信息 */
    g_cpu3_comm_display_params.local_led_version = 0x0100;
    g_cpu3_comm_display_params.language          = 0;

    /* 数据源默认 */
    g_cpu3_comm_display_params.screen_source_oil   = 0;
    g_cpu3_comm_display_params.screen_source_water = 0;
    g_cpu3_comm_display_params.screen_source_d     = 0;
    g_cpu3_comm_display_params.screen_source_t     = 0;

    /* 显示类默认 */
    g_cpu3_comm_display_params.screen_decimal  = 2;
    g_cpu3_comm_display_params.screen_str_mode = 0;

    /* ========== 串口默认：保持和你现在 usart.c 一致 ========== */

    /* COM1 = USART6: 4800 8N1 DSM */
    g_cpu3_comm_display_params.com1.baudrate = 4800;
    g_cpu3_comm_display_params.com1.databits = 8;
    g_cpu3_comm_display_params.com1.parity   = COM_PARITY_NONE;
    g_cpu3_comm_display_params.com1.stopbits = COM_STOPBITS_1;
    g_cpu3_comm_display_params.com1.protocol = COM_PROTO_DSM;

    /* COM2 = USART2: 4800 8N1 Modbus RTU */
    g_cpu3_comm_display_params.com2.baudrate = 4800;
    g_cpu3_comm_display_params.com2.databits = 8;
    g_cpu3_comm_display_params.com2.parity   = COM_PARITY_NONE;
    g_cpu3_comm_display_params.com2.stopbits = COM_STOPBITS_1;
    g_cpu3_comm_display_params.com2.protocol = COM_PROTO_MODBUS_RTU;

    /* COM3 = USART3: 4800 8N2 Wartsila */
    g_cpu3_comm_display_params.com3.baudrate = 4800;
    g_cpu3_comm_display_params.com3.databits = 8;
    g_cpu3_comm_display_params.com3.parity   = COM_PARITY_NONE;
    g_cpu3_comm_display_params.com3.stopbits = COM_STOPBITS_2;
    g_cpu3_comm_display_params.com3.protocol = COM_PROTO_WARTSILA;
}

/* ========================== 重配全部串口 ========================== */
void Cpu3_ReinitAllUarts(void)
{
    /* COM1 = USART6 */
    Cpu3_ReinitOneUart(&huart6, &g_cpu3_comm_display_params.com1);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);

    /* COM2 = USART2 */
    Cpu3_ReinitOneUart(&huart2, &g_cpu3_comm_display_params.com2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);

    /* COM3 = USART3 */
    Cpu3_ReinitOneUart(&huart3, &g_cpu3_comm_display_params.com3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
}
