/*
 * com_manager.c
 *  串口配置 & 协议分发管理模块实现
 *  Created on: 2025年12月10日
 *      Author: admin
 */

#include "com_manager.h"

#include "usart.h"                 /* huart2 / huart3 / huart6 & RX_BUF_SIZE 宏 */
#include "communicate.h"           /* COMx_SET_SEND_MODE / COMx_SET_RECV_MODE 等，如果需要 */
#include "DSM_communication.h"     /* DSM_CommunicationProcess(...) */
#include "wartsila_modbus_communication.h"  /* 你自己的 Wartsila 协议头文件 */
#include "system_parameter.h"      /* 内含 g_deviceParams，如果你用 DeviceParameters 做配置来源 */

#include <string.h>

/* === 这些变量在 usart.c 中定义，这里用 extern 引用 === */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern volatile uint8_t com1_rx_ready;
extern volatile uint8_t com2_rx_ready;
extern volatile uint8_t com3_rx_ready;

extern volatile uint8_t UART2_RX_LEN;
extern volatile uint8_t UART3_RX_LEN;
extern volatile uint8_t UART6_RX_LEN;

extern uint8_t UART2_RX_BUF[UART2_RX_BUF_SIZE];
extern uint8_t UART3_RX_BUF[UART3_RX_BUF_SIZE];
extern uint8_t UART6_RX_BUF[UART6_RX_BUF_SIZE];

/* 你的全局设备参数结构体（system_parameter.c 里定义） */
extern volatile DeviceParameters g_deviceParams;

/* === 全局 COM 配置数组（默认值可按需要修改） === */
ComConfig g_com_config[COM_PORT_NUM] =
{
    /* COM_PORT1 -> USART6 (COM1) 默认 4800, 8N1, DSM 协议 */
    {
        .baudrate = 4800,
        .databits = 8,
        .parity   = COM_PARITY_NONE,
        .stopbits = COM_STOPBITS_1,
        .protocol = COM_PROTO_DSM,
    },
    /* COM_PORT2 -> USART2 (COM2) 默认 4800, 8N1, Modbus RTU */
    {
        .baudrate = 4800,
        .databits = 8,
        .parity   = COM_PARITY_NONE,
        .stopbits = COM_STOPBITS_1,
        .protocol = COM_PROTO_MODBUS_RTU,
    },
    /* COM_PORT3 -> USART3 (COM3) 默认 4800, 8N2, Wartsila */
    {
        .baudrate = 4800,
        .databits = 8,
        .parity   = COM_PARITY_NONE,
        .stopbits = COM_STOPBITS_2,
        .protocol = COM_PROTO_WARTSILA,
    },
};

/* ========================================================================== */
/*                         内部工具函数声明                                   */
/* ========================================================================== */

static UART_HandleTypeDef * COM_GetUartHandle(ComPortIndex com);
static void COM_EnableRxDMAAndIdle(ComPortIndex com);

/* 将“枚举值/代码”转换为具体波特率，可根据你寄存器的定义修改 */
static uint32_t COM_BaudCodeToValue(uint8_t code);

/* 从 g_deviceParams 中把串口相关配置读到 g_com_config[] */
/* !!! TODO: 这里需要你根据 DeviceParameters 里实际字段名修改 !!! */
static void COM_LoadConfigFromDeviceParams(void);

/* ========================================================================== */
/*                            对外函数实现                                    */
/* ========================================================================== */

void COM_InitAllFromParams(void)
{
    /* 1. 从 g_deviceParams 里加载配置到 g_com_config[] */
    COM_LoadConfigFromDeviceParams();

    /* 2. 根据配置重新初始化各个 COM 口 */
    COM_ApplyConfigToUart(COM_PORT1);
    COM_ApplyConfigToUart(COM_PORT2);
    COM_ApplyConfigToUart(COM_PORT3);
}

void COM_ApplyConfigToUart(ComPortIndex com)
{
    UART_HandleTypeDef *huart = COM_GetUartHandle(com);
    if (huart == NULL) {
        return;
    }

    ComConfig *cfg = &g_com_config[com];

    /* 先 DeInit 再重新 Init，保证参数全生效 */
    HAL_UART_DeInit(huart);

    huart->Init.BaudRate = cfg->baudrate;
    huart->Init.WordLength = (cfg->databits == 9) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;

    switch (cfg->parity) {
    case COM_PARITY_EVEN:
        huart->Init.Parity = UART_PARITY_EVEN;
        break;
    case COM_PARITY_ODD:
        huart->Init.Parity = UART_PARITY_ODD;
        break;
    case COM_PARITY_NONE:
    default:
        huart->Init.Parity = UART_PARITY_NONE;
        break;
    }

    switch (cfg->stopbits) {
    case COM_STOPBITS_2:
        huart->Init.StopBits = UART_STOPBITS_2;
        break;
    case COM_STOPBITS_1:
    default:
        huart->Init.StopBits = UART_STOPBITS_1;
        break;
    }

    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK) {
        /* 这里可以根据需要打印错误或设置故障标志 */
        Error_Handler();
    }

    /* 重新开启 IDLE 中断 + DMA 接收（与原 usart.c 中逻辑保持一致） */
    COM_EnableRxDMAAndIdle(com);
}

/**
 * 协议分发函数：你在 App_MainLoop 里收到一帧数据后调用这个
 */
uint32_t COM_DispatchProtocol(ComPortIndex com,
                              const uint8_t *rx, uint16_t rx_len,
                              uint8_t *tx, uint16_t *tx_len)
{
    ComConfig *cfg = &g_com_config[com];

    if ((rx == NULL) || (tx == NULL) || (tx_len == NULL)) {
        return (uint32_t)(-1);
    }

    *tx_len = 0;   /* 先清零 */

    switch (cfg->protocol) {
    case COM_PROTO_DSM:
        /* 你的 DSM 协议处理函数 */
        return DSM_CommunicationProcess(rx, rx_len, tx, tx_len);

    case COM_PROTO_WARTSILA:
        /* TODO: 根据你实际的 Wartsila 协议处理函数名修改 */
        /* 比如： return Wartsila_CommunicationProcess(rx, rx_len, tx, tx_len); */
        return wartsila_modbus_process(rx, rx_len, tx, tx_len);  /* 如果函数名不对请自行改 */

    case COM_PROTO_MODBUS_RTU:
        /* 你的 Modbus RTU 处理函数（需要在 modbus_agreement.h 中声明） */
        return modbus_rtu_process(rx, rx_len, tx, tx_len);

    case COM_PROTO_TRANSPARENT:
        /* 透传：简单回环（可改成转发到另一个 COM 口） */
        memcpy(tx, rx, rx_len);
        *tx_len = rx_len;
        return 0;

    case COM_PROTO_NONE:
    default:
        /* 不处理，直接丢弃 */
        *tx_len = 0;
        return 0;
    }
}

/* ========================================================================== */
/*                         内部工具函数实现                                   */
/* ========================================================================== */

/* 根据 COM 索引返回对应的 UART 句柄 */
static UART_HandleTypeDef * COM_GetUartHandle(ComPortIndex com)
{
    switch (com) {
    case COM_PORT1:
        return &huart6;   /* COM1 -> USART6 */
    case COM_PORT2:
        return &huart2;   /* COM2 -> USART2 */
    case COM_PORT3:
        return &huart3;   /* COM3 -> USART3 */
    default:
        return NULL;
    }
}

/* 根据 COM 口重新开启 IDLE 中断 + DMA 接收
 * 注意：这里和你 usart.c 里的做法保持一致
 */
static void COM_EnableRxDMAAndIdle(ComPortIndex com)
{
    switch (com) {
    case COM_PORT1: /* USART6 / COM1 */
        __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart6, UART6_RX_BUF, UART6_RX_BUF_SIZE);
        break;

    case COM_PORT2: /* USART2 / COM2 */
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart2, UART2_RX_BUF, UART2_RX_BUF_SIZE);
        break;

    case COM_PORT3: /* USART3 / COM3 */
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart3, UART3_RX_BUF, UART3_RX_BUF_SIZE);
        break;

    default:
        break;
    }
}

/* 波特率码 -> 实际波特率，供 DeviceParameters 映射使用 */
static uint32_t COM_BaudCodeToValue(uint8_t code)
{
    switch (code) {
    case 0:  return 4800;
    case 1:  return 9600;
    case 2:  return 19200;
    case 3:  return 38400;
    case 4:  return 57600;
    case 5:  return 115200;
    default: return 4800;
    }
}
