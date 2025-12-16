/* com_manager.h
 *
 * 串口配置 & 协议分发管理模块
 * 用于管理 COM1(COM_PORT1, USART6), COM2(USART2), COM3(USART3)
 */

#ifndef COMMON_COM_MANAGER_H_
#define COMMON_COM_MANAGER_H_

#include <stdint.h>

/* === COM 口索引 === */
typedef enum {
    COM_PORT1 = 0,   /* 对应 USART6，COM1 */
    COM_PORT2 = 1,   /* 对应 USART2，COM2 */
    COM_PORT3 = 2,   /* 对应 USART3，COM3 */
    COM_PORT_NUM
} ComPortIndex;

/* === 协议类型（可在保持寄存器里配置） === */
typedef enum {
    COM_PROTO_NONE        = 0,   /* 不处理 */
    COM_PROTO_MODBUS_RTU  = 1,   /* 标准 Modbus RTU 协议 */
    COM_PROTO_DSM         = 2,   /* 你的 DSM 协议 */
    COM_PROTO_WARTSILA    = 3,   /* Wartsila 协议 */
    COM_PROTO_TRANSPARENT = 4,   /* 透传 / 回环 */
} ComProtocolType;

/* === 校验、停止位类型（与 HAL 的配置做简单映射） === */
typedef enum {
    COM_PARITY_NONE = 0,
    COM_PARITY_EVEN = 1,
    COM_PARITY_ODD  = 2,
} ComParityType;

typedef enum {
    COM_STOPBITS_1 = 0,
    COM_STOPBITS_2 = 1,
} ComStopBitsType;

/* === COM 口配置结构体 === */
typedef struct {
    uint32_t        baudrate;   /* 波特率：4800/9600/115200 等 */
    uint8_t         databits;   /* 数据位：一般为 8 或 9 */
    ComParityType   parity;     /* 校验位 */
    ComStopBitsType stopbits;   /* 停止位 */
    ComProtocolType protocol;   /* 当前协议类型 */
} ComConfig;

/* 全局配置数组：索引为 ComPortIndex */
extern ComConfig g_com_config[COM_PORT_NUM];

/**
 * @brief  从 DeviceParameters / FRAM 等全局参数加载串口配置，并应用到所有 COM 口
 *         - 内部包含：读取参数 -> 填充 g_com_config[] -> 调用 COM_ApplyConfigToUart()
 *         - 需要在参数加载完成后调用（例如系统上电初始化阶段）
 */
void COM_InitAllFromParams(void);

/**
 * @brief  按照 g_com_config[com] 对应的配置，重新初始化某一个 COM 口的 UART
 *         - 内部会调用 HAL_UART_DeInit + HAL_UART_Init
 *         - 并重新开启 IDLE 中断 + DMA 接收
 * @param  com: COM_PORT1 / COM_PORT2 / COM_PORT3
 */
void COM_ApplyConfigToUart(ComPortIndex com);

/**
 * @brief  串口接收一帧完成后，根据当前协议类型分发到不同协议处理函数
 * @param  com      : COM_PORT1 / COM_PORT2 / COM_PORT3
 * @param  rx       : 接收到的数据缓冲区
 * @param  rx_len   : 接收长度
 * @param  tx       : 输出应答缓冲区
 * @param  tx_len   : 输出应答长度（由协议处理函数写入）
 * @return 协议处理返回值，0 为成功，其它为错误码（由各协议栈定义）
 */
uint32_t COM_DispatchProtocol(ComPortIndex com,
                              const uint8_t *rx, uint16_t rx_len,
                              uint8_t *tx, uint16_t *tx_len);


#endif /* COMMON_COM_MANAGER_H_ */
