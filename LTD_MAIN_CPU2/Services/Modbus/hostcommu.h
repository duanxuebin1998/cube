#ifndef __HOSTCOMMU_H
#define __HOSTCOMMU_H
#include "main.h"

#define RS485_SET_RECV_MODE()  HAL_GPIO_WritePin(CPU2_485_SEL_GPIO_Port, CPU2_485_SEL_Pin, GPIO_PIN_SET)
#define RS485_SET_SEND_MODE()  HAL_GPIO_WritePin(CPU2_485_SEL_GPIO_Port, CPU2_485_SEL_Pin, GPIO_PIN_RESET)

#define HOSTCOMMU_SENDLENGTH 1000

/**
 * @brief 执行主板通信中的 HostCommuInit 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int HostCommuInit(void);
/**
 * @brief 处理主板通信中的 HostCommuProcess 逻辑。
 *
 * @param rcvbuff 数据缓冲区。
 * @param rcvcount 业务参数。
 */
void HostCommuProcess(uint8_t *rcvbuff, int rcvcount);
/* 在主循环中输出 HostCommuProcess 延后的异常日志，避免中断里直接打印。 */
void HostCommu_ProcessDeferredLogs(void);

#endif	  

