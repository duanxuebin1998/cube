#ifndef __COMMU_H
#define __COMMU_H
#include "stateformodbus.h"
#include "system_parameter.h"
#include "main.h"
extern uint16_t HoldingRegisterArray[HOLEREGISTER_STOP]; /* 保持寄存器数组 */

extern volatile bool wait_response;

#define COM1_SET_RECV_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_SET)
#define COM1_SET_SEND_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_RESET)
#define COM2_SET_RECV_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_SET)
#define COM2_SET_SEND_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_RESET)
#define COM3_SET_RECV_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_SET)
#define COM3_SET_SEND_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_RESET)
#define RS485_SET_RECV_MODE()  HAL_GPIO_WritePin(MAIN_BOARD_485_SEL_GPIO_Port, MAIN_BOARD_485_SEL_Pin, GPIO_PIN_SET)
#define RS485_SET_SEND_MODE()  HAL_GPIO_WritePin(MAIN_BOARD_485_SEL_GPIO_Port, MAIN_BOARD_485_SEL_Pin, GPIO_PIN_RESET)

static inline void COM1_SendMode(void) { COM1_SET_SEND_MODE(); }
static inline void COM1_RecvMode(void) { COM1_SET_RECV_MODE(); }

static inline void COM2_SendMode(void) { COM2_SET_SEND_MODE(); }
static inline void COM2_RecvMode(void) { COM2_SET_RECV_MODE(); }

static inline void COM3_SendMode(void) { COM3_SET_SEND_MODE(); }
static inline void COM3_RecvMode(void) { COM3_SET_RECV_MODE(); }



/**
 * @brief 处理Modbus 协议中的 HostCommuProcess 逻辑。
 *
 * @param rcv 业务参数。
 * @param len 数据长度。
 */
bool HostCommuProcess(uint8_t*rcv,int len);
/**
 * @brief 执行Modbus 协议中的 PollingInputData 逻辑。
 */
void PollingInputData(void);
/**
 * @brief 判断是否仍应显示等待 CPU2 首次状态和当前连接协议快照的同步页面。
 * @return true 表示继续显示通讯尝试页，false 表示进入正常状态页或故障页。
 */
bool CPU2_CommShouldShowStartup(void);
/**
 * @brief 判断 CPU2 状态/参数/协议快照是否完整、共享协议是否兼容且通信故障未锁存。
 * @return true 表示允许访问 CPU2 参数和普通命令，false 表示同步未完成、协议不兼容或故障已锁存。
 */
bool CPU2_CommIsAvailable(void);
/**
 * @brief 判断指定 CPU2 命令是否可下发；取消命令在当前连接协议已确认后可绕过其余参数刷新。
 * @param cmd 待下发命令。
 * @return true 表示当前通信状态允许下发该命令。
 */
bool CPU2_CommCanSendCommand(CommandType cmd);
/**
 * @brief 在 UART5 错误中断中记录待处理标志并解除当前等待。
 * @note 仅允许在 ISR 中置标志，不在中断内打印、计数或修改设备故障状态。
 */
void CPU2_CommNotifyUartErrorFromISR(void);
/**
 * @brief 发送Modbus 协议中的 CPU2_CombinatePackage_Send 逻辑。
 *
 * @param f_code 业务参数。
 * @param startadd 业务参数。
 * @param registercnt 业务参数。
 * @param holddata 数据缓冲区。
 */
bool CPU2_CombinatePackage_Send(uint8_t f_code,uint16_t startadd,uint16_t registercnt,uint32_t* holddata);
/**
 * @brief 发送Modbus 协议中的 sendToCPU2 逻辑。
 *
 * @param arr 业务参数。
 * @param len 数据长度。
 * @param flag_fromhost 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool sendToCPU2(uint8_t*arr,uint16_t len,bool flag_fromhost);








#endif

