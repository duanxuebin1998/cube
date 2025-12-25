#ifndef __COMMU_H
#define __COMMU_H
#include "stateformodbus.h"
#include "main.h"
extern uint16_t HoldingRegisterArray[HOLEREGISTER_STOP]; //保持寄存器数组

extern int count_com_CPU2;
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



void CommuToCPU2Init(void);
void HostCommuProcess(uint8_t*rcv,int len);
void PollingInputData(void);
void CPU2_CombinatePackage_Send(uint8_t f_code,uint16_t startadd,uint16_t registercnt,uint32_t* holddata);
void sendToCPU2(uint8_t*arr,uint16_t len,bool flag_fromhost);








#endif

