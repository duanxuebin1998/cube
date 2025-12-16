#ifndef PERIPHERALS_BPM_H_
#define PERIPHERALS_BPM_H_
#include "main.h"


#define BPM_MAX_DELAY 40

// 接收状态
typedef enum {
    BPM_IDLE_STATE,//空闲状态
    BPM_RECEIVING_STATE,  // 接收中
    BPM_RECEIVE_COMPLETE_STATE,  // 接收完成
    BPM_SEND_STATE,//发送状态
} BPM_State;

// 定义BPM接收数据解包的结构体
typedef struct {
    BPM_State bpm_state;
    GPIO_PinState ioState[32];// IO电平
    unsigned char  duration[32]; // 状态持续时间
    unsigned char currentBit;//当前位数
    unsigned char bpm_rxBuffer[32];//BPM接收数据
    unsigned char  bpm_rxDataLength;
} BPM_ReceiveData;
extern BPM_ReceiveData receivedDataBuffer;
//BPM_State

void bpm_init(void);
void bpm_receive(void);
#endif /* PERIPHERALS_GPIO_H_ */
