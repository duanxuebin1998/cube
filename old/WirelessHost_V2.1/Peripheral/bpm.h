#ifndef PERIPHERALS_BPM_H_
#define PERIPHERALS_BPM_H_
#include "main.h"

#define POWER_MIN_FREQUENCE 130000
#define POWER_MAX_FREQUENCE 180000
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
    GPIO_PinState ioState[64];// IO电平
    unsigned char  duration[64]; // 状态持续时间
    unsigned char currentBit;//当前位数
    unsigned char bpm_rxBuffer[64];//BPM接收数据
    unsigned char  bpm_rxDataLength;
    unsigned int current_frequency;//当前频率
} BPM_ReceiveData;
extern BPM_ReceiveData receivedDataBuffer;
//BPM_State

void bpm_init(void);
void bpm_receive(void);
uint8_t getBcc(uint8_t *data, uint16_t length);
#endif /* PERIPHERALS_GPIO_H_ */
