/*
 * @FilePath     : master\peripherals\bpm.c
 * @Description  :BPM接收与发送
 * @Author       : Aubon
 * @Date         : 2023-11-10 08:46:57
 * @LastEditors  : Duan
 * @LastEditTime : 2023-11-10 08:57:22
 * Copyright 2023 Aubon, All Rights Reserved.
 * 2023-11-10 08:46:57
 */
#include "bpm.h"
#include "timer.h"

BPM_ReceiveData receivedDataBuffer;
volatile char bpm_rxBuffer[32];


unsigned int unpackBPM(void);
static unsigned int unpackBPM_bit(unsigned char currentBit);
void bpm_init(void)
{
    receivedDataBuffer.bpm_state =BPM_IDLE_STATE ;
    receivedDataBuffer.current_frequency = POWER_MIN_FREQUENCE;
}

void bpm_receive()
{
	unsigned int ret;
	static unsigned int error =0;
	uint8_t bcc;
    receivedDataBuffer.currentBit = 0;
    receivedDataBuffer.bpm_rxDataLength = 0;
    receivedDataBuffer.duration[0] =0;
    receivedDataBuffer.ioState[0] =GPIO_PIN_RESET;
    while(receivedDataBuffer.bpm_state ==BPM_IDLE_STATE );
    while(receivedDataBuffer.bpm_state !=BPM_IDLE_STATE )
    {
        if(receivedDataBuffer.bpm_state == BPM_RECEIVE_COMPLETE_STATE)//接收完成准备解包
        {
        	ret = unpackBPM();
//        	HAL_UART_Transmit_DMA(&huart2, &receivedDataBuffer.duration[0], receivedDataBuffer.currentBit) ;
        	if(ret ==1)
        	{
//        		HAL_UART_Transmit_DMA(&huart2, &receivedDataBuffer.duration[0], receivedDataBuffer.currentBit+5) ;
        	}
            receivedDataBuffer.currentBit = 0;//清零接收
            receivedDataBuffer.duration[0] =0;
            while(receivedDataBuffer.bpm_state == BPM_RECEIVE_COMPLETE_STATE );//等待下一个字节的到来
        }
    }
//    if(error !=0)
//    {
//        auto_change_out_frequence();//改变接收频率
//    }
    if(receivedDataBuffer.bpm_rxDataLength >= 5)//转发到上位机
    {
    	bcc = getBcc(receivedDataBuffer.bpm_rxBuffer,receivedDataBuffer.bpm_rxDataLength-1);
//    	HAL_UART_Transmit_DMA(&huart2, &bcc, 1) ;
//    	HAL_UART_Transmit_DMA(&huart2, &bpm_rxBuffer[receivedDataBuffer.bpm_rxDataLength-1], 1) ;
        if(bcc !=receivedDataBuffer.bpm_rxBuffer[receivedDataBuffer.bpm_rxDataLength-1])
//        if(0 !=receivedDataBuffer.bpm_rxBuffer[receivedDataBuffer.bpm_rxDataLength-1])
        {
        	HAL_UART_Transmit_DMA(&huart2, &receivedDataBuffer.bpm_rxBuffer[0], receivedDataBuffer.bpm_rxDataLength) ;
			if(error !=0)
			{
				HAL_Delay(50);
				auto_change_out_frequence();//改变接收频率
				HAL_Delay(50);
				error = 0;
			}
			else
			{
				error ++;
			}
        }
        else
        {
        	error = 0;
        	HAL_UART_Transmit_DMA(&huart2, &receivedDataBuffer.bpm_rxBuffer[0], receivedDataBuffer.bpm_rxDataLength) ;
        }
        //50MS的接收超时，超过50MS没有收到数据自动扫频
        __HAL_TIM_SET_COUNTER(&htim4,0);
        HAL_TIM_Base_Start_IT(&htim4);
    }
}
uint8_t getBcc(uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint8_t bcc = 0;
    while(length--)
    {
        bcc ^= *data++;
    }

    return bcc;
}
unsigned int unpackBPM(void)
{
    int i=0;
    unsigned char currentBit =0;
    if(unpackBPM_bit(currentBit) ==0)//起始位正确
    {
        currentBit++;
        for(i=0;i<8;i++)//解包
        {
            if(unpackBPM_bit(currentBit) ==0)
            {
                receivedDataBuffer.bpm_rxBuffer[receivedDataBuffer.bpm_rxDataLength]>>=1;
                currentBit++;
            }
            else if(unpackBPM_bit(currentBit) ==1)
            {
                receivedDataBuffer.bpm_rxBuffer[receivedDataBuffer.bpm_rxDataLength]>>=1;
                receivedDataBuffer.bpm_rxBuffer[receivedDataBuffer.bpm_rxDataLength] |=0x80;
                currentBit+=2;
            }
            else
            {
                return 2;//数据位解包异常
            }
        }
        if(unpackBPM_bit(currentBit) !=0)//结束位判断
        {
            receivedDataBuffer.currentBit = 0;//清零接收
            receivedDataBuffer.duration[0] =0;
            receivedDataBuffer.bpm_rxDataLength++;//接收字节长度+1
            return 0;//解包正常
        }
        else
        {
            return 3;//结束位异常
        }
    }
    else
    {
        return 1;//起始位解包异常
    }
}
static unsigned int unpackBPM_bit(unsigned char currentBit)
{
    if(receivedDataBuffer.duration[currentBit] >15)//数据0
    {
        return 0;
    }
    else
    {
        if(receivedDataBuffer.duration[currentBit+1] < 15)//数据1
        {
            return 1;
        }
    }
    return 2;
}


