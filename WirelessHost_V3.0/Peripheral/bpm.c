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
}

void bpm_receive()
{
	unsigned int ret;
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
        	if(ret != 0)
        	{
//        		HAL_UART_Transmit_DMA(&huart2, &receivedDataBuffer.duration[0], receivedDataBuffer.currentBit+5) ;
        	}
            receivedDataBuffer.currentBit = 0;//清零接收
            receivedDataBuffer.duration[0] =0;
            while(receivedDataBuffer.bpm_state == BPM_RECEIVE_COMPLETE_STATE );//等待下一个字节的到来
        }
    }
    if(receivedDataBuffer.bpm_rxDataLength != 0)//转发到上位机
    {
    	HAL_UART_Transmit_DMA(&huart2, &receivedDataBuffer.bpm_rxBuffer[0], receivedDataBuffer.bpm_rxDataLength) ;
    }
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


