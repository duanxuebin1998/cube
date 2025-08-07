#include "main.h"
#include "spi.h"
#include "usart.h"
#include "communicate.h"
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include "crc.h"

#include "modbus_agreement.h"




#define DEBUG_COMMUCPU2 1
#define ADERSS 0X08


bool flagOfFromHost = false;





static void CPU2_Response03Process(uint8_t*arr,uint16_t len);
static void CPU2_Response04Process(uint8_t*arr,uint16_t len);
static void CPU2_Response05Process(uint8_t*arr,uint16_t len);
static void CPU2_Response10Process(uint8_t*arr,uint16_t len);



/*与CPU2通讯初始化*/
void CommuToCPU2Init(void)
{
//    SPI1_Init();//与CPU2通信初始化
//    Timer3Init();
//    Timer3Start(3); //定时轮询设备常规测量数据
//    Timer8Init();   //总超时时间定时器
    PollingInputData();//初始化后先轮询一遍测量数据
}
/*与CPU2通讯接收包主处理过程*/
void CommuCPU2MainProcess(uint8_t*rcv,int len)
{
    #if DEBUG_COMMUCPU2
    int i;
    printf("CPU2com_RCV %d : ",len);
    for(i = 0;i < len;i++)
        printf("%02X ",rcv[i]);
    printf("\r\n");
    #endif
    if(len <= 3)
        return;
//    if(!SlaveCheckCRC(rcv,len))
//    {
//        printf("CPU2\r\n");
//        return;
//    }
    /* 解析数据 */
	if(rcv[0]!= ADERSS)
    {
        printf("CPU2\r\n");
        return;
    }
    functioncode = rcv[1];
    cnt_commutoCPU2 = 0;
//    Timer8Stop();
    switch(functioncode)
    {
        case FUNCTIONCODE_READ_HOLDREGISTER:
        {
            CPU2_Response03Process(rcv,len);
            break;
        }
        case FUNCTIONCODE_READ_INPUTREGISTER:
        {
            CPU2_Response04Process(rcv,len);
            break;
        }
        case FUNCTIONCODE_WRITE_COIL:
        {
            CPU2_Response05Process(rcv,len);
            break;
        }
        case FUNCTIONCODE_WRITE_MULREGISTER:
        {
            CPU2_Response10Process(rcv,len);
            break;
        }
    }
}
/* 轮询输入寄存器数据 */
void PollingInputData(void)
{
    static bool flag_poweronread = false;
    
    CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,INPUTREG_PATTERNOFWORK,INPUTREGISTER_END - INPUTREG_PATTERNOFWORK,NULL);

    

//    if(cnt_commutoCPU2 >= COMMU_ERROR_MAX)
//        setEquipStateError();
//    else
//    {
        if(flag_poweronread == false)
        {
//            CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_DECIMALPLACES,HOLDREGISTER_INPUTVAL_T - HOLDREGISTER_DECIMALPLACES + 1,NULL);
//            CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_PRESSURE_BOTADD,2,NULL);
//            CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_PRESSURE_BOTSWITCH,2,NULL);
//            CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_LANGUAGE,1,NULL);
//            CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_DRUMCIRCUMFERENCE,2,NULL);
//            CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_SCREENOFF,1,NULL);
//            
//            
            flag_poweronread = true;
        }
//    }
}
/*由屏幕向CPU2发送指令包*/
void CPU2_CombinatePackage_Send(uint8_t f_code,uint16_t startadd,uint16_t registercnt,uint8_t* holddata)
{
    static uint8_t arr[255];
    static int len = 0;
    uint16_t crc;
    int i;
    
    arr[len++] = ADERSS;
    arr[len++] = f_code;
    arr[len++] = startadd >> 8;
    arr[len++] = startadd & 0xFF;
    arr[len++] = registercnt >> 8;
    arr[len++] = registercnt & 0xFF;
    if(f_code == FUNCTIONCODE_WRITE_MULREGISTER && holddata != NULL)
    {
        arr[len++] = registercnt * 2;
        for(i = 0;i < registercnt * 2;i++)
            arr[len++] = holddata[i];
    }
    crc = CRC16((u8*)arr,len); 
    arr[len++] = crc & 0xff;
    arr[len++] = crc >> 8;
    sendToCPU2(arr,len,false);
	
    len = 0;
    functioncode = f_code;
    startaddress = startadd;
    registeramount = registercnt;
	
	HAL_Delay(100);
	if((f_code == FUNCTIONCODE_READ_HOLDREGISTER)||(f_code == FUNCTIONCODE_READ_INPUTREGISTER))
    {
//        SPI1_ReadSend(0xFF, SPI1_readbuff, registercnt*2+5);
//		CommuCPU2MainProcess(SPI1_readbuff,registercnt*2+5);
    }
	else
	{
//		SPI1_ReadSend(0xFF, SPI1_readbuff, registercnt*2+5);
		HAL_Delay(300);
	}
}
/*向CPU2发送数据包*/
void sendToCPU2(uint8_t*arr,uint16_t len,bool flag_fromhost)
{
//    Timer8Start(1);接收超时判定
    flagOfFromHost = flag_fromhost;
//    SPI1_Send(arr,len);
    cnt_commutoCPU2++;
    #if DEBUG_COMMUCPU2
    {
        int i;
        for(i = 0;i < len;i++)
            printf("%02X ",arr[i]);
        printf("\n");
    }
    #endif
}
/*解析CPU2的响应包0x03功能码*/
static void CPU2_Response03Process(uint8_t*arr,uint16_t len)
{
    int i;
    byteamount = arr[2];
    if(byteamount != registeramount * 2)
        return;
    for(i = 0;i < byteamount;i++)
        hold_register_value[(startaddress << 1) + i] = arr[3 + i];
    /* 刷新寄存器 */
    AnalysisHoldRegister();
}
/*解析CPU2的响应包0x04功能码*/
static void CPU2_Response04Process(uint8_t*arr,uint16_t len)
{
    int i;
    byteamount = arr[2];
    if(byteamount != registeramount * 2)
        return;
    for(i = 0;i < byteamount;i++)
        input_register_value[(startaddress << 1) + i] = arr[3 + i];
    /* 刷新设备测量数据 */
    Analysis04Register();
}
static void CPU2_Response05Process(uint8_t*arr,uint16_t len)
{
    
    
    
}


static void CPU2_Response10Process(uint8_t*arr,uint16_t len)
{
    
    
    
}

