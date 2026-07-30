
/*************************************************
Copyright(C),2025, Aubon
Filename: hart.c
Version:  V1.2
Date: 2022-03-22
Description: HERT部分
History: 2022-03-16(初版)
*************************************************/
#include <string.h>
#include "hart.h"
#include "ad5421.h"
#include "AoOutput/ao_output.h"
#include "system_parameter.h"
/* #include "timer5.h" */
#include "usart.h"


HartParametersTPYE HartParameters; /* 全局变量：HART参数结构体 */
float SetGlobalVariables = 1.0f; /* TEXT */

static int AnalyseRcvPackage(RCV_TYPE *RcvPackage,u8* HartCommand,u8* FlagofLongFrame);			/* hart接收解包、校验 */
static void InitializeSndPackage(SND_TYPE *sendframe,u8 HartCommand);							/* 设置长指令响应时发送数据包的定界符，地址，命令，通信状态，设备状态 */
static u8 CalculateBCC(SND_TYPE *sendframe,u8 datalen);											/* 计算发送包的BCC校验位 */
static u32 SetFloatData(float Variable);														/* 把FLOAT变量小端存储转化成大端存储 */
static int ResponseCommand0(RCV_TYPE *revframe,SND_TYPE *sendframe);							/* 响应指令0：读唯一标识 */
static int ResponseCommand1(RCV_TYPE *revframe,SND_TYPE *sendframe);							/* 响应指令1：读主变量 */
static int ResponseCommand2(RCV_TYPE *revframe,SND_TYPE *sendframe);							/* 响应指令2：读环路电流和量程百分比 */
static int ResponseCommand3(RCV_TYPE *revframe,SND_TYPE *sendframe);							/* 响应指令3：读动态变量和环路电流 */
static int ResponseCommand6(RCV_TYPE *revframe,SND_TYPE *sendframe);
/**
 * @brief 读取 AO 所选且有效的过程样本，并把 0.1 mm 值换算为 mm；无有效样本返回 0。
 *
 * @return AO 当前选择的过程样本有效时返回其 0.1 mm 定点值换算后的 mm 数值；样本获取失败或无效时返回 0.0。
 */
static float Hart_GetPrimaryVariable(void);
/**
 * @brief 按现有 HART 兼容口径读取实时温度次变量。
 *
 * 遇到实时温度或无线温度无效哨兵时返回 0.0；其他值直接将 g_measurement.debug_data.temperature 除以 100.0。
 *
 * @return 实时或无线温度为无效哨兵时返回 0.0；否则直接返回 g_measurement.debug_data.temperature / 100.0f，且不扣除原始编码的 200.00 ℃
 *         偏移。
 * @note 当前实现没有扣除 CPU2 温度原始编码中的 200.00 ℃ 偏移，本轮只据实说明既有行为，不改变 HART 数值口径。
 */
static float Hart_GetSecondaryVariable(void);
/**
 * @brief 把有效平均密度原始值换算为工程值；无效密度返回 0。
 *
 * @return 平均密度为 UNVALID_DENSITY 时返回 0.0；否则返回 RAW_TO_DENSITY 换算后的密度工程值，单位 kg/m3。
 */
static float Hart_GetTertiaryVariable(void);
/**
 * @brief 把长地址响应包转换成短地址响应包。
 *
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @param datalen 协议数据区的有效长度，单位字节。
 * @param HartCommand 当前 HART 命令号。
 * @return 返回按当前映射得到的长地址响应包转换成短地址响应包；非法输入使用 @brief 说明的兜底地址或无效值。
 */
static u8 ConvertToShortAddressResponsePacket(SND_TYPE *sendframe,u8 datalen,u8 HartCommand);	/* 把长地址响应包转换成短地址响应包 */
/**
 * @brief 在发送包数据前添加先导符0XFF。
 *
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @param datalen 协议数据区的有效长度，单位字节。
 * @param NumberOfPreambles HART 响应前导字节 0xFF 的数量。
 * @return 返回添加指定数量 0xFF 前导字节后的 HART 发送包总长度，单位字节。
 */
static u8 AddPreamble(SND_TYPE *sendframe,u8 datalen,u8 NumberOfPreambles);						/* 在发送包数据前添加先导符0XFF */

/**
 * @brief 把 HART 调制解调器 RTS 引脚置低。
 */
static inline void HART_RTS_LOW(void)
{
    HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 把 HART 调制解调器 RTS 引脚置高。
 */
static inline void HART_RTS_HIGH(void)
{
    HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 将 HART 收发方向置为发送空闲态，并初始化 HART 过程变量。
 */
void HartInit(void)
{
	HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_SET);
/* Timer5Init(); / *Timer5初始化* / */
	HartParameterInit();				/* Hart参数初始化 */ /* TEXT */
}

/**
 * @brief 用当前 AO 电流及实时过程变量初始化 HART 参数缓存。
 */
void HartParameterInit(void) /* TEXT */
{
	HartParameters.Current = AoOutput_GetCurrent_mA();
	HartParameters.MaxPrimaryVariable =  SetGlobalVariables;
	HartParameters.MinPrimaryVariable = SetGlobalVariables;
	HartParameters.PrimaryVariable =  SetGlobalVariables;
	HartParameters.SecondaryVariable =  SetGlobalVariables;
	HartParameters.TertiaryVariable =  SetGlobalVariables;
}

/**
 * @brief 读取 AO 所选且有效的过程样本，并把 0.1 mm 值换算为 mm；无有效样本返回 0。
 *
 * @return AO 当前选择的过程样本有效时返回其 0.1 mm 定点值换算后的 mm 数值；样本获取失败或无效时返回 0.0。
 */
static float Hart_GetPrimaryVariable(void)
{
    AoProcessSample sample;

    if ((AoOutput_GetSelectedProcessSample(&sample) != NO_ERROR) ||
        (sample.valid == 0U)) {
        return 0.0f;
    }
    return ((float)sample.value_01mm) / 10.0f;
}

/**
 * @brief 按现有 HART 兼容口径读取实时温度次变量。
 *
 * 遇到实时温度或无线温度无效哨兵时返回 0.0；其他值直接将 g_measurement.debug_data.temperature 除以 100.0。
 *
 * @return 实时或无线温度为无效哨兵时返回 0.0；否则直接返回 g_measurement.debug_data.temperature / 100.0f，且不扣除原始编码的 200.00 ℃
 *         偏移。
 * @note 当前实现没有扣除 CPU2 温度原始编码中的 200.00 ℃ 偏移，本轮只据实说明既有行为，不改变 HART 数值口径。
 */
static float Hart_GetSecondaryVariable(void)
{
    if ((g_measurement.debug_data.temperature == UNVALID_TEMPERATURE_REALTIME) ||
        (g_measurement.debug_data.temperature == UNVALID_TEMPERATURE_WIRELESS)) {
        return 0.0f;
    }

    return ((float)g_measurement.debug_data.temperature) / 100.0f;
}

/**
 * @brief 把有效平均密度原始值换算为工程值；无效密度返回 0。
 *
 * @return 平均密度为 UNVALID_DENSITY 时返回 0.0；否则返回 RAW_TO_DENSITY 换算后的密度工程值，单位 kg/m3。
 */
static float Hart_GetTertiaryVariable(void)
{
    if (g_measurement.density_distribution.average_density == UNVALID_DENSITY) {
        return 0.0f;
    }

    return RAW_TO_DENSITY(g_measurement.density_distribution.average_density);
}
/**
 * @brief HART通信主机指令包处理函数,处理不同指令并发送响应包。
 *
 * @param RcvBuff 已经接收完成的 HART 请求帧缓冲区。
 * @param SendBuff 用于输出 HART 响应帧的缓冲区。
 * @param Sendlen 用于返回 HART 响应帧长度的输出参数，单位字节。
 * @return 返回 HART 请求解析或命令响应结果码；同时通过 SendBuff 和 Sendlen 输出响应帧。
 */
u8 HartCommunicationProcess(u8* RcvBuff,u8* SendBuff,volatile u8* Sendlen)
{
	int ret;
	u8 HartCommand;						/* HART指令代码 */
	u8 FlagofLongFrame;					/* 接收包长短帧标志位 1：长帧 0：短帧 */
	RCV_TYPE *RcvPackage;				/* 定义接收包共联体 */
	SND_TYPE *SndPackage;				/* 定义发送包共联体 */

	if (Sendlen == NULL) {
		return 1U;
	}
	*Sendlen = 0U;
	if ((RcvBuff == NULL) || (SendBuff == NULL)) {
		return 1U;
	}
	if (g_deviceParams.ao_output.work_mode != AO_WORK_MODE_HART_SLAVE_OUTPUT) {
		return 0U;
	}
	RcvPackage = (RCV_TYPE*)RcvBuff;	/* RevBuff强转接收包结构共联体 */
	SndPackage = (SND_TYPE*)SendBuff;	/* SendBuff强转发送包结构共联体 */
	ret = AnalyseRcvPackage(RcvPackage,&HartCommand,&FlagofLongFrame); /* hart接收解包、校验，读取指令代码 */
	if(ret!=0)
	{	
		printf("接收包异常： %d\r\n",ret );	/* 打印接收包返回代码 */
		int i;
		printf("Rcv %d:\t", USART2_RX_LEN);
		for (i = 0; i < USART2_RX_LEN; i++) {
			printf("%02X ", USART2_RX_BUF[i]);
		}
		printf("\n");
		return ret;
	}
	else
	{
		switch(HartCommand)	/* 根据收到的HRAT指令按长地址格式组包响应 */
		{
			case COMMAND0_READ_UNIQUE_IDENTIFIE:
			{
				printf("收到指令0 \r\n" );
				*Sendlen = ResponseCommand0(RcvPackage,SndPackage);
				break;
			}
			case COMMAND1_READ_PRIMARY_VARIABLE:
			{
				printf("收到指令1 \r\n" );
				*Sendlen = ResponseCommand1(RcvPackage,SndPackage);
				break;
			}
			case COMMAND2_READ_CURRENT:
			{
				printf("收到指令2 \r\n" );
				*Sendlen = ResponseCommand2(RcvPackage,SndPackage);
				break;
			}
			case COMMAND3_READ_DYNAMIC_VARIABLES_AND_CURRENT:
			{
				printf("收到指令3 \r\n" );
				*Sendlen = ResponseCommand3(RcvPackage,SndPackage);
				break;
			}
			case COMMAND6_WRITE_POLLING_ADDRESS:
			{
				printf("收到指令6 \r\n" );
				*Sendlen = ResponseCommand6(RcvPackage,SndPackage);
				break;
			}
			default:
				printf("不支持的指令： %d\r\n" ,HartCommand);
				break;				
		}
	}
	if(FlagofLongFrame == 0) /* 如果接收包为短地址 */
	{	
		 *Sendlen = ConvertToShortAddressResponsePacket(SndPackage, *Sendlen,HartCommand); /* 将长地址发送包转换成短地址发送包 */
	}
	*Sendlen = CalculateBCC(SndPackage,*Sendlen); /* 计算并赋值BCC校验位 */
	*Sendlen = AddPreamble(SndPackage,*Sendlen,NUMBER_OF_PREAMBLES); /* 发送包添加先导符 */
	return 0;	
}
/**
 * @brief hart接收解包、校验，读取指令代码。
 *
 * HartCommand - HART指令代码地址。
 * 1 - 定界符异常。
 * 2 - BCC校验错误。
 * 3 - 地址错误。
 *
 * @param RcvPackage 已经完成接收的 HART 请求包。
 * @param HartCommand 当前 HART 命令号。
 * @param pFlagofLongFrame 用于返回 HART 请求是否采用长帧地址格式。
 * @return 0 表示 HART 帧地址和 BCC 均有效；1 表示定界符异常，2 表示 BCC 错误，3 表示短地址或长地址不匹配。
 */
static int AnalyseRcvPackage(RCV_TYPE *RcvPackage,u8* HartCommand,u8* pFlagofLongFrame)
{
	int  i;						/* 循环计数 */
	int  datalen;				/* 从定界符到数据的字节数 */
	u8 chk;						/* 计算出的BCC校验值 */
	char chkdata;				/* 接收到的BCC校验数据 */
	switch(RcvPackage->data[0])	/* 根据定界符判断是长帧还是短帧 */
	{
		case IS_SHORT_FRAME:	/* 短帧 */
		{
			*pFlagofLongFrame = 0X0;						/* 短帧标志位置0 */
			datalen = RcvPackage->ShortFrame.BytesCount+4;	/* 定界符+地址+命令字节+数据总长度+数据 */
			chkdata = RcvPackage->data[datalen];			/* 把接收到的BCC校验值赋值给chkdata */
			*HartCommand = RcvPackage->data[2];		 		/* 读取命令码 */
			printf("从机地址： %d\r\n" ,HartParameters.PollingAddress);
			if((RcvPackage->data[1]&0X3F)!= HartParameters.PollingAddress) /* 如果短地址不为轮询地址 */
			{	
				return 3;
			}
			break;
		}
		case IS_LONG_FRAME:		/* 长帧 */
		{
			*pFlagofLongFrame = 0X1;						/* 长帧标志位置1 */
			datalen = RcvPackage->LongFrame.BytesCount+8;	/* 定界符+地址(5)+命令字节+数据总长度+数据 */
			chkdata = RcvPackage->data[datalen];			/* 把接收到的BCC校验值赋值给chkdata */
			*HartCommand = RcvPackage->data[6];		    	/* 读取命令码 */
			if(((RcvPackage->data[1]&0X3F) != MANUFACTURER_ID)
			   |(RcvPackage->data[2] != DEVICE_TYPE)
			   |(RcvPackage->data[3] != DEVICE_ID_1)
			   |(RcvPackage->data[4] != DEVICE_ID_2)
			   |(RcvPackage->data[5] != DEVICE_ID_3))		/* 如果长地址错误 */
			{	
				return 3;
			}
			break;
		}
		default:
			return 1;				/* 定界符异常 */
	}
	chk = RcvPackage->data[0];
	for(i=1;i<datalen;i++)			/* 开始计算BCC校验值 */
	{
		chk ^= RcvPackage->data[i];
	}
	if(chk == chkdata)				/* 校验值正常返回0 */
	{
		return 0;
	}
	else							/* 校验值异常返回2 */
	{
		return 2;
	}
}
/**
 * @brief 设置长指令响应时发送数据包的定界符，地址，命令，通信状态，设备状态。
 *
 * HartCommand - HART指令代码。
 *
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @param HartCommand 当前 HART 命令号。
 */
static void InitializeSndPackage(SND_TYPE *sendframe,u8 HartCommand)
{
	sendframe->Command1.Delimiter = 0X86;				/* 长地址定界符 */
	sendframe->Command1.Address[0] = MANUFACTURER_ID;	/* 长地址 */
	sendframe->Command1.Address[1] = DEVICE_TYPE;
	sendframe->Command1.Address[2] = DEVICE_ID_1;
	sendframe->Command1.Address[3] = DEVICE_ID_2;
	sendframe->Command1.Address[4] = DEVICE_ID_3;
	sendframe->Command1.Command=HartCommand;			/* 命令 */
	sendframe->Command1.CommunicationStatus = HartParameters.CommunicationStatus;	/* 通信状态 */
	sendframe->Command1.DeviceStatus = HartParameters.DeviceStatus;					/* 设备状态 */
}
/**
 * @brief 计算发送包的BCC校验位。
 *
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @param datalen 协议数据区的有效长度，单位字节。
 * @return 返回 datalen + 1，即在 sendframe->data[datalen] 写入异或 BCC 后的总数据长度；返回值不是 BCC 校验字节本身。
 */
static u8 CalculateBCC(SND_TYPE *sendframe,u8 datalen)
{
	int  i;
	u8 chk;							/* 计算出的BCC校验值 */
	chk = sendframe->data[0];
	for(i=1;i<datalen;i++)			/* 开始计算BCC校验值 */
	{
		chk ^= sendframe->data[i];
	}
	sendframe->data[datalen]= chk;	/* 赋值 */
	return datalen+1;				/* 返回数据长度 */
}
/**
 * @brief 把FLOAT变量小端存储转化成大端存储。
 *
 * @param Variable 待按 HART 线格式编码的单精度浮点变量。
 * @return 返回输入 Float32 原始位模式完成字节反序后的 32 位线格式值。
 */
static u32 SetFloatData(float Variable)
{
	u32* data=(u32*)(&Variable);
	*data = ((*data & 0xff000000) >> 24)
		  | ((*data & 0x00ff0000) >>  8)
		  | ((*data & 0x0000ff00) <<  8)
		  | ((*data & 0x000000ff) << 24);
	return *data;
}
/**
 * @brief 响应指令0：读唯一标识。
 *
 * @param revframe 已经解包的 HART 请求帧对象。命令处理器只读命令号、数据长度和请求数据，不修改接收帧。
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @return 返回 HART 命令 0 响应数据区的有效长度，单位字节。
 */
static int ResponseCommand0(RCV_TYPE *revframe,SND_TYPE *sendframe)
{
	int datalen;
	
	InitializeSndPackage(sendframe,0);				/* 初始化定界符，地址，命令，通信状态，设备状态 */
    sendframe->Command0.BytesCount=21u;				/* 数据字节数 */
	sendframe->Command0.Date[0]=254u;				/* 统一固定值 */
	sendframe->Command0.Date[1]=MANUFACTURER_ID;	/* 制造商ID，HCF登记 */
	sendframe->Command0.Date[2]=DEVICE_TYPE;		/* 制造商设备类型 */
	sendframe->Command0.Date[3]=5;					/* 主设备到从设备的最少同步前导码数量 */
	sendframe->Command0.Date[4]=0X07;				/* 通用命令版本号，版本7 */
	sendframe->Command0.Date[5]=0x01;				/* 设备软件版本（254和255保留） */
	sendframe->Command0.Date[6]=0x01;				/* 设备版本水平 */
	sendframe->Command0.Date[7]=0x01;				/* 高5位表示硬件版本号（31保留）低3位表示物理信号为Bell202 电流信号 */
	sendframe->Command0.Date[8]=0x08;				/* 保留 */
	sendframe->Command0.Date[9]=DEVICE_ID_1;		/* 设备ID ，同种类型设备的序列号，3个字节 */
	sendframe->Command0.Date[10]=DEVICE_ID_2;
	sendframe->Command0.Date[11]=DEVICE_ID_3;		
	sendframe->Command0.Date[12]=5u;				/* 从设备到主设备的最少同步前导码数量 */
	sendframe->Command0.Date[13]=0x04;				/* 最大设备变量数，主设备希望能读取的设备变量的个数。 */
	sendframe->Command0.Date[14]=0u;				/* 配置改变记数器，2个字节 */
	sendframe->Command0.Date[15]=0u;
	sendframe->Command0.Date[16]=0u;				/* 扩展设备状态：0 设备正常；0x01 设备没有故障但需要维护；0x02 设备变量报警状态 */
	sendframe->Command0.Date[17]=0;					/* 制造商ID，由HCF分配，2个字节 */
	sendframe->Command0.Date[18]=0;
	datalen = sendframe->Command0.BytesCount+8;		/* 定界符+地址+命令字节+数据总长度+数据 */
	return datalen;									/* 返回发送包长度 */
}
/**
 * @brief 响应指令1：读主变量。
 *
 * @param revframe 已经解包的 HART 请求帧对象。命令处理器只读命令号、数据长度和请求数据，不修改接收帧。
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @return 返回 HART 命令 1 响应数据区的有效长度，单位字节。
 */
static int ResponseCommand1(RCV_TYPE *revframe,SND_TYPE *sendframe)
{
	int datalen;
	HartParameters.PrimaryVariable = Hart_GetPrimaryVariable();
	
	InitializeSndPackage(sendframe,1);													/* 初始化定界符，地址，命令，通信状态，设备状态 */
	sendframe->Command1.BytesCount = 7;													/* 数据字节数 */
	sendframe->Command1.PrimaryVariableUnitsCode = MILLIMETERS;							/* 主变量单位：mm */
	sendframe->Command1.PrimaryVariable = SetFloatData(HartParameters.PrimaryVariable);
	datalen = sendframe->Command1.BytesCount+8;											/* 定界符+地址+命令字节+数据总长度+数据 */
	return datalen;	
}
/**
 * @brief 响应指令2：读环路电流和量程百分比。
 *
 * @param revframe 已经解包的 HART 请求帧对象。命令处理器只读命令号、数据长度和请求数据，不修改接收帧。
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @return 返回 HART 命令 2 响应数据区的有效长度，单位字节。
 */
static int ResponseCommand2(RCV_TYPE *revframe,SND_TYPE *sendframe)
{
	int datalen;
	HartParameters.Current = AoOutput_GetCurrent_mA();
	
	InitializeSndPackage(sendframe,2);									/* 初始化定界符，地址，命令，通信状态，设备状态 */
	sendframe->Command2.BytesCount = 10;								/* 数据字节数 */
	sendframe->Command2.Current = SetFloatData(HartParameters.Current);
	sendframe->Command2.PrimaryVariablePercentofRange = SetFloatData(AoOutput_GetPercentOfRange());
	datalen = sendframe->Command2.BytesCount+8;	
	return datalen;
}
/**
 * @brief 响应指令3：读动态变量和环路电流。
 *
 * @param revframe 已经解包的 HART 请求帧对象。命令处理器只读命令号、数据长度和请求数据，不修改接收帧。
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @return 返回 HART 命令 3 响应数据区的有效长度，单位字节。
 */
static int ResponseCommand3(RCV_TYPE *revframe,SND_TYPE *sendframe)
{
	int datalen;
	HartParameters.Current = AoOutput_GetCurrent_mA();
	HartParameters.PrimaryVariable = Hart_GetPrimaryVariable();
    HartParameters.SecondaryVariable = Hart_GetSecondaryVariable();
    HartParameters.TertiaryVariable = Hart_GetTertiaryVariable();
    HartParameters.FourthVariable = 0.0f;
	
	InitializeSndPackage(sendframe,3);														/* 初始化定界符，地址，命令，通信状态，设备状态 */
	sendframe->Command3.BytesCount = 26;													/* 数据字节数 */
	sendframe->Command3.Current = SetFloatData(HartParameters.Current);						/* 当前电流值 */
	sendframe->Command3.PrimaryVariableUnitsCode = MILLIMETERS;								/* 主变量单位：mm */
	sendframe->Command3.PrimaryVariable = SetFloatData(HartParameters.PrimaryVariable);		/* 主变量：液位 */
	sendframe->Command3.SecondaryVariableUnitsCode = DEGREES_CELSIUS;						/* 第二变量单位：℃ */
	sendframe->Command3.SecondaryVariable = SetFloatData(HartParameters.SecondaryVariable);	/* 第二变量：温度 */
	sendframe->Command3.TertiaryVariableUnitsCode = KG_CUM;									/* 第三变量单位：℃ */
	sendframe->Command3.TertiaryVariable = SetFloatData(HartParameters.TertiaryVariable);	/* 第三变量：密度 */
	sendframe->Command3.FourthVariableUnitsCode = MEGAPASCALS;								/* 第四变量单位：MPa */
	sendframe->Command3.FourthVariable = SetFloatData(HartParameters.FourthVariable);		/* 第四变量：压力 */
	datalen = sendframe->Command3.BytesCount+8;												/* 定界符+地址(5)+命令字节+数据总长度+数据 */
	return datalen;
}
/**
 * @brief 响应指令6：设置轮询地址。
 *
 * @param revframe 已经解包的 HART 请求帧对象。命令处理器只读命令号、数据长度和请求数据，不修改接收帧。
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @return 返回 HART 命令 6 响应数据区的有效长度，单位字节。
 */
static int ResponseCommand6(RCV_TYPE *revframe,SND_TYPE *sendframe)
{
	int datalen;
	HartParameters.PollingAddress = revframe->LongFrame.Date[0]; /* 读取命令中的轮询地址 */
	
	InitializeSndPackage(sendframe,6);
	sendframe->Command6.BytesCount = 4; /* 数据字节数 */
	sendframe->Command6.PollingAddress = HartParameters.PollingAddress;
	sendframe->Command6.EnableLoopCurrent = 3 ; /* 电流环使能设置 */
	datalen = sendframe->Command6.BytesCount+8;	/* 定界符+地址(5)+命令字节+数据总长度+数据 */
	return datalen;
}

/**
 * @brief 把长地址响应包转换成短地址响应包。
 *
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @param datalen 协议数据区的有效长度，单位字节。
 * @param HartCommand 当前 HART 命令号。
 * @return 返回按当前映射得到的长地址响应包转换成短地址响应包；非法输入使用 @brief 说明的兜底地址或无效值。
 */
static u8 ConvertToShortAddressResponsePacket(SND_TYPE *sendframe,u8 datalen,u8 HartCommand)
{
	int i = datalen;
	sendframe->data[0] = 0X06;			/* 短地址定界符 */
	sendframe->data[1] = HartParameters.PollingAddress; /* 短包时地址为轮询地址 */
	for(i=2;i<datalen-4;i++)			
	{
		sendframe->data[i]=sendframe->data[i+4];
	}
	datalen -= 4;
	return datalen;
}

/**
 * @brief 在发送包数据前添加先导符0XFF。
 *
 * @param sendframe 待填充或发送的 HART 响应帧对象。命令处理器在其中写入状态、响应数据和数据长度，最终 BCC 由统一封装流程追加。
 * @param datalen 协议数据区的有效长度，单位字节。
 * @param NumberOfPreambles HART 响应前导字节 0xFF 的数量。
 * @return 返回添加指定数量 0xFF 前导字节后的 HART 发送包总长度，单位字节。
 */
static u8 AddPreamble(SND_TYPE *sendframe,u8 datalen,u8 NumberOfPreambles)
{
	int i = datalen;
	for(i=datalen;i>=0;i--)			
	{
		sendframe->data[i+NumberOfPreambles]=sendframe->data[i];
	}
	for(i=NumberOfPreambles-1;i>=0;i--)			
	{
		sendframe->data[i]=0XFF;
	}
	datalen = datalen+NumberOfPreambles;
	return datalen;
}
