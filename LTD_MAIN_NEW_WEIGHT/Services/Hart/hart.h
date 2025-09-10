#ifndef HART_H
#define HART_H
#include "main.h"
//#include "usart3.h"
//#include "sd2421.h"
#define MAXREVEIVECNT 128

#define HART_RTS1	PAout(1) 						/*HART模式控制引脚，0 - 发送;1 - 接收*/
#define HART_RTS2	PCout(15) 						/*HART模式控制引脚，0 - 发送;1 - 接收*/
#define IS_SHORT_FRAME				0X02			/*短帧定界符内容*/
#define IS_LONG_FRAME				0X82 			/*长帧定界符内容*/
#define NUMBER_OF_PREAMBLES			0X06			/*HART发送包前导符的数量*/
/*HART 指令代码*/
#define	COMMAND0_READ_UNIQUE_IDENTIFIE				0/*读唯一标识*/
#define	COMMAND1_READ_PRIMARY_VARIABLE				1/*读主变量*/
#define	COMMAND2_READ_CURRENT						2/*读环路电流和量程百分比*/
#define	COMMAND3_READ_DYNAMIC_VARIABLES_AND_CURRENT	3/*读动态变量和环路电流*/
#define	COMMAND6_WRITE_POLLING_ADDRESS				6/*设置轮询地址*/

/*HART 工程单位代码 */
#define MILLIMETERS		49		/*mm*/
#define DEGREES_CELSIUS 32		/*℃*/
#define KG_CUM			92		/*KG/M3*/
#define MEGAPASCALS		237		/*Mpa*/

/*HART 设备长地址*/
/*目前是VEGA液位计地址，正常需要HCF认证*/
#define	MANUFACTURER_ID	0X55	/*制造商ID号*/
#define	DEVICE_TYPE		0X55	/*产品设备类型ID，由厂商定义，HCF 登记*/
#define	DEVICE_ID_1		0X55	/*设备ID，同种类型设备的序列号,每个设备都不一样*/
#define	DEVICE_ID_2 	0X55
#define	DEVICE_ID_3		0X55


typedef struct 
{
	u8 CommunicationStatus;		/*通信状态*/
	u8 DeviceStatus;			/*设备状态*/
	u8 PollingAddress;			/*轮询地址*/
	float Current;				/*电流值*/
	float MaxPrimaryVariable;	/*主变量值上限*/
	float MinPrimaryVariable;	/*主变量值下限*/
	float PrimaryVariable;		/*主变量：液位*/
	float SecondaryVariable;	/*第二变量：温度*/
	float TertiaryVariable;		/*第三变量：密度*/
	float FourthVariable;		/*第四变量：压力*/
}HartParametersTPYE;			/*HART通信中的参数结构体类型*/
#pragma pack(push)
#pragma pack(1)					/*设置编译器将结构体数据强制连续排列*/

/*定义接收数据包的结构体和联合体*/
typedef struct 
{
	u8 Delimiter;				/*定界符*/
	u8 Address;					/*短地址*/
	u8 Command;					/*命令*/
	u8 BytesCount;				/*数据字节数*/
	u8 Date[];					/*设置从机的参数数据*/
}RevShortFrame;					/*声明短帧接收包结构体类型*/
typedef struct 
{
	u8 Delimiter;				/*定界符*/
	u8 Address[5];				/*长地址*/
	u8 Command;					/*命令*/
	u8 BytesCount;				/*数据字节数*/
	u8 Date[];					/*设置从机的参数数据*/
}RevLongFrame;					/*声明长帧接收包结构体类型*/
typedef union 
{
	RevShortFrame ShortFrame;	/*定义短帧接收包结构体*/
	RevLongFrame LongFrame; 	/*定义长帧接收包结构体*/
	u8 data[MAXREVEIVECNT];		/*定义接收包数组*/
} RCV_TYPE;						/*声明接收包共用体类型*/

/*定义发送数据包的结构体和联合体*/
/*由于HART协议中float型变量是大端存储，这里以u32类型定义*/
typedef struct 
{
	u8 Delimiter;				/*定界符*/
	u8 Address[5];				/*短地址*/
	u8 Command;					/*命令*/
	u8 BytesCount;				/*数据字节数*/
	u8 CommunicationStatus;		/*通信状态*/
	u8 DeviceStatus;			/*设备状态*/
	u8 Date[19];				/*响应数据*/
}_Command0;						/*声明Command0发送包结构体类型*/

typedef struct 
{
	u8 Delimiter;				/*定界符*/
	u8 Address[5];				/*长地址*/
	u8 Command;					/*命令*/
	u8 BytesCount;				/*数据字节数*/
	u8 CommunicationStatus;		/*通信状态*/
	u8 DeviceStatus;			/*设备状态*/
	u8 PrimaryVariableUnitsCode;/*主变量单位代码*/
	u32 PrimaryVariable;		/*主变量*/
}_Command1;						/*声明Command1发送包结构体类型*/
typedef struct 
{
	u8 Delimiter;				/*定界符*/
	u8 Address[5];				/*长地址*/
	u8 Command;					/*命令*/
	u8 BytesCount;				/*数据字节数*/
	u8 CommunicationStatus;		/*通信状态*/
	u8 DeviceStatus;			/*设备状态*/
	u32 Current;				/*主变量电流,单位毫安*/
    u32 PrimaryVariablePercentofRange;/*主变量占量程的百分比*/
}_Command2;						/*声明Command2发送包结构体类型*/
typedef struct 
{
	u8 Delimiter;				/*定界符*/
	u8 Address[5];				/*长地址*/
	u8 Command;					/*命令*/
	u8 BytesCount;				/*数据字节数*/
	u8 CommunicationStatus;		/*通信状态*/
	u8 DeviceStatus;			/*设备状态*/
	u32 Current;				/*主变量电流,单位毫安*/
	u8 PrimaryVariableUnitsCode;/*主变量单位代码*/
	u32 PrimaryVariable;		/*主变量：液位*/
	u8 SecondaryVariableUnitsCode;/*第二变量单位代码*/
	u32 SecondaryVariable;		/*第二变量：温度*/
	u8 TertiaryVariableUnitsCode;/*第三变量单位代码*/
	u32 TertiaryVariable;		/*第三变量：密度*/
	u8 FourthVariableUnitsCode;	/*第四变量位代码*/
	u32 FourthVariable;			/*第四变量：压力*/
}_Command3;						/*声明Command3发送包结构体类型*/
typedef struct 
{
	u8 Delimiter;				/*定界符*/
	u8 Address[5];				/*长地址*/
	u8 Command;					/*命令*/
	u8 BytesCount;				/*数据字节数*/
	u8 CommunicationStatus;		/*通信状态*/
	u8 DeviceStatus;			/*设备状态*/
	u8 PollingAddress;			/*轮询地址*/
	u8 EnableLoopCurrent;		/*0=环路电流禁止；1=环路电流允许*/
}_Command6;						/*声明Command6发送包结构体类型*/

typedef union 
{
	_Command0 Command0;			/*定义Command0发送包结构体*/
	_Command1 Command1;			/*定义Command1发送包结构体*/
	_Command2 Command2;			/*定义Command2发送包结构体*/
	_Command3 Command3;			/*定义Command3发送包结构体*/
	_Command6 Command6;			/*定义Command6发送包结构体*/
	u8 data[MAXREVEIVECNT];		/*定义发送包数组*/
} SND_TYPE;						/*声明发送包共用体类型*/
#pragma pack(pop)


void HartInit(void);																			/*Hart初始化*/
void HartParameterInit(void);																	/*Hart参数初始化*/
u8 HartCommunicationProcess(u8* rcvbuff,u8* SendBuff,u8* Sendlen);														/*HART通信上位机指令包处理函数,处理不同指令并发送响应包。*/
static int AnalyseRcvPackage(RCV_TYPE *RcvPackage,u8* HartCommand,u8* FlagofLongFrame);			/*hart接收解包、校验*/
static void InitializeSndPackage(SND_TYPE *sendframe,u8 HartCommand);							/*设置长指令响应时发送数据包的定界符，地址，命令，通信状态，设备状态*/
static u8 CalculateBCC(SND_TYPE *sendframe,u8 datalen);											/*计算发送包的BCC校验位*/
static u32 SetFloatData(float Variable);														/*把FLOAT变量小端存储转化成大端存储*/
static int ResponseCommand0(RCV_TYPE *revframe,SND_TYPE *sendframe);							/*响应指令0：读唯一标识*/
static int ResponseCommand1(RCV_TYPE *revframe,SND_TYPE *sendframe);							/*响应指令1：读主变量*/
static int ResponseCommand2(RCV_TYPE *revframe,SND_TYPE *sendframe);							/*响应指令2：读环路电流和量程百分比*/
static int ResponseCommand3(RCV_TYPE *revframe,SND_TYPE *sendframe);							/*响应指令3：读动态变量和环路电流*/
static int ResponseCommand6(RCV_TYPE *revframe,SND_TYPE *sendframe);							/*响应指令6：设置轮询地址*/
static u8 ConvertToShortAddressResponsePacket(SND_TYPE *sendframe,u8 datalen,u8 HartCommand);	/*把长地址响应包转换成短地址响应包*/
static u8 AddPreamble(SND_TYPE *sendframe,u8 datalen,u8 NumberOfPreambles);						/*在发送包数据前添加先导符0XFF*/
#endif

