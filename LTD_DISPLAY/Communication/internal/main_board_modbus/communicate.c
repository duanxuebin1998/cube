#include "main.h"
#include "spi.h"
#include "usart.h"
#include "communicate.h"
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include "my_crc.h"
#include "modbus_agreement.h"
#include "system_parameter.h"
#include "dataanalysis_modbus.h"

#define DEBUG_COMMUCPU2 0
#define ADERSS 0X01

volatile bool wait_response = false; //主控板响应标志位

/*保持寄存器*/
//static const int HoldingregisterAddress = 0x00; //保持寄存器起始地址
//static const int HoldingregisterAmount = HOLEREGISTER_STOP + 1; //保持寄存器总数
int HoldingRegisterArray[HOLEREGISTER_STOP] = { 0 }; //保持寄存器数组
/*输入寄存器*/
//static const int InputregisterAddress = 0x00; //输入寄存器起始地址
//static const int InputRegisterAmount = INPUTREGISTER_AMOUNT; //输入寄存器总数
static uint16_t InputRegisterArray[INPUTREGISTER_AMOUNT] = { 0 };    //输入寄存器数组

/*接收到的命令包数据暂存变量*/
static int RCV_functioncode = 0;
static int RCV_startaddress = 0;
static int RCV_registercnt = 0;
static int SlaveTempBuffer[INPUTREGISTER_AMOUNT];

//static void CPU2_Response03Process(char const *revframe);
static void CPU2_Response03Process(uint8_t const *revframe);
static void CPU2_Response04Process(char const *revframe);
static void CPU2_Response10Process(uint8_t *arr, uint16_t len);
static void PresetRegister(bool registertype, int const *registervalue);

/*与CPU2通讯初始化*/
void CommuToCPU2Init(void) {
	PollingInputData();    //初始化后先轮询一遍测量数据
}
/*与CPU2通讯接收包主处理过程*/
void HostCommuProcess(uint8_t *rcv, int len) {
#if DEBUG_COMMUCPU2
    int i;
    printf("CPU3com_RCV %d : ",len);
    for(i = 0;i < len;i++)
        printf("%02X ",rcv[i]);
    printf("\r\n");
    #endif
	if (len <= 3)
		return;
	if (!SlaveCheckCRC(rcv, len)) {
		printf("CPU3 CRC ERROR");
		return;
	}
	/* 解析数据 */
	if (rcv[0] != ADERSS) {
		printf("CPU3 Address Error");
		return;
	}
	RCV_functioncode = rcv[1];
	cnt_commutoCPU2 = 0;
	switch (RCV_functioncode) {
	case FUNCTIONCODE_READ_HOLDREGISTER: {
		CPU2_Response03Process(rcv);
#if DEBUG_COMMUCPU2
            printf("CPU3 Response03Process\r\n");
//            print_device_params();
#endif
		break;
	}
	case FUNCTIONCODE_READ_INPUTREGISTER: {
		CPU2_Response04Process(rcv);
		break;
	}
	case FUNCTIONCODE_WRITE_MULREGISTER: {
		CPU2_Response10Process(rcv, len);
#if DEBUG_COMMUCPU2
//            print_device_params();
#endif
		break;
	}
	}
}
///* 轮询输入寄存器数据 */
//void PollingInputData(void)
//{
//    static bool flag_poweronread = false;
//    CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_DEVICEPARAM_COMMAND,HOLEREGISTER_STOP-HOLDREGISTER_DEVICEPARAM_COMMAND,NULL);
////    print_device_params();
//    HAL_Delay(100);
//    CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,REG_DEVICE_STATUS_WORK_MODE,REG_SINGLE_POINT_MEAS_TEMP-REG_DEVICE_STATUS_WORK_MODE,NULL);
//    HAL_Delay(100);
//    CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,REG_SINGLE_POINT_MEAS_TEMP,REG_DENSITY_DIST_POINT_BASE-REG_SINGLE_POINT_MEAS_TEMP,NULL);
//
////    CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,HOLDREGISTER_DEVICEPARAM_COMMAND,HOLEREGISTER_STOP,NULL);
//
//
////    if(cnt_commutoCPU2 >= COMMU_ERROR_MAX)
////        setEquipStateError();
////    else
////    {
//        if(flag_poweronread == false)//第一次上电后读取一次
//        {
//
//            flag_poweronread = true;
//        }
////    }
//}
typedef struct {
	uint8_t func;     // 功能码：3 或 4
	uint16_t start;   // 起始寄存器
	uint16_t len;     // 寄存器个数
} PollGroup;
#include <stdbool.h>

/* 上电阶段要读取的组：
 * - 包含设备参数（保持寄存器）
 * - 也可以顺便把输入寄存器读一遍
 */
static const PollGroup poweron_groups[] = {
/* 保持寄存器组 1：设备参数前半段 */
{
FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, (uint16_t) (HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION - HOLDREGISTER_DEVICEPARAM_COMMAND) },

/* 保持寄存器组 2：设备参数后半段 */
{
FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION, (uint16_t) (HOLEREGISTER_STOP - HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION) },

/* 输入寄存器组 3：设备状态 */
{
FUNCTIONCODE_READ_INPUTREGISTER, REG_DEVICE_STATUS_WORK_MODE, (uint16_t) (REG_SINGLE_POINT_MEAS_TEMP - REG_DEVICE_STATUS_WORK_MODE) },

/* 输入寄存器组 4：单点 / 分布测量结果 */
{
FUNCTIONCODE_READ_INPUTREGISTER, REG_SINGLE_POINT_MEAS_TEMP, (uint16_t) (REG_DENSITY_DIST_AVG_TEMP - REG_SINGLE_POINT_MEAS_TEMP) } };

#define POWERON_GROUP_COUNT  (sizeof(poweron_groups) / sizeof(poweron_groups[0]))

/* 正常运行时轮询的组：
 * 只保留输入寄存器，不再读保持寄存器
 */
static const PollGroup runtime_groups[] = {
/* 输入寄存器组 1：设备状态 */
{
FUNCTIONCODE_READ_INPUTREGISTER, REG_DEVICE_STATUS_WORK_MODE, (uint16_t) (REG_SINGLE_POINT_MEAS_TEMP - REG_DEVICE_STATUS_WORK_MODE) },

/* 输入寄存器组 2：单点 / 分布测量结果 */
{
FUNCTIONCODE_READ_INPUTREGISTER, REG_SINGLE_POINT_MEAS_TEMP, (uint16_t) (REG_DENSITY_DIST_AVG_TEMP - REG_SINGLE_POINT_MEAS_TEMP) } };

#define RUNTIME_GROUP_COUNT  (sizeof(runtime_groups) / sizeof(runtime_groups[0]))

/* 轮询输入寄存器数据 */
void PollingInputData(void) {
	/* 上电阶段是否已经完成 */
	static bool poweron_done = false;
	static uint8_t poweron_index = 0;

	/* 正常轮询阶段当前组索引 */
	static uint8_t runtime_index = 0;

	/* ---------- 上电阶段：每次调用发一个 poweron_groups ---------- */
	if (!poweron_done) {
		if (poweron_index < POWERON_GROUP_COUNT) {
			const PollGroup *g = &poweron_groups[poweron_index];

			CPU2_CombinatePackage_Send(g->func, g->start, g->len, NULL);

			poweron_index++;

			if (poweron_index >= POWERON_GROUP_COUNT) {
				poweron_done = true; /* 上电读取全部完成 */
			}
		}
		return; /* 上电阶段结束本次调用，不再发 runtime 组 */
	}

	/* ---------- 正常运行阶段：只轮询输入寄存器 ---------- */

	/* 如果某些状态不需要轮询，可以在这里加条件，例如：
	 * if (g_measurement.device_status.device_state == STATE_AI_SPREADPOINTOVER)
	 *     return;
	 */

	{
		const PollGroup *g = &runtime_groups[runtime_index];

		CPU2_CombinatePackage_Send(g->func, g->start, g->len, NULL);

		runtime_index++;
		if (runtime_index >= RUNTIME_GROUP_COUNT) {
			runtime_index = 0;
		}
	}
}

/*由屏幕向CPU2发送指令包*/
void CPU2_CombinatePackage_Send(uint8_t f_code, uint16_t startadd, uint16_t registercnt, uint32_t *holddata) {
	uint8_t arr[1024];
	int len = 0;
	uint16_t crc;
	int i;
	const uint16_t *regs = (const uint16_t*) holddata;   // 关键修正：按16位寄存器解释
	arr[len++] = ADERSS;
	arr[len++] = f_code;
	arr[len++] = startadd >> 8;
	arr[len++] = startadd & 0xFF;
	arr[len++] = registercnt >> 8;
	arr[len++] = registercnt & 0xFF;
	if (f_code == FUNCTIONCODE_WRITE_MULREGISTER && holddata != NULL) {
		arr[len++] = registercnt * 2;
		/* === Word Swap: 交换寄存器顺序 === */
		for (i = 0; i < registercnt; i += 2) {
			uint16_t low_word = regs[i + 1]; // 原本的高字
			uint16_t high_word = regs[i];     // 原本的低字

			// 低字先发（高字节→低字节）
			arr[len++] = (uint8_t) (low_word >> 8);
			arr[len++] = (uint8_t) (low_word & 0xFF);

			// 高字后发（高字节→低字节）
			arr[len++] = (uint8_t) (high_word >> 8);
			arr[len++] = (uint8_t) (high_word & 0xFF);
		}
	}
	crc = CRC16_Calculate((u8*) arr, len);
	arr[len++] = crc & 0xff;
	arr[len++] = crc >> 8;
#if DEBUG_COMMUCPU2
	printf("CPU2 send %d bytes:", len);
#endif
	sendToCPU2(arr, len, false);
	//全局变量赋值，用于接收CPU2的响应包处理
	RCV_functioncode = f_code;
	RCV_startaddress = startadd;
	RCV_registercnt = registercnt;
	// 等待接收完成
	uint32_t timeout = HAL_GetTick();
	while (wait_response) {
		if (HAL_GetTick() - timeout > 1000) // 100ms超时
				{
			printf("Wait response timeout!\n");
			wait_response = false;    // 防止一直 True
			return;
		}
	}
	HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN);  // 处理接收到的数据
}
/*向CPU2发送数据包*/
void sendToCPU2(uint8_t *arr, uint16_t len, bool flag_fromhost) {
	RS485_SET_SEND_MODE();  // 切换到发送
	HAL_UART_Transmit_DMA(&huart5, arr, len);
	wait_response = true; // 设置等待响应标志位
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
///*解析CPU2的响应包0x03功能码*/
//static void CPU2_Response03Process(char const *revframe)
//{
//	int i, j;
//	int byteamount;
//	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));
//		for (i = 0, j = 0; i < RCV_registercnt; i++, j = j + 2) {
//			SlaveTempBuffer[i] = (revframe[j + 3] << 8) + revframe[j + 4];
//		}
//		/*写保持寄存器*/
//		WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
//		PresetRegister(false, SlaveTempBuffer);
//		/*更新对应参数,若需存储则写入铁电*/
//	    byteamount = revframe[2];
//	    if(byteamount != RCV_registercnt * 2)
//	        return;
//	    for(i = 0;i < byteamount;i++)
//	    	HoldingRegisterArray[(RCV_startaddress << 1) + i] = revframe[3 + i];
//		AnalysisHoldRegister();
//		ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
//}
/* 解析CPU2的响应包 0x03 功能码 */
static void CPU2_Response03Process(uint8_t const *revframe) {
	int i;
	int byteamount;

	// Modbus RTU: revframe[0]=地址, revframe[1]=功能码(0x03), revframe[2]=字节数
	byteamount = revframe[2];

	// 简单防御：返回的字节数必须是寄存器数 * 2
	if (byteamount != RCV_registercnt * 2) {
		return;
	}

	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));

	// 1. 把数据区解析成寄存器值，填到 SlaveTempBuffer
	for (i = 0; i < RCV_registercnt; i++) {
		uint16_t reg = ((uint16_t) revframe[3 + i * 2] << 8) | (uint16_t) revframe[3 + i * 2 + 1];
		SlaveTempBuffer[i] = reg;
	}

	// 2. 写保持寄存器（根据项目逻辑，这里我不动你的调用顺序）
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	PresetRegister(false, SlaveTempBuffer);

	// 3. 更新 HoldingRegisterArray 里对应的寄存器（注意，这里是按“寄存器”写）
	for (i = 0; i < RCV_registercnt; i++) {
		HoldingRegisterArray[RCV_startaddress + i] = SlaveTempBuffer[i];
		// 如果不需要 SlaveTempBuffer，也可以直接用 reg 写：
		// HoldingRegisterArray[RCV_startaddress + i] =
		//     ((uint16_t)revframe[3 + i * 2] << 8) | (uint16_t)revframe[3 + i * 2 + 1];
	}

	// 4. 解析保持寄存器并刷新 g_deviceParams
	AnalysisHoldRegister();
	ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
}

/*解析CPU2的响应包0x04功能码*/
static void CPU2_Response04Process(char const *revframe) {
	int i, j;
	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));
	for (i = 0, j = 0; i < RCV_registercnt; i++, j = j + 2) {
		SlaveTempBuffer[i] = (revframe[j + 3] << 8) + revframe[j + 4];
	}
//	printf("CPU2_Response04Process: RCV_registercnt = %d\r\n", RCV_registercnt);
	/*写保持寄存器*/
	PresetRegister(true, SlaveTempBuffer);
	read_measurement_result_from_InputRegisters(InputRegisterArray);
}

static void CPU2_Response10Process(uint8_t *arr, uint16_t len) {

}

/*
 写寄存器
 registertype --> false - 保持寄存器
 --> true - 输入寄存器
 */
static void PresetRegister(bool registertype, int const *registervalue) {
	int range;
	int i;
	int j;

	range = RCV_startaddress + RCV_registercnt;
	if (registertype) {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			InputRegisterArray[i] = registervalue[j];
//			printf("InputRegisterArray[%d] = %d\r\n", i, InputRegisterArray[i]);
		}
	} else {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			HoldingRegisterArray[i] = registervalue[j];
//			printf("HoldingRegisterArray[%d] = %d\r\n", i, HoldingRegisterArray[i]);
		}
	}
}

