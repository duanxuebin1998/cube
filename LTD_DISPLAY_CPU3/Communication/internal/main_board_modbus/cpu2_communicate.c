#include "cpu2_communicate.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include "my_crc.h"
#include "system_parameter.h"
#include "dataanalysis_modbus.h"
#include "wartsila_modbus_communication.h"

#define DEBUG_COMMUCPU2 1
#define ADERSS 0X01

volatile bool wait_response = false; //主控板响应标志位

/*保持寄存器*/
uint16_t HoldingRegisterArray[HOLEREGISTER_STOP] = { 0 }; //保持寄存器数组
/*输入寄存器*/
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
    printf("RCV from CPU2 %d : ",len);
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
#if DEBUG_COMMUCPU2
            printf("CPU3 Response04Process\r\n");
//            print_device_params();
#endif
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
/* 输入寄存器组 1：设备状态 */
{FUNCTIONCODE_READ_INPUTREGISTER, REG_DEVICE_STATUS_WORK_MODE, (uint16_t) (REG_SINGLE_POINT_MEAS_TEMP - REG_DEVICE_STATUS_WORK_MODE) },

/* 输入寄存器组 2：单点 / 分布测量结果 */
{FUNCTIONCODE_READ_INPUTREGISTER, REG_SINGLE_POINT_MEAS_TEMP, (uint16_t) (REG_DENSITY_DIST_POINT_BASE - REG_SINGLE_POINT_MEAS_TEMP) },
/* 保持寄存器组 3：设备参数前半段 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, (uint16_t) (HOLDREGISTER_DEVICEPARAM_RESERVED11 - HOLDREGISTER_DEVICEPARAM_COMMAND) },

/* 保持寄存器组 4：设备参数后半段 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RESERVED11, (uint16_t) (HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE - HOLDREGISTER_DEVICEPARAM_RESERVED11) }
};

#define POWERON_GROUP_COUNT  (sizeof(poweron_groups) / sizeof(poweron_groups[0]))

/* 正常运行时轮询的组：
 * 只保留输入寄存器，不再读保持寄存器
 */
static const PollGroup runtime_groups[] = {
/* 输入寄存器组 1：设备状态 */
{
FUNCTIONCODE_READ_INPUTREGISTER, REG_DEVICE_STATUS_WORK_MODE, (uint16_t) (REG_SINGLE_POINT_MEAS_TEMP - REG_DEVICE_STATUS_WORK_MODE) },

/* 输入寄存器组 2：单点 / 密度测量结果 */
{
FUNCTIONCODE_READ_INPUTREGISTER, REG_SINGLE_POINT_MEAS_TEMP, (uint16_t) (REG_DENSITY_DIST_POINT_BASE - REG_SINGLE_POINT_MEAS_TEMP) } ,
/* 保持寄存器组 3：设备参数前半段 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, (uint16_t) (HOLDREGISTER_DEVICEPARAM_RESERVED11 - HOLDREGISTER_DEVICEPARAM_COMMAND) },

/* 保持寄存器组 4：设备参数后半段 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RESERVED11, (uint16_t) (HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE - HOLDREGISTER_DEVICEPARAM_RESERVED11) }
};

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
				 DeviceParams_StoreToRegisters(g_holding_regs);/* 把读取到的设备参数存入瓦锡兰保持寄存器 */
//				print_device_params();
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
    // 特定设备状态下，优先按测点数读取分布测量点
    if( (g_measurement.device_status.device_state == STATE_WARTSILA_DENSITY_OVER)
    || (g_measurement.device_status.device_state == STATE_SPREADPOINTOVER)
	  || (g_measurement.device_status.device_state == STATE_GB_SPREADPOINTOVER)
	  || (g_measurement.device_status.device_state == STATE_SYNTHETICING_OVER)
	  || (g_measurement.device_status.device_state == STATE_COM_METER_DENSITY_OVER)
	  || (g_measurement.device_status.device_state == STATE_INTERVAL_DENSITY_OVER)) {
        // 根据 REG_DENSITY_DIST_MEAS_POINTS 的值拉点数据
        RequestDensityDistPoints_ByCount();
        // 你可以在读完后置一个标志，避免每次都重复读
        // g_measurement.flags.density_points_fetched = true;
        return;
    }
}
#define MAX_REGS_PER_READ  100   /* 单帧最多读多少寄存器，根据你从机限制调整 */

/**
 * @brief 根据测量点数量读取分布测量点的所有数据
 *        调用前需要确保 g_measurement.density_distribution.measurement_points 已经正确更新。
 */
void RequestDensityDistPoints_ByCount(void)
{
    uint16_t points = (uint16_t)g_measurement.density_distribution.measurement_points;

    if (points == 0) {
        printf("分布测量点数为 0，不读取点数据。\n");
        return;
    }
    if (points > MAX_MEASUREMENT_POINTS) {
        printf("分布测量点数异常：%u > MAX_MEASUREMENT_POINTS=%u，裁剪。\n",
               points, (unsigned)MAX_MEASUREMENT_POINTS);
        points = MAX_MEASUREMENT_POINTS;
    }

    uint16_t start = REG_DENSITY_DIST_POINT_BASE;
    uint32_t total_regs = (uint32_t)points * (uint32_t)REG_DENSITY_DIST_POINT_SIZE;

    printf("准备读取分布测量点数据：点数=%u，每点寄存器=%u，总寄存器=%lu，起始地址=%u\n",
           points,
           (unsigned)REG_DENSITY_DIST_POINT_SIZE,
           (unsigned long)total_regs,
           (unsigned)start);

    while (total_regs > 0) {
        uint16_t this_len;

        if (total_regs > MAX_REGS_PER_READ) {
            this_len = MAX_REGS_PER_READ;
        } else {
            this_len = (uint16_t)total_regs;
        }

        /* 这里用读输入寄存器功能码（假设你把这些点映射到 04 号） */
        CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,
                                   start,
                                   this_len,
                                   NULL);

        start      += this_len;
        total_regs -= this_len;
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
	printf("send to CPU2 %d bytes:", len);
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

