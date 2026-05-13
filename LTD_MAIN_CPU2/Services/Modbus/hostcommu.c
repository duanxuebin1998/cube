#include "hostcommu.h"
#include "hostcommu_modbus.h"
#include "usart.h"
#include <string.h>
#include "stateformodbus.h"
#include "crc.h"
#include <ctype.h>
#include <stdbool.h>
#include "my_crc.h"
#include "stdio.h"
#include "error_log.h"

#define DEBUG_HOSTCOMMU 0
// 常量定义
#define MAXRCVLENGTH 256 // Modbus帧最大接收长度（RTU模式一般为256字节）
#define HOSTCOMMU_ERROR_LOG_INTERVAL_MS 1000U

typedef enum {
	HOSTCOMMU_LOG_LENGTH = 0,
	HOSTCOMMU_LOG_ADDRESS,
	HOSTCOMMU_LOG_CRC,
	HOSTCOMMU_LOG_FUNCTION,
	HOSTCOMMU_LOG_REGISTER_RANGE,
	HOSTCOMMU_LOG_COUNT
} HostCommuLogType;

typedef struct {
	uint8_t pending;
	uint32_t last_tick;
	uint32_t suppressed;
	uint32_t pending_suppressed;
	int rcvcount;
	unsigned int addr;
	int functioncode;
	int startaddress;
	int registeramount;
	const char *reason;
} HostCommuDeferredLog;

// 发送缓冲区（静态分配）
static uint8_t  HCOM_SendBuff[HOSTCOMMU_SENDLENGTH]; // Modbus响应帧缓冲区
static int HCOM_SendCount = 0;                   // 发送缓冲区当前数据长度
static HostCommuDeferredLog s_hostcommu_deferred_logs[HOSTCOMMU_LOG_COUNT] = {0};

/**
 * @brief 记录主机通信异常日志请求。
 * @note 该函数可能在 UART5 中断上下文被调用，只保存必要字段，不做 printf。
 */
static void HostCommu_RecordDeferredLog(HostCommuLogType type,
                                        const char *reason,
                                        int rcvcount,
                                        unsigned int addr,
                                        int functioncode,
                                        int startaddress,
                                        int registeramount)
{
	uint32_t now;
	uint32_t elapsed;
	HostCommuDeferredLog *log;

	if (type >= HOSTCOMMU_LOG_COUNT) {
		return;
	}

	log = &s_hostcommu_deferred_logs[type];
	now = HAL_GetTick();
	elapsed = now - log->last_tick;
	/* 已有待输出日志或 1 秒限频窗口内的同类异常，只累计抑制次数。 */
	if ((log->pending != 0U) ||
		((log->last_tick != 0U) &&
		 (elapsed < HOSTCOMMU_ERROR_LOG_INTERVAL_MS))) {
		log->suppressed++;
		return;
	}

	log->last_tick = now;
	log->pending_suppressed = log->suppressed;
	log->suppressed = 0U;
	log->rcvcount = rcvcount;
	log->addr = addr;
	log->functioncode = functioncode;
	log->startaddress = startaddress;
	log->registeramount = registeramount;
	log->reason = reason;
	log->pending = 1U;
}

/**
 * @brief 在主循环中输出主机通信异常日志。
  * @note 统一故障日志会走 printf，因此必须延后到主循环，避免拉长串口中断时间。
 */
void HostCommu_ProcessDeferredLogs(void)
{
	HostCommuDeferredLog log;
	char detail[128];
	uint32_t primask;

	for (HostCommuLogType type = HOSTCOMMU_LOG_LENGTH;
	     type < HOSTCOMMU_LOG_COUNT;
	     type = (HostCommuLogType)(type + 1)) {
		/* 复制 pending 数据时短暂关中断，避免 UART5 中断同时改写缓存。 */
		primask = __get_PRIMASK();
		__disable_irq();
		if (s_hostcommu_deferred_logs[type].pending == 0U) {
			if (primask == 0U) {
				__enable_irq();
			}
			continue;
		}
		log = s_hostcommu_deferred_logs[type];
		s_hostcommu_deferred_logs[type].pending = 0U;
		if (primask == 0U) {
			__enable_irq();
		}

		switch (type) {
		case HOSTCOMMU_LOG_LENGTH:
			snprintf(detail, sizeof(detail),
			         "长度：%d,最大=%d",
			         log.rcvcount,
			         MAXRCVLENGTH);
			break;
		case HOSTCOMMU_LOG_ADDRESS:
		case HOSTCOMMU_LOG_CRC:
			snprintf(detail, sizeof(detail),
			         "地址：%u,长度：%d",
			         log.addr,
			         log.rcvcount);
			break;
		case HOSTCOMMU_LOG_FUNCTION:
		case HOSTCOMMU_LOG_REGISTER_RANGE:
		default:
			snprintf(detail, sizeof(detail),
			         "功能码：%d,起始地址：%d,寄存器数：%d",
			         log.functioncode,
			         log.startaddress,
			         log.registeramount);
			break;
		}
		if (log.pending_suppressed != 0U) {
			size_t used = strlen(detail);
			if (used < sizeof(detail)) {
				snprintf(&detail[used],
				         sizeof(detail) - used,
				         ",抑制：%lu",
				         (unsigned long)log.pending_suppressed);
			}
		}

		// 错误	阶段：错误报警	模块：通信	操作：接收主机帧	原因：log.reason	处理：继续尝试	详情：detail
		ErrorLog_WarnDetail(ERROR_LOG_MODULE_COMM,
		                    ERROR_LOG_OP_HOST_FRAME,
		                    log.reason,
		                    ERROR_LOG_ACTION_CONTINUE,
		                    detail);
	}
}

static void HostCommuResumeRxDMA(void) {
	/* UART5 IDLE IRQ stops RX DMA first; restore RX here for bad frames or TX start failures. */
	RS485_SET_RECV_MODE();
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	HAL_UART_Receive_DMA(&huart5, UART5_RX_BUF, UART5_RX_BUF_SIZE);
}

/**
 * @brief Modbus从站初始化函数
 *
 * @return int 初始化状态（0表示成功）
 */
int HostCommuInit(void) {
	int ret = 0;

	// 设置从站地址（默认为01）
	SetSlaveaddress(01);

	return ret;
}

/**
 * @brief Modbus通信处理主函数
 *
 * @param rcvbuff 接收到的Modbus帧数据
 * @param rcvcount 接收到的数据长度
 * @param commu_num 通信通道号（用于多接口系一）
 */
void HostCommuProcess(uint8_t *rcvbuff, int rcvcount) {
	int functioncode;    // Modbus功能码
	int startaddress;    // 寄存器起始地址
	int registeramount;  // 寄存器数量
	uint16_t crc;        // CRC校验值

	// 调试输出：打印接收到的原始帧数据（仅在调试模式启用时）
#if DEBUG_HOSTCOMMU
	int i;
	printf("主机接收 %d 字节:\t", rcvcount);
	for (i = 0; i < rcvcount; i++) {
		printf("%02X ", rcvbuff[i]);
	}
	printf("\n");
#endif

	// 检查1: 接收数据长度有效性
	if ((rcvcount <= 3) || (rcvcount >= MAXRCVLENGTH)) {
		// 帧长度过短或过长都不合法
#if DEBUG_HOSTCOMMU
		printf("主机通信: 长度%d异常\r\n", rcvcount);
#endif
		HostCommu_RecordDeferredLog(HOSTCOMMU_LOG_LENGTH,
		                            ERROR_LOG_REASON_LENGTH_ERROR,
		                            rcvcount,
		                            0U,
		                            0,
		                            0,
		                            0);
		HostCommuResumeRxDMA();
	}
	// 检查2: 目标地址校验
	else if (SlaveCheckAddress(rcvbuff, rcvcount) == false) {
		// 地址不匹配，不是发给本机的请求
#if DEBUG_HOSTCOMMU
		printf("主机通信: 地址%d异常\r\n", rcvbuff[0]);
#endif
		HostCommu_RecordDeferredLog(HOSTCOMMU_LOG_ADDRESS,
		                            ERROR_LOG_REASON_ADDRESS_ERROR,
		                            rcvcount,
		                            (unsigned int)rcvbuff[0],
		                            0,
		                            0,
		                            0);
		HostCommuResumeRxDMA();
	}
	// 检查3: CRC校验
	else if (SlaveCheckCRC(rcvbuff, rcvcount) == false) {
		// CRC校验失败，记录一一告警
		HostCommu_RecordDeferredLog(HOSTCOMMU_LOG_CRC,
		                            ERROR_LOG_REASON_PARAM_CRC,
		                            rcvcount,
		                            (unsigned int)rcvbuff[0],
		                            0,
		                            0,
		                            0);
		HostCommuResumeRxDMA();
	}
	// 处理有效请求
	else {
		// 解析功能码 (第2字节)
		functioncode = rcvbuff[1];

		// 解析起始地址 (第3-4字节，高位在前)
		startaddress = (rcvbuff[2] << 8) + rcvbuff[3];

		// 解析寄存器数量 (第5-6字节，高位在前)
		registeramount = (rcvbuff[4] << 8) + rcvbuff[5];

		// 更新接收参数状态（用于调试/监控）
		UpdateRcvPara(functioncode, startaddress, registeramount);

		// 准备发送缓冲区（重置长度）
		HCOM_SendCount = 0;

		// 检查4: 功能码合法性
		if (FunctionCheckIllPack(HCOM_SendBuff, &HCOM_SendCount) == false) {
			// 非支持的功能码
#if DEBUG_HOSTCOMMU
			printf("主机通信: 功能码%d异常\r\n", functioncode);
#endif
			HostCommu_RecordDeferredLog(HOSTCOMMU_LOG_FUNCTION,
			                            ERROR_LOG_REASON_FUNCTION_ERROR,
			                            rcvcount,
			                            (unsigned int)rcvbuff[0],
			                            functioncode,
			                            startaddress,
			                            registeramount);
		}
		// 检查5: 数据地址合法性
		else if (IllegalDataAddressPack(HCOM_SendBuff, &HCOM_SendCount) == false) {
			// 请求的寄存器地址或数量超出范围
#if DEBUG_HOSTCOMMU
			printf("主机通信: 起始地址%d 寄存器数%d异常\r\n", startaddress, registeramount);
#endif
			HostCommu_RecordDeferredLog(HOSTCOMMU_LOG_REGISTER_RANGE,
			                            ERROR_LOG_REASON_REGISTER_RANGE,
			                            rcvcount,
			                            (unsigned int)rcvbuff[0],
			                            functioncode,
			                            startaddress,
			                            registeramount);
		}
		// 处理支持的合法请求
		else {
			// 根据功能码调用对应的处理函数
			switch (functioncode) {
			case FUNCTIONCODE_READ_HOLDREGISTER:  // 03 - 读保持寄存器
				HCOM_SendCount = Response03Process(rcvbuff, HCOM_SendBuff);
				break;

			case FUNCTIONCODE_READ_INPUTREGISTER: // 04 - 读输入寄存器
				HCOM_SendCount = Response04Process(rcvbuff, HCOM_SendBuff);
				break;

			case FUNCTIONCODE_WRITE_MULREGISTER:  // 16 - 写多个寄存器
				HCOM_SendCount = Response10Process(rcvbuff, HCOM_SendBuff);
				break;
			}
		}

		//添加CRC校验到响应帧尾部
		crc = CRC16_Calculate(HCOM_SendBuff, HCOM_SendCount);
		HCOM_SendBuff[HCOM_SendCount] = crc & 0xff;    // CRC低字节
		HCOM_SendBuff[HCOM_SendCount + 1] = crc >> 8;   // CRC高字节
		HCOM_SendCount += 2;
#if DEBUG_HOSTCOMMU
		printf("主机通信: 发送%d字节:", HCOM_SendCount);
		for (int i = 0; i < HCOM_SendCount; i++) {
			printf(" %02X", (unsigned char) HCOM_SendBuff[i]);
		}
		printf("\r\n");
#endif
		//切换发送模式
		RS485_SET_SEND_MODE();  // 切换到发送模式
		if (HAL_UART_Transmit_DMA(&huart5, (uint8_t*)HCOM_SendBuff, HCOM_SendCount) != HAL_OK) {  // send response
			/* If TX DMA does not start, TxCpltCallback will not run, so force the bus back to RX here. */
			HostCommuResumeRxDMA();
		}
	}
}
