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

#define DEBUG_HOSTCOMMU 0 /* 主机通信调试打印开关。 */
/* 常量定义 */
#define MAXRCVLENGTH 256 /* Modbus帧最大接收长度（RTU模式一般为256字节） */
#define HOSTCOMMU_ERROR_LOG_INTERVAL_MS 1000U /* 主机通信错误日志限频间隔，单位 ms。 */

typedef enum {
	/* 主机 Modbus 延迟日志的错误分类索引。 */
	HOSTCOMMU_LOG_LENGTH = 0, /* 主机 Modbus 帧长度错误日志类别。 */
	HOSTCOMMU_LOG_ADDRESS, /* 主机 Modbus 从站地址不匹配日志类别。 */
	HOSTCOMMU_LOG_CRC, /* 主机 Modbus CRC 校验失败日志类别。 */
	HOSTCOMMU_LOG_FUNCTION, /* 主机 Modbus 功能码不支持日志类别。 */
	HOSTCOMMU_LOG_REGISTER_RANGE, /* 主机 Modbus 寄存器范围越界日志类别。 */
	HOSTCOMMU_LOG_COUNT /* 主机通信日志类别总数，仅用于数组容量。 */
} HostCommuLogType;

typedef struct {
	/* Modbus 高频错误的延迟日志槽；合并抑制期间的重复错误，并保留最后一帧关键字段。 */
	uint8_t pending; /* 当前延迟日志槽是否保存了一条等待输出的错误记录。 */
	uint32_t last_tick; /* 最近一次tick，采用 HAL 单调毫秒节拍；超时判断必须使用无符号差值。 */
	uint32_t suppressed; /* 当前日志抑制窗口内被合并的同类错误次数。 */
	uint32_t pending_suppressed; /* 待输出日志记录附带的已抑制同类错误次数。 */
	int rcvcount; /* 触发该日志的 Modbus 实际接收字节数。 */
	unsigned int addr; /* 触发该日志的 Modbus 从站地址。 */
	int functioncode; /* 触发该日志的 Modbus 功能码。 */
	int startaddress; /* 触发该日志的 Modbus 起始寄存器地址。 */
	int registeramount; /* 触发该日志的 Modbus 请求寄存器数量。 */
	const char *reason; /* 延迟输出时使用的只读中文错误原因。 */
} HostCommuDeferredLog;

/* 发送缓冲区（静态分配） */
static uint8_t  HCOM_SendBuff[HOSTCOMMU_SENDLENGTH]; /* Modbus响应帧缓冲区 */
static int HCOM_SendCount = 0;                   /* 发送缓冲区当前数据长度 */
static HostCommuDeferredLog s_hostcommu_deferred_logs[HOSTCOMMU_LOG_COUNT] = {0};

/**
 * @brief 记录主机通信异常日志请求。
 * @note 该函数可能在 UART5 中断上下文被调用，只保存必要字段，不做 printf。
 *
 * @param type 类型。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param rcvcount 接收缓冲区中的有效帧长度，单位字节。
 * @param addr 接收帧中的 Modbus 从站地址字节，用于在延后日志中还原请求现场。
 * @param functioncode 本次 Modbus 或 HART 请求的功能码。
 * @param startaddress 本次 Modbus 访问的起始寄存器地址。
 * @param registeramount 本次连续访问的寄存器数量。
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
 *
 * 函数依次检查帧长、地址、CRC、功能码和寄存器范围五类延后日志；每类日志在中断侧只保存最近一次详情并累计限频期间的抑制次数。
 * 复制 pending 日志时先保存 PRIMASK 并短暂关闭中断，清除 pending 后按原中断状态恢复，避免 UART5 中断同时改写共享缓存。
 * 帧长错误输出实际长度与最大长度，地址或 CRC 错误输出地址和长度，功能码或寄存器范围错误输出功能码、起始地址和寄存器数量；存在抑制记录时追加累计次数。
 * 格式化完成后通过统一通信告警接口在线程态打印，单类日志一次只消费一份快照。
 *
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
		/* 进入临界区，保护主板通信共享状态，避免中断同时修改。 */
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

		ErrorLog_WarnDetail(ERROR_LOG_MODULE_COMM,
		                    ERROR_LOG_OP_HOST_FRAME,
		                    log.reason,
		                    ERROR_LOG_ACTION_CONTINUE,
		                    detail);
	}
}

/**
 * @brief 恢复 UART5 DMA 接收并重新使能空闲中断。
 */
static void HostCommuResumeRxDMA(void) {
	/* UART5 IDLE IRQ stops RX DMA first; restore RX here for bad frames or TX start failures. */
	(void)CPU2_UartRestartRxDMA(&huart5);
}

/**
 * @brief 把 CPU2 主机 Modbus 从站地址初始化为默认地址 1。
 *
 * @return 固定返回 0；当前初始化只设置内存中的默认从站地址，不执行可失败的外设访问。
 */
int HostCommuInit(void) {
	int ret = 0;

	/* 设置从站地址（默认为01） */
	SetSlaveaddress(01);

	return ret;
}

/**
 * @brief 校验并处理 CPU2 UART5 收到的一帧主机 Modbus RTU 请求，生成响应或恢复 DMA 接收。
 *
 * 处理顺序为帧长、从站地址和 CRC 校验；合法请求解析功能码、起始地址和数量，再分派 FC03、FC04 或 FC10，并为正常或异常响应追加低字节在前的 CRC16。
 * 帧长、地址或 CRC 不合法时只记录延后日志并恢复 UART5 DMA 接收；响应 DMA 启动失败时同样立即退回接收，避免 RS485 总线停留在发送方向。
 *
 * @param rcvbuff 接收到的 Modbus 帧数据；已经由 UART5 空闲中断结束接收，有效字节范围由 rcvcount 指定。
 * @param rcvcount 接收到的数据长度，单位字节；对应 rcvbuff 中实际有效的帧范围，必须大于 3 且小于 MAXRCVLENGTH。
 */
void HostCommuProcess(uint8_t *rcvbuff, int rcvcount) {
	int functioncode;    /* Modbus功能码 */
	int startaddress;    /* 寄存器起始地址 */
	int registeramount;  /* 寄存器数量 */
	uint16_t crc;        /* CRC校验值 */

	/* 调试输出：打印接收到的原始帧数据（仅在调试模式启用时） */
#if DEBUG_HOSTCOMMU
	int i;
	printf("主机接收 %d 字节:\t", rcvcount);
	for (i = 0; i < rcvcount; i++) {
		printf("%02X ", rcvbuff[i]);
	}
	printf("\n");
#endif

	/* 检查1: 接收数据长度有效性 */
	if ((rcvcount <= 3) || (rcvcount >= MAXRCVLENGTH)) {
		/* 帧长度过短或过长都不合法 */
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
	/* 检查2: 目标地址校验 */
	else if (SlaveCheckAddress(rcvbuff, rcvcount) == false) {
		/* 地址不匹配，不是发给本机的请求 */
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
	/* 检查3: CRC校验 */
	else if (SlaveCheckCRC(rcvbuff, rcvcount) == false) {
		/* CRC校验失败，记录一一告警 */
		HostCommu_RecordDeferredLog(HOSTCOMMU_LOG_CRC,
		                            ERROR_LOG_REASON_PARAM_CRC,
		                            rcvcount,
		                            (unsigned int)rcvbuff[0],
		                            0,
		                            0,
		                            0);
		HostCommuResumeRxDMA();
	}
	/* 处理有效请求 */
	else {
		/* 解析功能码 (第2字节) */
		functioncode = rcvbuff[1];

		/* 解析起始地址 (第3-4字节，高位在前) */
		startaddress = (rcvbuff[2] << 8) + rcvbuff[3];

		/* 解析寄存器数量 (第5-6字节，高位在前) */
		registeramount = (rcvbuff[4] << 8) + rcvbuff[5];

		/* 更新接收参数状态（用于调试/监控） */
		UpdateRcvPara(functioncode, startaddress, registeramount);

		/* 准备发送缓冲区（重置长度） */
		HCOM_SendCount = 0;

		/* 检查4: 功能码合法性 */
		if (FunctionCheckIllPack(HCOM_SendBuff, &HCOM_SendCount) == false) {
			/* 非支持的功能码 */
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
		/* 检查5: 数据地址合法性 */
		else if (IllegalDataAddressPack(HCOM_SendBuff, &HCOM_SendCount) == false) {
			/* 请求的寄存器地址或数量超出范围 */
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
		/* 处理支持的合法请求 */
		else {
			/* 根据功能码调用对应的处理函数 */
			switch (functioncode) {
			case FUNCTIONCODE_READ_HOLDREGISTER:  /* 03 - 读保持寄存器 */
				HCOM_SendCount = Response03Process(rcvbuff, HCOM_SendBuff);
				break;

			case FUNCTIONCODE_READ_INPUTREGISTER: /* 04 - 读输入寄存器 */
				HCOM_SendCount = Response04Process(rcvbuff, HCOM_SendBuff);
				break;

			case FUNCTIONCODE_WRITE_MULREGISTER:  /* 16 - 写多个寄存器 */
				HCOM_SendCount = Response10Process(rcvbuff, HCOM_SendBuff);
				break;
			}
		}

		/* 添加CRC校验到响应帧尾部 */
		crc = CRC16_Calculate(HCOM_SendBuff, HCOM_SendCount);
		HCOM_SendBuff[HCOM_SendCount] = crc & 0xff;    /* CRC低字节 */
		HCOM_SendBuff[HCOM_SendCount + 1] = crc >> 8;   /* CRC高字节 */
		HCOM_SendCount += 2;
#if DEBUG_HOSTCOMMU
		printf("主机通信: 发送%d字节:", HCOM_SendCount);
		for (int i = 0; i < HCOM_SendCount; i++) {
			printf(" %02X", (unsigned char) HCOM_SendBuff[i]);
		}
		printf("\r\n");
#endif
		/* 切换发送模式 */
		RS485_SET_SEND_MODE();  /* 切换到发送模式 */
		if (HAL_UART_Transmit_DMA(&huart5, (uint8_t*)HCOM_SendBuff, HCOM_SendCount) != HAL_OK) {  /* send response */
			/* If TX DMA does not start, TxCpltCallback will not run, so force the bus back to RX here. */
			HostCommuResumeRxDMA();
		}
	}
}
