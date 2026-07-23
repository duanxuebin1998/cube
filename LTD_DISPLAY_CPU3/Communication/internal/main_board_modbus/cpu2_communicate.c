#include "cpu2_communicate.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include "my_crc.h"
#include "system_parameter.h"
#include "dataanalysis_modbus.h"
#include "wartsila_modbus_communication.h"
#include "wartsila_modbus_data_analysis.h"
#include "cpu3_debug_log.h"
#include "app_main.h"

#define ADERSS 0X01
#define CPU2_RESPONSE_TIMEOUT_MS 1000U /* CPU2 单次响应等待超时，单位 ms。 */
#define CPU2_COMM_STATUS_LOG_INTERVAL_MS 5000U /* CPU2通信健康摘要周期，避免阻塞调试串口影响轮询。 */
#define CPU2_COMM_RESYNC_FAILURE_LIMIT 3U /* 连续失败达到三次后才废弃快照并完整重同步。 */
#define CPU2_COMM_FAILURE_LIMIT 10U /* CPU2 连续请求未获得合法响应的置错阈值。 */
#define CPU2_MAX_EXTERNAL_WRITE_REGISTERS 122U /* RTU 最大 123 个寄存器；共享字段按 32 位对齐后取最大偶数。 */
#define CPU2_FIXED_POINT_RESULT_REGISTER_COUNT (REG_DENSITY_DIST_AVG_TEMP - REG_SINGLE_POINT_MEAS_TEMP)
#define CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT (2U * REG_SIZE_U32)
#define CPU2_PROFILE_HEADER_REGISTER_COUNT REG_DENSITY_DIST_SUMMARY_REG_COUNT
#define CPU2_PROFILE_POINT_REGISTER_CAPACITY (MAX_MEASUREMENT_POINTS * REG_DENSITY_DIST_POINT_SIZE)
#define CPU2_PROFILE_POINTS_PER_FRAME 8U
#define CPU2_PROFILE_REGISTERS_PER_FRAME (CPU2_PROFILE_POINTS_PER_FRAME * REG_DENSITY_DIST_POINT_SIZE)
#define CPU2_PROFILE_FRAME_INTERVAL_MS 50U
#define CPU2_PROFILE_ROUND_RETRY_DELAY_MS 500U
#define CPU2_PROFILE_NO_PROGRESS_TIMEOUT_MS 20000U
#define CPU2_PROFILE_FAILED_RETRY_DELAY_MS 2000U
#define CPU2_PROFILE_BLOCK_RETRY_MAX 3U
#define CPU2_PROFILE_ROUND_MAX 3U

volatile bool wait_response = false; /* 主控板响应标志位 */
static bool s_cpu2_has_status_snapshot = false; /* CPU3 本次上电是否收到过覆盖状态和错误码的 CPU2 响应。 */
static bool s_cpu2_has_parameter_snapshot = false; /* CPU3 是否已完整读取 CPU2 保持寄存器参数。 */
static bool s_cpu2_has_protocol_snapshot = false; /* 当前 CPU2 连接是否已读回共享协议版本。 */
static uint32_t s_cpu2_protocol_version = 0U; /* 当前连接独立探测到的 CPU2 共享协议版本。 */
static bool s_cpu2_has_fixed_point_snapshot = false; /* 固定点结果与双代际是否已通过前后双读校验。 */
static uint32_t s_cpu2_consecutive_failure_count = 0U; /* CPU2 连续请求失败次数。 */
static bool s_cpu2_comm_fault_active = false; /* CPU3 本机通信故障是否等待状态帧恢复。 */
static uint32_t s_cpu2_snapshot_generation = 0U; /* 每次CPU2公开快照失效时递增，供SI识别通信会话切换。 */
static bool s_cpu2_parameter_refresh_requested = false; /* 外部参数写后是否要求重新确认 CPU2 参数。 */
static bool s_cpu2_snapshot_resync_requested = false; /* 内部通信失败后是否要求完整重同步。 */
static volatile uint32_t s_cpu2_uart_error_pending = HAL_UART_ERROR_NONE; /* UART5中断只锁存错误位，主循环统一分类计数。 */
static uint32_t s_cpu2_request_uart_error_code = HAL_UART_ERROR_NONE; /* 当前失败请求对应的UART5硬件错误位。 */
static Cpu2CommHealthSnapshot s_cpu2_comm_health = {0}; /* CPU3本机RAM通信健康计数，上电清零。 */
static Cpu2CommFailureReason s_cpu2_response_failure_reason = CPU2_COMM_FAIL_NONE; /* 当前响应校验失败原因。 */

/* 保持寄存器 */
uint16_t HoldingRegisterArray[HOLDREGISTER_AMOUNT] = { 0 }; /* 保持寄存器数组 */
/* 输入寄存器 */
static uint16_t InputRegisterArray[INPUTREGISTER_AMOUNT] = { 0 };    /* 输入寄存器数组 */

typedef enum {
	CPU2_FIXED_POINT_READ_COUNTER_BEFORE = 0,
	CPU2_FIXED_POINT_READ_RESULTS,
	CPU2_FIXED_POINT_READ_COUNTER_AFTER
} Cpu2FixedPointSnapshotStage;

typedef struct {
	uint32_t complete_latched;
	uint32_t complete_counter;
	uint32_t measurement_points;
	uint32_t profile_source;
} Cpu2ProfileSnapshotKey;

typedef enum {
	CPU2_PROFILE_SYNC_IDLE = 0,
	CPU2_PROFILE_SYNC_HEADER_BEFORE,
	CPU2_PROFILE_SYNC_POINTS,
	CPU2_PROFILE_SYNC_HEADER_AFTER,
	CPU2_PROFILE_SYNC_RETRY_WAIT
} Cpu2ProfileSyncStage;

typedef struct {
	Cpu2ProfileSyncStage stage;
	Cpu2ProfileSyncStage retry_stage;
	Cpu2ProfileSnapshotKey target_key;
	uint32_t snapshot_generation;
	uint32_t started_tick;
	uint32_t last_progress_tick;
	uint32_t last_frame_tick;
	uint32_t retry_due_tick;
	uint32_t point_register_offset;
	uint8_t block_retry_count;
	uint8_t round;
} Cpu2ProfileSyncContext;

static Cpu2FixedPointSnapshotStage s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_COUNTER_BEFORE;
static uint16_t s_cpu2_fixed_point_results[CPU2_FIXED_POINT_RESULT_REGISTER_COUNT];
static uint16_t s_cpu2_fixed_point_counter_before[CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT];
static uint16_t s_cpu2_fixed_point_counter_after[CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT];
static uint16_t *s_cpu2_private_input_destination = NULL;
static uint16_t s_cpu2_private_input_capacity = 0U;
static uint16_t s_cpu2_private_input_start = 0U;
static bool s_cpu2_private_input_captured = false;
static Cpu2ProfileSyncContext s_cpu2_profile_sync = {0};
static uint16_t s_cpu2_profile_header_before[CPU2_PROFILE_HEADER_REGISTER_COUNT];
static uint16_t s_cpu2_profile_header_after[CPU2_PROFILE_HEADER_REGISTER_COUNT];
static uint16_t s_cpu2_profile_candidate_points[CPU2_PROFILE_POINT_REGISTER_CAPACITY];
static uint16_t s_cpu2_profile_published_header[CPU2_PROFILE_HEADER_REGISTER_COUNT];
static uint16_t s_cpu2_profile_published_points[CPU2_PROFILE_POINT_REGISTER_CAPACITY];
static DensityDistribution s_cpu2_profile_candidate_distribution;
static DensityDistribution s_cpu2_profile_published_distribution;
static Cpu2ProfileSnapshotKey s_cpu2_profile_published_key = {0};
static uint32_t s_cpu2_profile_published_cycle = 0U;
static bool s_cpu2_profile_published_valid = false;
static bool s_cpu2_profile_failed_key_valid = false;
static Cpu2ProfileSnapshotKey s_cpu2_profile_failed_key = {0};
static uint32_t s_cpu2_profile_failed_snapshot_generation = 0U;
static uint32_t s_cpu2_profile_failed_retry_due_tick = 0U;
static bool s_cpu2_profile_sync_fault_active = false;
static DeviceState s_cpu2_raw_device_state = STATE_INIT;
static uint32_t s_cpu2_raw_error_code = NO_ERROR;
static DeviceState s_cpu2_last_raw_device_state = STATE_INIT;
static bool s_cpu2_last_raw_device_state_valid = false;
static bool s_cpu2_profile_completion_state_pending = false;
static bool s_cpu2_profile_commit_waiting_for_complete_state = false;

/* 接收到的命令包数据暂存变量 */
static int RCV_functioncode = 0; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static int RCV_startaddress = 0; /* Modbus 协议地址配置，影响协议寻址或硬件访问。 */
static int RCV_registercnt = 0; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static int SlaveTempBuffer[LTD_MODBUS_MAX_READ_REGISTERS]; /* 单帧寄存器数据缓冲区。 */

/* static void CPU2_Response03Process(char const *revframe); */
static void CPU2_Response03Process(uint8_t const *revframe);
static void CPU2_Response04Process(uint8_t const *revframe);
static void CPU2_Response10Process(uint8_t *arr, uint16_t len);
static void PresetRegister(bool registertype, int const *registervalue);
static Cpu2CommFailureReason CPU2_ResponseFrameFailureReason(uint8_t const *rcv, int len);
static bool CPU2_ResponseContainsDeviceStatus(void);
static bool CPU2_ResponseContainsProtocolVersion(void);
static void CPU2_CommMarkValidResponse(void);
static void CPU2_InvalidatePublicSnapshotFreshness(void);
static void CPU2_CommRecordFailure(Cpu2CommFailureReason reason);
static void CPU2_CommRecordUartFlags(uint32_t uart_error_code);
static void CPU2_ProfileInvalidatePublishedSnapshot(void);
static void CPU2_ResetFixedPointSnapshotHandshake(bool invalidate_snapshot);
static bool CPU2_ReadInputRegistersPrivate(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs);
static bool CPU2_PollFixedPointSnapshotHandshake(void);
static bool CPU2_ProfileSyncPoll(void);
static void CPU2_ProfileObservePublicHeader(void);
static void CPU2_ProfileApplyPublicStatusGate(void);
static const char *CPU2_CommFailureReasonText(Cpu2CommFailureReason reason);
static const char *CPU2_CommLinkStateText(void);
static void CPU2_CommLogRequestFailure(void);
static bool CPU2_CombinatePackage_SendWire(uint8_t f_code,
                                           uint16_t startadd,
                                           uint16_t registercnt,
                                           uint32_t *holddata);

/* 返回CPU2请求失败原因的现场可读名称。 */
static const char *CPU2_CommFailureReasonText(Cpu2CommFailureReason reason)
{
	switch (reason) {
	case CPU2_COMM_FAIL_TIMEOUT:
		return "响应超时";
	case CPU2_COMM_FAIL_CRC:
		return "CRC校验失败";
	case CPU2_COMM_FAIL_ADDRESS:
		return "从机地址不匹配";
	case CPU2_COMM_FAIL_FUNCTION:
		return "功能码或写回显不匹配";
	case CPU2_COMM_FAIL_LENGTH:
		return "帧长度不匹配";
	case CPU2_COMM_FAIL_UART:
		return "UART硬件异常";
	case CPU2_COMM_FAIL_TX_DMA:
		return "发送DMA启动失败";
	case CPU2_COMM_FAIL_NONE:
	default:
		return "无";
	}
}

/* 返回CPU2链路当前对外门禁状态。 */
static const char *CPU2_CommLinkStateText(void)
{
	if (s_cpu2_comm_fault_active) {
		return "通信故障";
	}
	if (CPU2_CommIsProtocolMismatch()) {
		return "协议不匹配";
	}
	if (CPU2_CommIsAvailable()) {
		return "可用";
	}
	return "同步中";
}

/*
 * 函数用途：校验 CPU2 响应是否与当前请求的功能码、长度和回显字段一致。
 * 调用场景：CRC 和从机地址校验通过后、清除连续请求失败计数前调用。
 * 关键约束：异常响应、错功能码和不完整数据帧均不得刷新通信有效状态。
 */
static Cpu2CommFailureReason CPU2_ResponseFrameFailureReason(uint8_t const *rcv, int len)
{
	uint32_t expected_byte_count;

	if ((rcv == NULL) || (len < 5)) {
		return CPU2_COMM_FAIL_LENGTH;
	}
	if (rcv[1] != (uint8_t)RCV_functioncode) {
		return CPU2_COMM_FAIL_FUNCTION;
	}

	switch (RCV_functioncode) {
	case FUNCTIONCODE_READ_HOLDREGISTER:
	case FUNCTIONCODE_READ_INPUTREGISTER:
		expected_byte_count = (uint32_t)RCV_registercnt * 2U;
		return ((expected_byte_count <= UINT8_MAX) &&
				(rcv[2] == (uint8_t)expected_byte_count) &&
				(len == (int)(expected_byte_count + 5U))) ?
			   CPU2_COMM_FAIL_NONE : CPU2_COMM_FAIL_LENGTH;

	case FUNCTIONCODE_WRITE_MULREGISTER:
		if (len != 8) {
			return CPU2_COMM_FAIL_LENGTH;
		}
		return ((rcv[2] == (uint8_t)((uint16_t)RCV_startaddress >> 8)) &&
				(rcv[3] == (uint8_t)RCV_startaddress) &&
				(rcv[4] == (uint8_t)((uint16_t)RCV_registercnt >> 8)) &&
				(rcv[5] == (uint8_t)RCV_registercnt)) ?
			   CPU2_COMM_FAIL_NONE : CPU2_COMM_FAIL_FUNCTION;

	default:
		return CPU2_COMM_FAIL_FUNCTION;
	}
}

/*
 * 函数用途：判断当前 0x04 响应是否完整覆盖设备状态和错误码字段。
 * 调用场景：输入寄存器响应写入缓存后决定是否允许恢复本机通信故障。
 * 关键约束：非状态分组不得用掉线前的旧缓存提前清除通信故障。
 */
static bool CPU2_ResponseContainsDeviceStatus(void)
{
	return (RCV_functioncode == FUNCTIONCODE_READ_INPUTREGISTER) &&
		   LtdModbus_RangeContains((uint16_t)RCV_startaddress,
							   (uint16_t)RCV_registercnt,
							   REG_DEVICE_STATUS_DEVICE_STATE,
							   REG_SIZE_U32) &&
		   LtdModbus_RangeContains((uint16_t)RCV_startaddress,
							   (uint16_t)RCV_registercnt,
							   REG_DEVICE_STATUS_ERROR_CODE,
							   REG_SIZE_U32);
}

/*
 * 函数用途：判断当前 0x03 响应是否完整覆盖共享协议版本字段。
 * 调用场景：保持寄存器响应解析完成后确认当前 CPU2 连接的协议版本来源。
 * 关键约束：通信故障恢复后必须重新读回该字段，不能沿用掉线前缓存开放写入口。
 */
static bool CPU2_ResponseContainsProtocolVersion(void)
{
	return (RCV_functioncode == FUNCTIONCODE_READ_HOLDREGISTER) &&
		   LtdModbus_RangeContains((uint16_t)RCV_startaddress,
							   (uint16_t)RCV_registercnt,
							   HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION,
							   REG_SIZE_U32);
}

/*
 * 函数用途：记录 CPU2 合法响应并清除连续请求失败计数。
 * 调用场景：响应通过长度、CRC 和从机地址校验后由主循环调用。
 * 关键约束：本函数不建立状态快照；只有包含状态和错误码的 0x04 响应才开放写入口。
 */
static void CPU2_CommMarkValidResponse(void)
{
	uint32_t previous_consecutive_count = s_cpu2_comm_health.consecutive_failure_count;
	Cpu2CommFailureReason previous_reason = s_cpu2_comm_health.last_failure_reason;

	s_cpu2_comm_health.success_count++;
	s_cpu2_consecutive_failure_count = 0U;
	s_cpu2_comm_health.consecutive_failure_count = 0U;
	if (previous_consecutive_count > 0U) {
		CPU3_LOG_INFO("CPU2",
					  "收到合法响应，连续失败计数清零 原因=%s 之前连续失败=%lu",
					  CPU2_CommFailureReasonText(previous_reason),
					  (unsigned long)previous_consecutive_count);
	}
}

/*
 * 函数用途：关闭所有对外公开快照门禁并请求完整重同步。
 * 调用场景：CPU2连续请求失败达到三次重同步阈值时调用。
 * 关键约束：只处理数据新鲜度和完整重同步请求；连续十次报警计数由独立逻辑维护。
 */
static void CPU2_InvalidatePublicSnapshotFreshness(void)
{
	s_cpu2_has_status_snapshot = false;
	s_cpu2_has_parameter_snapshot = false;
	s_cpu2_has_protocol_snapshot = false;
	s_cpu2_protocol_version = 0U;
	s_cpu2_snapshot_generation++;
	CPU2_ProfileInvalidatePublishedSnapshot();
	CPU2_ResetFixedPointSnapshotHandshake(true);
	s_cpu2_snapshot_resync_requested = true;
	CPU3_LOG_WARNING("CPU2",
					 "连续失败达到%u次，公开快照已失效并请求完整重同步",
					 (unsigned int)CPU2_COMM_RESYNC_FAILURE_LIMIT);
}

/*
 * 函数用途：记录一次未获得合法CPU2响应的请求，并按连续失败阈值升级恢复动作。
 * 调用场景：主循环处理响应超时、非法响应、UART 错误或 TX DMA 启动失败时调用。
 * 关键约束：前两次失败只计数并重试，第三次失败才废弃公开快照并完整重同步；
 *           连续十次失败仍置CPU2通信超时故障，ISR只置待处理标志。
 */
static void CPU2_CommRecordFailure(Cpu2CommFailureReason reason)
{
	bool fault_was_active = s_cpu2_comm_fault_active;

	switch (reason) {
	case CPU2_COMM_FAIL_TIMEOUT:
		s_cpu2_comm_health.timeout_count++;
		break;
	case CPU2_COMM_FAIL_CRC:
		s_cpu2_comm_health.crc_count++;
		break;
	case CPU2_COMM_FAIL_ADDRESS:
		s_cpu2_comm_health.address_count++;
		break;
	case CPU2_COMM_FAIL_FUNCTION:
		s_cpu2_comm_health.function_count++;
		break;
	case CPU2_COMM_FAIL_LENGTH:
		s_cpu2_comm_health.length_count++;
		break;
	case CPU2_COMM_FAIL_TX_DMA:
		s_cpu2_comm_health.tx_dma_start_fail_count++;
		break;
	case CPU2_COMM_FAIL_UART:
		s_cpu2_comm_health.uart_failure_count++;
		break;
	case CPU2_COMM_FAIL_NONE:
	default:
		break;
	}
	s_cpu2_comm_health.total_failure_count++;
	s_cpu2_comm_health.last_failure_reason = reason;
	s_cpu2_comm_health.consecutive_failure_count++;
	if (s_cpu2_comm_health.consecutive_failure_count > s_cpu2_comm_health.max_consecutive_failure_count) {
		s_cpu2_comm_health.max_consecutive_failure_count = s_cpu2_comm_health.consecutive_failure_count;
	}

	if (s_cpu2_consecutive_failure_count < CPU2_COMM_FAILURE_LIMIT) {
		s_cpu2_consecutive_failure_count++;
	}
	if (s_cpu2_consecutive_failure_count == CPU2_COMM_RESYNC_FAILURE_LIMIT) {
		CPU2_InvalidatePublicSnapshotFreshness();
	}

	if (s_cpu2_consecutive_failure_count >= CPU2_COMM_FAILURE_LIMIT) {
		s_cpu2_comm_fault_active = true;
	}
	if ((!fault_was_active) && s_cpu2_comm_fault_active) {
		CPU3_LOG_CRITICAL("CPU2",
						  "连续失败达到%u次，CPU2通信故障已锁存",
						  (unsigned int)CPU2_COMM_FAILURE_LIMIT);
	}

	if (s_cpu2_comm_fault_active) {
		g_measurement.device_status.device_state = STATE_ERROR;
		g_measurement.device_status.error_code = CPU2_COMM_TIMEOUT;
	}
}

/*
 * 函数用途：把一次UART5事务中的硬件错误位分别累计到健康计数。
 * 调用场景：主循环解除同步等待并取得ISR锁存的HAL错误码后调用。
 * 关键约束：多个硬件错误位可分别累计，但整笔事务只增加一次总失败和连续失败。
 */
static void CPU2_CommRecordUartFlags(uint32_t uart_error_code)
{
	if ((uart_error_code & HAL_UART_ERROR_ORE) != 0U) {
		s_cpu2_comm_health.uart_ore_count++;
	}
	if ((uart_error_code & HAL_UART_ERROR_FE) != 0U) {
		s_cpu2_comm_health.uart_fe_count++;
	}
	if ((uart_error_code & HAL_UART_ERROR_NE) != 0U) {
		s_cpu2_comm_health.uart_ne_count++;
	}
	if ((uart_error_code & HAL_UART_ERROR_PE) != 0U) {
		s_cpu2_comm_health.uart_pe_count++;
	}
}

/*
 * 函数用途：记录 UART5 错误并解除当前同步等待。
 * 调用场景：HAL UART 错误回调中调用。
 * 关键约束：本函数可能处于 ISR 上下文，只置标志，不打印、不计数、不改设备状态。
 */
void CPU2_CommNotifyUartErrorFromISR(uint32_t uart_error_code)
{
	if (wait_response) {
		s_cpu2_uart_error_pending |= uart_error_code;
		wait_response = false;
	}
}

void CPU2_CommGetHealthSnapshot(Cpu2CommHealthSnapshot *out_snapshot)
{
	if (out_snapshot == NULL) {
		return;
	}

	*out_snapshot = s_cpu2_comm_health;
}

/*
 * 函数用途：输出当前失败请求的地址、功能码、计数和原始帧证据。
 * 调用场景：主循环完成失败分类和健康计数后调用。
 * 关键约束：同一笔失败只在统一出口打印一次，避免CRC等解析分支重复刷屏。
 */
static void CPU2_CommLogRequestFailure(void)
{
	CPU3_LOG_WARNING("CPU2",
					 "请求失败 原因=%s 功能码=0x%02X 起始地址=0x%04X 寄存器数=%u 接收长度=%u UART标志=0x%08lX 连续=%lu/%u 总失败=%lu",
					 CPU2_CommFailureReasonText(s_cpu2_response_failure_reason),
					 (unsigned int)(uint8_t)RCV_functioncode,
					 (unsigned int)(uint16_t)RCV_startaddress,
					 (unsigned int)(uint16_t)RCV_registercnt,
					 (unsigned int)UART5_RX_LEN,
					 (unsigned long)s_cpu2_request_uart_error_code,
					 (unsigned long)s_cpu2_comm_health.consecutive_failure_count,
					 (unsigned int)CPU2_COMM_FAILURE_LIMIT,
					 (unsigned long)s_cpu2_comm_health.total_failure_count);

	if ((UART5_RX_LEN > 0U) &&
		(s_cpu2_response_failure_reason != CPU2_COMM_FAIL_TIMEOUT) &&
		(s_cpu2_response_failure_reason != CPU2_COMM_FAIL_UART) &&
		(s_cpu2_response_failure_reason != CPU2_COMM_FAIL_TX_DMA)) {
		Cpu3Log_Frame(CPU3_LOG_LEVEL_WARNING,
					  "CPU2",
					  "接收失败帧",
					  UART5_RX_BUF,
					  UART5_RX_LEN);
	}
}

void CPU2_CommDebugTask(void)
{
	static uint32_t last_log_tick = 0U;
	uint32_t now = HAL_GetTick();

	if ((now - last_log_tick) < CPU2_COMM_STATUS_LOG_INTERVAL_MS) {
		return;
	}
	last_log_tick = now;

	CPU3_LOG_INFO("CPU2",
				  "链路=%s CPU2协议=%lu CPU3协议=%u 快照=状态%u/参数%u/固定点%u 成功=%lu 失败=%lu 连续=%lu 最大连续=%lu 最近=%s",
				  CPU2_CommLinkStateText(),
				  (unsigned long)s_cpu2_protocol_version,
				  (unsigned int)DEVICE_PROTOCOL_VERSION,
				  s_cpu2_has_status_snapshot ? 1U : 0U,
				  s_cpu2_has_parameter_snapshot ? 1U : 0U,
				  s_cpu2_has_fixed_point_snapshot ? 1U : 0U,
				  (unsigned long)s_cpu2_comm_health.success_count,
				  (unsigned long)s_cpu2_comm_health.total_failure_count,
				  (unsigned long)s_cpu2_comm_health.consecutive_failure_count,
				  (unsigned long)s_cpu2_comm_health.max_consecutive_failure_count,
				  CPU2_CommFailureReasonText(s_cpu2_comm_health.last_failure_reason));
	if (s_cpu2_comm_health.total_failure_count > 0U) {
		CPU3_LOG_INFO("CPU2",
					  "失败分类 超时=%lu CRC=%lu 地址=%lu 功能=%lu 长度=%lu UART=%lu(ORE=%lu/FE=%lu/NE=%lu/PE=%lu) TXDMA=%lu",
					  (unsigned long)s_cpu2_comm_health.timeout_count,
					  (unsigned long)s_cpu2_comm_health.crc_count,
					  (unsigned long)s_cpu2_comm_health.address_count,
					  (unsigned long)s_cpu2_comm_health.function_count,
					  (unsigned long)s_cpu2_comm_health.length_count,
					  (unsigned long)s_cpu2_comm_health.uart_failure_count,
					  (unsigned long)s_cpu2_comm_health.uart_ore_count,
					  (unsigned long)s_cpu2_comm_health.uart_fe_count,
					  (unsigned long)s_cpu2_comm_health.uart_ne_count,
					  (unsigned long)s_cpu2_comm_health.uart_pe_count,
					  (unsigned long)s_cpu2_comm_health.tx_dma_start_fail_count);
	}
}

/*
 * 函数用途：判断当前连接是否已确认使用 CPU3 支持的共享协议。
 * 调用场景：启动显示、快照门禁、命令下发和运行期轮询决策。
 * 关键约束：必须同时具备当前连接协议快照和值相等，不能使用默认值或掉线前缓存。
 */
bool CPU2_CommIsProtocolCompatible(void)
{
	return s_cpu2_has_protocol_snapshot &&
		   (s_cpu2_protocol_version == DEVICE_PROTOCOL_VERSION);
}

/*
 * 函数用途：判断当前连接是否已经确认存在共享协议版本不匹配。
 * 调用场景：CPU3 状态页区分协议不匹配、同步未完成和通信超时。
 * 关键约束：协议快照无效时返回 false，不能把未知版本误报为协议不匹配。
 */
bool CPU2_CommIsProtocolMismatch(void)
{
	return s_cpu2_has_protocol_snapshot &&
		   (s_cpu2_protocol_version != DEVICE_PROTOCOL_VERSION);
}

/*
 * 函数用途：判断是否仍处于等待 CPU2 首次状态和当前连接协议快照的同步阶段。
 * 调用场景：CPU3 状态页选择通讯尝试页前调用。
 * 关键约束：协议字段未实际读回时不得用默认值或掉线前缓存显示协议不匹配/兼容；
 *           已确认协议不匹配时退出等待页显示兼容性提示，第十次连续失败后由故障页接管。
 */
bool CPU2_CommShouldShowStartup(void)
{
	bool fixed_point_snapshot_pending;

	if (CPU2_CommIsProtocolMismatch()) {
		return false;
	}

	fixed_point_snapshot_pending = CPU2_CommIsProtocolCompatible() &&
		(!s_cpu2_has_fixed_point_snapshot);

	return ((!s_cpu2_has_status_snapshot) ||
			(!s_cpu2_has_protocol_snapshot) ||
			fixed_point_snapshot_pending) &&
		   (!s_cpu2_comm_fault_active);
}

/*
 * 函数用途：判断 CPU2 状态和参数快照是否完整、协议是否兼容且通信故障未锁存。
 * 调用场景：CPU3 菜单或外部协议访问 CPU2 参数和命令前调用。
 * 关键约束：冷启动补读、参数刷新、协议不匹配和通信故障期间均禁止写入。
 */
bool CPU2_CommIsAvailable(void)
{
	return s_cpu2_has_status_snapshot &&
		   s_cpu2_has_parameter_snapshot &&
		   s_cpu2_has_protocol_snapshot &&
		   s_cpu2_has_fixed_point_snapshot &&
		   (!s_cpu2_comm_fault_active) &&
		   CPU2_CommIsProtocolCompatible();
}

bool CPU2_CommHasRuntimeSnapshot(void)
{
	return s_cpu2_has_status_snapshot &&
		   s_cpu2_has_protocol_snapshot &&
		   s_cpu2_has_fixed_point_snapshot &&
		   (!s_cpu2_comm_fault_active) &&
		   CPU2_CommIsProtocolCompatible();
}

/*
 * 函数用途：返回CPU2公开快照会话代际。
 * 调用场景：SI本地投影在通信恢复后判断旧发布结果是否必须作废。
 * 关键约束：该代际只表示CPU3本机快照连续性，不属于CPU2/CPU3共享协议字段。
 */
uint32_t CPU2_CommGetSnapshotGeneration(void)
{
	return s_cpu2_snapshot_generation;
}

/*
 * 函数用途：判断指定 CPU2 命令在当前通信状态下是否允许下发。
 * 调用场景：菜单或外部协议准备写命令寄存器前调用。
 * 关键约束：普通命令继续依赖完整参数快照；取消命令在参数刷新期间保持可达，
 *           但仍要求状态快照、协议兼容且通信故障未锁存。
 */
bool CPU2_CommCanSendCommand(CommandType cmd)
{
	if (cmd != CMD_CANCEL_MEASUREMENT) {
		return CPU2_CommIsAvailable();
	}

	return s_cpu2_has_status_snapshot &&
		   s_cpu2_has_protocol_snapshot &&
		   (!s_cpu2_comm_fault_active) &&
		   CPU2_CommIsProtocolCompatible();
}

/*
 * 函数用途：从 CPU3 已确认的参数快照复制 LTD 保持寄存器。
 * 调用场景：CPU3 对外以 LTD 协议独立响应 FC03 读请求时调用。
 * 关键约束：同步未完成、协议不匹配、通信故障或地址越界时不得返回旧缓存。
 */
bool CPU2_CommReadHoldingSnapshot(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs)
{
	if ((out_regs == NULL) ||
		!LtdModbus_RangeWithin(startadd, registercnt, HOLDREGISTER_AMOUNT) ||
		(!CPU2_CommIsAvailable())) {
		return false;
	}

	/* g_deviceParams 只在 CPU2 快照或成功写入后更新，组表后再复制可覆盖菜单成功写入的新值。 */
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	memcpy(out_regs, &HoldingRegisterArray[startadd],
		   (size_t)registercnt * sizeof(uint16_t));
	return true;
}

/*
 * 函数用途：从 CPU3 已确认的状态快照复制 LTD 输入寄存器。
 * 调用场景：CPU3 对外以 LTD 协议独立响应 FC04 读请求时调用。
 * 关键约束：只复制 CPU2 最近一次合法响应形成的数组，不用默认结构生成伪状态。
 */
bool CPU2_CommReadInputSnapshot(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs)
{
	if ((out_regs == NULL) ||
		!LtdModbus_RangeWithin(startadd, registercnt, INPUTREGISTER_AMOUNT) ||
		(!CPU2_CommIsAvailable())) {
		return false;
	}

	memcpy(out_regs, &InputRegisterArray[startadd],
		   (size_t)registercnt * sizeof(uint16_t));
	return true;
}

/*
 * 函数用途：使当前参数快照失效并请求重新读取 CPU2 全部保持寄存器参数。
 * 调用场景：外部协议写参数成功或失败后，重新确认 CPU2 的实际生效值。
 * 关键约束：刷新完成前普通写和依赖 CPU2 参数的外部读均不得返回成功。
 */
void CPU2_CommRequestParameterRefresh(void)
{
	s_cpu2_has_parameter_snapshot = false;
	s_cpu2_parameter_refresh_requested = true;
}

/*
 * 函数用途：把 LTD 外部 FC10 的寄存器序列写穿到 CPU2，并以 CPU2 ACK 作为成功依据。
 * 调用场景：CPU3 外部 LTD Modbus 从站处理写多个保持寄存器请求时调用。
 * 关键约束：共享参数均为 32 位字段，只接受偶数地址和偶数数量；参数写成功后也使快照失效，
 *           下一次对外读取必须等待 CPU2 全量补读确认，不能把请求影子当作 CPU2 实际值。
 */
bool CPU2_CommWriteHoldingRegisters(uint16_t startadd,
										uint16_t registercnt,
										const uint16_t *wire_regs)
{
	uint32_t host_values[CPU2_MAX_EXTERNAL_WRITE_REGISTERS / 2U];
	uint32_t command_value;
	bool command_only;
	bool ret;

	if ((wire_regs == NULL) ||
		(registercnt == 0U) ||
		(registercnt > CPU2_MAX_EXTERNAL_WRITE_REGISTERS) ||
		!LtdModbus_HoldingWriteRangeIsValid(startadd, registercnt)) {
		return false;
	}

	/* 现有发送入口接收本机 uint32_t 值，并负责转换为共享协议的高字在前线序。 */
	for (uint16_t i = 0U; i < registercnt; i += 2U) {
		host_values[i / 2U] = ((uint32_t)wire_regs[i] << 16) |
									 (uint32_t)wire_regs[i + 1U];
	}

	command_only = (startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) &&
				   (registercnt == REG_STRIDE);
	command_value = ((uint32_t)wire_regs[0] << 16) | (uint32_t)wire_regs[1];
	if (command_only && !LtdModbus_CommandIsImplemented(command_value)) {
		return false;
	}
	ret = CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
									startadd,
									registercnt,
									host_values);
	if (!ret) {
		return false;
	}

	if (command_only) {
		g_deviceParams.command = (CommandType)command_value;
		WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	} else {
		CPU2_CommRequestParameterRefresh();
	}
	return true;
}

/* 已发起的非命令写失败时，CPU2 是否实际应用无法由响应确定，必须重新确认参数。 */
static bool CPU2_CommFinishFailedRequest(bool parameter_write_attempted)
{
	CPU2_CommRecordFailure(s_cpu2_response_failure_reason);
	CPU2_CommLogRequestFailure();
	if (parameter_write_attempted) {
		CPU2_CommRequestParameterRefresh();
	}
	return false;
}

/*
 * 函数用途：重置固定点私有读取阶段，并按需使已确认快照失效。
 * 调用场景：冷启动、通信故障、三阶段任一读取失败或前后计数变化时。
 * 关键约束：候选缓冲从不直接暴露；失效后必须完成一轮全新握手才能再次开放。
 */
static void CPU2_ResetFixedPointSnapshotHandshake(bool invalidate_snapshot)
{
	s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_COUNTER_BEFORE;
	s_cpu2_private_input_destination = NULL;
	s_cpu2_private_input_capacity = 0U;
	s_cpu2_private_input_start = 0U;
	s_cpu2_private_input_captured = false;
	memset(s_cpu2_fixed_point_results, 0, sizeof(s_cpu2_fixed_point_results));
	memset(s_cpu2_fixed_point_counter_before, 0, sizeof(s_cpu2_fixed_point_counter_before));
	memset(s_cpu2_fixed_point_counter_after, 0, sizeof(s_cpu2_fixed_point_counter_after));
	if (invalidate_snapshot) {
		s_cpu2_has_fixed_point_snapshot = false;
	}
}

/*
 * 函数用途：把一次CPU2输入寄存器响应捕获到调用方私有缓冲，不写公开缓存。
 * 调用场景：固定点计数前读、24个结果寄存器候选读取和计数后读。
 * 关键约束：发送入口同步返回，退出前无条件撤销捕获目标，失败不得留下半帧目的地。
 */
static bool CPU2_ReadInputRegistersPrivate(uint16_t startadd,
											uint16_t registercnt,
											uint16_t *out_regs)
{
	bool request_ok;
	bool captured;

	if ((out_regs == NULL) || (registercnt == 0U) ||
		(startadd >= INPUTREGISTER_AMOUNT) ||
		(registercnt > (uint16_t)(INPUTREGISTER_AMOUNT - startadd)) ||
		(s_cpu2_private_input_destination != NULL)) {
		return false;
	}

	s_cpu2_private_input_destination = out_regs;
	s_cpu2_private_input_capacity = registercnt;
	s_cpu2_private_input_start = startadd;
	s_cpu2_private_input_captured = false;
	request_ok = CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,
												 startadd,
												 registercnt,
												 NULL);
	captured = s_cpu2_private_input_captured;
	s_cpu2_private_input_destination = NULL;
	s_cpu2_private_input_capacity = 0U;
	s_cpu2_private_input_start = 0U;
	s_cpu2_private_input_captured = false;
	return request_ok && captured;
}

static uint32_t CPU2_ReadU32FromPrivateRegs(const uint16_t *regs, uint16_t offset)
{
	return ((uint32_t)regs[offset] << 16) | (uint32_t)regs[offset + 1U];
}

/*
 * 函数用途：把已验证同代的固定点结果块与两个代际一次提交到公开缓存。
 * 调用场景：计数后读与计数前读完全一致后。
 * 关键约束：公开寄存器和g_measurement在同一短临界区更新，外部读者不会看到六字段混代。
 */
static void CPU2_CommitFixedPointSnapshot(void)
{
	uint32_t primask;
	uint32_t measurement_counter = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_counter_after, 0U);
	uint32_t monitoring_counter = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_counter_after, REG_SIZE_U32);

	primask = __get_PRIMASK();
	__disable_irq();
	memcpy(&InputRegisterArray[REG_SINGLE_POINT_MEAS_TEMP],
		   s_cpu2_fixed_point_results,
		   sizeof(s_cpu2_fixed_point_results));
	memcpy(&InputRegisterArray[REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER],
		   s_cpu2_fixed_point_counter_after,
		   sizeof(s_cpu2_fixed_point_counter_after));

	g_measurement.single_point_measurement.temperature = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 0U);
	g_measurement.single_point_measurement.density = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 2U);
	g_measurement.single_point_measurement.temperature_position = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 4U);
	g_measurement.single_point_measurement.standard_density = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 6U);
	g_measurement.single_point_measurement.vcf20 = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 8U);
	g_measurement.single_point_measurement.weight_density = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 10U);
	g_measurement.single_point_monitoring.temperature = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_fixed_point_results, REG_SINGLE_POINT_MON_TEMP - REG_SINGLE_POINT_MEAS_TEMP);
	g_measurement.single_point_monitoring.density = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_fixed_point_results, REG_SINGLE_POINT_MON_DENSITY - REG_SINGLE_POINT_MEAS_TEMP);
	g_measurement.single_point_monitoring.temperature_position = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_fixed_point_results, REG_SINGLE_POINT_MON_TEMP_POS - REG_SINGLE_POINT_MEAS_TEMP);
	g_measurement.single_point_monitoring.standard_density = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_fixed_point_results, REG_SINGLE_POINT_MON_STD_DENSITY - REG_SINGLE_POINT_MEAS_TEMP);
	g_measurement.single_point_monitoring.vcf20 = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_fixed_point_results, REG_SINGLE_POINT_MON_VCF20 - REG_SINGLE_POINT_MEAS_TEMP);
	g_measurement.single_point_monitoring.weight_density = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_fixed_point_results, REG_SINGLE_POINT_MON_WEIGHT_DENSITY - REG_SINGLE_POINT_MEAS_TEMP);
	g_measurement.measurement_complete_counter = measurement_counter;
	g_measurement.monitoring_sample_counter = monitoring_counter;
	s_cpu2_has_fixed_point_snapshot = true;
	__DMB();
	if (primask == 0U) {
		__enable_irq();
	}
}

/*
 * 函数用途：每次调用推进固定点快照握手的一个Modbus请求。
 * 调用场景：上电全量同步尾部，以及每轮普通运行轮询之后。
 * 关键约束：顺序固定为计数前读、私有结果读、计数后读；失败或变化仅丢弃本轮候选，
 *           已发布快照继续有效，连续三次通信失败由全局恢复策略统一失效。
 */
static bool CPU2_PollFixedPointSnapshotHandshake(void)
{
	switch (s_cpu2_fixed_point_stage) {
	case CPU2_FIXED_POINT_READ_COUNTER_BEFORE:
		if (!CPU2_ReadInputRegistersPrivate(REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER,
											 CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT,
											 s_cpu2_fixed_point_counter_before)) {
			CPU2_ResetFixedPointSnapshotHandshake(false);
			return false;
		}
		s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_RESULTS;
		return false;

	case CPU2_FIXED_POINT_READ_RESULTS:
		if (!CPU2_ReadInputRegistersPrivate(REG_SINGLE_POINT_MEAS_TEMP,
											 CPU2_FIXED_POINT_RESULT_REGISTER_COUNT,
											 s_cpu2_fixed_point_results)) {
			CPU2_ResetFixedPointSnapshotHandshake(false);
			return false;
		}
		s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_COUNTER_AFTER;
		return false;

	case CPU2_FIXED_POINT_READ_COUNTER_AFTER:
		if (!CPU2_ReadInputRegistersPrivate(REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER,
											 CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT,
											 s_cpu2_fixed_point_counter_after)) {
			CPU2_ResetFixedPointSnapshotHandshake(false);
			return false;
		}
		if (memcmp(s_cpu2_fixed_point_counter_before,
				   s_cpu2_fixed_point_counter_after,
				   sizeof(s_cpu2_fixed_point_counter_before)) != 0) {
			CPU2_ResetFixedPointSnapshotHandshake(false);
			return false;
		}
		CPU2_CommitFixedPointSnapshot();
		CPU2_ResetFixedPointSnapshotHandshake(false);
		return true;

	default:
		CPU2_ResetFixedPointSnapshotHandshake(true);
		return false;
	}
}

static void CPU2_ProfileWriteU32ToInput(uint16_t address, uint32_t value)
{
	InputRegisterArray[address] = (uint16_t)(value >> 16);
	InputRegisterArray[address + 1U] = (uint16_t)value;
}

static bool CPU2_ProfileTickReached(uint32_t now, uint32_t due)
{
	return ((int32_t)(now - due) >= 0);
}

static bool CPU2_ProfileIsCompleteState(DeviceState state)
{
	switch (state) {
	case STATE_WARTSILA_DENSITY_OVER:
	case STATE_SPREADPOINTOVER:
	case STATE_GB_SPREADPOINTOVER:
	case STATE_SYNTHETICING_OVER:
	case STATE_COM_METER_DENSITY_OVER:
	case STATE_INTERVAL_DENSITY_OVER:
		return true;
	default:
		return false;
	}
}

static DeviceState CPU2_ProfileBusyStateFromComplete(DeviceState state)
{
	switch (state) {
	case STATE_WARTSILA_DENSITY_OVER:
		return STATE_WARTSILA_DENSITY_MEASURING;
	case STATE_GB_SPREADPOINTOVER:
		return STATE_GB_SPREADPOINTING;
	case STATE_SYNTHETICING_OVER:
		return STATE_SYNTHETICING;
	case STATE_COM_METER_DENSITY_OVER:
		return STATE_METER_DENSITY;
	case STATE_INTERVAL_DENSITY_OVER:
		return STATE_INTERVAL_DENSITY;
	case STATE_SPREADPOINTOVER:
	default:
		return STATE_SPREADPOINTING;
	}
}

static Cpu2ProfileSnapshotKey CPU2_ProfileKeyFromHeader(const uint16_t *header)
{
	Cpu2ProfileSnapshotKey key;

	key.complete_latched = CPU2_ReadU32FromPrivateRegs(
		header,
		(uint16_t)(REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED - REG_DENSITY_DIST_AVG_TEMP));
	key.complete_counter = CPU2_ReadU32FromPrivateRegs(
		header,
		(uint16_t)(REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER - REG_DENSITY_DIST_AVG_TEMP));
	key.measurement_points = CPU2_ReadU32FromPrivateRegs(
		header,
		(uint16_t)(REG_DENSITY_DIST_MEAS_POINTS - REG_DENSITY_DIST_AVG_TEMP));
	key.profile_source = CPU2_ReadU32FromPrivateRegs(
		header,
		(uint16_t)(REG_DENSITY_DIST_PROFILE_SOURCE - REG_DENSITY_DIST_AVG_TEMP));
	return key;
}

static bool CPU2_ProfileKeyIsValid(const Cpu2ProfileSnapshotKey *key)
{
	return (key->complete_latched != 0U) &&
		   (key->measurement_points > 0U) &&
		   (key->measurement_points <= MAX_MEASUREMENT_POINTS) &&
		   (key->profile_source >= (uint32_t)PROFILE_SOURCE_STANDARD) &&
		   (key->profile_source <= (uint32_t)PROFILE_SOURCE_SI);
}

static bool CPU2_ProfileKeyEqual(const Cpu2ProfileSnapshotKey *left,
								 const Cpu2ProfileSnapshotKey *right)
{
	return (left->complete_latched == right->complete_latched) &&
		   (left->complete_counter == right->complete_counter) &&
		   (left->measurement_points == right->measurement_points) &&
		   (left->profile_source == right->profile_source);
}

/*
 * 函数用途：清除一次点阵最终失败留下的完整代际和自动重试计时。
 * 调用场景：通信会话失效、新代际出现或候选最终同步成功后。
 * 关键约束：只清CPU3本机失败记录，不修改CPU2原始状态和已发布点阵。
 */
static void CPU2_ProfileClearFailedKey(void)
{
	memset(&s_cpu2_profile_failed_key, 0, sizeof(s_cpu2_profile_failed_key));
	s_cpu2_profile_failed_key_valid = false;
	s_cpu2_profile_failed_snapshot_generation = 0U;
	s_cpu2_profile_failed_retry_due_tick = 0U;
}

/*
 * 函数用途：把CPU2最近一次原始状态恢复到CPU3公开状态区。
 * 调用场景：本机20-13撤销、点阵重试恢复或CPU2真实故障需要优先显示时。
 * 关键约束：不得用CPU3本机点阵故障继续遮盖CPU2发布的真实错误码。
 */
static void CPU2_ProfileRestoreRawStatus(void)
{
	g_measurement.device_status.device_state = s_cpu2_raw_device_state;
	g_measurement.device_status.error_code = s_cpu2_raw_error_code;
	CPU2_ProfileWriteU32ToInput(REG_DEVICE_STATUS_DEVICE_STATE,
							(uint32_t)s_cpu2_raw_device_state);
	CPU2_ProfileWriteU32ToInput(REG_DEVICE_STATUS_ERROR_CODE,
							s_cpu2_raw_error_code);
}

/*
 * 函数用途：同步期间恢复上一次已确认点阵，保持旧代际标识不变。
 * 调用场景：普通轮询先读到新完成头、候选点阵尚未通过前后复核时。
 * 关键约束：无历史快照时公开区清零；完成状态由独立门禁保持为忙。
 */
static void CPU2_ProfileRestorePublishedSnapshot(void)
{
	static const DensityDistribution empty_distribution = {0};

	if (s_cpu2_profile_published_valid) {
		g_measurement.density_distribution = s_cpu2_profile_published_distribution;
		memcpy(&InputRegisterArray[REG_DENSITY_DIST_AVG_TEMP],
			   s_cpu2_profile_published_header,
			   sizeof(s_cpu2_profile_published_header));
		memcpy(&InputRegisterArray[REG_DENSITY_DIST_POINT_BASE],
			   s_cpu2_profile_published_points,
			   sizeof(s_cpu2_profile_published_points));
	} else {
		g_measurement.density_distribution = empty_distribution;
		memset(&InputRegisterArray[REG_DENSITY_DIST_AVG_TEMP],
			   0,
			   sizeof(s_cpu2_profile_published_header));
		memset(&InputRegisterArray[REG_DENSITY_DIST_POINT_BASE],
			   0,
			   sizeof(s_cpu2_profile_published_points));
	}
}

/*
 * 函数用途：使CPU3已发布分布点阵失效，并清除依赖旧通信会话的状态门禁。
 * 调用场景：任一CPU2请求失败导致公开快照整体失效时。
 * 关键约束：恢复通信后必须重新执行完整头部和点阵复核，不能按重置后的相同计数复用旧结果。
 */
static void CPU2_ProfileInvalidatePublishedSnapshot(void)
{
	memset(&s_cpu2_profile_published_distribution, 0, sizeof(s_cpu2_profile_published_distribution));
	memset(&s_cpu2_profile_published_key, 0, sizeof(s_cpu2_profile_published_key));
	memset(s_cpu2_profile_published_header, 0, sizeof(s_cpu2_profile_published_header));
	memset(s_cpu2_profile_published_points, 0, sizeof(s_cpu2_profile_published_points));
	s_cpu2_profile_published_cycle = 0U;
	s_cpu2_profile_published_valid = false;
	CPU2_ProfileClearFailedKey();
	s_cpu2_profile_sync_fault_active = false;
	s_cpu2_profile_completion_state_pending = false;
	s_cpu2_profile_commit_waiting_for_complete_state = false;
	s_cpu2_last_raw_device_state_valid = false;
	CPU2_ProfileRestorePublishedSnapshot();
}

/*
 * 函数用途：取消已被CPU2正常作废的旧分布同步任务。
 * 调用场景：新测量、命令切换或Point0使完成锁存清零时。
 * 关键约束：正常作废不计入块重试、整轮重试或20-13最终故障。
 */
static void CPU2_ProfileCancelPendingSync(const char *reason)
{
	uint32_t complete_counter;

	if (s_cpu2_profile_sync.stage == CPU2_PROFILE_SYNC_IDLE) {
		return;
	}

	complete_counter = s_cpu2_profile_sync.target_key.complete_counter;
	memset(&s_cpu2_profile_sync, 0, sizeof(s_cpu2_profile_sync));
	memset(s_cpu2_profile_header_before, 0, sizeof(s_cpu2_profile_header_before));
	memset(s_cpu2_profile_header_after, 0, sizeof(s_cpu2_profile_header_after));
	memset(s_cpu2_profile_candidate_points, 0, sizeof(s_cpu2_profile_candidate_points));
	s_cpu2_profile_completion_state_pending = false;
	s_cpu2_profile_commit_waiting_for_complete_state = false;
	CPU3_LOG_INFO("CPU2分布",
				  "同步取消 完成计数=%lu 原因=%s",
				  (unsigned long)complete_counter,
				  reason);
}

static void CPU2_ProfileStart(const Cpu2ProfileSnapshotKey *observed)
{
	uint32_t now = HAL_GetTick();

	memset(&s_cpu2_profile_sync, 0, sizeof(s_cpu2_profile_sync));
	s_cpu2_profile_sync.stage = CPU2_PROFILE_SYNC_HEADER_BEFORE;
	s_cpu2_profile_sync.target_key = *observed;
	s_cpu2_profile_sync.snapshot_generation = s_cpu2_snapshot_generation;
	s_cpu2_profile_sync.started_tick = now;
	s_cpu2_profile_sync.last_progress_tick = now;
	s_cpu2_profile_sync.last_frame_tick = now - CPU2_PROFILE_FRAME_INTERVAL_MS;
	s_cpu2_profile_sync.round = 1U;
	memset(s_cpu2_profile_header_before, 0, sizeof(s_cpu2_profile_header_before));
	memset(s_cpu2_profile_header_after, 0, sizeof(s_cpu2_profile_header_after));
	memset(s_cpu2_profile_candidate_points, 0, sizeof(s_cpu2_profile_candidate_points));
}

static void CPU2_ProfilePublishError(void)
{
	uint32_t now = HAL_GetTick();

	s_cpu2_profile_failed_key = s_cpu2_profile_sync.target_key;
	s_cpu2_profile_failed_key_valid = true;
	s_cpu2_profile_failed_snapshot_generation = s_cpu2_profile_sync.snapshot_generation;
	s_cpu2_profile_failed_retry_due_tick = now + CPU2_PROFILE_FAILED_RETRY_DELAY_MS;
	s_cpu2_profile_sync_fault_active = true;
	s_cpu2_profile_sync.stage = CPU2_PROFILE_SYNC_IDLE;
	if (s_cpu2_raw_device_state == STATE_ERROR) {
		/* CPU2真实故障优先，20-13只保留为CPU3本机同步状态。 */
		CPU2_ProfileRestoreRawStatus();
	} else {
		g_measurement.device_status.device_state = STATE_ERROR;
		g_measurement.device_status.error_code = CPU2_PROFILE_SYNC_FAILED;
		CPU2_ProfileWriteU32ToInput(REG_DEVICE_STATUS_DEVICE_STATE, (uint32_t)STATE_ERROR);
		CPU2_ProfileWriteU32ToInput(REG_DEVICE_STATUS_ERROR_CODE, (uint32_t)CPU2_PROFILE_SYNC_FAILED);
	}
	CPU3_LOG_CRITICAL("CPU2分布",
					  "同步最终失败 完成计数=%lu 点数=%lu 来源=%lu 会话=%lu 已尝试=%u轮 耗时=%lums",
					  (unsigned long)s_cpu2_profile_failed_key.complete_counter,
					  (unsigned long)s_cpu2_profile_failed_key.measurement_points,
					  (unsigned long)s_cpu2_profile_failed_key.profile_source,
					  (unsigned long)s_cpu2_profile_failed_snapshot_generation,
					  (unsigned)s_cpu2_profile_sync.round,
					  (unsigned long)(now - s_cpu2_profile_sync.started_tick));
}

static void CPU2_ProfileScheduleRoundRetry(const char *reason)
{
	uint32_t now = HAL_GetTick();

	if (s_cpu2_profile_sync.round >= CPU2_PROFILE_ROUND_MAX) {
		CPU2_ProfilePublishError();
		return;
	}

	s_cpu2_profile_sync.round++;
	s_cpu2_profile_sync.block_retry_count = 0U;
	s_cpu2_profile_sync.point_register_offset = 0U;
	s_cpu2_profile_sync.retry_stage = CPU2_PROFILE_SYNC_HEADER_BEFORE;
	s_cpu2_profile_sync.retry_due_tick = now + CPU2_PROFILE_ROUND_RETRY_DELAY_MS;
	s_cpu2_profile_sync.stage = CPU2_PROFILE_SYNC_RETRY_WAIT;
	memset(s_cpu2_profile_header_before, 0, sizeof(s_cpu2_profile_header_before));
	memset(s_cpu2_profile_header_after, 0, sizeof(s_cpu2_profile_header_after));
	memset(s_cpu2_profile_candidate_points, 0, sizeof(s_cpu2_profile_candidate_points));
	CPU3_LOG_WARNING("CPU2分布",
					 "同步整轮重试 原因=%s 下一轮=%u/%u",
					 reason,
					 (unsigned)s_cpu2_profile_sync.round,
					 (unsigned)CPU2_PROFILE_ROUND_MAX);
}

static void CPU2_ProfileHandleBlockFailure(Cpu2ProfileSyncStage failed_stage)
{
	static const uint32_t retry_delays_ms[CPU2_PROFILE_BLOCK_RETRY_MAX] = {100U, 300U, 1000U};
	uint32_t now = HAL_GetTick();

	if ((now - s_cpu2_profile_sync.last_progress_tick) >=
		CPU2_PROFILE_NO_PROGRESS_TIMEOUT_MS) {
		CPU2_ProfilePublishError();
		return;
	}

	if (s_cpu2_profile_sync.block_retry_count < CPU2_PROFILE_BLOCK_RETRY_MAX) {
		uint8_t retry_index = s_cpu2_profile_sync.block_retry_count;

		s_cpu2_profile_sync.block_retry_count++;
		s_cpu2_profile_sync.retry_stage = failed_stage;
		s_cpu2_profile_sync.retry_due_tick = now + retry_delays_ms[retry_index];
		s_cpu2_profile_sync.stage = CPU2_PROFILE_SYNC_RETRY_WAIT;
		CPU3_LOG_WARNING("CPU2分布",
						 "同步块重试 阶段=%u 尝试=%u/%u 等待=%lums",
						 (unsigned)failed_stage,
						 (unsigned)s_cpu2_profile_sync.block_retry_count,
						 (unsigned)CPU2_PROFILE_BLOCK_RETRY_MAX,
						 (unsigned long)retry_delays_ms[retry_index]);
		return;
	}

	CPU2_ProfileScheduleRoundRetry("单块重试耗尽");
}

static void CPU2_ProfileMarkBlockSuccess(void)
{
	s_cpu2_profile_sync.last_progress_tick = HAL_GetTick();
	if (s_cpu2_profile_sync.block_retry_count != 0U) {
		CPU3_LOG_INFO("CPU2分布",
					  "同步块重试成功 尝试=%u/%u",
					  (unsigned)s_cpu2_profile_sync.block_retry_count,
					  (unsigned)CPU2_PROFILE_BLOCK_RETRY_MAX);
	}
	s_cpu2_profile_sync.block_retry_count = 0U;
}

static void CPU2_ProfileBuildCandidate(void)
{
	DensityDistribution *candidate = &s_cpu2_profile_candidate_distribution;
	uint16_t i;

	memset(candidate, 0, sizeof(*candidate));
	candidate->average_temperature = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_AVG_TEMP - REG_DENSITY_DIST_AVG_TEMP));
	candidate->average_density = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_AVG_DENSITY - REG_DENSITY_DIST_AVG_TEMP));
	candidate->average_standard_density = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_AVG_STD_DENSITY - REG_DENSITY_DIST_AVG_TEMP));
	candidate->average_vcf20 = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_AVG_VCF20 - REG_DENSITY_DIST_AVG_TEMP));
	candidate->average_weight_density = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_AVG_WEIGHT_DENSITY - REG_DENSITY_DIST_AVG_TEMP));
	candidate->measurement_points = s_cpu2_profile_sync.target_key.measurement_points;
	candidate->Density_oil_level = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_OIL_LEVEL - REG_DENSITY_DIST_AVG_TEMP));
	candidate->profile_complete_latched = s_cpu2_profile_sync.target_key.complete_latched;
	candidate->profile_complete_counter = s_cpu2_profile_sync.target_key.complete_counter;
	candidate->profile_source = s_cpu2_profile_sync.target_key.profile_source;
	candidate->profile_blocked_by_process = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS - REG_DENSITY_DIST_AVG_TEMP));
	candidate->profile_temp_deviation_alarm = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM - REG_DENSITY_DIST_AVG_TEMP));
	candidate->profile_density_deviation_alarm = CPU2_ReadU32FromPrivateRegs(
		s_cpu2_profile_header_after,
		(uint16_t)(REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM - REG_DENSITY_DIST_AVG_TEMP));

	for (i = 0U; i < (uint16_t)candidate->measurement_points; i++) {
		uint16_t point_offset = (uint16_t)(i * REG_DENSITY_DIST_POINT_SIZE);
		DensityMeasurement *point = &candidate->single_density_data[i];

		point->temperature = CPU2_ReadU32FromPrivateRegs(s_cpu2_profile_candidate_points, point_offset);
		point->density = CPU2_ReadU32FromPrivateRegs(s_cpu2_profile_candidate_points, (uint16_t)(point_offset + 2U));
		point->temperature_position = CPU2_ReadU32FromPrivateRegs(s_cpu2_profile_candidate_points, (uint16_t)(point_offset + 4U));
		point->standard_density = CPU2_ReadU32FromPrivateRegs(s_cpu2_profile_candidate_points, (uint16_t)(point_offset + 6U));
		point->vcf20 = CPU2_ReadU32FromPrivateRegs(s_cpu2_profile_candidate_points, (uint16_t)(point_offset + 8U));
		point->weight_density = CPU2_ReadU32FromPrivateRegs(s_cpu2_profile_candidate_points, (uint16_t)(point_offset + 10U));
	}
}

/*
 * 函数用途：把前后代际一致的候选头和N个点一次提交为CPU3公开快照。
 * 调用场景：HEADER_AFTER确认完成计数、点数、来源和完成锁存均未变化后。
 * 关键约束：N之后的公开尾部必须清零；完成态与点阵在同一临界区开放。
 */
static void CPU2_ProfileCommitCandidate(void)
{
	DensityDistribution *candidate = &s_cpu2_profile_candidate_distribution;
	uint32_t point_register_count;
	uint32_t primask;
	bool recovered_from_sync_fault = s_cpu2_profile_sync_fault_active;

	CPU2_ProfileBuildCandidate();
	point_register_count = candidate->measurement_points * (uint32_t)REG_DENSITY_DIST_POINT_SIZE;
	s_cpu2_profile_published_distribution = *candidate;
	s_cpu2_profile_published_key = s_cpu2_profile_sync.target_key;
	s_cpu2_profile_published_cycle = g_measurement.si_profile_runtime.cycle_counter;
	memcpy(s_cpu2_profile_published_header,
		   s_cpu2_profile_header_after,
		   sizeof(s_cpu2_profile_published_header));
	memset(s_cpu2_profile_published_points, 0, sizeof(s_cpu2_profile_published_points));
	memcpy(s_cpu2_profile_published_points,
		   s_cpu2_profile_candidate_points,
		   (size_t)point_register_count * sizeof(uint16_t));

	primask = __get_PRIMASK();
	__disable_irq();
	g_measurement.density_distribution = *candidate;
	memcpy(&InputRegisterArray[REG_DENSITY_DIST_AVG_TEMP],
		   s_cpu2_profile_published_header,
		   sizeof(s_cpu2_profile_published_header));
	memset(&InputRegisterArray[REG_DENSITY_DIST_POINT_BASE],
		   0,
		   sizeof(s_cpu2_profile_published_points));
	memcpy(&InputRegisterArray[REG_DENSITY_DIST_POINT_BASE],
		   s_cpu2_profile_published_points,
		   (size_t)point_register_count * sizeof(uint16_t));
	if (recovered_from_sync_fault ||
		(s_cpu2_profile_completion_state_pending &&
		 CPU2_ProfileIsCompleteState(s_cpu2_raw_device_state))) {
		CPU2_ProfileRestoreRawStatus();
	}
	s_cpu2_profile_completion_state_pending = false;
	s_cpu2_profile_commit_waiting_for_complete_state =
		!CPU2_ProfileIsCompleteState(s_cpu2_raw_device_state);
	s_cpu2_profile_published_valid = true;
	s_cpu2_profile_sync_fault_active = false;
	CPU2_ProfileClearFailedKey();
	s_cpu2_profile_sync.stage = CPU2_PROFILE_SYNC_IDLE;
	__DMB();
	if (primask == 0U) {
		__enable_irq();
	}

	CPU3_LOG_INFO("CPU2分布",
				  "同步完成 完成计数=%lu 点数=%lu 轮次=%u",
				  (unsigned long)candidate->profile_complete_counter,
				  (unsigned long)candidate->measurement_points,
				  (unsigned)s_cpu2_profile_sync.round);
	if (recovered_from_sync_fault) {
		CPU3_LOG_INFO("CPU2分布", "同代自动重试成功，20-13已清除");
	}
}

/*
 * 函数用途：每次主循环调用最多推进一个分布点阵Modbus请求。
 * 调用场景：普通轮询前优先调度，覆盖普通、国标、每米、区间、瓦锡兰、综合和SI结果。
 * 关键约束：每帧固定最多8点，失败按100/300/1000ms重试；三轮耗尽或20秒无进展才报错。
 */
static bool CPU2_ProfileSyncPoll(void)
{
	Cpu2ProfileSnapshotKey before;
	Cpu2ProfileSnapshotKey after;
	Cpu2ProfileSnapshotKey restart_key;
	uint32_t now = HAL_GetTick();
	uint32_t total_registers;
	uint32_t remaining;
	uint16_t read_count;
	bool request_ok;

	if (s_cpu2_profile_sync.stage == CPU2_PROFILE_SYNC_IDLE) {
		return false;
	}
	if (s_cpu2_profile_sync.snapshot_generation != s_cpu2_snapshot_generation) {
		/* 通信会话变化后轮次、退避、偏移和计时全部归零，禁止沿用旧任务预算。 */
		restart_key = s_cpu2_profile_sync.target_key;
		CPU2_ProfileStart(&restart_key);
		now = HAL_GetTick();
	}
	if ((now - s_cpu2_profile_sync.last_progress_tick) >=
		CPU2_PROFILE_NO_PROGRESS_TIMEOUT_MS) {
		CPU2_ProfilePublishError();
		return true;
	}
	if (s_cpu2_profile_sync.stage == CPU2_PROFILE_SYNC_RETRY_WAIT) {
		if (!CPU2_ProfileTickReached(now, s_cpu2_profile_sync.retry_due_tick)) {
			/* 点阵退避期间不插入普通小帧，避免成功小帧打断本次连续失败判定。 */
			return true;
		}
		s_cpu2_profile_sync.stage = s_cpu2_profile_sync.retry_stage;
	}
	if ((now - s_cpu2_profile_sync.last_frame_tick) < CPU2_PROFILE_FRAME_INTERVAL_MS) {
		return true;
	}
	s_cpu2_profile_sync.last_frame_tick = now;

	switch (s_cpu2_profile_sync.stage) {
	case CPU2_PROFILE_SYNC_HEADER_BEFORE:
		request_ok = CPU2_ReadInputRegistersPrivate(REG_DENSITY_DIST_AVG_TEMP,
											 CPU2_PROFILE_HEADER_REGISTER_COUNT,
											 s_cpu2_profile_header_before);
		if (!request_ok) {
			CPU2_ProfileHandleBlockFailure(CPU2_PROFILE_SYNC_HEADER_BEFORE);
			return true;
		}
		CPU2_ProfileMarkBlockSuccess();
		before = CPU2_ProfileKeyFromHeader(s_cpu2_profile_header_before);
		if (!CPU2_ProfileKeyIsValid(&before)) {
			if (before.complete_latched == 0U) {
				CPU2_ProfileCancelPendingSync("完成锁存已清除");
				return true;
			}
			CPU2_ProfileScheduleRoundRetry("候选头无效");
			return true;
		}
		s_cpu2_profile_sync.target_key = before;
		s_cpu2_profile_sync.point_register_offset = 0U;
		memset(s_cpu2_profile_candidate_points, 0, sizeof(s_cpu2_profile_candidate_points));
		s_cpu2_profile_sync.stage = CPU2_PROFILE_SYNC_POINTS;
		return true;

	case CPU2_PROFILE_SYNC_POINTS:
		total_registers = s_cpu2_profile_sync.target_key.measurement_points *
			(uint32_t)REG_DENSITY_DIST_POINT_SIZE;
		remaining = total_registers - s_cpu2_profile_sync.point_register_offset;
		read_count = (remaining > CPU2_PROFILE_REGISTERS_PER_FRAME) ?
			(uint16_t)CPU2_PROFILE_REGISTERS_PER_FRAME : (uint16_t)remaining;
		request_ok = CPU2_ReadInputRegistersPrivate(
			(uint16_t)(REG_DENSITY_DIST_POINT_BASE + s_cpu2_profile_sync.point_register_offset),
			read_count,
			&s_cpu2_profile_candidate_points[s_cpu2_profile_sync.point_register_offset]);
		if (!request_ok) {
			CPU2_ProfileHandleBlockFailure(CPU2_PROFILE_SYNC_POINTS);
			return true;
		}
		CPU2_ProfileMarkBlockSuccess();
		s_cpu2_profile_sync.point_register_offset += read_count;
		if (s_cpu2_profile_sync.point_register_offset >= total_registers) {
			s_cpu2_profile_sync.stage = CPU2_PROFILE_SYNC_HEADER_AFTER;
		}
		return true;

	case CPU2_PROFILE_SYNC_HEADER_AFTER:
		request_ok = CPU2_ReadInputRegistersPrivate(REG_DENSITY_DIST_AVG_TEMP,
											 CPU2_PROFILE_HEADER_REGISTER_COUNT,
											 s_cpu2_profile_header_after);
		if (!request_ok) {
			CPU2_ProfileHandleBlockFailure(CPU2_PROFILE_SYNC_HEADER_AFTER);
			return true;
		}
		CPU2_ProfileMarkBlockSuccess();
		after = CPU2_ProfileKeyFromHeader(s_cpu2_profile_header_after);
		if (!CPU2_ProfileKeyIsValid(&after)) {
			if (after.complete_latched == 0U) {
				CPU2_ProfileCancelPendingSync("完成锁存已清除");
				return true;
			}
			CPU2_ProfileScheduleRoundRetry("候选头无效");
			return true;
		}
		if (!CPU2_ProfileKeyEqual(&s_cpu2_profile_sync.target_key, &after)) {
			CPU2_ProfileScheduleRoundRetry("前后代际不一致");
			return true;
		}
		CPU2_ProfileCommitCandidate();
		return true;

	case CPU2_PROFILE_SYNC_RETRY_WAIT:
	case CPU2_PROFILE_SYNC_IDLE:
	default:
		return false;
	}
}

/*
 * 函数用途：观察普通轮询读到的分布头，并在新完成代际出现时启动私有候选同步。
 * 调用场景：公开0x04响应完整覆盖分布头后、对外返回主循环前。
 * 关键约束：新头只作为触发证据；候选完成前立即恢复上次已发布代际。
 */
static void CPU2_ProfileObservePublicHeader(void)
{
	Cpu2ProfileSnapshotKey observed = CPU2_ProfileKeyFromHeader(
		&InputRegisterArray[REG_DENSITY_DIST_AVG_TEMP]);
	uint32_t now = HAL_GetTick();
	bool observed_valid = CPU2_ProfileKeyIsValid(&observed);
	bool already_published = s_cpu2_profile_published_valid &&
		CPU2_ProfileKeyEqual(&observed, &s_cpu2_profile_published_key);
	bool terminal_failed = s_cpu2_profile_failed_key_valid &&
		(s_cpu2_profile_failed_snapshot_generation == s_cpu2_snapshot_generation) &&
		CPU2_ProfileKeyEqual(&observed, &s_cpu2_profile_failed_key);
	bool failed_retry_due = terminal_failed &&
		CPU2_ProfileTickReached(now, s_cpu2_profile_failed_retry_due_tick);
	bool sync_canceled = false;

	if (observed.complete_latched == 0U) {
		s_cpu2_profile_commit_waiting_for_complete_state = false;
		if (s_cpu2_profile_sync.stage != CPU2_PROFILE_SYNC_IDLE) {
			CPU2_ProfileCancelPendingSync("CPU2已作废本轮结果");
			sync_canceled = true;
		}
	}

	if (s_cpu2_profile_sync_fault_active &&
		((observed.complete_latched == 0U) || !terminal_failed)) {
		s_cpu2_profile_sync_fault_active = false;
		CPU2_ProfileClearFailedKey();
		CPU2_ProfileRestoreRawStatus();
		terminal_failed = false;
		failed_retry_due = false;
	}

	if (observed_valid && !already_published &&
		(!terminal_failed || failed_retry_due)) {
		if ((s_cpu2_profile_sync.stage == CPU2_PROFILE_SYNC_IDLE) ||
			(s_cpu2_profile_sync.snapshot_generation != s_cpu2_snapshot_generation) ||
			!CPU2_ProfileKeyEqual(&observed, &s_cpu2_profile_sync.target_key)) {
			if (failed_retry_due) {
				CPU3_LOG_INFO("CPU2分布",
							  "同代自动重试 完成计数=%lu 会话=%lu",
							  (unsigned long)observed.complete_counter,
							  (unsigned long)s_cpu2_snapshot_generation);
			}
			s_cpu2_profile_commit_waiting_for_complete_state = false;
			CPU2_ProfileStart(&observed);
		}
	}

	if ((s_cpu2_profile_sync.stage != CPU2_PROFILE_SYNC_IDLE) || terminal_failed || sync_canceled) {
		CPU2_ProfileRestorePublishedSnapshot();
	}
}

/*
 * 函数用途：在点阵提交前把CPU2原始分布完成态转换为对应测量中状态。
 * 调用场景：每次完整状态分组解析后。
 * 关键约束：CPU2真实故障优先于本机20-13；成功提交后才恢复CPU2原始完成态。
 */
static void CPU2_ProfileApplyPublicStatusGate(void)
{
	DeviceState parsed_state = g_measurement.device_status.device_state;

	s_cpu2_raw_device_state = parsed_state;
	s_cpu2_raw_error_code = g_measurement.device_status.error_code;
	if (CPU2_ProfileIsCompleteState(parsed_state) &&
		(!s_cpu2_last_raw_device_state_valid ||
		 !CPU2_ProfileIsCompleteState(s_cpu2_last_raw_device_state))) {
		if (s_cpu2_profile_commit_waiting_for_complete_state) {
			s_cpu2_profile_completion_state_pending = false;
			s_cpu2_profile_commit_waiting_for_complete_state = false;
		} else {
			s_cpu2_profile_completion_state_pending = true;
		}
	}
	s_cpu2_last_raw_device_state = parsed_state;
	s_cpu2_last_raw_device_state_valid = true;

	if (s_cpu2_profile_sync_fault_active && (parsed_state != STATE_ERROR)) {
		g_measurement.device_status.device_state = STATE_ERROR;
		g_measurement.device_status.error_code = CPU2_PROFILE_SYNC_FAILED;
		CPU2_ProfileWriteU32ToInput(REG_DEVICE_STATUS_DEVICE_STATE, (uint32_t)STATE_ERROR);
		CPU2_ProfileWriteU32ToInput(REG_DEVICE_STATUS_ERROR_CODE, (uint32_t)CPU2_PROFILE_SYNC_FAILED);
		return;
	}

	if (s_cpu2_profile_completion_state_pending && CPU2_ProfileIsCompleteState(parsed_state)) {
		DeviceState busy_state = CPU2_ProfileBusyStateFromComplete(parsed_state);

		g_measurement.device_status.device_state = busy_state;
		CPU2_ProfileWriteU32ToInput(REG_DEVICE_STATUS_DEVICE_STATE, (uint32_t)busy_state);
	}
}

/* 与CPU2通讯接收包主处理过程 */
bool HostCommuProcess(uint8_t *rcv, int len) {
	Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG,
				  "CPU2",
				  "接收",
				  rcv,
				  (len > 0) ? (uint16_t)len : 0U);
	s_cpu2_response_failure_reason = CPU2_COMM_FAIL_NONE;
	if (len <= 3) {
		s_cpu2_response_failure_reason = CPU2_COMM_FAIL_LENGTH;
		return false;
	}
	if (!SlaveCheckCRC(rcv, len)) {
		s_cpu2_response_failure_reason = CPU2_COMM_FAIL_CRC;
		return false;
	}
	/* 解析数据 */
	if (rcv[0] != ADERSS) {
		s_cpu2_response_failure_reason = CPU2_COMM_FAIL_ADDRESS;
		return false;
	}
	s_cpu2_response_failure_reason = CPU2_ResponseFrameFailureReason(rcv, len);
	if (s_cpu2_response_failure_reason != CPU2_COMM_FAIL_NONE) {
		return false;
	}
	CPU2_CommMarkValidResponse();
	switch (RCV_functioncode) {
	case FUNCTIONCODE_READ_HOLDREGISTER: {
		CPU2_Response03Process(rcv);
		break;
	}
	case FUNCTIONCODE_READ_INPUTREGISTER: {
		CPU2_Response04Process(rcv);
		break;
	}
	case FUNCTIONCODE_WRITE_MULREGISTER: {
		CPU2_Response10Process(rcv, len);
		break;
	}
	}
	return true;
}

typedef struct {
	uint8_t func;     /* 功能码：3 或 4 */
	uint16_t start;   /* 起始寄存器 */
	uint16_t len;     /* 寄存器个数 */
} PollGroup;

/* 轮询表直接使用stateformodbus.h中的现行Modbus地址。 */
static const PollGroup poweron_groups[] = {
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_DEVICE_STATUS_WORK_MODE,
	 REG_DEVICE_STATUS_BLOCK_END - REG_DEVICE_STATUS_WORK_MODE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_DEBUG_BASE,
	 REG_DEBUG_BLOCK_END - REG_DEBUG_BASE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_OIL_MEASUREMENT_OIL_LEVEL,
	 REG_PROCESS_BLOCK_END - REG_OIL_MEASUREMENT_OIL_LEVEL},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_SINGLE_POINT_MEAS_TEMP,
	 REG_FIXED_RESULT_BLOCK_END - REG_SINGLE_POINT_MEAS_TEMP},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_AO_OUTPUT_RUNTIME_BASE,
	 REG_AO_OUTPUT_RUNTIME_BLOCK_END - REG_AO_OUTPUT_RUNTIME_BASE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_RELAY_ALARM_RUNTIME_BASE,
	 REG_RELAY_ALARM_RUNTIME_BLOCK_END - REG_RELAY_ALARM_RUNTIME_BASE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_WIRELESS_PAIRING_RESULT,
	 REG_WIRELESS_PAIRING_BLOCK_END - REG_WIRELESS_PAIRING_RESULT},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_GENERAL,
	 HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT + REG_STRIDE - HOLDREG_BASE_GENERAL},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_MOTOR,
	 HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE + REG_STRIDE - HOLDREG_BASE_MOTOR},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_OIL_HEIGHT,
	 HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE + REG_STRIDE - HOLDREG_BASE_OIL_HEIGHT},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_WATER,
	 HOLDREGISTER_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD + REG_STRIDE - HOLDREG_BASE_WATER},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_CORRECTION,
	 HOLDREGISTER_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE + REG_STRIDE - HOLDREG_BASE_CORRECTION},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_MEAS_CONFIG,
	 HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE + REG_STRIDE - HOLDREG_BASE_MEAS_CONFIG},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_AO,
	 HOLDREGISTER_AO_SIMULATION_ENABLE + REG_STRIDE - HOLDREG_BASE_AO},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE,
	 HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_END - HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_SI_WARTSILA,
	 HOLDREGISTER_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL + REG_STRIDE - HOLDREG_BASE_SI_WARTSILA},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_RESERVED,
	 HOLDREGISTER_DEVICEPARAM_RESERVED29 + REG_STRIDE - HOLDREG_BASE_RESERVED},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_METADATA,
	 HOLDREGISTER_DEVICEPARAM_CRC + REG_STRIDE - HOLDREG_BASE_METADATA},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND,
	 HOLDREGISTER_PROTOCOL_CAPABILITIES + REG_STRIDE - HOLDREGISTER_DEVICEPARAM_COMMAND}
};

#define POWERON_GROUP_COUNT  (sizeof(poweron_groups) / sizeof(poweron_groups[0]))

/* 正常运行时轮询的组：
 * 只保留输入寄存器，不再读保持寄存器
 */
static const PollGroup runtime_groups[] = {
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_DEVICE_STATUS_WORK_MODE,
	 REG_DEVICE_STATUS_BLOCK_END - REG_DEVICE_STATUS_WORK_MODE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_DEBUG_BASE,
	 REG_DEBUG_BLOCK_END - REG_DEBUG_BASE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_OIL_MEASUREMENT_OIL_LEVEL,
	 REG_PROCESS_BLOCK_END - REG_OIL_MEASUREMENT_OIL_LEVEL},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_SINGLE_POINT_MEAS_TEMP,
	 REG_FIXED_RESULT_BLOCK_END - REG_SINGLE_POINT_MEAS_TEMP},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_AO_OUTPUT_RUNTIME_BASE,
	 REG_AO_OUTPUT_RUNTIME_BLOCK_END - REG_AO_OUTPUT_RUNTIME_BASE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_RELAY_ALARM_RUNTIME_BASE,
	 REG_RELAY_ALARM_RUNTIME_BLOCK_END - REG_RELAY_ALARM_RUNTIME_BASE},
	{FUNCTIONCODE_READ_INPUTREGISTER, REG_WIRELESS_PAIRING_RESULT,
	 REG_WIRELESS_PAIRING_BLOCK_END - REG_WIRELESS_PAIRING_RESULT}
};

#define RUNTIME_GROUP_COUNT  (sizeof(runtime_groups) / sizeof(runtime_groups[0]))

/* 参数更新时，一次性补读系统参数。 */
static const PollGroup refresh_hold_groups[] = {
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_GENERAL,
	 HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT + REG_STRIDE - HOLDREG_BASE_GENERAL},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_MOTOR,
	 HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE + REG_STRIDE - HOLDREG_BASE_MOTOR},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_OIL_HEIGHT,
	 HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE + REG_STRIDE - HOLDREG_BASE_OIL_HEIGHT},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_WATER,
	 HOLDREGISTER_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD + REG_STRIDE - HOLDREG_BASE_WATER},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_CORRECTION,
	 HOLDREGISTER_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE + REG_STRIDE - HOLDREG_BASE_CORRECTION},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_MEAS_CONFIG,
	 HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE + REG_STRIDE - HOLDREG_BASE_MEAS_CONFIG},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_AO,
	 HOLDREGISTER_AO_SIMULATION_ENABLE + REG_STRIDE - HOLDREG_BASE_AO},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE,
	 HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_END - HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_SI_WARTSILA,
	 HOLDREGISTER_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL + REG_STRIDE - HOLDREG_BASE_SI_WARTSILA},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_RESERVED,
	 HOLDREGISTER_DEVICEPARAM_RESERVED29 + REG_STRIDE - HOLDREG_BASE_RESERVED},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREG_BASE_METADATA,
	 HOLDREGISTER_DEVICEPARAM_CRC + REG_STRIDE - HOLDREG_BASE_METADATA},
	{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND,
	 HOLDREGISTER_PROTOCOL_CAPABILITIES + REG_STRIDE - HOLDREGISTER_DEVICEPARAM_COMMAND}
};

#define REFRESH_HOLD_GROUP_COUNT (sizeof(refresh_hold_groups) / sizeof(refresh_hold_groups[0]))

/* 轮询输入寄存器数据 */
void PollingInputData(void) {
	/* 上电阶段是否已经完成 */
	static bool poweron_done = false;
	static uint8_t poweron_index = 0;

	/* 正常轮询阶段当前组索引 */
	static uint8_t runtime_index = 0;
	static bool runtime_fixed_snapshot_pending = false;
	static bool hold_refresh_pending = false;
	static uint8_t hold_refresh_index = 0;
	static bool param_flag_valid = false;
	static uint32_t last_param_update_flag = 0;
	static uint32_t refresh_target_flag = 0;

	/* 连续第三次失败才废弃旧公开快照；下一轮从状态、参数和协议开始完整重同步。 */
	if (s_cpu2_snapshot_resync_requested) {
		poweron_done = false;
		poweron_index = 0;
		runtime_index = 0;
		runtime_fixed_snapshot_pending = false;
		hold_refresh_pending = false;
		hold_refresh_index = 0;
		param_flag_valid = false;
		s_cpu2_parameter_refresh_requested = false;
		CPU2_ResetFixedPointSnapshotHandshake(true);
		s_cpu2_snapshot_resync_requested = false;
	}

	/* 通信故障恢复必须先取得包含设备状态的 0x04 响应，禁止旧缓存提前清故障。 */
	if (s_cpu2_comm_fault_active) {
		const PollGroup *status_group = &runtime_groups[0];
		poweron_done = false;
		poweron_index = 0;
		runtime_fixed_snapshot_pending = false;
		CPU2_ResetFixedPointSnapshotHandshake(true);
		hold_refresh_pending = false;
		hold_refresh_index = 0;
		param_flag_valid = false;
		(void)CPU2_CombinatePackage_SendWire(status_group->func,
									   status_group->start,
									   status_group->len,
									   NULL);
		return;
	}

	/* 上电和恢复后先读取稳定的协议版本地址，旧协议未确认前不得访问新扩展区。 */
	if (!s_cpu2_has_protocol_snapshot) {
		(void)CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
										   HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION,
										   REG_SIZE_U32,
										   NULL);
		return;
	}

	/* 协议不匹配表示链路在线但共享契约不可用，只保留版本探测作为兼容状态心跳。 */
	if (!CPU2_CommIsProtocolCompatible()) {
		poweron_done = false;
		poweron_index = 0;
		runtime_index = 0;
		runtime_fixed_snapshot_pending = false;
		hold_refresh_pending = false;
		hold_refresh_index = 0;
		param_flag_valid = false;
		s_cpu2_has_parameter_snapshot = false;
		s_cpu2_parameter_refresh_requested = false;
		CPU2_ResetFixedPointSnapshotHandshake(true);
		(void)CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
										   HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION,
										   REG_SIZE_U32,
										   NULL);
		return;
	}

	/* ---------- 上电阶段：普通组完成后再执行固定点私有快照握手 ---------- */
	if (!poweron_done) {
		if (poweron_index < POWERON_GROUP_COUNT) {
			const PollGroup *g = &poweron_groups[poweron_index];

			if (!CPU2_CombinatePackage_SendWire(g->func, g->start, g->len, NULL)) {
				return;
			}
			poweron_index++;
			return;
		}

		if (!CPU2_PollFixedPointSnapshotHandshake()) {
			return;
		}

		poweron_done = true;
		s_cpu2_has_parameter_snapshot = true;
		s_cpu2_parameter_refresh_requested = false;
		runtime_fixed_snapshot_pending = false;
		if (s_cpu2_profile_sync.stage != CPU2_PROFILE_SYNC_IDLE) {
			Cpu2ProfileSnapshotKey restart_key = s_cpu2_profile_sync.target_key;

			/* 全量重同步完成后重新计算点阵轮次和超时，旧会话耗时不得带入新任务。 */
			CPU2_ProfileStart(&restart_key);
		}
		DeviceParams_StoreToRegisters(g_holding_regs);
		last_param_update_flag = g_measurement.device_status.parameter_update_flag;
		refresh_target_flag = last_param_update_flag;
		param_flag_valid = true;
		CPU3_LOG_INFO("CPU2",
					  "完整同步完成 协议=%lu 状态快照=1 参数快照=1 固定点快照=1",
					  (unsigned long)s_cpu2_protocol_version);
		return;
	}

	/* 上电尾部已取得SI生命周期后再推进分布候选；本次调用最多发送一个相关请求。 */
	if (CPU2_ProfileSyncPoll()) {
		return;
	}

	/* 外部参数写后不能依赖本地影子，必须主动重读确认 CPU2 实际值。 */
	if (s_cpu2_parameter_refresh_requested) {
		hold_refresh_pending = true;
		hold_refresh_index = 0;
		s_cpu2_parameter_refresh_requested = false;
	}

	/* ---------- 参数更新后：一次性补读保持寄存器 ---------- */
	if (hold_refresh_pending) {
		if (hold_refresh_index < REFRESH_HOLD_GROUP_COUNT) {
			const PollGroup *g = &refresh_hold_groups[hold_refresh_index];
			if (!CPU2_CombinatePackage_SendWire(g->func, g->start, g->len, NULL)) {
				return;
			}
			hold_refresh_index++;
		}

		if (hold_refresh_index >= REFRESH_HOLD_GROUP_COUNT) {
			hold_refresh_pending = false;
			hold_refresh_index = 0;
			last_param_update_flag = refresh_target_flag;
			param_flag_valid = true;
			DeviceParams_StoreToRegisters(g_holding_regs);
			s_cpu2_has_parameter_snapshot = true;
		}
		return;
	}

	/* ---------- 正常运行阶段：固定点握手与普通输入组交替推进 ---------- */
	if (runtime_fixed_snapshot_pending) {
		if (CPU2_PollFixedPointSnapshotHandshake()) {
			runtime_fixed_snapshot_pending = false;
		}
		return;
	}

	/* 如果某些状态不需要轮询，可以在这里加条件，例如：
	 * if (g_measurement.device_status.device_state == STATE_AI_SPREADPOINTOVER)
	 *     return;
	 */

	{
		const PollGroup *g = &runtime_groups[runtime_index];

		if (!CPU2_CombinatePackage_SendWire(g->func, g->start, g->len, NULL)) {
			return;
		}

		runtime_index++;
		if (runtime_index >= RUNTIME_GROUP_COUNT) {
			runtime_index = 0;
			runtime_fixed_snapshot_pending = CPU2_CommIsProtocolCompatible();
		}
	}

	if (!param_flag_valid) {
		last_param_update_flag = g_measurement.device_status.parameter_update_flag;
		refresh_target_flag = last_param_update_flag;
		param_flag_valid = true;
	} else if (g_measurement.device_status.parameter_update_flag != last_param_update_flag) {
		refresh_target_flag = g_measurement.device_status.parameter_update_flag;
		hold_refresh_pending = true;
		hold_refresh_index = 0;
		s_cpu2_has_parameter_snapshot = false;
		return;
	}

}

/*
 * 函数用途：判断SI点阵拉取时允许接受的生命周期阶段。
 * 调用场景：区分正常Complete候选和CPU3重启后Point0前遗留的上一轮快照。
 * 关键约束：上一轮快照只允许PREPARING、ABORTED或FAILED，不能放宽到Point0后的活动阶段。
 */
static bool CPU2_SiProfileFetchPhaseAllowed(uint32_t phase, bool previous_snapshot)
{
	if (!previous_snapshot) {
		return phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE;
	}

	return (phase == (uint32_t)SI_PROFILE_PHASE_PREPARING) ||
		   (phase == (uint32_t)SI_PROFILE_PHASE_ABORTED) ||
		   (phase == (uint32_t)SI_PROFILE_PHASE_FAILED);
}

/*
 * 函数用途：查询已经由统一异步状态机确认并发布的SI Profile点阵代际。
 * 调用场景：正常Complete候选发布，或CPU3重启后恢复Point0前保留的上一轮结果。
 * 关键约束：本函数不发起UART请求；异步同步未完成时返回false，调用方继续保持忙。
 */
static bool CPU2_CommGetPublishedSiProfile(Cpu2SiProfileCandidateKey *out_key,
											 bool previous_snapshot)
{
	uint32_t current_cycle;
	uint32_t current_phase;

	if ((out_key == NULL) ||
		(!CPU2_CommHasRuntimeSnapshot()) ||
		(!s_cpu2_profile_published_valid) ||
		(s_cpu2_profile_published_key.profile_source != (uint32_t)PROFILE_SOURCE_SI)) {
		return false;
	}

	current_cycle = g_measurement.si_profile_runtime.cycle_counter;
	current_phase = g_measurement.si_profile_runtime.phase;
	if ((current_cycle != s_cpu2_profile_published_cycle) ||
		(!CPU2_SiProfileFetchPhaseAllowed(current_phase, previous_snapshot))) {
		return false;
	}

	out_key->cycle_counter = current_cycle;
	out_key->complete_counter = s_cpu2_profile_published_key.complete_counter;
	out_key->measurement_points = s_cpu2_profile_published_key.measurement_points;
	out_key->profile_source = s_cpu2_profile_published_key.profile_source;
	out_key->phase = current_phase;
	return true;
}

/*
 * 函数用途：拉取CPU2当前Complete阶段的SI候选点阵。
 * 调用场景：CPU3观察到本周期完成计数沿后执行最终发布。
 * 关键约束：只接受Complete阶段，并沿用完整分块前后代际复核。
 */
bool CPU2_CommFetchSiProfileCandidate(Cpu2SiProfileCandidateKey *out_key)
{
	return CPU2_CommGetPublishedSiProfile(out_key, false);
}

/*
 * 函数用途：拉取CPU2在Point0前仍保留的上一轮完整SI点阵。
 * 调用场景：CPU3冷启动时处于PREPARING，或Point0前已经ABORTED/FAILED。
 * 关键约束：只接受旧SI来源、完成锁存、有效点数和稳定代际，不重建Profile时间。
 */
bool CPU2_CommFetchSiPreviousSnapshot(Cpu2SiProfileCandidateKey *out_key)
{
	return CPU2_CommGetPublishedSiProfile(out_key, true);
}

/*
 * 函数用途：使用stateformodbus.h中的现行地址发起CPU2标准Modbus事务。
 * 调用场景：菜单、周期轮询、快照握手和外部LTD网关共用。
 * 关键约束：读取按标准125寄存器上限拆帧；写入必须等待CPU2合法ACK。
 */
bool CPU2_CombinatePackage_Send(uint8_t f_code,
								uint16_t startadd,
								uint16_t registercnt,
								uint32_t *holddata)
{
	uint16_t processed = 0U;

	if ((registercnt == 0U) ||
		((f_code != FUNCTIONCODE_READ_HOLDREGISTER) &&
		 (f_code != FUNCTIONCODE_READ_INPUTREGISTER) &&
		 (f_code != FUNCTIONCODE_WRITE_MULREGISTER))) {
		return false;
	}
	if (f_code == FUNCTIONCODE_WRITE_MULREGISTER) {
		if ((registercnt > LTD_MODBUS_MAX_WRITE_REGISTERS) ||
			!LtdModbus_HoldingWriteRangeIsValid(startadd, registercnt)) {
			return false;
		}
		/*
		 * CPU3只用状态白名单提前抑制明显不安全的持久参数写；
		 * 当前命令、待执行命令和故障恢复活动仍由CPU2在FC10入口最终裁决。
		 */
		if (LtdModbus_HoldingWriteTouchesPersistent(startadd, registercnt) &&
			!DeviceState_AllowsPersistentParamWrite(
				g_measurement.device_status.device_state)) {
			return false;
		}
		return CPU2_CombinatePackage_SendWire(f_code, startadd, registercnt, holddata);
	}
	if ((f_code == FUNCTIONCODE_READ_HOLDREGISTER) &&
		!LtdModbus_RangeWithin(startadd, registercnt, HOLDREGISTER_AMOUNT)) {
		return false;
	}
	if ((f_code == FUNCTIONCODE_READ_INPUTREGISTER) &&
		!LtdModbus_RangeWithin(startadd, registercnt, INPUTREGISTER_AMOUNT)) {
		return false;
	}

	while (processed < registercnt) {
		uint16_t remaining = (uint16_t)(registercnt - processed);
		uint16_t chunk = (remaining > LTD_MODBUS_MAX_READ_REGISTERS) ?
			LTD_MODBUS_MAX_READ_REGISTERS : remaining;

		if (!CPU2_CombinatePackage_SendWire(f_code,
										(uint16_t)(startadd + processed),
										chunk,
										NULL)) {
			return false;
		}
		processed = (uint16_t)(processed + chunk);
	}
	return true;
}

/* 使用现行Modbus地址执行一次CPU2事务。 */
static bool CPU2_CombinatePackage_SendWire(uint8_t f_code,
										   uint16_t startadd,
										   uint16_t registercnt,
										   uint32_t *holddata) {
	bool cancel_command_allowed = false;
	bool parameter_write_attempted = false;
	bool command_write = (f_code == FUNCTIONCODE_WRITE_MULREGISTER) &&
		(startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) &&
		(registercnt == REG_STRIDE);

	/* 每个板间事务只在边界报告进度，等待响应循环内部不得给看门狗续命。 */
	CPU3_WatchdogReportProgress();

	/* 普通写必须通过完整快照门禁；参数刷新期间只放行协议兼容的取消命令。 */
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) &&
		command_write && (registercnt == 2U) &&
		(holddata != NULL)) {
		cancel_command_allowed = CPU2_CommCanSendCommand((CommandType)(*holddata));
	}
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) && !command_write) {
		parameter_write_attempted = true;
	}
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) && !CPU2_CommIsAvailable()) {
		if (!cancel_command_allowed) {
			CPU3_WatchdogReportProgress();
			return false;
		}
	}
	uint8_t arr[1024];
	int len = 0;
	uint16_t crc;
	int i;
	const uint16_t *regs = (const uint16_t*) holddata;   /* 关键修正：按16位寄存器解释 */
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
			uint16_t low_word = regs[i + 1]; /* 原本的高字 */
			uint16_t high_word = regs[i];     /* 原本的低字 */

			/* 低字先发（高字节→低字节） */
			arr[len++] = (uint8_t) (low_word >> 8);
			arr[len++] = (uint8_t) (low_word & 0xFF);

			/* 高字后发（高字节→低字节） */
			arr[len++] = (uint8_t) (high_word >> 8);
			arr[len++] = (uint8_t) (high_word & 0xFF);
		}
	}
	crc = CRC16_Calculate((u8*) arr, len);
	arr[len++] = crc & 0xff;
	arr[len++] = crc >> 8;
	Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG,
				  "CPU2",
				  "发送",
				  arr,
				  (uint16_t)len);
	/* 先发布本次期望字段，避免极快响应到达时仍沿用上一请求。 */
	RCV_functioncode = f_code;
	RCV_startaddress = startadd;
	RCV_registercnt = registercnt;
	UART5_RX_LEN = 0U;
	s_cpu2_uart_error_pending = HAL_UART_ERROR_NONE;
	s_cpu2_request_uart_error_code = HAL_UART_ERROR_NONE;
	s_cpu2_response_failure_reason = CPU2_COMM_FAIL_TX_DMA;
	if (!sendToCPU2(arr, len, false)) {
		CPU3_WatchdogReportProgress();
		return CPU2_CommFinishFailedRequest(parameter_write_attempted);
	}
	/* 等待接收完成 */
	uint32_t timeout = HAL_GetTick();
	while (wait_response) {
		/* 先处理异常边界，避免Modbus 协议状态机带故障继续运行。 */
		if ((HAL_GetTick() - timeout) > CPU2_RESPONSE_TIMEOUT_MS)
				{
			wait_response = false;    /* 防止一直 True */
			s_cpu2_response_failure_reason = CPU2_COMM_FAIL_TIMEOUT;
			CPU3_WatchdogReportProgress();
			return CPU2_CommFinishFailedRequest(parameter_write_attempted);
		}
	}
	if (s_cpu2_uart_error_pending != HAL_UART_ERROR_NONE) {
		uint32_t uart_error_code = s_cpu2_uart_error_pending;
		s_cpu2_uart_error_pending = HAL_UART_ERROR_NONE;
		s_cpu2_request_uart_error_code = uart_error_code;
		CPU2_CommRecordUartFlags(uart_error_code);
		s_cpu2_response_failure_reason = CPU2_COMM_FAIL_UART;
		CPU3_WatchdogReportProgress();
		return CPU2_CommFinishFailedRequest(parameter_write_attempted);
	}
	if (!HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN)) {
		CPU3_WatchdogReportProgress();
		return CPU2_CommFinishFailedRequest(parameter_write_attempted);
	}
	CPU3_WatchdogReportProgress();
	return true;
}
/* 向CPU2发送数据包 */
bool sendToCPU2(uint8_t *arr, uint16_t len, bool flag_fromhost) {
	RS485_SET_SEND_MODE();  /* switch to transmit */
	wait_response = true; /* wait for CPU2 response */
	if (HAL_UART_Transmit_DMA(&huart5, arr, len) != HAL_OK) {
		/* Fall back to RX immediately if TX DMA cannot start. */
		RS485_SET_RECV_MODE();
		__HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_PE) != RESET) __HAL_UART_CLEAR_PEFLAG(&huart5);
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(&huart5);
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_FE) != RESET) __HAL_UART_CLEAR_FEFLAG(&huart5);
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_NE) != RESET) __HAL_UART_CLEAR_NEFLAG(&huart5);
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		HAL_UART_DMAStop(&huart5);
		if (HAL_UART_Receive_DMA(&huart5, UART5_RX_BUF, UART5_RX_BUF_SIZE) == HAL_OK) {
			__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		} else {
			CPU3_UartScheduleRxRecovery(&huart5);
		}
		wait_response = false;
		return false;
	}
	return true;
}
/* 解析CPU2的响应包 0x03 功能码 */
static void CPU2_Response03Process(uint8_t const *revframe) {
	int i;
	int byteamount;
	bool response_contains_protocol = CPU2_ResponseContainsProtocolVersion();
	bool previous_protocol_valid = s_cpu2_has_protocol_snapshot;
	uint32_t previous_protocol_version = s_cpu2_protocol_version;
	uint16_t protocol_offset = 0U;

	/* Modbus RTU: revframe[0]=地址, revframe[1]=功能码(0x03), revframe[2]=字节数 */
	byteamount = revframe[2];

	/* 简单防御：返回的字节数必须是寄存器数 * 2 */
	if (byteamount != RCV_registercnt * 2) {
		return;
	}

	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));

	/* 1. 把数据区解析成寄存器值，填到 SlaveTempBuffer */
	for (i = 0; i < RCV_registercnt; i++) {
		uint16_t reg = ((uint16_t) revframe[3 + i * 2] << 8) | (uint16_t) revframe[3 + i * 2 + 1];
		SlaveTempBuffer[i] = reg;
	}
	if (response_contains_protocol) {
		protocol_offset = (uint16_t)(HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION -
									 (uint16_t)RCV_startaddress);
		s_cpu2_protocol_version = ((uint32_t)(uint16_t)SlaveTempBuffer[protocol_offset] << 16) |
							  (uint32_t)(uint16_t)SlaveTempBuffer[protocol_offset + 1U];
		s_cpu2_has_protocol_snapshot = true;
		if ((!previous_protocol_valid) ||
			(previous_protocol_version != s_cpu2_protocol_version)) {
			if (s_cpu2_protocol_version == DEVICE_PROTOCOL_VERSION) {
				CPU3_LOG_INFO("CPU2",
							  "协议版本确认匹配 CPU2=%lu CPU3=%u",
							  (unsigned long)s_cpu2_protocol_version,
							  (unsigned int)DEVICE_PROTOCOL_VERSION);
			} else {
				CPU3_LOG_WARNING("CPU2",
								 "协议版本不匹配 CPU2=%lu CPU3=%u，普通命令与快照访问保持关闭",
								 (unsigned long)s_cpu2_protocol_version,
								 (unsigned int)DEVICE_PROTOCOL_VERSION);
			}
		}
	}

	/* 独立协议探测只更新私有兼容状态，不能把其余未读取参数覆盖为旧值或零值。 */
	if (response_contains_protocol &&
		(RCV_startaddress == HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION) &&
		(RCV_registercnt == REG_SIZE_U32)) {
		return;
	}

	/* 2. 写保持寄存器（根据项目逻辑，这里我不动你的调用顺序） */
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	PresetRegister(false, SlaveTempBuffer);

	/* 3. 按现行直接地址解析保持寄存器并刷新g_deviceParams。 */
	AnalysisHoldRegister();
	ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
}

/* 解析CPU2的响应包0x04功能码 */
static void CPU2_Response04Process(uint8_t const *revframe) {
	int i, j;
	bool response_contains_status = CPU2_ResponseContainsDeviceStatus();
	bool comm_fault_was_active = s_cpu2_comm_fault_active;
	bool response_contains_profile_header =
		LtdModbus_RangeContains((uint16_t)RCV_startaddress,
							   (uint16_t)RCV_registercnt,
							   REG_DENSITY_DIST_AVG_TEMP,
							   CPU2_PROFILE_HEADER_REGISTER_COUNT);
	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));
	for (i = 0, j = 0; i < RCV_registercnt; i++, j = j + 2) {
		SlaveTempBuffer[i] = (revframe[j + 3] << 8) + revframe[j + 4];
	}

	/* 固定点三阶段读取只写私有候选，任何中间响应都不得触碰公开输入缓存。 */
	if (s_cpu2_private_input_destination != NULL) {
		if (((uint16_t)RCV_startaddress != s_cpu2_private_input_start) ||
			((uint16_t)RCV_registercnt > s_cpu2_private_input_capacity)) {
			return;
		}
		for (i = 0; i < RCV_registercnt; i++) {
			s_cpu2_private_input_destination[i] = (uint16_t)SlaveTempBuffer[i];
		}
		s_cpu2_private_input_captured = true;
		return;
	}

/* printf("CPU2_Response04Process: RCV_registercnt = %d\r\n", RCV_registercnt); */
	/* 写保持寄存器 */
	PresetRegister(true, SlaveTempBuffer);
	read_measurement_result_from_InputRegisters(InputRegisterArray);
	if (response_contains_profile_header) {
		CPU2_ProfileObservePublicHeader();
	}
	if (response_contains_status) {
		s_cpu2_has_status_snapshot = true;
		CPU2_ProfileApplyPublicStatusGate();
	}
	if (s_cpu2_comm_fault_active) {
		if (response_contains_status) {
			s_cpu2_comm_fault_active = false;
		} else {
			g_measurement.device_status.device_state = STATE_ERROR;
			g_measurement.device_status.error_code = CPU2_COMM_TIMEOUT;
		}
	}
	if (comm_fault_was_active && (!s_cpu2_comm_fault_active)) {
		CPU3_LOG_INFO("CPU2", "状态快照已恢复，CPU2通信故障锁存已清除");
	}
}

/**
 * @brief 处理 CPU2 对写多个寄存器指令的响应帧。
 * @param arr 响应帧数据缓冲区。
 * @param len 响应帧长度。
 * @note 当前保留接口，后续若需要确认 0x10 写入结果可在此解析。
 */
static void CPU2_Response10Process(uint8_t *arr, uint16_t len) {

}

/*
 写寄存器
 registertype --> false - 保持寄存器
 --> true - 输入寄存器
 */
static void PresetRegister(bool registertype, int const *registervalue) {
	int i;
	uint16_t *registers = registertype ? InputRegisterArray : HoldingRegisterArray;

	for (i = 0; i < RCV_registercnt; i++) {
		registers[RCV_startaddress + i] = (uint16_t)registervalue[i];
	}
}
