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
#include "wartsila_modbus_data_analysis.h"

#define DEBUG_COMMUCPU2 0
#define ADERSS 0X01
#define CPU2_RESPONSE_TIMEOUT_MS 1000U /* CPU2 单次响应等待超时，单位 ms。 */
#define CPU2_COMM_FAILURE_LIMIT 10U /* CPU2 连续请求未获得合法响应的置错阈值。 */
#define CPU2_MAX_EXTERNAL_WRITE_REGISTERS 122U /* RTU 最大 123 个寄存器；共享字段按 32 位对齐后取最大偶数。 */
#define CPU2_FIXED_POINT_RESULT_REGISTER_COUNT (REG_DENSITY_DIST_AVG_TEMP - REG_SINGLE_POINT_MEAS_TEMP)
#define CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT (REG_ENG - REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER)

volatile bool wait_response = false; /* 主控板响应标志位 */
static bool s_cpu2_has_status_snapshot = false; /* CPU3 本次上电是否收到过覆盖状态和错误码的 CPU2 响应。 */
static bool s_cpu2_has_parameter_snapshot = false; /* CPU3 是否已完整读取 CPU2 保持寄存器参数。 */
static bool s_cpu2_has_protocol_snapshot = false; /* 当前 CPU2 连接是否已读回共享协议版本。 */
static bool s_cpu2_has_fixed_point_snapshot = false; /* 固定点结果与双代际是否已通过前后双读校验。 */
static uint32_t s_cpu2_consecutive_failure_count = 0U; /* CPU2 连续请求失败次数。 */
static bool s_cpu2_comm_fault_active = false; /* CPU3 本机通信故障是否等待状态帧恢复。 */
static bool s_cpu2_parameter_refresh_requested = false; /* 外部参数写后是否要求重新确认 CPU2 参数。 */
static bool s_cpu2_snapshot_resync_requested = false; /* 内部通信失败后是否要求完整重同步。 */
static volatile bool s_cpu2_uart_error_pending = false; /* UART5 中断仅置位，主循环统一计入失败。 */

/* 保持寄存器 */
uint16_t HoldingRegisterArray[HOLEREGISTER_STOP] = { 0 }; /* 保持寄存器数组 */
/* 输入寄存器 */
static uint16_t InputRegisterArray[INPUTREGISTER_AMOUNT] = { 0 };    /* 输入寄存器数组 */

typedef enum {
	CPU2_FIXED_POINT_READ_COUNTER_BEFORE = 0,
	CPU2_FIXED_POINT_READ_RESULTS,
	CPU2_FIXED_POINT_READ_COUNTER_AFTER
} Cpu2FixedPointSnapshotStage;

static Cpu2FixedPointSnapshotStage s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_COUNTER_BEFORE;
static uint16_t s_cpu2_fixed_point_results[CPU2_FIXED_POINT_RESULT_REGISTER_COUNT];
static uint16_t s_cpu2_fixed_point_counter_before[CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT];
static uint16_t s_cpu2_fixed_point_counter_after[CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT];
static uint16_t *s_cpu2_private_input_destination = NULL;
static uint16_t s_cpu2_private_input_capacity = 0U;
static bool s_cpu2_private_input_captured = false;

/* 接收到的命令包数据暂存变量 */
static int RCV_functioncode = 0; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static int RCV_startaddress = 0; /* Modbus 协议地址配置，影响协议寻址或硬件访问。 */
static int RCV_registercnt = 0; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static int SlaveTempBuffer[INPUTREGISTER_AMOUNT]; /* Modbus 协议数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */

/* static void CPU2_Response03Process(char const *revframe); */
static void CPU2_Response03Process(uint8_t const *revframe);
static void CPU2_Response04Process(uint8_t const *revframe);
static void CPU2_Response10Process(uint8_t *arr, uint16_t len);
static void PresetRegister(bool registertype, int const *registervalue);
static bool CPU2_ResponseFrameIsValid(uint8_t const *rcv, int len);
static bool CPU2_ResponseContainsDeviceStatus(void);
static bool CPU2_ResponseContainsProtocolVersion(void);
static void CPU2_CommMarkValidResponse(void);
static void CPU2_InvalidatePublicSnapshotFreshness(void);
static void CPU2_CommRecordFailure(void);
static void CPU2_ResetFixedPointSnapshotHandshake(bool invalidate_snapshot);
static bool CPU2_ReadInputRegistersPrivate(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs);
static bool CPU2_PollFixedPointSnapshotHandshake(void);

static void RequestDensityDistPoints_ByCount(void);

/*
 * 函数用途：校验 CPU2 响应是否与当前请求的功能码、长度和回显字段一致。
 * 调用场景：CRC 和从机地址校验通过后、清除连续请求失败计数前调用。
 * 关键约束：异常响应、错功能码和不完整数据帧均不得刷新通信有效状态。
 */
static bool CPU2_ResponseFrameIsValid(uint8_t const *rcv, int len)
{
	uint32_t expected_byte_count;

	if ((rcv == NULL) || (len < 5) ||
		(rcv[1] != (uint8_t)RCV_functioncode)) {
		return false;
	}

	switch (RCV_functioncode) {
	case FUNCTIONCODE_READ_HOLDREGISTER:
	case FUNCTIONCODE_READ_INPUTREGISTER:
		expected_byte_count = (uint32_t)RCV_registercnt * 2U;
		return (expected_byte_count <= UINT8_MAX) &&
			   (rcv[2] == (uint8_t)expected_byte_count) &&
			   (len == (int)(expected_byte_count + 5U));

	case FUNCTIONCODE_WRITE_MULREGISTER:
		return (len == 8) &&
			   (rcv[2] == (uint8_t)((uint16_t)RCV_startaddress >> 8)) &&
			   (rcv[3] == (uint8_t)RCV_startaddress) &&
			   (rcv[4] == (uint8_t)((uint16_t)RCV_registercnt >> 8)) &&
			   (rcv[5] == (uint8_t)RCV_registercnt);

	default:
		return false;
	}
}

/*
 * 函数用途：判断当前 0x04 响应是否完整覆盖设备状态和错误码字段。
 * 调用场景：输入寄存器响应写入缓存后决定是否允许恢复本机通信故障。
 * 关键约束：非状态分组不得用掉线前的旧缓存提前清除通信故障。
 */
static bool CPU2_ResponseContainsDeviceStatus(void)
{
	uint32_t response_start = (uint32_t)RCV_startaddress;
	uint32_t response_end = response_start + (uint32_t)RCV_registercnt;
	uint32_t required_end = (uint32_t)REG_DEVICE_STATUS_ERROR_CODE + REG_SIZE_U32;

	return (RCV_functioncode == FUNCTIONCODE_READ_INPUTREGISTER) &&
		   (response_start <= (uint32_t)REG_DEVICE_STATUS_DEVICE_STATE) &&
		   (response_end >= required_end);
}

/*
 * 函数用途：判断当前 0x03 响应是否完整覆盖共享协议版本字段。
 * 调用场景：保持寄存器响应解析完成后确认当前 CPU2 连接的协议版本来源。
 * 关键约束：通信故障恢复后必须重新读回该字段，不能沿用掉线前缓存开放写入口。
 */
static bool CPU2_ResponseContainsProtocolVersion(void)
{
	uint32_t response_start = (uint32_t)RCV_startaddress;
	uint32_t response_end = response_start + (uint32_t)RCV_registercnt;
	uint32_t required_end = (uint32_t)HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION + REG_SIZE_U32;

	return (RCV_functioncode == FUNCTIONCODE_READ_HOLDREGISTER) &&
		   (response_start <= (uint32_t)HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION) &&
		   (response_end >= required_end);
}

/*
 * 函数用途：记录 CPU2 合法响应并清除连续请求失败计数。
 * 调用场景：响应通过长度、CRC 和从机地址校验后由主循环调用。
 * 关键约束：本函数不建立状态快照；只有包含状态和错误码的 0x04 响应才开放写入口。
 */
static void CPU2_CommMarkValidResponse(void)
{
	s_cpu2_consecutive_failure_count = 0U;
}

/*
 * 函数用途：内部请求未获得合法响应时，立即关闭所有对外公开快照门禁。
 * 调用场景：响应超时、非法响应、UART错误或TX DMA启动失败。
 * 关键约束：只处理数据新鲜度和完整重同步请求；连续十次报警计数由独立逻辑维护。
 */
static void CPU2_InvalidatePublicSnapshotFreshness(void)
{
	s_cpu2_has_status_snapshot = false;
	s_cpu2_has_parameter_snapshot = false;
	s_cpu2_has_protocol_snapshot = false;
	CPU2_ResetFixedPointSnapshotHandshake(true);
	s_cpu2_snapshot_resync_requested = true;
}

/*
 * 函数用途：先使公开快照失效，再记录一次未获得合法 CPU2 响应的请求。
 * 调用场景：主循环处理响应超时、非法响应、UART 错误或 TX DMA 启动失败时调用。
 * 关键约束：首次失败即关闭新鲜度门禁；计数仍饱和在十次报警阈值，ISR 只置待处理标志。
 */
static void CPU2_CommRecordFailure(void)
{
	CPU2_InvalidatePublicSnapshotFreshness();

	if (s_cpu2_consecutive_failure_count < CPU2_COMM_FAILURE_LIMIT) {
		s_cpu2_consecutive_failure_count++;
	}

	if (s_cpu2_consecutive_failure_count >= CPU2_COMM_FAILURE_LIMIT) {
		s_cpu2_comm_fault_active = true;
	}

	if (s_cpu2_comm_fault_active) {
		g_measurement.device_status.device_state = STATE_ERROR;
		g_measurement.device_status.error_code = CPU2_COMM_TIMEOUT;
	}
}

/*
 * 函数用途：记录 UART5 错误并解除当前同步等待。
 * 调用场景：HAL UART 错误回调中调用。
 * 关键约束：本函数可能处于 ISR 上下文，只置标志，不打印、不计数、不改设备状态。
 */
void CPU2_CommNotifyUartErrorFromISR(void)
{
	if (wait_response) {
		s_cpu2_uart_error_pending = true;
		wait_response = false;
	}
}

/*
 * 函数用途：判断是否仍处于等待 CPU2 首次状态和当前连接协议快照的同步阶段。
 * 调用场景：CPU3 状态页选择通讯尝试页前调用。
 * 关键约束：协议字段未实际读回时不得用默认值或掉线前缓存显示协议不匹配/兼容；
 *           连续第十次请求未获得合法响应后返回 false，让故障页接管显示。
 */
bool CPU2_CommShouldShowStartup(void)
{
	bool fixed_point_snapshot_pending = s_cpu2_has_protocol_snapshot &&
		(g_deviceParams.protocolVersion == DEVICE_PROTOCOL_VERSION) &&
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
		   (g_deviceParams.protocolVersion == DEVICE_PROTOCOL_VERSION);
}

bool CPU2_CommHasRuntimeSnapshot(void)
{
	return s_cpu2_has_status_snapshot &&
		   s_cpu2_has_protocol_snapshot &&
		   s_cpu2_has_fixed_point_snapshot &&
		   (!s_cpu2_comm_fault_active) &&
		   (g_deviceParams.protocolVersion == DEVICE_PROTOCOL_VERSION);
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
		   (g_deviceParams.protocolVersion == DEVICE_PROTOCOL_VERSION);
}

/*
 * 函数用途：从 CPU3 已确认的参数快照复制 LTD 保持寄存器。
 * 调用场景：CPU3 对外以 LTD 协议独立响应 FC03 读请求时调用。
 * 关键约束：同步未完成、协议不匹配、通信故障或地址越界时不得返回旧缓存。
 */
bool CPU2_CommReadHoldingSnapshot(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs)
{
	if ((out_regs == NULL) || (registercnt == 0U) ||
		(startadd >= HOLEREGISTER_STOP) ||
		(registercnt > (uint16_t)(HOLEREGISTER_STOP - startadd)) ||
		(!CPU2_CommIsAvailable())) {
		return false;
	}

	/* g_deviceParams 只在 CPU2 快照或成功写入后更新，组表后再复制可覆盖菜单成功写入的新值。 */
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	memcpy(out_regs,
	       &HoldingRegisterArray[startadd],
	       (size_t)registercnt * sizeof(out_regs[0]));
	return true;
}

/*
 * 函数用途：从 CPU3 已确认的状态快照复制 LTD 输入寄存器。
 * 调用场景：CPU3 对外以 LTD 协议独立响应 FC04 读请求时调用。
 * 关键约束：只复制 CPU2 最近一次合法响应形成的数组，不用默认结构生成伪状态。
 */
bool CPU2_CommReadInputSnapshot(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs)
{
	if ((out_regs == NULL) || (registercnt == 0U) ||
		(startadd >= INPUTREGISTER_AMOUNT) ||
		(registercnt > (uint16_t)(INPUTREGISTER_AMOUNT - startadd)) ||
		(!CPU2_CommIsAvailable())) {
		return false;
	}

	memcpy(out_regs,
	       &InputRegisterArray[startadd],
	       (size_t)registercnt * sizeof(out_regs[0]));
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

	if ((wire_regs == NULL) || (registercnt == 0U) ||
		(registercnt > CPU2_MAX_EXTERNAL_WRITE_REGISTERS) ||
		((startadd & 1U) != 0U) || ((registercnt & 1U) != 0U) ||
		(startadd >= HOLEREGISTER_STOP) ||
		(registercnt > (uint16_t)(HOLEREGISTER_STOP - startadd))) {
		return false;
	}

	/* 现有发送入口接收本机 uint32_t 值，并负责转换为共享协议的高字在前线序。 */
	for (uint16_t i = 0U; i < registercnt; i += 2U) {
		host_values[i / 2U] = ((uint32_t)wire_regs[i] << 16) |
									 (uint32_t)wire_regs[i + 1U];
	}

	command_only = (startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) && (registercnt == 2U);
	command_value = ((uint32_t)wire_regs[0] << 16) | (uint32_t)wire_regs[1];
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
	CPU2_CommRecordFailure();
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
	s_cpu2_private_input_captured = false;
	request_ok = CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,
												 startadd,
												 registercnt,
												 NULL);
	captured = s_cpu2_private_input_captured;
	s_cpu2_private_input_destination = NULL;
	s_cpu2_private_input_capacity = 0U;
	s_cpu2_private_input_captured = false;
	return request_ok && captured;
}

static uint32_t CPU2_ReadU32FromPrivateRegs(const uint16_t *regs, uint16_t offset)
{
	return ((uint32_t)regs[offset] << 16) | (uint32_t)regs[offset + 1U];
}

/*
 * 函数用途：把已验证同代的24个固定点结果寄存器与两个代际一次提交到公开缓存。
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
	g_measurement.single_point_monitoring.temperature = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 12U);
	g_measurement.single_point_monitoring.density = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 14U);
	g_measurement.single_point_monitoring.temperature_position = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 16U);
	g_measurement.single_point_monitoring.standard_density = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 18U);
	g_measurement.single_point_monitoring.vcf20 = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 20U);
	g_measurement.single_point_monitoring.weight_density = CPU2_ReadU32FromPrivateRegs(s_cpu2_fixed_point_results, 22U);
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
 * 关键约束：顺序固定为计数前读、私有结果读、计数后读；失败或变化从第一阶段重来。
 */
static bool CPU2_PollFixedPointSnapshotHandshake(void)
{
	switch (s_cpu2_fixed_point_stage) {
	case CPU2_FIXED_POINT_READ_COUNTER_BEFORE:
		if (!CPU2_ReadInputRegistersPrivate(REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER,
											 CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT,
											 s_cpu2_fixed_point_counter_before)) {
			CPU2_ResetFixedPointSnapshotHandshake(true);
			return false;
		}
		s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_RESULTS;
		return false;

	case CPU2_FIXED_POINT_READ_RESULTS:
		if (!CPU2_ReadInputRegistersPrivate(REG_SINGLE_POINT_MEAS_TEMP,
											 CPU2_FIXED_POINT_RESULT_REGISTER_COUNT,
											 s_cpu2_fixed_point_results)) {
			CPU2_ResetFixedPointSnapshotHandshake(true);
			return false;
		}
		s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_COUNTER_AFTER;
		return false;

	case CPU2_FIXED_POINT_READ_COUNTER_AFTER:
		if (!CPU2_ReadInputRegistersPrivate(REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER,
											 CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT,
											 s_cpu2_fixed_point_counter_after)) {
			CPU2_ResetFixedPointSnapshotHandshake(true);
			return false;
		}
		if (memcmp(s_cpu2_fixed_point_counter_before,
				   s_cpu2_fixed_point_counter_after,
				   sizeof(s_cpu2_fixed_point_counter_before)) != 0) {
			CPU2_ResetFixedPointSnapshotHandshake(true);
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

/* 与CPU2通讯接收包主处理过程 */
bool HostCommuProcess(uint8_t *rcv, int len) {
#if DEBUG_COMMUCPU2
    int i;
    printf("收到CPU2数据 %d 字节: ",len);
    for(i = 0;i < len;i++)
        printf("%02X ",rcv[i]);
    printf("\r\n");
#endif
	if (len <= 3)
		return false;
	if (!SlaveCheckCRC(rcv, len)) {
		printf("CPU3 CRC校验错误");
		return false;
	}
	/* 解析数据 */
	if (rcv[0] != ADERSS) {
		printf("CPU3地址错误");
		return false;
	}
	if (!CPU2_ResponseFrameIsValid(rcv, len)) {
		printf("CPU3响应格式错误");
		return false;
	}
	CPU2_CommMarkValidResponse();
	switch (RCV_functioncode) {
	case FUNCTIONCODE_READ_HOLDREGISTER: {
		CPU2_Response03Process(rcv);
#if DEBUG_COMMUCPU2
            printf("CPU3处理03响应\r\n");
/* print_device_params(); */
#endif
		break;
	}
	case FUNCTIONCODE_READ_INPUTREGISTER: {
#if DEBUG_COMMUCPU2
            printf("CPU3处理04响应\r\n");
/* print_device_params(); */
#endif
		CPU2_Response04Process(rcv);
		break;
	}
	case FUNCTIONCODE_WRITE_MULREGISTER: {
		CPU2_Response10Process(rcv, len);
#if DEBUG_COMMUCPU2
/* print_device_params(); */
#endif
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
#include <stdbool.h>

#define INPUT_TAIL_REGISTER_COUNT ((uint16_t)(REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER - REG_WIRELESS_PAIRING_RESULT))

/* 上电阶段要读取的组：
 * - 包含设备参数（保持寄存器）
 * - 也可以顺便把输入寄存器读一遍
 */
static const PollGroup poweron_groups[] = {
/* 输入寄存器组 1：设备状态 */
{FUNCTIONCODE_READ_INPUTREGISTER, REG_DEVICE_STATUS_WORK_MODE, (uint16_t) (REG_SINGLE_POINT_MEAS_TEMP - REG_DEVICE_STATUS_WORK_MODE) },

/* 输入寄存器组 2：单点 / 分布测量结果 */
{FUNCTIONCODE_READ_INPUTREGISTER, REG_DENSITY_DIST_AVG_TEMP, (uint16_t) (REG_DENSITY_DIST_POINT_BASE - REG_DENSITY_DIST_AVG_TEMP) },

/* 输入寄存器组 3：无线滑环匹配状态与继电器报警输出运行态 */
{FUNCTIONCODE_READ_INPUTREGISTER, REG_WIRELESS_PAIRING_RESULT, INPUT_TAIL_REGISTER_COUNT },

/* 保持寄存器组 4：设备参数前半段 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, (uint16_t) (HOLDREGISTER_DEVICEPARAM_RESERVED11 - HOLDREGISTER_DEVICEPARAM_COMMAND) },

/* 保持寄存器组 5：设备参数中段 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RESERVED11, (uint16_t) (HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE - HOLDREGISTER_DEVICEPARAM_RESERVED11) },

/* 保持寄存器组 6：AO/指令参数/尺带补偿 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE, (uint16_t) (HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE - HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE) },

/* 保持寄存器组 7：继电器报警输出配置与元信息 */
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE, (uint16_t) (HOLEREGISTER_STOP - HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE) }
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
FUNCTIONCODE_READ_INPUTREGISTER, REG_DENSITY_DIST_AVG_TEMP, (uint16_t) (REG_DENSITY_DIST_POINT_BASE - REG_DENSITY_DIST_AVG_TEMP) },

/* 输入寄存器组 3：无线滑环匹配状态与继电器报警输出运行态 */
{
FUNCTIONCODE_READ_INPUTREGISTER, REG_WIRELESS_PAIRING_RESULT, INPUT_TAIL_REGISTER_COUNT }
};

#define RUNTIME_GROUP_COUNT  (sizeof(runtime_groups) / sizeof(runtime_groups[0]))

/* 参数更新时，一次性补读系统参数。 */
static const PollGroup refresh_hold_groups[] = {
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_COMMAND, (uint16_t) (HOLDREGISTER_DEVICEPARAM_RESERVED11 - HOLDREGISTER_DEVICEPARAM_COMMAND) },
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RESERVED11, (uint16_t) (HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE - HOLDREGISTER_DEVICEPARAM_RESERVED11) },
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE, (uint16_t) (HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE - HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE) },
{FUNCTIONCODE_READ_HOLDREGISTER, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE, (uint16_t) (HOLEREGISTER_STOP - HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE) }
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

	/* 首次失败即废弃旧公开快照；下一轮从状态、参数和协议开始完整重同步。 */
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
		(void)CPU2_CombinatePackage_Send(status_group->func,
									   status_group->start,
									   status_group->len,
									   NULL);
		return;
	}

	/* ---------- 上电阶段：普通组完成后再执行固定点私有快照握手 ---------- */
	if (!poweron_done) {
		if (poweron_index < POWERON_GROUP_COUNT) {
			const PollGroup *g = &poweron_groups[poweron_index];

			if (!CPU2_CombinatePackage_Send(g->func, g->start, g->len, NULL)) {
				return;
			}
			poweron_index++;
			return;
		}

		/* 已确认旧协议时不读取协议21新增地址，保留严格版本不匹配门禁。 */
		if ((!s_cpu2_has_protocol_snapshot) ||
			(g_deviceParams.protocolVersion != DEVICE_PROTOCOL_VERSION)) {
			poweron_done = true;
			s_cpu2_has_parameter_snapshot = true;
			s_cpu2_parameter_refresh_requested = false;
			DeviceParams_StoreToRegisters(g_holding_regs);
			last_param_update_flag = g_measurement.device_status.parameter_update_flag;
			refresh_target_flag = last_param_update_flag;
			param_flag_valid = true;
			return;
		}

		if (!CPU2_PollFixedPointSnapshotHandshake()) {
			return;
		}

		poweron_done = true;
		s_cpu2_has_parameter_snapshot = true;
		s_cpu2_parameter_refresh_requested = false;
		runtime_fixed_snapshot_pending = false;
		DeviceParams_StoreToRegisters(g_holding_regs);
		last_param_update_flag = g_measurement.device_status.parameter_update_flag;
		refresh_target_flag = last_param_update_flag;
		param_flag_valid = true;
/* print_device_params(); */
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
			if (!CPU2_CombinatePackage_Send(g->func, g->start, g->len, NULL)) {
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

		if (!CPU2_CombinatePackage_Send(g->func, g->start, g->len, NULL)) {
			return;
		}

		runtime_index++;
		if (runtime_index >= RUNTIME_GROUP_COUNT) {
			runtime_index = 0;
			runtime_fixed_snapshot_pending = s_cpu2_has_protocol_snapshot &&
				(g_deviceParams.protocolVersion == DEVICE_PROTOCOL_VERSION);
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

    /* 特定设备状态下，优先按测点数读取分布测量点 */
    if( (g_measurement.device_status.device_state == STATE_WARTSILA_DENSITY_OVER)
    || (g_measurement.device_status.device_state == STATE_SPREADPOINTOVER)
	  || (g_measurement.device_status.device_state == STATE_GB_SPREADPOINTOVER)
	  || (g_measurement.device_status.device_state == STATE_SYNTHETICING_OVER)
	  || (g_measurement.device_status.device_state == STATE_COM_METER_DENSITY_OVER)
	  || (g_measurement.device_status.device_state == STATE_INTERVAL_DENSITY_OVER)) {
        /* 根据 REG_DENSITY_DIST_MEAS_POINTS 的值拉点数据 */
        RequestDensityDistPoints_ByCount();
        /* 你可以在读完后置一个标志，避免每次都重复读 */
        /* g_measurement.flags.density_points_fetched = true; */
        return;
    }
}
#define MAX_REGS_PER_READ  100   /* 单帧最多读多少寄存器，根据你从机限制调整 */

/**
 * @brief 根据测量点数量读取分布测量点的所有数据
 *        调用前需要确保 g_measurement.density_distribution.measurement_points 已经正确更新。
 */
static void RequestDensityDistPoints_ByCount(void)
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
		if (!CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,
									start,
									this_len,
									NULL)) {
			return;
		}

        start      += this_len;
        total_regs -= this_len;
    }
}

/*
 * 函数用途：刷新SI候选结果的紧凑头和生命周期尾部。
 * 调用场景：分块拉取点阵前后各调用一次。
 * 关键约束：两个区域任一读取失败时不得继续使用缓存中的旧头部。
 */
static bool CPU2_ReadSiProfileCandidateHeader(void)
{
	uint16_t header_count = (uint16_t)(REG_DENSITY_DIST_POINT_BASE - REG_DENSITY_DIST_AVG_TEMP);
	uint16_t runtime_count = (uint16_t)(REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER - REG_DENSITY_DIST_SI_PROFILE_PHASE);

	if (!CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,
									  REG_DENSITY_DIST_AVG_TEMP,
									  header_count,
									  NULL)) {
		return false;
	}

	return CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,
									 REG_DENSITY_DIST_SI_PROFILE_PHASE,
									 runtime_count,
									 NULL);
}

static Cpu2SiProfileCandidateKey CPU2_GetSiProfileCandidateKey(void)
{
	Cpu2SiProfileCandidateKey key;

	key.cycle_counter = g_measurement.si_profile_runtime.cycle_counter;
	key.complete_counter = g_measurement.density_distribution.profile_complete_counter;
	key.measurement_points = g_measurement.density_distribution.measurement_points;
	key.profile_source = g_measurement.density_distribution.profile_source;
	key.phase = g_measurement.si_profile_runtime.phase;
	return key;
}

static bool CPU2_SiProfileCandidateKeyEqual(const Cpu2SiProfileCandidateKey *left,
											const Cpu2SiProfileCandidateKey *right)
{
	return (left->cycle_counter == right->cycle_counter) &&
		   (left->complete_counter == right->complete_counter) &&
		   (left->measurement_points == right->measurement_points) &&
		   (left->profile_source == right->profile_source) &&
		   (left->phase == right->phase);
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
 * 函数用途：读取一代完整SI Profile点阵并执行分块前后代际复核。
 * 调用场景：正常Complete候选拉取，或CPU3重启后恢复Point0前仍保留的上一轮结果。
 * 关键约束：本函数同步分块读取；失败时调用方不得发布部分点阵，并应从头节流重试。
 */
static bool CPU2_CommFetchSiProfilePayload(Cpu2SiProfileCandidateKey *out_key,
											bool previous_snapshot)
{
	Cpu2SiProfileCandidateKey before;
	Cpu2SiProfileCandidateKey after;
	uint16_t start;
	uint32_t remaining;

	if ((out_key == NULL) || (!CPU2_CommIsAvailable())) {
		return false;
	}

	if (!CPU2_ReadSiProfileCandidateHeader()) {
		return false;
	}
	before = CPU2_GetSiProfileCandidateKey();
	if ((g_measurement.density_distribution.profile_complete_latched == 0U) ||
		(before.profile_source != (uint32_t)PROFILE_SOURCE_SI) ||
		(!CPU2_SiProfileFetchPhaseAllowed(before.phase, previous_snapshot)) ||
		(before.measurement_points == 0U) ||
		(before.measurement_points > MAX_MEASUREMENT_POINTS)) {
		return false;
	}

	start = REG_DENSITY_DIST_POINT_BASE;
	remaining = before.measurement_points * (uint32_t)REG_DENSITY_DIST_POINT_SIZE;
	while (remaining > 0U) {
		uint16_t read_count = (remaining > MAX_REGS_PER_READ) ?
			(uint16_t)MAX_REGS_PER_READ : (uint16_t)remaining;

		if (!CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_INPUTREGISTER,
										  start,
										  read_count,
										  NULL)) {
			return false;
		}

		start = (uint16_t)(start + read_count);
		remaining -= read_count;
	}

	if (!CPU2_ReadSiProfileCandidateHeader()) {
		return false;
	}
	after = CPU2_GetSiProfileCandidateKey();
	if ((g_measurement.density_distribution.profile_complete_latched == 0U) ||
		(!CPU2_SiProfileCandidateKeyEqual(&before, &after))) {
		return false;
	}

	*out_key = after;
	return true;
}

/*
 * 函数用途：拉取CPU2当前Complete阶段的SI候选点阵。
 * 调用场景：CPU3观察到本周期完成计数沿后执行最终发布。
 * 关键约束：只接受Complete阶段，并沿用完整分块前后代际复核。
 */
bool CPU2_CommFetchSiProfileCandidate(Cpu2SiProfileCandidateKey *out_key)
{
	return CPU2_CommFetchSiProfilePayload(out_key, false);
}

/*
 * 函数用途：拉取CPU2在Point0前仍保留的上一轮完整SI点阵。
 * 调用场景：CPU3冷启动时处于PREPARING，或Point0前已经ABORTED/FAILED。
 * 关键约束：只接受旧SI来源、完成锁存、有效点数和稳定代际，不重建Profile时间。
 */
bool CPU2_CommFetchSiPreviousSnapshot(Cpu2SiProfileCandidateKey *out_key)
{
	return CPU2_CommFetchSiProfilePayload(out_key, true);
}

/* 由屏幕向CPU2发送指令包 */
bool CPU2_CombinatePackage_Send(uint8_t f_code, uint16_t startadd, uint16_t registercnt, uint32_t *holddata) {
	bool cancel_command_allowed = false;
	bool parameter_write_attempted = false;

	/* 普通写必须通过完整快照门禁；参数刷新期间只放行协议兼容的取消命令。 */
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) &&
		(startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) &&
		(registercnt == 2U) &&
		(holddata != NULL)) {
		cancel_command_allowed = CPU2_CommCanSendCommand((CommandType)(*holddata));
	}
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) &&
		!((startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) && (registercnt == 2U))) {
		parameter_write_attempted = true;
	}
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) && !CPU2_CommIsAvailable()) {
		if (!cancel_command_allowed) {
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
#if DEBUG_COMMUCPU2
	printf("发送到CPU2 %d 字节:", len);
#endif
	/* 先发布本次期望字段，避免极快响应到达时仍沿用上一请求。 */
	RCV_functioncode = f_code;
	RCV_startaddress = startadd;
	RCV_registercnt = registercnt;
	UART5_RX_LEN = 0U;
	s_cpu2_uart_error_pending = false;
	if (!sendToCPU2(arr, len, false)) {
		return CPU2_CommFinishFailedRequest(parameter_write_attempted);
	}
	/* 等待接收完成 */
	uint32_t timeout = HAL_GetTick();
	while (wait_response) {
		/* 先处理异常边界，避免Modbus 协议状态机带故障继续运行。 */
		if ((HAL_GetTick() - timeout) > CPU2_RESPONSE_TIMEOUT_MS)
				{
			printf("等待响应超时！\n");
			wait_response = false;    /* 防止一直 True */
			return CPU2_CommFinishFailedRequest(parameter_write_attempted);
		}
	}
	if (s_cpu2_uart_error_pending) {
		s_cpu2_uart_error_pending = false;
		return CPU2_CommFinishFailedRequest(parameter_write_attempted);
	}
	if (!HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN)) {
		return CPU2_CommFinishFailedRequest(parameter_write_attempted);
	}
	return true;
}
/* 向CPU2发送数据包 */
bool sendToCPU2(uint8_t *arr, uint16_t len, bool flag_fromhost) {
	RS485_SET_SEND_MODE();  /* switch to transmit */
	wait_response = true; /* wait for CPU2 response */
	if (HAL_UART_Transmit_DMA(&huart5, arr, len) != HAL_OK) {
		/* Fall back to RX immediately if TX DMA cannot start. */
		RS485_SET_RECV_MODE();
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_PE) != RESET) __HAL_UART_CLEAR_PEFLAG(&huart5);
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_OREFLAG(&huart5);
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_FE) != RESET) __HAL_UART_CLEAR_FEFLAG(&huart5);
		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_NE) != RESET) __HAL_UART_CLEAR_NEFLAG(&huart5);
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		HAL_UART_DMAStop(&huart5);
		HAL_UART_Receive_DMA(&huart5, UART5_RX_BUF, UART5_RX_BUF_SIZE);
		wait_response = false;
		return false;
	}
#if DEBUG_COMMUCPU2
    {
        int i;
        for(i = 0;i < len;i++)
            printf("%02X ",arr[i]);
        printf("\n");
    }
    #endif
	return true;
}
/* 解析CPU2的响应包 0x03 功能码 */
static void CPU2_Response03Process(uint8_t const *revframe) {
	int i;
	int byteamount;
	bool response_contains_protocol = CPU2_ResponseContainsProtocolVersion();

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

	/* 2. 写保持寄存器（根据项目逻辑，这里我不动你的调用顺序） */
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	PresetRegister(false, SlaveTempBuffer);

	/* 3. 更新 HoldingRegisterArray 里对应的寄存器（注意，这里是按“寄存器”写） */
	for (i = 0; i < RCV_registercnt; i++) {
		HoldingRegisterArray[RCV_startaddress + i] = SlaveTempBuffer[i];
	}

	/* 4. 解析保持寄存器并刷新 g_deviceParams */
	AnalysisHoldRegister();
	ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
	if (response_contains_protocol) {
		s_cpu2_has_protocol_snapshot = true;
	}
}

/* 解析CPU2的响应包0x04功能码 */
static void CPU2_Response04Process(uint8_t const *revframe) {
	int i, j;
	bool response_contains_status = CPU2_ResponseContainsDeviceStatus();
	memset(SlaveTempBuffer, 0, sizeof(SlaveTempBuffer));
	for (i = 0, j = 0; i < RCV_registercnt; i++, j = j + 2) {
		SlaveTempBuffer[i] = (revframe[j + 3] << 8) + revframe[j + 4];
	}

	/* 固定点三阶段读取只写私有候选，任何中间响应都不得触碰公开输入缓存。 */
	if (s_cpu2_private_input_destination != NULL) {
		if ((uint16_t)RCV_registercnt > s_cpu2_private_input_capacity) {
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
	if (response_contains_status) {
		s_cpu2_has_status_snapshot = true;
	}
	if (s_cpu2_comm_fault_active) {
		if (response_contains_status) {
			s_cpu2_comm_fault_active = false;
		} else {
			g_measurement.device_status.device_state = STATE_ERROR;
			g_measurement.device_status.error_code = CPU2_COMM_TIMEOUT;
		}
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
	int range;
	int i;
	int j;

	range = RCV_startaddress + RCV_registercnt;
	if (registertype) {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			InputRegisterArray[i] = registervalue[j];
/* printf("InputRegisterArray[%d] = %d\r\n", i, InputRegisterArray[i]); */
		}
	} else {
		for (i = RCV_startaddress, j = 0; i < range; i++, j++) {
			HoldingRegisterArray[i] = registervalue[j];
/* printf("HoldingRegisterArray[%d] = %d\r\n", i, HoldingRegisterArray[i]); */
		}
	}
}
