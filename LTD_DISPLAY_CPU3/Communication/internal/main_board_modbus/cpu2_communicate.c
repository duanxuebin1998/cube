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

/* CPU2 主板内部 Modbus 从站地址 0x01；保留历史拼写以兼容现有调用，构造请求帧时作为目标地址。 */
#define ADERSS 0X01
#define CPU2_RESPONSE_TIMEOUT_MS 1000U /* CPU2 单次响应等待超时，单位 ms。 */
#define CPU2_COMM_STATUS_LOG_INTERVAL_MS 5000U /* CPU2通信健康摘要周期，避免阻塞调试串口影响轮询。 */
#define CPU2_COMM_RESYNC_FAILURE_LIMIT 3U /* 连续失败达到三次后才废弃快照并完整重同步。 */
#define CPU2_COMM_FAILURE_LIMIT 10U /* CPU2 连续请求未获得合法响应的置错阈值。 */
#define CPU2_MAX_EXTERNAL_WRITE_REGISTERS 122U /* RTU 最大 123 个寄存器；共享字段按 32 位对齐后取最大偶数。 */
/* CPU2 固定结果区从单点测量温度到密度分布摘要之前的寄存器数量；由共享寄存器地址差计算，保证 CPU3 缓存长度随映射同步。 */
#define CPU2_FIXED_POINT_RESULT_REGISTER_COUNT (REG_DENSITY_DIST_AVG_TEMP - REG_SINGLE_POINT_MEAS_TEMP)
/* 单点测量完成计数和监测采样计数合计占用的寄存器数量；两个计数均为 UInt32，因此按 2 × REG_SIZE_U32 计算。 */
#define CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT (2U * REG_SIZE_U32)
/* 密度剖面摘要头的寄存器数量；直接复用共享寄存器表计数，防止 CPU2/CPU3 布局漂移。 */
#define CPU2_PROFILE_HEADER_REGISTER_COUNT REG_DENSITY_DIST_SUMMARY_REG_COUNT
/* 完整密度剖面测点区的最大寄存器容量；由最大测点数乘单点固定步长计算。 */
#define CPU2_PROFILE_POINT_REGISTER_CAPACITY (MAX_MEASUREMENT_POINTS * REG_DENSITY_DIST_POINT_SIZE)
/* CPU3 每帧从 CPU2 拉取的密度剖面测点数 8；用于限制 FC04 分块长度并控制串口占用时间。 */
#define CPU2_PROFILE_POINTS_PER_FRAME 8U
/* 单帧密度剖面点数据对应的寄存器数量；由每帧点数乘单点寄存器步长计算。 */
#define CPU2_PROFILE_REGISTERS_PER_FRAME (CPU2_PROFILE_POINTS_PER_FRAME * REG_DENSITY_DIST_POINT_SIZE)
/* 连续剖面分块读取之间的最小间隔 50 ms；用于给内部总线和其它周期任务让出时间。 */
#define CPU2_PROFILE_FRAME_INTERVAL_MS 50U
/* 一轮剖面拉取失败后重新开始整轮读取前的等待时间 500 ms。 */
#define CPU2_PROFILE_ROUND_RETRY_DELAY_MS 500U
/* 剖面拉取无进展超时 20000 ms；在该窗口内摘要代际和已接收点数均未推进时判定本轮卡住。 */
#define CPU2_PROFILE_NO_PROGRESS_TIMEOUT_MS 20000U
/* 剖面拉取进入失败状态后的下一次重试间隔 2000 ms；避免持续占用内部总线。 */
#define CPU2_PROFILE_FAILED_RETRY_DELAY_MS 2000U
/* 同一剖面数据块允许的最大重试次数 3；超过后终止本轮并保留旧完整快照。 */
#define CPU2_PROFILE_BLOCK_RETRY_MAX 3U
/* 为获得同一代完整剖面允许重新开始的最大轮数 3；防止代际持续变化造成无限循环。 */
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
static bool s_cpu2_factory_restore_refresh_pending = false; /* 恢复出厂ACK后等待CPU2发布持久化完成代次。 */
static uint32_t s_cpu2_confirmed_command_argument_mask = 0U; /* 本连接内逐字段ACK确认、尚未被对应命令消费的命令参数位图。 */
static bool s_cpu2_snapshot_resync_requested = false; /* 内部通信失败后是否要求完整重同步。 */
static volatile uint32_t s_cpu2_uart_error_pending = HAL_UART_ERROR_NONE; /* UART5中断只锁存错误位，主循环统一分类计数。 */
static uint32_t s_cpu2_request_uart_error_code = HAL_UART_ERROR_NONE; /* 当前失败请求对应的UART5硬件错误位。 */
static Cpu2CommHealthSnapshot s_cpu2_comm_health = {0}; /* CPU3本机RAM通信健康计数，上电清零。 */
static Cpu2CommFailureReason s_cpu2_response_failure_reason = CPU2_COMM_FAIL_NONE; /* 当前响应校验失败原因。 */
static uint8_t s_cpu2_last_modbus_exception = CPU2_MODBUS_RESULT_OK; /* 最近一次合法标准异常帧的异常码。 */

/* 保持寄存器 */
uint16_t HoldingRegisterArray[HOLDREGISTER_AMOUNT] = { 0 }; /* 保持寄存器数组 */
/* 输入寄存器 */
static uint16_t InputRegisterArray[INPUTREGISTER_AMOUNT] = { 0 };    /* 输入寄存器数组 */

typedef enum {
	/* 固定结果一致性读取阶段；用前后完成计数包围结果区读取，防止发布撕裂快照。 */
	CPU2_FIXED_POINT_READ_COUNTER_BEFORE = 0, /* 先读取固定结果完成计数作为一致性基线。 */
	CPU2_FIXED_POINT_READ_RESULTS, /* 在基线之后读取固定结果寄存器块。 */
	CPU2_FIXED_POINT_READ_COUNTER_AFTER /* 结果读取完成后再次读取完成计数并复核未变化。 */
} Cpu2FixedPointSnapshotStage;

typedef struct {
	/* CPU2 剖面快照的完成锁存、完成计数、点数和来源键。 */
	uint32_t complete_latched; /* CPU2 已锁存剖面完成状态的值，用于和完成计数组合判断。 */
	uint32_t complete_counter; /* 剖面完整结果提交计数；每成功提交一份新点阵后递增。 */
	uint32_t measurement_points; /* 本次剖面声明的有效测点数，读取点阵前必须校验容量。 */
	uint32_t profile_source; /* 本次剖面结果来源，取值遵循 ProfileSource。 */
} Cpu2ProfileSnapshotKey;

typedef enum {
	/* 密度剖面点阵同步阶段；前后读取头部，中间分块抓取点阵，失败后进入有界重试等待。 */
	CPU2_PROFILE_SYNC_IDLE = 0, /* 当前没有剖面点阵同步任务。 */
	CPU2_PROFILE_SYNC_HEADER_BEFORE, /* 读取点阵前的剖面头和组合键。 */
	CPU2_PROFILE_SYNC_POINTS, /* 按容量分块读取剖面点阵寄存器。 */
	CPU2_PROFILE_SYNC_HEADER_AFTER, /* 点阵读取后再次读取剖面头并执行一致性比较。 */
	CPU2_PROFILE_SYNC_RETRY_WAIT /* 同步失败后等待退避时间到期再重试。 */
} Cpu2ProfileSyncStage;

typedef struct {
	/* CPU2 剖面点阵分阶段同步上下文；记录目标键、进度、超时节拍、分块偏移和有界重试状态。 */
	Cpu2ProfileSyncStage stage; /* 诊断记录对应的驱动操作阶段。 */
	Cpu2ProfileSyncStage retry_stage; /* 发生失败时需要重新进入的同步阶段。 */
	Cpu2ProfileSnapshotKey target_key; /* 本轮同步必须保持不变的目标剖面组合键。 */
	uint32_t snapshot_generation; /* 本轮同步开始时锁存的 CPU2 快照代际。 */
	uint32_t started_tick; /* 本轮剖面同步开始的 HAL 毫秒节拍，用于限制整轮最大耗时。 */
	uint32_t last_progress_tick; /* 最近一次成功推进同步阶段或分块偏移的 HAL 毫秒节拍。 */
	uint32_t last_frame_tick; /* 最近一次收到有效 CPU2 响应帧的 HAL 毫秒节拍。 */
	uint32_t retry_due_tick; /* 同步失败后允许再次进入 retry_stage 的最早 HAL 毫秒节拍。 */
	uint32_t point_register_offset; /* 下一分块相对剖面点阵起址的 16 位寄存器偏移。 */
	uint8_t block_retry_count; /* 当前剖面分块已经重试的次数；达到上限后本轮同步失败并进入退避等待。 */
	uint8_t round; /* 当前完整同步轮次编号，用于限制跨轮重试。 */
} Cpu2ProfileSyncContext;

/* 固定结果区防撕裂读取的当前阶段。 */
static Cpu2FixedPointSnapshotStage s_cpu2_fixed_point_stage = CPU2_FIXED_POINT_READ_COUNTER_BEFORE;
/* 两次完成计数之间读取的固定结果寄存器候选缓冲区。 */
static uint16_t s_cpu2_fixed_point_results[CPU2_FIXED_POINT_RESULT_REGISTER_COUNT];
/* 读取固定结果前抓取的完成计数寄存器。 */
static uint16_t s_cpu2_fixed_point_counter_before[CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT];
/* 读取固定结果后抓取的完成计数寄存器。 */
static uint16_t s_cpu2_fixed_point_counter_after[CPU2_FIXED_POINT_COUNTER_REGISTER_COUNT];
/* 当前 CPU2 私有输入寄存器读取请求的目标缓冲区。 */
static uint16_t *s_cpu2_private_input_destination = NULL;
/* 当前私有输入目标缓冲区可写入的 16 位字容量。 */
static uint16_t s_cpu2_private_input_capacity = 0U;
/* 当前私有输入读取请求的起始寄存器地址。 */
static uint16_t s_cpu2_private_input_start = 0U;
/* 当前私有输入读取响应已经完整复制到目标缓冲区的标志。 */
static bool s_cpu2_private_input_captured = false;
/* CPU2 密度剖面分块同步状态机上下文。 */
static Cpu2ProfileSyncContext s_cpu2_profile_sync = {0};
/* 读取剖面点阵前抓取的头部一致性寄存器。 */
static uint16_t s_cpu2_profile_header_before[CPU2_PROFILE_HEADER_REGISTER_COUNT];
/* 读取剖面点阵后复核的头部一致性寄存器。 */
static uint16_t s_cpu2_profile_header_after[CPU2_PROFILE_HEADER_REGISTER_COUNT];
/* 当前同步轮次尚未发布的剖面点阵候选寄存器缓冲区。 */
static uint16_t s_cpu2_profile_candidate_points[CPU2_PROFILE_POINT_REGISTER_CAPACITY];
/* 最近一次通过前后头一致性校验的剖面头部。 */
static uint16_t s_cpu2_profile_published_header[CPU2_PROFILE_HEADER_REGISTER_COUNT];
/* 最近一次完整发布的剖面点阵寄存器缓冲区。 */
static uint16_t s_cpu2_profile_published_points[CPU2_PROFILE_POINT_REGISTER_CAPACITY];
/* 从候选寄存器解码得到、尚未提交的密度分布结构。 */
static DensityDistribution s_cpu2_profile_candidate_distribution;
/* 最近一次已经原子提交给显示和外部协议的密度分布。 */
static DensityDistribution s_cpu2_profile_published_distribution;
/* 最近一次已发布剖面的组合快照键。 */
static Cpu2ProfileSnapshotKey s_cpu2_profile_published_key = {0};
/* 最近一次已发布剖面的 CPU2 测量周期计数。 */
static uint32_t s_cpu2_profile_published_cycle = 0U;
/* 已发布剖面缓冲区可供消费者读取的标志。 */
static bool s_cpu2_profile_published_valid = false;
/* 最近失败剖面键是否已经记录的标志。 */
static bool s_cpu2_profile_failed_key_valid = false;
/* 最近一次同步失败的剖面组合键，用于抑制同一坏快照的紧密重试。 */
static Cpu2ProfileSnapshotKey s_cpu2_profile_failed_key = {0};
/* 记录失败时的 CPU2 快照代际。 */
static uint32_t s_cpu2_profile_failed_snapshot_generation = 0U;
/* 失败剖面允许再次同步的最早 HAL 毫秒节拍。 */
static uint32_t s_cpu2_profile_failed_retry_due_tick = 0U;
/* 剖面同步故障已经上报且尚未恢复的状态。 */
static bool s_cpu2_profile_sync_fault_active = false;
/* 最近一帧 CPU2 原始设备状态，尚未经过剖面发布门控修正。 */
static DeviceState s_cpu2_raw_device_state = STATE_INIT;
/* 最近一帧 CPU2 原始故障码。 */
static uint32_t s_cpu2_raw_error_code = NO_ERROR;
/* 上一轮 CPU2 原始设备状态，用于检测状态边沿。 */
static DeviceState s_cpu2_last_raw_device_state = STATE_INIT;
/* 上一轮原始设备状态基线已经建立的标志。 */
static bool s_cpu2_last_raw_device_state_valid = false;
/* CPU2 已报告剖面完成、但完整点阵尚未发布的挂起标志。 */
static bool s_cpu2_profile_completion_state_pending = false;
/* 完整点阵已就绪、等待 CPU2 完成状态确认后提交的标志。 */
static bool s_cpu2_profile_commit_waiting_for_complete_state = false;

/* CPU3 到 CPU2 的内部通信请求类型；用于将普通读取、参数写入和命令写入分别纳入统计与故障归因。 */
typedef enum {
	/* CPU3 发往 CPU2 的内部 Modbus 请求类别。 */
	CPU2_COMM_REQUEST_READ = 0, /* 普通 CPU2 寄存器读取请求。 */
	CPU2_COMM_REQUEST_PARAMETER_WRITE, /* CPU2 参数保持寄存器写入请求。 */
	CPU2_COMM_REQUEST_COMMAND_WRITE /* CPU2 瞬时命令寄存器写入请求。 */
} Cpu2CommRequestKind;

/* 接收到的命令包数据暂存变量 */
static int RCV_functioncode = 0; /* 当前正在解析的 Modbus 功能码。 */
static int RCV_startaddress = 0; /* Modbus 协议地址配置，影响协议寻址或硬件访问。 */
static int RCV_registercnt = 0; /* 当前 Modbus 请求声明的寄存器数量。 */
static int SlaveTempBuffer[LTD_MODBUS_MAX_READ_REGISTERS]; /* 单帧寄存器数据缓冲区。 */

/* static void CPU2_Response03Process(char const *revframe); */
/**
 * @brief 解析CPU2的响应包 0x03 功能码。
 *
 * @param revframe CPU2 板间 Modbus 响应帧缓冲区；函数按功能码解析寄存器数据、写入确认或异常码。
 */
static void CPU2_Response03Process(uint8_t const *revframe);
/**
 * @brief 解析CPU2的响应包0x04功能码。
 *
 * @param revframe CPU2 板间 Modbus 响应帧缓冲区；函数按功能码解析寄存器数据、写入确认或异常码。
 */
static void CPU2_Response04Process(uint8_t const *revframe);
/**
 * @brief 处理 CPU2 对写多个寄存器指令的响应帧。
 * @param arr 响应帧数据缓冲区。
 * @param len 响应帧长度。
 * @note 当前保留接口，后续若需要确认 0x10 写入结果可在此解析。
 */
static void CPU2_Response10Process(uint8_t *arr, uint16_t len);
/**
 * @brief 把当前 CPU2 响应中的连续寄存器值写入 CPU3 保持寄存器或输入寄存器镜像。
 *
 * registertype --> false - 保持寄存器。
 * > true - 输入寄存器。
 *
 * @param registertype > false - 保持寄存器。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 */
static void PresetRegister(bool registertype, int const *registervalue);
/**
 * @brief 校验 CPU2 响应是否与当前请求的功能码、长度和回显字段一致。
 *
 * @details 调用场景：CRC 和从机地址校验通过后、清除连续请求失败计数前调用。
 * @note 关键约束：标准异常帧由 HostCommuProcess 单独识别；错功能码和不完整数据帧不得刷新通信有效状态。
 *
 * @param rcv 已经接收完成、等待校验或解析的板间通信帧。
 * @param len 输入数据的有效长度，单位字节。
 * @return 返回 CPU2_COMM_FAIL_NONE 表示响应功能码、长度和回显字段一致；否则返回 CPU2_COMM_FAIL_LENGTH 或
 *         CPU2_COMM_FAIL_FUNCTION。
 */
static Cpu2CommFailureReason CPU2_ResponseFrameFailureReason(uint8_t const *rcv, int len);
/**
 * @brief 判断当前 0x04 响应是否完整覆盖设备状态和错误码字段。
 *
 * @details 调用场景：输入寄存器响应写入缓存后决定是否允许恢复本机通信故障。
 * @note 关键约束：非状态分组不得用掉线前的旧缓存提前清除通信故障。
 *
 * @return true 表示当前响应功能码为 0x04，且读取区间完整覆盖设备状态和错误码两个 UInt32 字段；false 表示功能码不是 0x04，或任一字段没有被完整覆盖。
 */
static bool CPU2_ResponseContainsDeviceStatus(void);
/**
 * @brief 判断当前 0x03 响应是否完整覆盖共享协议版本字段。
 *
 * @details 调用场景：保持寄存器响应解析完成后确认当前 CPU2 连接的协议版本来源。
 * @note 关键约束：通信故障恢复后必须重新读回该字段，不能沿用掉线前缓存开放写入口。
 *
 * @return true 表示当前响应功能码为 0x03，且读取区间完整覆盖共享协议版本 UInt32 字段；false 表示功能码不是 0x03，或协议版本字段没有被完整覆盖。
 */
static bool CPU2_ResponseContainsProtocolVersion(void);
/**
 * @brief 记录 CPU2 合法响应并清除连续请求失败计数。
 *
 * @details 调用场景：响应通过长度、CRC 和从机地址校验后由主循环调用。
 * @note 关键约束：本函数不建立状态快照；只有包含状态和错误码的 0x04 响应才开放写入口。
 */
static void CPU2_CommMarkValidResponse(void);
/**
 * @brief 关闭所有对外公开快照门禁并请求完整重同步。
 *
 * @details 调用场景：CPU2连续请求失败达到三次重同步阈值时调用。
 * @note 关键约束：只处理数据新鲜度和完整重同步请求；连续十次报警计数由独立逻辑维护。
 */
static void CPU2_InvalidatePublicSnapshotFreshness(void);
/**
 * @brief 记录一次未获得合法CPU2响应的请求，并按连续失败阈值升级恢复动作。
 *
 * @details 调用场景：主循环处理响应超时、非法响应、UART 错误或 TX DMA 启动失败时调用。
 * @note 关键约束：前两次失败只计数并重试，第三次失败才废弃公开快照并完整重同步；
 *           连续十次失败仍置CPU2通信超时故障，ISR只置待处理标志。
 *
 * @param reason CPU2 板间通信失败原因枚举；用于区分启动失败、响应超时、帧错误、Modbus 异常和重试耗尽等失败阶段。
 */
static void CPU2_CommRecordFailure(Cpu2CommFailureReason reason);
/**
 * @brief 把一次UART5事务中的硬件错误位分别累计到健康计数。
 *
 * @details 调用场景：主循环解除同步等待并取得ISR锁存的HAL错误码后调用。
 * @note 关键约束：多个硬件错误位可分别累计，但整笔事务只增加一次总失败和连续失败。
 *
 * @param uart_error_code UART故障。
 */
static void CPU2_CommRecordUartFlags(uint32_t uart_error_code);
/**
 * @brief 使CPU3已发布分布点阵失效，并清除依赖旧通信会话的状态门禁。
 *
 * @details 调用场景：任一CPU2请求失败导致公开快照整体失效时。
 * @note 关键约束：恢复通信后必须重新执行完整头部和点阵复核，不能按重置后的相同计数复用旧结果。
 */
static void CPU2_ProfileInvalidatePublishedSnapshot(void);
/**
 * @brief 重置固定点私有读取阶段，并按需使已确认快照失效。
 *
 * @details 调用场景：冷启动、通信故障、三阶段任一读取失败或前后计数变化时。
 * @note 关键约束：候选缓冲从不直接暴露；失效后必须完成一轮全新握手才能再次开放。
 *
 * @param invalidate_snapshot true 表示同时作废现有固定点快照，false 表示只复位握手状态。
 */
static void CPU2_ResetFixedPointSnapshotHandshake(bool invalidate_snapshot);
/**
 * @brief 把一次CPU2输入寄存器响应捕获到调用方私有缓冲，不写公开缓存。
 *
 * @details 调用场景：固定点计数前读、24个结果寄存器候选读取和计数后读。
 * @note 关键约束：发送入口同步返回，退出前无条件撤销捕获目标，失败不得留下半帧目的地。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param out_regs 用于接收本次读取的连续寄存器快照。
 * @return true 表示请求成功且响应已完整捕获到调用方私有缓冲；false 表示参数或区间无效、请求失败、回调未捕获响应，或响应长度或地址与请求不一致。
 */
static bool CPU2_ReadInputRegistersPrivate(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs);
/**
 * @brief 每次调用推进固定点快照握手的一个Modbus请求。
 *
 * @details 调用场景：上电全量同步尾部，以及每轮普通运行轮询之后。
 * @note 关键约束：顺序固定为计数前读、私有结果读、计数后读；失败或变化仅丢弃本轮候选，
 *           已发布快照继续有效，连续三次通信失败由全局恢复策略统一失效。
 *
 * @return true 表示本次调用完成握手并发布了一代一致的固定点快照；false 表示握手尚在推进、当前阶段请求失败或需要重试，或两次代次读取不一致而未发布。
 */
static bool CPU2_PollFixedPointSnapshotHandshake(void);
/**
 * @brief 每次主循环调用最多推进一个分布点阵Modbus请求。
 *
 * @details 调用场景：普通轮询前优先调度，覆盖普通、国标、每米、区间、瓦锡兰、综合和SI结果。
 * @note 关键约束：每帧固定最多8点，失败按100/300/1000ms重试；三轮耗尽或20秒无进展才报错。
 *
 * @return true 表示分布点阵同步任务处于活动态，本次调度槽已用于读块、退避等待、重试处理、发布成功或发布错误，普通小帧不应再占用本槽；false 表示状态机为空闲态或落入非法阶段，本次没有活动点阵任务占用调度。
 */
static bool CPU2_ProfileSyncPoll(void);
/**
 * @brief 观察普通轮询读到的分布头，并在新完成代际出现时启动私有候选同步。
 *
 * @details 调用场景：公开0x04响应完整覆盖分布头后、对外返回主循环前。
 * @note 关键约束：新头只作为触发证据；候选完成前立即恢复上次已发布代际。
 */
static void CPU2_ProfileObservePublicHeader(void);
/**
 * @brief 在点阵提交前把CPU2原始分布完成态转换为对应测量中状态。
 *
 * @details 调用场景：每次完整状态分组解析后。
 * @note 关键约束：CPU2真实故障优先于本机20-13；成功提交后才恢复CPU2原始完成态。
 */
static void CPU2_ProfileApplyPublicStatusGate(void);
/**
 * @brief 返回CPU2请求失败原因的现场可读名称。
 *
 * @param reason CPU2 板间通信失败原因枚举；用于区分启动失败、响应超时、帧错误、Modbus 异常和重试耗尽等失败阶段。
 * @return 返回CPU2请求失败原因的现场可读名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *CPU2_CommFailureReasonText(Cpu2CommFailureReason reason);
/**
 * @brief 返回CPU2链路当前对外门禁状态。
 *
 * @return 返回CPU2链路当前对外门禁状态对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *CPU2_CommLinkStateText(void);
/**
 * @brief 输出当前失败请求的地址、功能码、计数和原始帧证据。
 *
 * @details 调用场景：主循环完成失败分类和健康计数后调用。
 * @note 关键约束：同一笔失败只在统一出口打印一次，避免CRC等解析分支重复刷屏。
 */
static void CPU2_CommLogRequestFailure(void);
/**
 * @brief 使用现行Modbus地址执行一次CPU2事务。
 *
 * @param f_code CPU2 板间 Modbus 请求功能码。
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param holddata 待发送的保持寄存器数据；解释方式由功能码和寄存器数量决定。
 * @return true 表示请求帧已发送，并收到地址、功能、长度、CRC 和事务语义均合法的 CPU2 响应；false 表示前置参数无效、发送或接收失败、超时、响应校验失败，或 CPU2 返回受支持的异常。
 */
static bool CPU2_CombinatePackage_SendWire(uint8_t f_code,
                                           uint16_t startadd,
                                           uint16_t registercnt,
                                           uint32_t *holddata);

/**
 * @brief 只接受当前 CPU2 共享契约实际可能返回的标准异常码。
 *
 * @param exception_code 异常码。
 * @return true 表示异常码是非法功能、非法地址、非法数据、从机故障或从机忙等当前共享契约支持值；false 表示异常码为零、未知或不应由 CPU2 返回。
 */
static bool CPU2_ModbusExceptionIsSupported(uint8_t exception_code)
{
	switch (exception_code) {
	case CPU2_MODBUS_EX_ILLEGAL_FUNCTION:
	case CPU2_MODBUS_EX_ILLEGAL_ADDRESS:
	case CPU2_MODBUS_EX_ILLEGAL_VALUE:
	case CPU2_MODBUS_EX_SLAVE_DEVICE_FAILURE:
	case CPU2_MODBUS_EX_SLAVE_DEVICE_BUSY:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 返回CPU2请求失败原因的现场可读名称。
 *
 * @param reason CPU2 板间通信失败原因枚举；用于区分启动失败、响应超时、帧错误、Modbus 异常和重试耗尽等失败阶段。
 * @return 返回CPU2请求失败原因的现场可读名称对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
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

/**
 * @brief 返回CPU2链路当前对外门禁状态。
 *
 * @return 返回CPU2链路当前对外门禁状态对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
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

/**
 * @brief 校验 CPU2 响应是否与当前请求的功能码、长度和回显字段一致。
 *
 * @details 调用场景：CRC 和从机地址校验通过后、清除连续请求失败计数前调用。
 * @note 关键约束：标准异常帧由 HostCommuProcess 单独识别；错功能码和不完整数据帧不得刷新通信有效状态。
 *
 * @param rcv 已经接收完成、等待校验或解析的板间通信帧。
 * @param len 输入数据的有效长度，单位字节。
 * @return 返回 CPU2_COMM_FAIL_NONE 表示响应功能码、长度和回显字段一致；否则返回 CPU2_COMM_FAIL_LENGTH 或
 *         CPU2_COMM_FAIL_FUNCTION。
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

/**
 * @brief 判断当前 0x04 响应是否完整覆盖设备状态和错误码字段。
 *
 * @details 调用场景：输入寄存器响应写入缓存后决定是否允许恢复本机通信故障。
 * @note 关键约束：非状态分组不得用掉线前的旧缓存提前清除通信故障。
 *
 * @return true 表示当前响应功能码为 0x04，且读取区间完整覆盖设备状态和错误码两个 UInt32 字段；false 表示功能码不是 0x04，或任一字段没有被完整覆盖。
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

/**
 * @brief 判断当前 0x03 响应是否完整覆盖共享协议版本字段。
 *
 * @details 调用场景：保持寄存器响应解析完成后确认当前 CPU2 连接的协议版本来源。
 * @note 关键约束：通信故障恢复后必须重新读回该字段，不能沿用掉线前缓存开放写入口。
 *
 * @return true 表示当前响应功能码为 0x03，且读取区间完整覆盖共享协议版本 UInt32 字段；false 表示功能码不是 0x03，或协议版本字段没有被完整覆盖。
 */
static bool CPU2_ResponseContainsProtocolVersion(void)
{
	return (RCV_functioncode == FUNCTIONCODE_READ_HOLDREGISTER) &&
		   LtdModbus_RangeContains((uint16_t)RCV_startaddress,
							   (uint16_t)RCV_registercnt,
							   HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION,
							   REG_SIZE_U32);
}

/**
 * @brief 记录 CPU2 合法响应并清除连续请求失败计数。
 *
 * @details 调用场景：响应通过长度、CRC 和从机地址校验后由主循环调用。
 * @note 关键约束：本函数不建立状态快照；只有包含状态和错误码的 0x04 响应才开放写入口。
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

/**
 * @brief 关闭所有对外公开快照门禁并请求完整重同步。
 *
 * @details 调用场景：CPU2连续请求失败达到三次重同步阈值时调用。
 * @note 关键约束：只处理数据新鲜度和完整重同步请求；连续十次报警计数由独立逻辑维护。
 */
static void CPU2_InvalidatePublicSnapshotFreshness(void)
{
	s_cpu2_has_status_snapshot = false;
	s_cpu2_has_parameter_snapshot = false;
	s_cpu2_has_protocol_snapshot = false;
	s_cpu2_confirmed_command_argument_mask = 0U;
	s_cpu2_factory_restore_refresh_pending = false;
	s_cpu2_protocol_version = 0U;
	s_cpu2_snapshot_generation++;
	CPU2_ProfileInvalidatePublishedSnapshot();
	CPU2_ResetFixedPointSnapshotHandshake(true);
	s_cpu2_snapshot_resync_requested = true;
	CPU3_LOG_WARNING("CPU2",
					 "连续失败达到%u次，公开快照已失效并请求完整重同步",
					 (unsigned int)CPU2_COMM_RESYNC_FAILURE_LIMIT);
}

/**
 * @brief 记录一次未获得合法CPU2响应的请求，并按连续失败阈值升级恢复动作。
 *
 * @details 调用场景：主循环处理响应超时、非法响应、UART 错误或 TX DMA 启动失败时调用。
 * @note 关键约束：前两次失败只计数并重试，第三次失败才废弃公开快照并完整重同步；
 *           连续十次失败仍置CPU2通信超时故障，ISR只置待处理标志。
 *
 * @param reason CPU2 板间通信失败原因枚举；用于区分启动失败、响应超时、帧错误、Modbus 异常和重试耗尽等失败阶段。
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

/**
 * @brief 把一次UART5事务中的硬件错误位分别累计到健康计数。
 *
 * @details 调用场景：主循环解除同步等待并取得ISR锁存的HAL错误码后调用。
 * @note 关键约束：多个硬件错误位可分别累计，但整笔事务只增加一次总失败和连续失败。
 *
 * @param uart_error_code UART故障。
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

/**
 * @brief 记录 UART5 错误并解除当前同步等待。
 *
 * @details 调用场景：HAL UART 错误回调中调用。
 * @note 关键约束：本函数可能处于 ISR 上下文，只置标志，不打印、不计数、不改设备状态。
 *
 * @param uart_error_code UART故障。
 */
void CPU2_CommNotifyUartErrorFromISR(uint32_t uart_error_code)
{
	if (wait_response) {
		s_cpu2_uart_error_pending |= uart_error_code;
		wait_response = false;
	}
}

/**
 * @brief 原子读取 CPU2 板间通信健康统计快照。
 *
 * @param out_snapshot 用于接收一次原子复制的 CPU2 板间通信健康统计快照。
 */
void CPU2_CommGetHealthSnapshot(Cpu2CommHealthSnapshot *out_snapshot)
{
	if (out_snapshot == NULL) {
		return;
	}

	*out_snapshot = s_cpu2_comm_health;
}

/**
 * @brief 输出当前失败请求的地址、功能码、计数和原始帧证据。
 *
 * @details 调用场景：主循环完成失败分类和健康计数后调用。
 * @note 关键约束：同一笔失败只在统一出口打印一次，避免CRC等解析分支重复刷屏。
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

/**
 * @brief 按通信健康摘要周期汇总 CPU2 链路、快照、成功失败计数和 UART 错误到异步日志。
 */
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

/**
 * @brief 判断当前连接是否已确认使用 CPU3 支持的共享协议。
 *
 * @details 调用场景：启动显示、快照门禁、命令下发和运行期轮询决策。
 * @note 关键约束：必须同时具备当前连接协议快照和值相等，不能使用默认值或掉线前缓存。
 *
 * @return true 表示当前连接已确认使用 CPU3 支持的共享协议；false 表示当前连接尚未确认使用 CPU3 支持的共享协议。
 */
bool CPU2_CommIsProtocolCompatible(void)
{
	return s_cpu2_has_protocol_snapshot &&
		   (s_cpu2_protocol_version == DEVICE_PROTOCOL_VERSION);
}

/**
 * @brief 判断当前连接是否已经确认存在共享协议版本不匹配。
 *
 * @details 调用场景：CPU3 状态页区分协议不匹配、同步未完成和通信超时。
 * @note 关键约束：协议快照无效时返回 false，不能把未知版本误报为协议不匹配。
 *
 * @return true 表示当前连接已经确认存在共享协议版本不匹配；false 表示当前连接尚未确认存在共享协议版本不匹配。
 */
bool CPU2_CommIsProtocolMismatch(void)
{
	return s_cpu2_has_protocol_snapshot &&
		   (s_cpu2_protocol_version != DEVICE_PROTOCOL_VERSION);
}

/**
 * @brief 判断是否仍处于等待 CPU2 首次状态和当前连接协议快照的同步阶段。
 *
 * @details 调用场景：CPU3 状态页选择通讯尝试页前调用。
 * @note 关键约束：协议字段未实际读回时不得用默认值或掉线前缓存显示协议不匹配/兼容；
 *           已确认协议不匹配时退出等待页显示兼容性提示，第十次连续失败后由故障页接管。
 *
 * @return true 表示仍处于等待 CPU2 首次状态和当前连接协议快照的同步阶段；false 表示已不再处于等待 CPU2 首次状态和当前连接协议快照的同步阶段。
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

/**
 * @brief 判断 CPU2 状态和参数快照是否完整、协议是否兼容且通信故障未锁存。
 *
 * @details 调用场景：CPU3 菜单或外部协议访问 CPU2 参数和命令前调用。
 * @note 关键约束：固定点结果使用独立门禁，不得阻塞无关参数访问。
 *
 * @return true 表示 CPU2 状态、参数和协议三类快照均已取得，通信故障未锁存，且 CPU2 或 CPU3 共享协议兼容；false 表示任一快照缺失、通信故障已锁存，或共享协议版本不兼容。
 */
bool CPU2_CommIsAvailable(void)
{
	return s_cpu2_has_status_snapshot &&
		   s_cpu2_has_parameter_snapshot &&
		   s_cpu2_has_protocol_snapshot &&
		   (!s_cpu2_comm_fault_active) &&
		   CPU2_CommIsProtocolCompatible();
}

/**
 * @brief 判断普通 CPU2 运行态快照和当前通信会话是否可用。
 *
 * @details 调用场景：外部协议读取设备状态、错误码、液位等非参数运行数据前调用。
 * @note 关键约束：固定点结果和参数快照分别使用独立门禁，不能扩大成全部运行数据失效。
 *
 * @return true 表示普通 CPU2 运行态快照和当前通信会话可用；false 表示普通 CPU2 运行态快照和当前通信会话不可用。
 */
bool CPU2_CommHasRuntimeSnapshot(void)
{
	return s_cpu2_has_status_snapshot &&
		   s_cpu2_has_protocol_snapshot &&
		   (!s_cpu2_comm_fault_active) &&
		   CPU2_CommIsProtocolCompatible();
}

/**
 * @brief 判断固定点测量和监测结果是否已在当前 CPU2 会话内完成一致性握手。
 *
 * @details 调用场景：LTD、DSM、SI 和 Wärtsilä 读取固定点派生字段前调用。
 * @note 关键约束：固定点快照必须建立在有效运行态会话上，掉线前结果不得继续发布。
 *
 * @return true 表示固定点测量和监测结果已在当前 CPU2 会话内完成一致性握手；false 表示固定点测量和监测结果尚未在当前 CPU2 会话内完成一致性握手。
 */
bool CPU2_CommHasFixedPointSnapshot(void)
{
	return CPU2_CommHasRuntimeSnapshot() &&
		   s_cpu2_has_fixed_point_snapshot;
}

/**
 * @brief 返回CPU2公开快照会话代际。
 *
 * @details 调用场景：SI本地投影在通信恢复后判断旧发布结果是否必须作废。
 * @note 关键约束：该代际只表示CPU3本机快照连续性，不属于CPU2/CPU3共享协议字段。
 *
 * @return 返回最近一次成功发布 CPU2 一致快照时递增的会话代际值。
 */
uint32_t CPU2_CommGetSnapshotGeneration(void)
{
	return s_cpu2_snapshot_generation;
}

/**
 * @brief 把命令参数保持寄存器范围转换成逐字段确认位图。
 *
 * @details 调用场景：七个命令参数完成FC10写入并取得合法ACK后调用。
 * @note 关键约束：调用范围必须完整位于七个连续32位字段内，位序与寄存器顺序一致。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @return 返回命令参数保持寄存器范围转换成逐字段确认位图对应的位掩码；各位含义由相邻枚举或宏定义。
 */
static uint32_t CPU2_CommCommandArgumentMaskForRange(uint16_t startadd,
												 uint16_t registercnt)
{
	uint32_t end;
	uint32_t address;
	uint32_t mask = 0U;

	if (!LtdModbus_HoldingWriteIsCommandArgumentOnly(startadd, registercnt)) {
		return 0U;
	}
	end = (uint32_t)startadd + (uint32_t)registercnt;
	for (address = (uint32_t)startadd; address < end; address += REG_STRIDE) {
		uint32_t field =
			(address - (uint32_t)HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL) /
			REG_STRIDE;
		mask |= (uint32_t)1U << field;
	}
	return mask;
}

/**
 * @brief 返回CPU2业务实际消费的命令前置参数位图。
 *
 * @details 调用场景：完整参数快照刷新期间判断某条命令能否依靠定向确认值下发，并在ACK后消费对应资格。
 * @note 关键约束：只映射当前CPU2消费点；无参数命令返回0，不能凭其它字段的确认资格越过完整快照门禁。
 *
 * @param cmd 命令值。该值是 CPU2 业务命令枚举；函数按该命令判断必需前置参数、发送条件、自中断能力或板间确认关系。
 * @return 返回CPU2业务实际消费的命令前置参数位图对应的位掩码；各位含义由相邻枚举或宏定义。
 */
static uint32_t CPU2_CommRequiredCommandArgumentMask(CommandType cmd)
{
	switch (cmd) {
	case CMD_CALIBRATE_OIL:
	case CMD_CORRECT_OIL:
		return CPU2_CommCommandArgumentMaskForRange(
			HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL, REG_STRIDE);
	case CMD_CALIBRATE_WATER:
		return CPU2_CommCommandArgumentMaskForRange(
			HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL, REG_STRIDE);
	case CMD_FIND_BOTTOM:
	case CMD_CALIBRATE_TANKHEIGHT:
		return CPU2_CommCommandArgumentMaskForRange(
			HOLDREGISTER_DEVICEPARAM_CALIBRATE_TANK_HEIGHT, REG_STRIDE);
	case CMD_MEASURE_SINGLE:
		return CPU2_CommCommandArgumentMaskForRange(
			HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION, REG_STRIDE);
	case CMD_MONITOR_SINGLE:
	case CMD_WARTSILA_DENSITY_RANGE:
		return CPU2_CommCommandArgumentMaskForRange(
			HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION, REG_STRIDE);
	case CMD_RUN_TO_POSITION:
		return CPU2_CommCommandArgumentMaskForRange(
			HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,
			REG_STRIDE);
	case CMD_MOVE_UP:
	case CMD_MOVE_DOWN:
	case CMD_FORCE_MOVE_UP:
	case CMD_FORCE_MOVE_DOWN:
		return CPU2_CommCommandArgumentMaskForRange(
			HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE, REG_STRIDE);
	default:
		return 0U;
	}
}

/**
 * @brief 在CPU2确认接收命令后消费该命令对应的参数确认资格。
 *
 * @details 调用场景：所有屏幕和外部协议共用的UART5命令成功出口。
 * @note 关键约束：只消费业务实际使用的字段；恢复出厂会覆盖整套参数，因此清除全部资格。
 *
 * @param cmd 命令值。该值是 CPU2 业务命令枚举；函数按该命令判断必需前置参数、发送条件、自中断能力或板间确认关系。
 */
static void CPU2_CommConsumeCommandArgumentConfirmation(CommandType cmd)
{
	if (cmd == CMD_RESTORE_FACTORY) {
		s_cpu2_confirmed_command_argument_mask = 0U;
		return;
	}

	s_cpu2_confirmed_command_argument_mask &=
		~CPU2_CommRequiredCommandArgumentMask(cmd);
}

/**
 * @brief 判断指定 CPU2 命令在当前通信状态下是否允许下发。
 *
 * @details 调用场景：菜单或外部协议准备写命令寄存器前调用。
 * @note 关键约束：普通命令依赖完整参数快照；刷新期间只有所需字段均已由ACK确认的带参命令可下发；
 *           取消命令保持原特例，但仍要求状态快照、协议兼容且通信故障未锁存。
 *
 * @param cmd 命令值。该值是 CPU2 业务命令枚举；函数按该命令判断必需前置参数、发送条件、自中断能力或板间确认关系。
 * @return true 表示指定 CPU2 命令在当前通信状态下允许下发；false 表示指定 CPU2 命令在当前通信状态下不允许下发。
 */
bool CPU2_CommCanSendCommand(CommandType cmd)
{
	if (cmd != CMD_CANCEL_MEASUREMENT) {
		uint32_t required_mask = CPU2_CommRequiredCommandArgumentMask(cmd);

		if (CPU2_CommIsAvailable()) {
			return true;
		}
		return (required_mask != 0U) &&
			   CPU2_CommHasRuntimeSnapshot() &&
			   ((s_cpu2_confirmed_command_argument_mask & required_mask) ==
				required_mask);
	}

	return s_cpu2_has_status_snapshot &&
		   s_cpu2_has_protocol_snapshot &&
		   (!s_cpu2_comm_fault_active) &&
		   CPU2_CommIsProtocolCompatible();
}

/**
 * @brief 从 CPU3 已确认的参数快照复制 LTD 保持寄存器。
 *
 * @details 调用场景：CPU3 对外以 LTD 协议独立响应 FC03 读请求时调用。
 * @note 关键约束：同步未完成、协议不匹配、通信故障或地址越界时不得返回旧缓存。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param out_regs 用于接收本次读取的连续寄存器快照。
 * @return true 表示参数快照已确认有效，请求区间合法且已复制到输出；false 表示输出指针为空、数量为零、区间越界，或当前尚无可发布的保持寄存器快照。
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

/**
 * @brief 判断 LTD 输入寄存器范围是否触及固定点结果或其代次字段。
 *
 * @details 调用场景：LTD FC04 复制输入寄存器快照前调用。
 * @note 关键约束：范围相交即整帧使用固定点门禁，不能拼接新运行态和旧固定点结果。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @return true 表示半开区间 [startadd, startadd + registercnt) 与固定点计数块或固定点结果块至少有一处重叠；false 表示两个块均未被本次输入寄存器范围触及。
 */
static bool CPU2_CommInputRangeTouchesFixedPoint(uint16_t startadd,
												 uint16_t registercnt)
{
	uint32_t range_start = (uint32_t)startadd;
	uint32_t range_end = range_start + (uint32_t)registercnt;
	uint32_t counter_start = (uint32_t)REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER;
	uint32_t counter_end =
		(uint32_t)REG_SINGLE_POINT_MON_SAMPLE_COUNTER + (uint32_t)REG_SIZE_U32;
	uint32_t result_start = (uint32_t)REG_SINGLE_POINT_MEAS_TEMP;
	uint32_t result_end = (uint32_t)REG_DENSITY_DIST_AVG_TEMP;

	return ((range_start < counter_end) && (range_end > counter_start)) ||
		   ((range_start < result_end) && (range_end > result_start));
}

/**
 * @brief 从 CPU3 已确认的状态快照复制 LTD 输入寄存器。
 *
 * @details 调用场景：CPU3 对外以 LTD 协议独立响应 FC04 读请求时调用。
 * @note 关键约束：普通输入只依赖运行态；固定点范围还必须完成固定点一致性握手。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param out_regs 用于接收本次读取的连续寄存器快照。
 * @return true 表示状态快照已确认有效，请求区间合法且已复制到输出；false 表示输出指针为空、数量为零、区间越界，或当前尚无可发布的输入寄存器快照。
 */
bool CPU2_CommReadInputSnapshot(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs)
{
	if ((out_regs == NULL) ||
		!LtdModbus_RangeWithin(startadd, registercnt, INPUTREGISTER_AMOUNT) ||
		(!CPU2_CommHasRuntimeSnapshot()) ||
		(CPU2_CommInputRangeTouchesFixedPoint(startadd, registercnt) &&
		 !CPU2_CommHasFixedPointSnapshot())) {
		return false;
	}

	memcpy(out_regs, &InputRegisterArray[startadd],
		   (size_t)registercnt * sizeof(uint16_t));
	return true;
}

/**
 * @brief 使当前完整参数快照失效并请求重新读取 CPU2 全部保持寄存器参数。
 *
 * @details 调用场景：参数写入已确认、写结果不确定或恢复出厂命令确认后调用。
 * @note 关键约束：刷新请求本身不得清除逐字段确认资格；结果不确定和会话失效由调用点显式清除。
 */
void CPU2_CommRequestParameterRefresh(void)
{
	s_cpu2_has_parameter_snapshot = false;
	s_cpu2_parameter_refresh_requested = true;
}

/**
 * @brief 在 CPU2 合法 FC10 ACK 后把本次确认值提交到 CPU3 参数镜像。
 *
 * @details 调用场景：外部协议或屏幕通过统一写入口成功写入非命令保持寄存器后调用。
 * @note 关键约束：必须先从当前 g_deviceParams 生成完整寄存器镜像，再覆盖本次范围并复用现有解析入口；
 *           未收到合法 ACK、标准异常或坏响应路径不得调用。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param wire_regs CPU2 已通过合法 FC10 ACK 确认的共享保持寄存器线序值，用于更新 CPU3 局部镜像。
 */
static void CPU2_CommApplyConfirmedHoldingWrite(uint16_t startadd,
											 uint16_t registercnt,
											 const uint16_t *wire_regs)
{
	WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	memcpy(&HoldingRegisterArray[startadd],
		   wire_regs,
		   (size_t)registercnt * sizeof(uint16_t));
	AnalysisHoldRegister();
	ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
}

/**
 * @brief 把 LTD 外部 FC10 的寄存器序列写穿到 CPU2，并以 CPU2 ACK 作为成功依据。
 *
 * @details 调用场景：CPU3 外部 LTD Modbus 从站处理写多个保持寄存器请求时调用。
 * @note 关键约束：共享参数均为 32 位字段，只接受偶数地址和偶数数量；合法 FC10 ACK 直接确认
 *           本次写值并更新局部镜像，完整参数快照统一在后台补读完成后恢复。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param wire_regs 待写入 CPU2 的连续 16 位共享寄存器线序值；相邻两个寄存器组成一个 32 位参数。
 * @return 返回标准 Modbus 结果：0 表示 CPU2 已确认；0x02、0x03、0x04、0x06 分别表示地址、数值、设备或忙错误，CPU2 异常码原样透传。
 */
uint8_t CPU2_CommWriteHoldingRegistersEx(uint16_t startadd,
                                         uint16_t registercnt,
                                         const uint16_t *wire_regs)
{
	uint32_t host_values[CPU2_MAX_EXTERNAL_WRITE_REGISTERS / 2U];
	uint32_t command_value;
	bool command_only;
	bool command_arguments_only;
	bool ret;

	s_cpu2_last_modbus_exception = CPU2_MODBUS_RESULT_OK;
	if ((wire_regs == NULL) ||
		(registercnt == 0U) ||
		(registercnt > CPU2_MAX_EXTERNAL_WRITE_REGISTERS) ||
		((startadd & 1U) != 0U) ||
		((registercnt & 1U) != 0U)) {
		return CPU2_MODBUS_EX_ILLEGAL_VALUE;
	}
	if (!LtdModbus_HoldingWriteRangeIsValid(startadd, registercnt)) {
		return CPU2_MODBUS_EX_ILLEGAL_ADDRESS;
	}

	/* 现有发送入口接收本机 uint32_t 值，并负责转换为共享协议的高字在前线序。 */
	for (uint16_t i = 0U; i < registercnt; i += 2U) {
		host_values[i / 2U] = ((uint32_t)wire_regs[i] << 16) |
									 (uint32_t)wire_regs[i + 1U];
	}

	command_only = (startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) &&
				   (registercnt == REG_STRIDE);
	command_arguments_only =
		LtdModbus_HoldingWriteIsCommandArgumentOnly(startadd, registercnt);
	command_value = ((uint32_t)wire_regs[0] << 16) | (uint32_t)wire_regs[1];
	if (command_only && !LtdModbus_CommandIsImplemented(command_value)) {
		return CPU2_MODBUS_EX_ILLEGAL_VALUE;
	}
	if (!CPU2_CommIsAvailable() &&
		!(command_only && CPU2_CommCanSendCommand((CommandType)command_value))) {
		return CPU2_MODBUS_EX_SLAVE_DEVICE_BUSY;
	}
	if (LtdModbus_HoldingWriteTouchesPersistent(startadd, registercnt) &&
		!LtdModbus_HoldingWriteIsCommandArgumentOnly(startadd, registercnt) &&
		!DeviceState_AllowsPersistentParamWrite(
			g_measurement.device_status.device_state)) {
		return CPU2_MODBUS_EX_SLAVE_DEVICE_BUSY;
	}
	ret = CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
									startadd,
									registercnt,
									host_values);
	if (!ret) {
		if (s_cpu2_last_modbus_exception != CPU2_MODBUS_RESULT_OK) {
			return s_cpu2_last_modbus_exception;
		}
		return CPU2_MODBUS_EX_SLAVE_DEVICE_FAILURE;
	}

	if (command_only) {
		/* 命令是一次性写槽，CPU2确认接收后CPU3不得长期保留旧命令影子。 */
		g_deviceParams.command = CMD_NONE;
		WriteDeviceParamsToHoldingRegisters(HoldingRegisterArray);
	} else {
		CPU2_CommApplyConfirmedHoldingWrite(startadd, registercnt, wire_regs);
		if (command_arguments_only) {
			s_cpu2_confirmed_command_argument_mask |=
				CPU2_CommCommandArgumentMaskForRange(startadd, registercnt);
		}
		CPU2_CommRequestParameterRefresh();
	}
	return CPU2_MODBUS_RESULT_OK;
}

/**
 * @brief 向 CPU2 连续写入板间 Modbus 保持寄存器。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param wire_regs 待写入 CPU2 的连续 16 位共享寄存器线序值；元素数量必须等于 registercnt。
 * @return true 表示连续保持寄存器写事务返回 CPU2_MODBUS_RESULT_OK；false 表示请求参数/区间无效、链路不可用、响应校验失败或 CPU2 返回 Modbus 异常。
 */
bool CPU2_CommWriteHoldingRegisters(uint16_t startadd,
									uint16_t registercnt,
									const uint16_t *wire_regs)
{
	return CPU2_CommWriteHoldingRegistersEx(startadd, registercnt, wire_regs) ==
		   CPU2_MODBUS_RESULT_OK;
}

/**
 * @brief 返回最近一次 CPU2 板间事务收到的标准 Modbus 异常码。
 *
 * @return 返回最近一次板间同步事务收到的标准 Modbus 异常码；0 表示没有标准异常响应。
 */
uint8_t CPU2_CommGetLastModbusException(void)
{
	return s_cpu2_last_modbus_exception;
}

/**
 * @brief 完成一次未取得合法响应的 CPU2 请求，并按事务确定性处理参数确认资格。
 *
 * @details 调用场景：发送后超时、UART错误、CRC/地址/功能码/长度错误或TX DMA启动失败。
 * @note 关键约束：普通读单次失败不清确认位；TX DMA未启动属于确定未发送；参数写结果不确定时
 *           清除全部确认位并补读，普通命令只消费对应确认位，恢复出厂仍强制全量补读。
 *
 * @param request_kind 刚完成或失败的 CPU2 板间异步请求类型。
 * @param request_command 命令。
 * @return 本函数固定返回 false，表示本次 CPU2 请求已按失败收口；它会记录失败结果，并按请求类型撤销或保留参数确认资格，不代表后续请求永久不可用。
 */
static bool CPU2_CommFinishFailedRequest(Cpu2CommRequestKind request_kind,
										 CommandType request_command)
{
	CPU2_CommRecordFailure(s_cpu2_response_failure_reason);
	CPU2_CommLogRequestFailure();
	if (s_cpu2_response_failure_reason == CPU2_COMM_FAIL_TX_DMA) {
		return false;
	}
	if (request_kind == CPU2_COMM_REQUEST_PARAMETER_WRITE) {
		s_cpu2_confirmed_command_argument_mask = 0U;
		CPU2_CommRequestParameterRefresh();
	} else if (request_kind == CPU2_COMM_REQUEST_COMMAND_WRITE) {
		CPU2_CommConsumeCommandArgumentConfirmation(request_command);
		if (request_command == CMD_RESTORE_FACTORY) {
			CPU2_CommRequestParameterRefresh();
		}
	}
	return false;
}

/**
 * @brief 重置固定点私有读取阶段，并按需使已确认快照失效。
 *
 * @details 调用场景：冷启动、通信故障、三阶段任一读取失败或前后计数变化时。
 * @note 关键约束：候选缓冲从不直接暴露；失效后必须完成一轮全新握手才能再次开放。
 *
 * @param invalidate_snapshot true 表示同时作废现有固定点快照，false 表示只复位握手状态。
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

/**
 * @brief 把一次CPU2输入寄存器响应捕获到调用方私有缓冲，不写公开缓存。
 *
 * @details 调用场景：固定点计数前读、24个结果寄存器候选读取和计数后读。
 * @note 关键约束：发送入口同步返回，退出前无条件撤销捕获目标，失败不得留下半帧目的地。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param out_regs 用于接收本次读取的连续寄存器快照。
 * @return true 表示请求成功且响应已完整捕获到调用方私有缓冲；false 表示参数或区间无效、请求失败、回调未捕获响应，或响应长度/地址与请求不一致。
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

/**
 * @brief 将连续两个高字在前的 Modbus 寄存器合并为 uint32。
 *
 * @param regs 连续寄存器值缓冲区。函数按既定寄存器数量只读 regs[]，并按高低字、字段偏移或协议映射解析业务值。
 * @param offset 相对起始位置的偏移量。
 * @return 返回将连续两个高字在前的 Modbus 寄存器合并为 uint32得到的主机整数值；函数只处理既定字节序或编码，不执行范围校验。
 */
static uint32_t CPU2_ReadU32FromPrivateRegs(const uint16_t *regs, uint16_t offset)
{
	return ((uint32_t)regs[offset] << 16) | (uint32_t)regs[offset + 1U];
}

/**
 * @brief 把已验证同代的固定点结果块与两个代际一次提交到公开缓存。
 *
 * @details 调用场景：计数后读与计数前读完全一致后。
 * @note 关键约束：公开寄存器和g_measurement在同一短临界区更新，外部读者不会看到六字段混代。
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

/**
 * @brief 每次调用推进固定点快照握手的一个Modbus请求。
 *
 * @details 调用场景：上电全量同步尾部，以及每轮普通运行轮询之后。
 * @note 关键约束：顺序固定为计数前读、私有结果读、计数后读；失败或变化仅丢弃本轮候选，
 *           已发布快照继续有效，连续三次通信失败由全局恢复策略统一失效。
 *
 * @return true 表示本次调用完成握手并发布了一代一致的固定点快照；false 表示握手尚在推进、当前阶段请求失败/需重试，或两次代次读取不一致而未发布。
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

/**
 * @brief 按高字在前顺序把 32 位值写入两个公开输入寄存器。
 *
 * @param address CPU2 输入寄存器数组中的目标起始地址；函数从该位置写入 32 位值的高、低两个 16 位字。
 * @param value 待编码写入线格式的 32 位数值。
 */
static void CPU2_ProfileWriteU32ToInput(uint16_t address, uint32_t value)
{
	InputRegisterArray[address] = (uint16_t)(value >> 16);
	InputRegisterArray[address + 1U] = (uint16_t)value;
}

/**
 * @brief 用有符号节拍差判断期限是否到达，保持 32 位计时回绕安全。
 *
 * @param now 本次判断使用的当前系统节拍，单位 ms；调用方在同一轮处理内复用该快照，避免多次取时造成边界漂移。
 * @param due 本次异步调度计划到期的 HAL 毫秒节拍。
 * @return true 表示 now 按 32 位回绕安全比较已经到达或超过 due；false 表示截止节拍尚未到达。
 */
static bool CPU2_ProfileTickReached(uint32_t now, uint32_t due)
{
	return ((int32_t)(now - due) >= 0);
}

/**
 * @brief 判断设备状态是否属于任一分布或 Profile 测量完成态。
 *
 * @param state CPU2 共享设备状态；各密度分布、瓦锡兰、区间、密度计和综合测量完成态均视为 Profile 完成。
 * @return true 表示设备状态属于任一分布或 Profile 测量完成态；false 表示设备状态不属于任一分布或 Profile 测量完成态。
 */
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

/**
 * @brief 把完成态还原为对应测量中状态，避免候选复核期间提前公开完成。
 *
 * @param state 已完成的 Profile 设备状态；函数在候选复核期间还原为对应测量中状态。
 * @return 返回完成态对应的测量中状态；覆盖瓦锡兰、国标分布、合成、计量、间隔和普通分布六类流程。
 */
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

/**
 * @brief 从分布头解析完成锁存、完成计数、点数和来源，组成候选代际键。
 *
 * @param header 分布测量头寄存器快照，至少覆盖完成锁存、完成计数、测点数和 profile 来源字段。
 * @return 返回由分布头完成锁存、完成计数、点数和来源组成的 Cpu2ProfileSnapshotKey。
 */
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

/**
 * @brief 校验候选已完成、点数在范围内且来源已定义。
 *
 * @param key 待校验的分布测量候选代际键；必须已完成、测点数合法且来源已定义。
 * @return true 表示上述校验全部通过；false 表示至少一项校验未通过。
 */
static bool CPU2_ProfileKeyIsValid(const Cpu2ProfileSnapshotKey *key)
{
	return (key->complete_latched != 0U) &&
		   (key->measurement_points > 0U) &&
		   (key->measurement_points <= MAX_MEASUREMENT_POINTS) &&
		   (key->profile_source >= (uint32_t)PROFILE_SOURCE_STANDARD) &&
		   (key->profile_source <= (uint32_t)PROFILE_SOURCE_SI);
}

/**
 * @brief 比较两份剖面候选代际键是否完全一致。
 *
 * @param left 区间左端值或左侧比较对象。
 * @param right 区间右端值或右侧比较对象。
 * @return true 表示完成锁存、完成计数、测量点数和 Profile 来源四个代际字段全部相同；false 表示两份候选不属于同一可发布代际。
 */
static bool CPU2_ProfileKeyEqual(const Cpu2ProfileSnapshotKey *left,
								 const Cpu2ProfileSnapshotKey *right)
{
	return (left->complete_latched == right->complete_latched) &&
		   (left->complete_counter == right->complete_counter) &&
		   (left->measurement_points == right->measurement_points) &&
		   (left->profile_source == right->profile_source);
}

/**
 * @brief 清除一次点阵最终失败留下的完整代际和自动重试计时。
 *
 * @details 调用场景：通信会话失效、新代际出现或候选最终同步成功后。
 * @note 关键约束：只清CPU3本机失败记录，不修改CPU2原始状态和已发布点阵。
 */
static void CPU2_ProfileClearFailedKey(void)
{
	memset(&s_cpu2_profile_failed_key, 0, sizeof(s_cpu2_profile_failed_key));
	s_cpu2_profile_failed_key_valid = false;
	s_cpu2_profile_failed_snapshot_generation = 0U;
	s_cpu2_profile_failed_retry_due_tick = 0U;
}

/**
 * @brief 把CPU2最近一次原始状态恢复到CPU3公开状态区。
 *
 * @details 调用场景：本机20-13撤销、点阵重试恢复或CPU2真实故障需要优先显示时。
 * @note 关键约束：不得用CPU3本机点阵故障继续遮盖CPU2发布的真实错误码。
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

/**
 * @brief 同步期间恢复上一次已确认点阵，保持旧代际标识不变。
 *
 * @details 调用场景：普通轮询先读到新完成头、候选点阵尚未通过前后复核时。
 * @note 关键约束：无历史快照时公开区清零；完成状态由独立门禁保持为忙。
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

/**
 * @brief 使CPU3已发布分布点阵失效，并清除依赖旧通信会话的状态门禁。
 *
 * @details 调用场景：任一CPU2请求失败导致公开快照整体失效时。
 * @note 关键约束：恢复通信后必须重新执行完整头部和点阵复核，不能按重置后的相同计数复用旧结果。
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

/**
 * @brief 取消已被CPU2正常作废的旧分布同步任务。
 *
 * @details 调用场景：新测量、命令切换或Point0使完成锁存清零时。
 * @note 关键约束：正常作废不计入块重试、整轮重试或20-13最终故障。
 *
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
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

/**
 * @brief 根据剖面头启动 CPU2 剖面结果分段同步。
 *
 * @param observed 本次从 CPU2 头寄存器观测到的候选代际键，用于建立或重启 profile 会话。
 */
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

/**
 * @brief 发布 CPU2 剖面同步失败状态和诊断原因。
 */
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

/**
 * @brief 单轮失败后按固定延时重启全量同步；轮次耗尽则发布最终故障。
 *
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 */
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

/**
 * @brief 记录剖面数据块读取失败并决定重试或终止。
 *
 * @param failed_stage SI Profile 拉取失败时所在的异步阶段。
 */
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

/**
 * @brief 记录剖面数据块同步进展并清零当前块重试计数。
 */
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

/**
 * @brief 从已同步的数据块组装并校验剖面候选快照。
 */
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

/**
 * @brief 把前后代际一致的候选头和N个点一次提交为CPU3公开快照。
 *
 * @details 调用场景：HEADER_AFTER确认完成计数、点数、来源和完成锁存均未变化后。
 * @note 关键约束：N之后的公开尾部必须清零；完成态与点阵在同一临界区开放。
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

/**
 * @brief 每次主循环调用最多推进一个分布点阵Modbus请求。
 *
 * @details 调用场景：普通轮询前优先调度，覆盖普通、国标、每米、区间、瓦锡兰、综合和SI结果。
 * @note 关键约束：每帧固定最多8点，失败按100/300/1000ms重试；三轮耗尽或20秒无进展才报错。
 *
 * @return true 表示分布点阵同步任务处于活动态，本次调度槽已用于读块、退避等待、重试处理、发布成功或发布错误，普通小帧不应再占用本槽；false 表示状态机为空闲态或落入非法阶段，本次没有活动点阵任务占用调度。
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

/**
 * @brief 观察普通轮询读到的分布头，并在新完成代际出现时启动私有候选同步。
 *
 * @details 调用场景：公开0x04响应完整覆盖分布头后、对外返回主循环前。
 * @note 关键约束：新头只作为触发证据；候选完成前立即恢复上次已发布代际。
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

/**
 * @brief 在点阵提交前把CPU2原始分布完成态转换为对应测量中状态。
 *
 * @details 调用场景：每次完整状态分组解析后。
 * @note 关键约束：CPU2真实故障优先于本机20-13；成功提交后才恢复CPU2原始完成态。
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

/**
 * @brief 校验并解析一帧 CPU2 板间 Modbus 响应，更新请求结果和对应数据镜像。
 *
 * @param rcv 已经接收完成、等待校验或解析的板间通信帧。
 * @param len 输入数据的有效长度，单位字节。
 * @return true 表示收到并处理了合法正常响应或标准 Modbus 异常响应；false 表示帧长、CRC、地址、功能码或响应结构校验失败。
 */
bool HostCommuProcess(uint8_t *rcv, int len) {
	Cpu3Log_Frame(CPU3_LOG_LEVEL_DEBUG,
				  "CPU2",
				  "接收",
				  rcv,
				  (len > 0) ? (uint16_t)len : 0U);
	s_cpu2_response_failure_reason = CPU2_COMM_FAIL_NONE;
	s_cpu2_last_modbus_exception = CPU2_MODBUS_RESULT_OK;
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
	if (rcv[1] == ((uint8_t)RCV_functioncode | 0x80U)) {
		if (len != 5) {
			s_cpu2_response_failure_reason = CPU2_COMM_FAIL_LENGTH;
			return false;
		}
		if (!CPU2_ModbusExceptionIsSupported(rcv[2])) {
			s_cpu2_response_failure_reason = CPU2_COMM_FAIL_FUNCTION;
			return false;
		}
		s_cpu2_last_modbus_exception = rcv[2];
		s_cpu2_comm_health.last_modbus_exception_code =
			s_cpu2_last_modbus_exception;
		CPU2_CommMarkValidResponse();
		CPU3_LOG_WARNING("CPU2",
						 "收到标准Modbus异常 功能码=0x%02X 异常码=0x%02X",
						 (unsigned int)RCV_functioncode,
						 (unsigned int)s_cpu2_last_modbus_exception);
		return true;
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

/* 上电首轮同步寄存器分组数量；由 poweron_groups 数组长度推导，新增分组无需手工改计数。 */
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

/* 运行期周期同步寄存器分组数量；由 runtime_groups 数组长度推导。 */
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

/* 保持寄存器强制刷新分组数量；由 refresh_hold_groups 数组长度推导，避免表与计数失配。 */
#define REFRESH_HOLD_GROUP_COUNT (sizeof(refresh_hold_groups) / sizeof(refresh_hold_groups[0]))

/**
 * @brief 按恢复、协议确认、全量同步、参数刷新和运行轮询的优先级，每次最多向 CPU2 发起一个板间请求。
 *
 * 连续失败触发重同步时先复位上电组、运行组、保持寄存器刷新、固定点握手和 SI 点阵同步状态；通信故障恢复阶段只轮询包含设备状态的输入组。
 * 取得协议版本且确认兼容后，依次完成上电参数组和固定点私有快照握手；运行期再交替推进 SI 点阵同步、保持寄存器整组刷新、普通输入组和固定点快照。
 * parameter_update_flag 变化或外部参数写确认会启动一次性保持寄存器补读，完成后再更新 CPU3 参数镜像和快照有效标志。
 *
 * @note 所有阶段都采用单请求推进；协议不兼容时只保留版本探测，参数和固定点公开快照必须等整轮同步完成后才标记有效。
 */
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
		s_cpu2_factory_restore_refresh_pending = false;
		s_cpu2_confirmed_command_argument_mask = 0U;
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
		s_cpu2_factory_restore_refresh_pending = false;
		s_cpu2_confirmed_command_argument_mask = 0U;
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
		s_cpu2_factory_restore_refresh_pending = false;
		s_cpu2_confirmed_command_argument_mask = 0U;
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
		s_cpu2_factory_restore_refresh_pending = false;
		s_cpu2_confirmed_command_argument_mask = 0U;
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

	/*
	 * 恢复出厂ACK不能立即补读，否则CPU2主循环尚未完成默认值保存时可能重新发布旧参数。
	 * 必须继续轮询状态，等待parameter_update_flag变化后再进入下方完整保持寄存器刷新。
	 */
	if (s_cpu2_factory_restore_refresh_pending) {
		s_cpu2_parameter_refresh_requested = false;
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
			s_cpu2_confirmed_command_argument_mask = 0U;
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
		s_cpu2_factory_restore_refresh_pending = false;
		s_cpu2_has_parameter_snapshot = false;
		return;
	}

}

/**
 * @brief 判断SI点阵拉取时允许接受的生命周期阶段。
 *
 * @details 调用场景：区分正常Complete候选和CPU3重启后Point0前遗留的上一轮快照。
 * @note 关键约束：上一轮快照只允许PREPARING、ABORTED或FAILED，不能放宽到Point0后的活动阶段。
 *
 * @param phase 当前状态机阶段。
 * @param previous_snapshot true 表示选择上一轮完整 SI 点阵，false 表示选择当前候选或已发布点阵。
 * @return true 表示对“SI点阵拉取时允许接受的生命周期阶段”的判断成立；false 表示对“SI点阵拉取时允许接受的生命周期阶段”的判断不成立。
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

/**
 * @brief 查询已经由统一异步状态机确认并发布的SI Profile点阵代际。
 *
 * @details 调用场景：正常Complete候选发布，或CPU3重启后恢复Point0前保留的上一轮结果。
 * @note 关键约束：本函数不发起UART请求；异步同步未完成时返回false，调用方继续保持忙。
 *
 * @param out_key 用于返回 SI Profile 点阵的代次、数量和来源标识。
 * @param previous_snapshot true 表示选择上一轮完整 SI 点阵，false 表示选择当前候选或已发布点阵。
 * @return true 表示指定当前或上一轮 SI Profile 已发布有效，且点阵和代际键已复制给调用方；false 表示输出参数无效，或请求的 Profile 尚无完整已发布代际。
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

/**
 * @brief 拉取CPU2当前Complete阶段的SI候选点阵。
 *
 * @details 调用场景：CPU3观察到本周期完成计数沿后执行最终发布。
 * @note 关键约束：只接受Complete阶段，并沿用完整分块前后代际复核。
 *
 * @param out_key 用于返回 SI Profile 点阵的代次、数量和来源标识。
 * @return true 表示当前 Complete 阶段的 SI 候选点阵已有完整发布快照；false 表示尚未发布、代际不完整，或输出参数无效。
 */
bool CPU2_CommFetchSiProfileCandidate(Cpu2SiProfileCandidateKey *out_key)
{
	return CPU2_CommGetPublishedSiProfile(out_key, false);
}

/**
 * @brief 拉取CPU2在Point0前仍保留的上一轮完整SI点阵。
 *
 * @details 调用场景：CPU3冷启动时处于PREPARING，或Point0前已经ABORTED/FAILED。
 * @note 关键约束：只接受旧SI来源、完成锁存、有效点数和稳定代际，不重建Profile时间。
 *
 * @param out_key 用于返回 SI Profile 点阵的代次、数量和来源标识。
 * @return true 表示 Point0 前保留的上一轮 SI 点阵已有完整发布快照；false 表示没有上一轮快照、快照无效，或输出参数无效。
 */
bool CPU2_CommFetchSiPreviousSnapshot(Cpu2SiProfileCandidateKey *out_key)
{
	return CPU2_CommGetPublishedSiProfile(out_key, true);
}

/**
 * @brief 使用stateformodbus.h中的现行地址发起CPU2标准Modbus事务。
 *
 * @details 调用场景：菜单、周期轮询、快照握手和外部LTD网关共用。
 * @note 关键约束：读取按标准125寄存器上限拆帧；写入必须等待CPU2合法ACK。
 *
 * @param f_code CPU2 板间 Modbus 请求功能码。
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param holddata 待发送的保持寄存器数据；解释方式由功能码和寄存器数量决定。
 * @return true 表示请求无需上线发送即可由有效快照满足，或现行地址已转换成线格式并完成 CPU2 事务；false 表示功能码、区间或参数无效，快照不可用，或线格式事务失败。
 */
bool CPU2_CombinatePackage_Send(uint8_t f_code,
								uint16_t startadd,
								uint16_t registercnt,
								uint32_t *holddata)
{
	uint16_t processed = 0U;

	s_cpu2_last_modbus_exception = CPU2_MODBUS_RESULT_OK;
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
		 * CPU3只用状态白名单提前抑制普通持久参数写；
		 * 命令前置参数需要先写后发命令，具体命令是否切换仍由CPU2原有命令入口裁决。
		 */
		if (LtdModbus_HoldingWriteTouchesPersistent(startadd, registercnt) &&
			!LtdModbus_HoldingWriteIsCommandArgumentOnly(startadd, registercnt) &&
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

/**
 * @brief 使用现行Modbus地址执行一次CPU2事务。
 *
 * @param f_code CPU2 板间 Modbus 请求功能码。
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param registercnt 本次连续访问的寄存器数量。
 * @param holddata 待发送的保持寄存器数据；解释方式由功能码和寄存器数量决定。
 * @return true 表示请求帧已发送，并收到地址、功能、长度、CRC 和事务语义均合法的 CPU2 响应；false 表示前置参数无效、发送/接收失败、超时、响应校验失败或 CPU2 返回受支持的异常。
 */
static bool CPU2_CombinatePackage_SendWire(uint8_t f_code,
										   uint16_t startadd,
										   uint16_t registercnt,
										   uint32_t *holddata) {
	bool command_write_allowed = false;
	bool command_write = (f_code == FUNCTIONCODE_WRITE_MULREGISTER) &&
		(startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) &&
		(registercnt == REG_STRIDE);
	Cpu2CommRequestKind request_kind = CPU2_COMM_REQUEST_READ;
	CommandType request_command = CMD_NONE;

	/* 每个板间事务只在边界报告进度，等待响应循环内部不得给看门狗续命。 */
	CPU3_WatchdogReportProgress();

	/* 普通写必须通过完整快照门禁；命令写统一复用取消特例或逐字段ACK确认门禁。 */
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) &&
		command_write && (registercnt == 2U) &&
		(holddata != NULL)) {
		command_write_allowed = CPU2_CommCanSendCommand((CommandType)(*holddata));
	}
	if (f_code == FUNCTIONCODE_WRITE_MULREGISTER) {
		request_kind = command_write ?
			CPU2_COMM_REQUEST_COMMAND_WRITE :
			CPU2_COMM_REQUEST_PARAMETER_WRITE;
		if (command_write && (holddata != NULL)) {
			request_command = (CommandType)(*holddata);
		}
	}
	if ((f_code == FUNCTIONCODE_WRITE_MULREGISTER) && !CPU2_CommIsAvailable()) {
		if (!command_write_allowed) {
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
	s_cpu2_last_modbus_exception = CPU2_MODBUS_RESULT_OK;
	s_cpu2_response_failure_reason = CPU2_COMM_FAIL_TX_DMA;
	if (!sendToCPU2(arr, len, false)) {
		CPU3_WatchdogReportProgress();
		return CPU2_CommFinishFailedRequest(request_kind, request_command);
	}
	/* 等待接收完成 */
	uint32_t timeout = HAL_GetTick();
	while (wait_response) {
		/* 等待 CPU2 应答超过协议门限后，清除等待标志并记录超时归因，再由统一失败出口决定重试或完整重同步。 */
		if ((HAL_GetTick() - timeout) > CPU2_RESPONSE_TIMEOUT_MS)
				{
			wait_response = false;    /* 防止一直 True */
			s_cpu2_response_failure_reason = CPU2_COMM_FAIL_TIMEOUT;
			CPU3_WatchdogReportProgress();
			return CPU2_CommFinishFailedRequest(request_kind, request_command);
		}
	}
	if (s_cpu2_uart_error_pending != HAL_UART_ERROR_NONE) {
		uint32_t uart_error_code = s_cpu2_uart_error_pending;
		s_cpu2_uart_error_pending = HAL_UART_ERROR_NONE;
		s_cpu2_request_uart_error_code = uart_error_code;
		CPU2_CommRecordUartFlags(uart_error_code);
		s_cpu2_response_failure_reason = CPU2_COMM_FAIL_UART;
		CPU3_WatchdogReportProgress();
		return CPU2_CommFinishFailedRequest(request_kind, request_command);
	}
	if (!HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN)) {
		CPU3_WatchdogReportProgress();
		return CPU2_CommFinishFailedRequest(request_kind, request_command);
	}
	if (s_cpu2_last_modbus_exception != CPU2_MODBUS_RESULT_OK) {
		CPU3_WatchdogReportProgress();
		return false;
	}
	if (command_write && (holddata != NULL)) {
		CommandType confirmed_command = (CommandType)(*holddata);

		CPU2_CommConsumeCommandArgumentConfirmation(confirmed_command);
		if (confirmed_command == CMD_RESTORE_FACTORY) {
			/*
			 * ACK只表示CPU2已接收命令，默认参数尚未必保存完成。
			 * 先关闭读取门禁，等待parameter_update_flag变化后再执行完整补读。
			 */
			s_cpu2_has_parameter_snapshot = false;
			s_cpu2_parameter_refresh_requested = false;
			s_cpu2_factory_restore_refresh_pending = true;
		}
	}
	CPU3_WatchdogReportProgress();
	return true;
}
/**
 * @brief 向CPU2发送数据包。
 *
 * @param arr 待打包并发送给 CPU2 的寄存器数据数组。
 * @param len 输入数据的有效长度，单位字节。
 * @param flag_fromhost true 表示数据来自外部主机写入，false 表示来自本机流程。
 * @return true 表示待发长度和缓冲有效，UART DMA 发送已成功启动并完成；false 表示参数无效、UART 忙/错误或发送等待超时。
 */
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
/**
 * @brief 解析CPU2的响应包 0x03 功能码。
 *
 * @param revframe CPU2 板间 Modbus 响应帧缓冲区；函数按功能码解析寄存器数据、写入确认或异常码。
 */
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
	/*
	 * 命令寄存器是一次性写槽。即使CPU2在主循环取命令前仍回读pending值，
	 * CPU3也不得把它重新发布成长期保持参数。
	 */
	if (LtdModbus_RangeContains((uint16_t)RCV_startaddress,
								(uint16_t)RCV_registercnt,
								HOLDREGISTER_DEVICEPARAM_COMMAND,
								REG_STRIDE)) {
		HoldingRegisterArray[HOLDREGISTER_DEVICEPARAM_COMMAND] = 0U;
		HoldingRegisterArray[HOLDREGISTER_DEVICEPARAM_COMMAND + 1U] = 0U;
	}

	/* 3. 按现行直接地址解析保持寄存器并刷新g_deviceParams。 */
	AnalysisHoldRegister();
	ReadDeviceParamsFromHoldingRegisters(HoldingRegisterArray);
}

/**
 * @brief 解析CPU2的响应包0x04功能码。
 *
 * @param revframe CPU2 板间 Modbus 响应帧缓冲区；函数按功能码解析寄存器数据、写入确认或异常码。
 */
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

/**
 * @brief 把当前 CPU2 响应中的连续寄存器值写入 CPU3 保持寄存器或输入寄存器镜像。
 *
 * registertype --> false - 保持寄存器。
 * > true - 输入寄存器。
 *
 * @param registertype > false - 保持寄存器。
 * @param registervalue 待写入 DSM 寄存器的 16 位值或连续寄存器数组。
 */
static void PresetRegister(bool registertype, int const *registervalue) {
	int i;
	uint16_t *registers = registertype ? InputRegisterArray : HoldingRegisterArray;

	for (i = 0; i < RCV_registercnt; i++) {
		registers[RCV_startaddress + i] = (uint16_t)registervalue[i];
	}
}
