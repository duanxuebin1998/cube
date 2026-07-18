#ifndef __COMMU_H
#define __COMMU_H
#include "stateformodbus.h"
#include "system_parameter.h"
#include "main.h"
extern uint16_t HoldingRegisterArray[HOLEREGISTER_STOP]; /* 保持寄存器数组 */

extern volatile bool wait_response;

typedef struct {
    uint32_t cycle_counter;
    uint32_t complete_counter;
    uint32_t measurement_points;
    uint32_t profile_source;
    uint32_t phase;
} Cpu2SiProfileCandidateKey;

typedef enum {
    CPU2_COMM_FAIL_NONE = 0,
    CPU2_COMM_FAIL_TIMEOUT,
    CPU2_COMM_FAIL_CRC,
    CPU2_COMM_FAIL_ADDRESS,
    CPU2_COMM_FAIL_FUNCTION,
    CPU2_COMM_FAIL_LENGTH,
    CPU2_COMM_FAIL_UART,
    CPU2_COMM_FAIL_TX_DMA
} Cpu2CommFailureReason;

typedef struct {
    uint32_t success_count;
    uint32_t timeout_count;
    uint32_t crc_count;
    uint32_t address_count;
    uint32_t function_count;
    uint32_t length_count;
    uint32_t uart_ore_count;
    uint32_t uart_fe_count;
    uint32_t uart_ne_count;
    uint32_t uart_pe_count;
    uint32_t uart_failure_count;
    uint32_t tx_dma_start_fail_count;
    uint32_t total_failure_count;
    uint32_t consecutive_failure_count;
    uint32_t max_consecutive_failure_count;
    Cpu2CommFailureReason last_failure_reason;
} Cpu2CommHealthSnapshot;

#define COM1_SET_RECV_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_SET)
#define COM1_SET_SEND_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_RESET)
#define COM2_SET_RECV_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_SET)
#define COM2_SET_SEND_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_RESET)
#define COM3_SET_RECV_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_SET)
#define COM3_SET_SEND_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_RESET)
#define RS485_SET_RECV_MODE()  HAL_GPIO_WritePin(MAIN_BOARD_485_SEL_GPIO_Port, MAIN_BOARD_485_SEL_Pin, GPIO_PIN_SET)
#define RS485_SET_SEND_MODE()  HAL_GPIO_WritePin(MAIN_BOARD_485_SEL_GPIO_Port, MAIN_BOARD_485_SEL_Pin, GPIO_PIN_RESET)

static inline void COM1_SendMode(void) { COM1_SET_SEND_MODE(); }
static inline void COM1_RecvMode(void) { COM1_SET_RECV_MODE(); }

static inline void COM2_SendMode(void) { COM2_SET_SEND_MODE(); }
static inline void COM2_RecvMode(void) { COM2_SET_RECV_MODE(); }

static inline void COM3_SendMode(void) { COM3_SET_SEND_MODE(); }
static inline void COM3_RecvMode(void) { COM3_SET_RECV_MODE(); }



/**
 * @brief 处理Modbus 协议中的 HostCommuProcess 逻辑。
 *
 * @param rcv 业务参数。
 * @param len 数据长度。
 */
bool HostCommuProcess(uint8_t*rcv,int len);
/**
 * @brief 执行Modbus 协议中的 PollingInputData 逻辑。
 */
void PollingInputData(void);
/**
 * @brief 判断是否仍应显示等待 CPU2 首次状态和当前连接协议快照的同步页面。
 * @return true 表示继续显示通讯尝试页，false 表示进入正常状态页或故障页。
 */
bool CPU2_CommShouldShowStartup(void);
/*
 * 函数用途：判断当前连接是否已读回协议版本且与 CPU3 共享协议严格一致。
 * 调用场景：CPU3 显示、命令门禁、外部协议和运行期快照访问。
 * 关键约束：协议快照无效时必须返回 false，不能使用默认值或掉线前缓存。
 */
bool CPU2_CommIsProtocolCompatible(void);
/*
 * 函数用途：判断当前连接是否已确认存在共享协议版本不匹配。
 * 调用场景：CPU3 状态页区分协议不匹配、同步未完成和通信超时。
 * 关键约束：协议快照尚未建立时返回 false，不能把未知状态误报为不匹配。
 */
bool CPU2_CommIsProtocolMismatch(void);
/**
 * @brief 判断 CPU2 状态/参数/协议快照是否完整、共享协议是否兼容且通信故障未锁存。
 * @return true 表示允许访问 CPU2 参数和普通命令，false 表示同步未完成、协议不兼容或故障已锁存。
 */
bool CPU2_CommIsAvailable(void);
/*
 * 函数用途：使 CPU2 参数快照失效，并请求主循环重新读取全部保持寄存器参数。
 * 调用场景：参数写入后的定向补读失败，CPU3 无法确认 CPU2 当前实际值时调用。
 * 关键约束：刷新完成前参数读写门禁必须保持关闭，不能继续开放旧镜像。
 */
void CPU2_CommRequestParameterRefresh(void);
/*
 * 函数用途：判断SI生命周期运行态是否来自连续、兼容的CPU2快照。
 * 调用场景：CPU3本地SI投影识别首次上线和通信重连。
 * 关键约束：参数补读期间不打断运行态连续性，通信故障和协议不匹配必须返回false。
 */
bool CPU2_CommHasRuntimeSnapshot(void);
/*
 * 函数用途：读取CPU3本机维护的CPU2公开快照会话代际。
 * 调用场景：外部协议本地投影判断通信恢复后是否需要丢弃旧缓存。
 * 关键约束：返回值不属于共享寄存器协议，不改变DEVICE_PROTOCOL_VERSION。
 */
uint32_t CPU2_CommGetSnapshotGeneration(void);
/**
 * @brief 判断指定 CPU2 命令是否可下发；取消命令在当前连接协议已确认后可绕过其余参数刷新。
 * @param cmd 待下发命令。
 * @return true 表示当前通信状态允许下发该命令。
 */
bool CPU2_CommCanSendCommand(CommandType cmd);
/*
 * 函数用途：读取CPU3本机累计的CPU2通信健康计数。
 * 调用场景：CPU3维护菜单的CPU2通讯页面刷新时调用。
 * 关键约束：计数仅存RAM、上电清零，不属于CPU2/CPU3共享协议。
 */
void CPU2_CommGetHealthSnapshot(Cpu2CommHealthSnapshot *out_snapshot);
/*
 * 函数用途：周期输出CPU2链路、协议、快照和健康计数摘要。
 * 调用场景：CPU3主循环每轮调用，函数内部按固定周期限流。
 * 关键约束：只能在主循环调用，禁止在UART5中断上下文中打印。
 */
void CPU2_CommDebugTask(void);
/**
 * @brief 从已确认的 CPU2 参数快照复制保持寄存器。
 * @param startadd 起始寄存器地址。
 * @param registercnt 寄存器数量。
 * @param out_regs 接收寄存器数据的缓冲区。
 * @return true 表示快照有效且复制成功，false 表示通信不可用或范围非法。
 */
bool CPU2_CommReadHoldingSnapshot(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs);
/**
 * @brief 从已确认的 CPU2 状态快照复制输入寄存器。
 * @param startadd 起始寄存器地址。
 * @param registercnt 寄存器数量。
 * @param out_regs 接收寄存器数据的缓冲区。
 * @return true 表示快照有效且复制成功，false 表示通信不可用或范围非法。
 */
bool CPU2_CommReadInputSnapshot(uint16_t startadd, uint16_t registercnt, uint16_t *out_regs);
/*
 * 函数用途：读取统一异步状态机已经确认发布的SI Profile候选代际。
 * 调用场景：CPU3观察到新的SI完成计数后调用。
 * 关键约束：本接口不发起UART请求；点阵未完整发布或生命周期不匹配时返回false。
 */
bool CPU2_CommFetchSiProfileCandidate(Cpu2SiProfileCandidateKey *out_key);
/*
 * 函数用途：读取CPU3已确认的、Point0前仍允许保留的上一轮完整SI快照。
 * 调用场景：CPU3冷启动后处于PREPARING，或Point0前已经ABORTED/FAILED。
 * 关键约束：接口不发起UART请求，也不生成或恢复Profile时间。
 */
bool CPU2_CommFetchSiPreviousSnapshot(Cpu2SiProfileCandidateKey *out_key);
/**
 * @brief 按 LTD 共享 Modbus 线序向 CPU2 写入完整 32 位字段。
 * @param startadd 起始保持寄存器地址，必须按 2 个寄存器对齐。
 * @param registercnt 寄存器数量，必须为非零偶数。
 * @param wire_regs 按 Modbus 高字在前、低字在后的寄存器数据。
 * @return true 表示 CPU2 已返回合法 ACK，false 表示门禁、范围或通信失败。
 */
bool CPU2_CommWriteHoldingRegisters(uint16_t startadd, uint16_t registercnt, const uint16_t *wire_regs);
/**
 * @brief 在 UART5 错误中断中记录待处理标志并解除当前等待。
 * @note 仅允许在 ISR 中置标志，不在中断内打印、计数或修改设备故障状态。
 */
void CPU2_CommNotifyUartErrorFromISR(uint32_t uart_error_code);
/**
 * @brief 发送Modbus 协议中的 CPU2_CombinatePackage_Send 逻辑。
 *
 * @param f_code 业务参数。
 * @param startadd 业务参数。
 * @param registercnt 业务参数。
 * @param holddata 数据缓冲区。
 */
bool CPU2_CombinatePackage_Send(uint8_t f_code,uint16_t startadd,uint16_t registercnt,uint32_t* holddata);
/**
 * @brief 发送Modbus 协议中的 sendToCPU2 逻辑。
 *
 * @param arr 业务参数。
 * @param len 数据长度。
 * @param flag_fromhost 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
bool sendToCPU2(uint8_t*arr,uint16_t len,bool flag_fromhost);








#endif

