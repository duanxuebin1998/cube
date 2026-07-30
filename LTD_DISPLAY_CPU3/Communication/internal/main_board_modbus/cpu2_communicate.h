#ifndef __COMMU_H
/* __COMMU_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define __COMMU_H
#include "stateformodbus.h"
#include "system_parameter.h"
#include "main.h"
extern uint16_t HoldingRegisterArray[HOLEREGISTER_STOP]; /* 保持寄存器数组 */

extern volatile bool wait_response;

/* CPU2 SI 剖面候选快照键；由周期、完成计数、测点数、来源和阶段共同标识一次可同步的测量结果。 */
typedef struct {
    /* CPU3 判定 SI 剖面候选结果是否属于同一测量周期的组合键。 */
    uint32_t cycle_counter; /* 剖面测量周期计数；每开始一个新周期递增，用于区分不同点阵。 */
    uint32_t complete_counter; /* 剖面完整结果提交计数；每成功提交一份新点阵后递增。 */
    uint32_t measurement_points; /* 本次剖面声明的有效测点数，读取点阵前必须校验容量。 */
    uint32_t profile_source; /* 本次剖面结果来源，取值遵循 ProfileSource。 */
    uint32_t phase; /* 当前剖面或状态机阶段；消费者必须按对应枚举解释。 */
} Cpu2SiProfileCandidateKey;

typedef enum {
    /* CPU2 内部通信最近一次失败的细分原因。 */
    CPU2_COMM_FAIL_NONE = 0, /* 最近一次 CPU2 通信没有失败。 */
    CPU2_COMM_FAIL_TIMEOUT, /* CPU2 响应等待超时。 */
    CPU2_COMM_FAIL_CRC, /* CPU2 响应 CRC 校验失败。 */
    CPU2_COMM_FAIL_ADDRESS, /* CPU2 响应站号不匹配。 */
    CPU2_COMM_FAIL_FUNCTION, /* CPU2 响应功能码或异常响应不符合请求。 */
    CPU2_COMM_FAIL_LENGTH, /* CPU2 响应长度与请求不一致。 */
    CPU2_COMM_FAIL_UART, /* CPU3 UART 外设报告接收或线路错误。 */
    CPU2_COMM_FAIL_TX_DMA /* CPU3 发送 DMA 未能启动。 */
} Cpu2CommFailureReason;

typedef struct {
    /* CPU2 内部 Modbus 通信健康快照；累计各类失败并保留连续失败峰值、最近原因和异常码。 */
    uint32_t success_count; /* CPU2 通信成功累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t timeout_count; /* 通信超时累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t crc_count; /* CRC 错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t address_count; /* 地址不匹配累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t function_count; /* 功能码错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t length_count; /* 长度错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t uart_ore_count; /* UART 接收溢出错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t uart_fe_count; /* UART 帧格式错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t uart_ne_count; /* UART 噪声错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t uart_pe_count; /* UART 奇偶校验错误累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t uart_failure_count; /* UART 硬件失败累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t tx_dma_start_fail_count; /* 发送 DMA 启动失败累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t total_failure_count; /* 所有 CPU2 通信失败累计次数；自对应上下文初始化后按事件递增，仅用于诊断统计，达到无符号上限后允许自然回绕。 */
    uint32_t consecutive_failure_count; /* 当前连续通信失败次数；任一成功响应都会将其清零。 */
    uint32_t max_consecutive_failure_count; /* 自统计初始化以来观察到的最大连续通信失败次数。 */
    Cpu2CommFailureReason last_failure_reason; /* 最近一次 CPU2 通信失败的细分原因。 */
    uint8_t last_modbus_exception_code; /* 最近一次 CPU2 返回的 Modbus 异常码；无异常响应时为零。 */
} Cpu2CommHealthSnapshot;

/* CPU2 内部 Modbus 处理结果 0x00：请求成功且响应数据可继续使用。 */
#define CPU2_MODBUS_RESULT_OK                 0x00U
/* CPU2 内部 Modbus 异常码：非法功能码；异常响应功能码需同时置最高位。 */
#define CPU2_MODBUS_EX_ILLEGAL_FUNCTION       0x01U
/* CPU2 内部 Modbus 异常码：非法数据地址；异常响应功能码需同时置最高位。 */
#define CPU2_MODBUS_EX_ILLEGAL_ADDRESS        0x02U
/* CPU2 内部 Modbus 异常码：非法数据值或长度；异常响应功能码需同时置最高位。 */
#define CPU2_MODBUS_EX_ILLEGAL_VALUE          0x03U
/* CPU2 内部 Modbus 异常码：从站内部处理失败；异常响应功能码需同时置最高位。 */
#define CPU2_MODBUS_EX_SLAVE_DEVICE_FAILURE   0x04U
/* CPU2 内部 Modbus 异常码：从站忙，当前无法完成请求；异常响应功能码需同时置最高位。 */
#define CPU2_MODBUS_EX_SLAVE_DEVICE_BUSY      0x06U

/* 将 CPU3 外部 COM1 RS485 收发器切换到接收方向；宏直接把对应方向控制 GPIO 置高，调用方必须遵守发送完成后的回切时序。 */
#define COM1_SET_RECV_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_SET)
/* 将 CPU3 外部 COM1 RS485 收发器切换到发送方向；宏直接把对应方向控制 GPIO 拉低，调用方必须遵守发送完成后的回切时序。 */
#define COM1_SET_SEND_MODE()  HAL_GPIO_WritePin(COM1_SEL_GPIO_Port, COM1_SEL_Pin, GPIO_PIN_RESET)
/* 将 CPU3 外部 COM2 RS485 收发器切换到接收方向；宏直接把对应方向控制 GPIO 置高，调用方必须遵守发送完成后的回切时序。 */
#define COM2_SET_RECV_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_SET)
/* 将 CPU3 外部 COM2 RS485 收发器切换到发送方向；宏直接把对应方向控制 GPIO 拉低，调用方必须遵守发送完成后的回切时序。 */
#define COM2_SET_SEND_MODE()  HAL_GPIO_WritePin(COM2_SEL_GPIO_Port, COM2_SEL_Pin, GPIO_PIN_RESET)
/* 将 CPU3 外部 COM3 RS485 收发器切换到接收方向；宏直接把对应方向控制 GPIO 置高，调用方必须遵守发送完成后的回切时序。 */
#define COM3_SET_RECV_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_SET)
/* 将 CPU3 外部 COM3 RS485 收发器切换到发送方向；宏直接把对应方向控制 GPIO 拉低，调用方必须遵守发送完成后的回切时序。 */
#define COM3_SET_SEND_MODE()  HAL_GPIO_WritePin(COM3_SEL_GPIO_Port, COM3_SEL_Pin, GPIO_PIN_RESET)
/* 将 CPU3 与 CPU2 板间 RS485 收发器切换到接收方向；宏将 MAIN_BOARD_485_SEL 控制脚置高。 */
#define RS485_SET_RECV_MODE()  HAL_GPIO_WritePin(MAIN_BOARD_485_SEL_GPIO_Port, MAIN_BOARD_485_SEL_Pin, GPIO_PIN_SET)
/* 将 CPU3 与 CPU2 板间 RS485 收发器切换到发送方向；宏将 MAIN_BOARD_485_SEL 控制脚拉低，发送完成后必须及时回切接收。 */
#define RS485_SET_SEND_MODE()  HAL_GPIO_WritePin(MAIN_BOARD_485_SEL_GPIO_Port, MAIN_BOARD_485_SEL_Pin, GPIO_PIN_RESET)

/**
 * @brief 以下包装分别控制三路外部 RS485 收发方向；CPU2 板间通信使用 UART5，不属于 COM1/2/3。
 */
static inline void COM1_SendMode(void) { COM1_SET_SEND_MODE(); }
/**
 * @brief 将外部 COM1 对应的 RS485 收发器切到接收方向。
 */
static inline void COM1_RecvMode(void) { COM1_SET_RECV_MODE(); }

/**
 * @brief 将外部 COM2 对应的 RS485 收发器切到发送方向。
 */
static inline void COM2_SendMode(void) { COM2_SET_SEND_MODE(); }
/**
 * @brief 将外部 COM2 对应的 RS485 收发器切到接收方向。
 */
static inline void COM2_RecvMode(void) { COM2_SET_RECV_MODE(); }

/**
 * @brief 将外部 COM3 对应的 RS485 收发器切到发送方向。
 */
static inline void COM3_SendMode(void) { COM3_SET_SEND_MODE(); }
/**
 * @brief 将外部 COM3 对应的 RS485 收发器切到接收方向。
 */
static inline void COM3_RecvMode(void) { COM3_SET_RECV_MODE(); }



/**
 * @brief 校验并解析一帧 CPU2 板间 Modbus 响应，更新请求结果和对应数据镜像。
 *
 * @param rcv 已经接收完成、等待校验或解析的板间通信帧。
 * @param len 输入数据的有效长度，单位字节。
 * @return true 表示收到并处理了合法正常响应或标准 Modbus 异常响应；false 表示帧长、CRC、地址、功能码或响应结构校验失败。
 */
bool HostCommuProcess(uint8_t*rcv,int len);
/**
  * @brief 按调度周期从 CPU2 轮询运行数据和参数快照。
 */
void PollingInputData(void);
/**
 * @brief 判断是否仍应显示等待 CPU2 首次状态和当前连接协议快照的同步页面。
 * @return true 表示继续显示通讯尝试页，false 表示进入正常状态页或故障页。
 */
bool CPU2_CommShouldShowStartup(void);
/**
 * @brief 判断当前连接是否已读回协议版本且与 CPU3 共享协议严格一致。
 *
 * @details 调用场景：CPU3 显示、命令门禁、外部协议和运行期快照访问。
 * @note 关键约束：协议快照无效时必须返回 false，不能使用默认值或掉线前缓存。
 */
bool CPU2_CommIsProtocolCompatible(void);
/**
 * @brief 判断当前连接是否已确认存在共享协议版本不匹配。
 *
 * @details 调用场景：CPU3 状态页区分协议不匹配、同步未完成和通信超时。
 * @note 关键约束：协议快照尚未建立时返回 false，不能把未知状态误报为不匹配。
 */
bool CPU2_CommIsProtocolMismatch(void);
/**
 * @brief 判断 CPU2 状态/参数/协议快照是否完整、共享协议是否兼容且通信故障未锁存。
 * @return true 表示允许访问 CPU2 参数和普通命令，false 表示同步未完成、协议不兼容或故障已锁存。
 */
bool CPU2_CommIsAvailable(void);
/**
 * @brief 使 CPU2 完整参数快照失效，并请求主循环重新读取全部保持寄存器参数。
 *
 * @details 调用场景：参数写入确认、写结果不确定或恢复出厂命令确认后调用。
 * @note 关键约束：刷新请求不清逐字段确认资格；刷新完成前参数读取和普通写入门禁保持关闭。
 */
void CPU2_CommRequestParameterRefresh(void);
/**
 * @brief 判断SI生命周期运行态是否来自连续、兼容的CPU2快照。
 *
 * @details 调用场景：CPU3本地SI投影识别首次上线和通信重连。
 * @note 关键约束：参数补读期间不打断运行态连续性，通信故障和协议不匹配必须返回false。
 */
bool CPU2_CommHasRuntimeSnapshot(void);
/**
 * @brief 判断当前 CPU2 会话内固定点结果是否完成一致性握手。
 *
 * @details 调用场景：外部协议准备读取单点测量或监测派生字段前调用。
 * @note 关键约束：运行态会话无效时必须同时返回 false，不能发布掉线前固定点结果。
 */
bool CPU2_CommHasFixedPointSnapshot(void);
/**
 * @brief 读取CPU3本机维护的CPU2公开快照会话代际。
 *
 * @details 调用场景：外部协议本地投影判断通信恢复后是否需要丢弃旧缓存。
 * @note 关键约束：返回值不属于共享寄存器协议，不改变DEVICE_PROTOCOL_VERSION。
 */
uint32_t CPU2_CommGetSnapshotGeneration(void);
/**
 * @brief 判断指定 CPU2 命令是否可下发；取消命令在当前连接协议已确认后可绕过其余参数刷新。
 * @param cmd 待下发命令。
 * @return true 表示当前通信状态允许下发该命令。
 */
bool CPU2_CommCanSendCommand(CommandType cmd);
/**
 * @brief 读取CPU3本机累计的CPU2通信健康计数。
 *
 * @details 调用场景：CPU3维护菜单的CPU2通讯页面刷新时调用。
 * @note 关键约束：计数仅存RAM、上电清零，不属于CPU2/CPU3共享协议。
 */
void CPU2_CommGetHealthSnapshot(Cpu2CommHealthSnapshot *out_snapshot);
/**
 * @brief 周期输出CPU2链路、协议、快照和健康计数摘要。
 *
 * @details 调用场景：CPU3主循环每轮调用，函数内部按固定周期限流。
 * @note 关键约束：只能在主循环调用，禁止在UART5中断上下文中打印。
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
/**
 * @brief 读取统一异步状态机已经确认发布的SI Profile候选代际。
 *
 * @details 调用场景：CPU3观察到新的SI完成计数后调用。
 * @note 关键约束：本接口不发起UART请求；点阵未完整发布或生命周期不匹配时返回false。
 */
bool CPU2_CommFetchSiProfileCandidate(Cpu2SiProfileCandidateKey *out_key);
/**
 * @brief 读取CPU3已确认的、Point0前仍允许保留的上一轮完整SI快照。
 *
 * @details 调用场景：CPU3冷启动后处于PREPARING，或Point0前已经ABORTED/FAILED。
 * @note 关键约束：接口不发起UART请求，也不生成或恢复Profile时间。
 */
bool CPU2_CommFetchSiPreviousSnapshot(Cpu2SiProfileCandidateKey *out_key);
/**
 * @brief 按 LTD 共享 Modbus 线序向 CPU2 写入完整 32 位字段。
 * @param startadd 起始保持寄存器地址，必须按 2 个寄存器对齐。
 * @param registercnt 寄存器数量，必须为非零偶数。
 * @param wire_regs 按 Modbus 高字在前、低字在后的寄存器数据。
 * @return true 表示 CPU2 已返回合法 ACK，false 表示门禁、范围、标准异常或通信失败。
 */
bool CPU2_CommWriteHoldingRegisters(uint16_t startadd, uint16_t registercnt, const uint16_t *wire_regs);
/**
 * @brief 按 LTD 共享 Modbus 线序写入并返回标准 Modbus 结果。
 * @note 非命令参数以合法 FC10 ACK 确认本次值并更新局部镜像，随后触发后台全量刷新。
 * @return 0 表示成功；CPU2 标准异常码原样返回；本地忙返回 0x06；
 *         UART、超时、CRC 等传输失败返回 0x04。
 */
uint8_t CPU2_CommWriteHoldingRegistersEx(uint16_t startadd,
                                         uint16_t registercnt,
                                         const uint16_t *wire_regs);
/**
 * @brief 读取最近一次同步事务收到的标准 Modbus 异常码。
 * @return 0 表示最近事务没有标准异常帧。
 */
uint8_t CPU2_CommGetLastModbusException(void);
/**
 * @brief 在 UART5 错误中断中记录待处理标志并解除当前等待。
 * @note 仅允许在 ISR 中置标志，不在中断内打印、计数或修改设备故障状态。
 */
void CPU2_CommNotifyUartErrorFromISR(uint32_t uart_error_code);
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
bool CPU2_CombinatePackage_Send(uint8_t f_code,uint16_t startadd,uint16_t registercnt,uint32_t* holddata);
/**
 * @brief 向CPU2发送数据包。
 *
 * @param arr 待打包并发送给 CPU2 的寄存器数据数组。
 * @param len 输入数据的有效长度，单位字节。
 * @param flag_fromhost true 表示数据来自外部主机写入，false 表示来自本机流程。
 * @return true 表示待发长度和缓冲有效，UART DMA 发送已成功启动并完成；false 表示参数无效、UART 忙或错误，或发送等待超时。
 */
bool sendToCPU2(uint8_t*arr,uint16_t len,bool flag_fromhost);








#endif
