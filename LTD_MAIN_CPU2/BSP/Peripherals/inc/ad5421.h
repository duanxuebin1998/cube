#ifndef sil__AD5421_H
/* sil__AD5421_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define sil__AD5421_H
#include "main.h"

/* AD5421 诊断阶段，用于区分总线访问失败和芯片主动报警。 */
#define AD5421_DIAG_STAGE_NONE             0U
/* AD5421 诊断阶段编码：SPI 访问仲裁忙，未取得总线所有权；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define AD5421_DIAG_STAGE_ACCESS_BUSY      1U
/* AD5421 诊断阶段编码：SPI 寄存器写入阶段失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define AD5421_DIAG_STAGE_SPI_WRITE        2U
/* AD5421 诊断阶段编码：SPI 读命令发送阶段失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define AD5421_DIAG_STAGE_SPI_READ_COMMAND 3U
/* AD5421 诊断阶段编码：SPI 读数据返回阶段失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define AD5421_DIAG_STAGE_SPI_READ_DATA    4U
/* AD5421 诊断阶段编码：控制寄存器回读校验失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define AD5421_DIAG_STAGE_CONTROL_READBACK 5U
/* AD5421 诊断阶段编码：器件故障状态寄存器报告异常；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define AD5421_DIAG_STAGE_FAULT_STATUS     6U

/* AD5421 诊断访问方向。 */
#define AD5421_DIAG_DIRECTION_NONE  0U
/* AD5421 诊断访问方向编码：写寄存器；与寄存器地址和诊断阶段一起记录，不能单独解释为故障原因。 */
#define AD5421_DIAG_DIRECTION_WRITE 1U
/* AD5421 诊断访问方向编码：读寄存器；与寄存器地址和诊断阶段一起记录，不能单独解释为故障原因。 */
#define AD5421_DIAG_DIRECTION_READ  2U

/* 保存最后一次有效 AD5421 故障现场，供任务态日志和测试读取。 */
typedef struct {
    /* AD5421 驱动故障诊断快照；以递增序号发布错误链、故障寄存器、期望/实测值和失败阶段。 */
    uint32_t sequence; /* 诊断快照发布序号；每次完整更新后递增，供读取方检测变化。 */
    uint32_t error_code; /* 本次 AD5421 诊断对应的完整统一故障码。 */
    uint32_t root_error_code; /* 故障链中最初的根因错误码，避免后续恢复错误覆盖来源。 */
    uint32_t fault_flags; /* 驱动故障状态的归一化位集合；各位含义由对应驱动接口定义。 */
    uint32_t fault_register; /* 驱动芯片故障寄存器的原始读回值。 */
    uint32_t hal_status; /* 失败操作对应的 HAL 返回状态。 */
    uint32_t expected_value; /* 驱动读写校验期望得到的寄存器值。 */
    uint32_t actual_value; /* 驱动实际读回的寄存器值。 */
    uint8_t stage; /* 诊断记录对应的驱动操作阶段。 */
    uint8_t direction; /* 诊断发生时的读写方向。 */
    uint8_t reg; /* AD5421 诊断涉及的寄存器地址。 */
    uint8_t reserved; /* 为保持历史二进制布局预留的字段；写入时保持约定值，禁止复用。 */
} AD5421DiagnosticSnapshot;

#define AD5421_CS_GPIO_PORT   GPIOB /* AD5421 片选 GPIO 端口。 */
#define AD5421_CS_PIN         GPIO_PIN_6 /* AD5421 片选 GPIO 引脚。 */

/* ============================== */
/* 寄存器地址宏定义 */
/* ============================== */
#define DELAY30US  delay_us(100) /* AD5421 时序使用的短延时宏。 */
#define SPI2WRITE00 SPI2_ReadWriteByte(0x00) /* SPI2 发送 0x00 并读取返回字节的宏。 */

/* 满量程电流范围 */
#define STARTFULLSCALE 3.2 /* AD5421 起始满量程电流值，单位 mA。 */
#define STOPFULLSCALE 24.0 /* AD5421 终止满量程电流值，单位 mA。 */
#define STARTFULLSCALE_MA_X1000 3200U /* AD5421 3.2 mA下限的千倍定点值。 */
#define STOPFULLSCALE_MA_X1000 24000U /* AD5421 24.0 mA上限的千倍定点值。 */
/* AD5421 寄存器指令 */
#define WRITEDAC 0x01u /* AD5421 写 DAC 寄存器命令。 */
#define WRITECONTROL 0x02u /* AD5421 写控制寄存器命令。 */
#define RESETAD5421REG 0x07u /* AD5421 复位寄存器命令。 */
#define NOOPAD5421 0x09u /* AD5421 空操作命令。 */
#define READCONTROL 0x82u /* AD5421 读控制寄存器命令。 */
#define READFAULT 0x85u /* AD5421 读故障寄存器命令。 */
/* SPI 看门狗开关 */
#define CUR_SPION_COMMAND 0xC000u /* SPI 看门狗开启，4s */
#define CUR_SPIOFF_COMMAND 0x1000u /* SPI 看门狗关闭 */
#define CUR_AUTO_FAULT_READBACK_OFF_COMMAND 0x0800u /* 禁止自动 fault 回读，允许专用寄存器回读 */
#define CUR_SPIOFF_READBACK_COMMAND (CUR_SPIOFF_COMMAND | CUR_AUTO_FAULT_READBACK_OFF_COMMAND) /* AD5421 关闭 SPI 看门狗并允许专用回读的控制字。 */

#define AD5421_FAULT_FLAG_SPI_WRITE    0x00000001u /* AD5421 故障位标志：AD5421 故障 FLAG SPI 写入。 */
#define AD5421_FAULT_FLAG_SPI_READ     0x00000002u /* AD5421 故障位标志：AD5421 故障 FLAG SPI 读取。 */
#define AD5421_FAULT_FLAG_READBACK     0x00000004u /* AD5421 故障位标志：AD5421 故障 FLAG 回读。 */
#define AD5421_FAULT_FLAG_PIN          0x00000008u /* 历史 PB8 位，当前硬件未接入 */
#define AD5421_FAULT_FLAG_STATUS       0x00000010u /* AD5421 故障位标志：AD5421 故障 FLAG 状态。 */


/* 电流设置 */
#define CURRENT_INIT 				3.5 /* AD5421 初始化阶段输出电流，单位 mA。 */
#define CURRENT_UNDERAOL 			3.6 /* 模拟量低于量程报警时输出电流，单位 mA。 */
#define CURRENT_GREATERAOH 			21.0 /* 模拟量高于量程报警时输出电流，单位 mA。 */
#define CURRENT_EQUIPMENT_ERROR 	22.0 /* 设备故障时输出电流，单位 mA。 */
#define CURRENT_CHECKMODE_MIN 		3.5 /* 校验模式最小输出电流，单位 mA。 */
#define CURRENT_CHECKMODE_MAX 		24.0 /* 校验模式最大输出电流，单位 mA。 */
#define CURRENT_CONFIGMODE 			23.0 /* 配置模式输出电流，单位 mA。 */
#define CURRENT_MEAMODE 			21.5 /* 测量模式默认输出电流，单位 mA。 */


/**
 * @brief 保留历史无参数初始化接口，供旧测试和调用点兼容使用。
 *
 * @details 调用场景：尚未迁移到AO服务显式初始电流接口的旧调用点。
 * @note 关键约束：默认采用当前AO上电电流参数，不改变旧接口返回语义。
 *
 * @return 返回整机错误码；NO_ERROR 表示已按上电默认电流完成 AD5421 初始化，其他值标识初始化失败阶段。
 */
uint32_t Ad5421Init(void);
/**
 * @brief 按0.01mA目标复位并初始化AD5421。
 *
 * @details 调用场景：AO服务根据禁用、固定或上电电流选择初始输出时调用。
 * @note 关键约束：会访问SPI并等待芯片稳定，不应在中断中调用。
 *
 * @param initial_mA_x100 AD5421 初始化完成后准备输出的起始电流，单位 0.01 mA。
 * @return 返回整机错误码；NO_ERROR 表示 AD5421 已复位、初始化并输出目标电流，其他值标识失败阶段。
 */
uint32_t AD5421_InitCurrentX100(uint32_t initial_mA_x100);
uint32_t AD5421_InitCurrentX1000(uint32_t initial_mA_x1000);
/**
 * @brief 按 mA 值换算并写入 AD5421 输出电流。
 *
 * @details 调用场景：AO 服务需要刷新模拟电流输出时调用。
 * @note 关键约束：会把输入限制在 AD5421 允许的 3.2-24.0mA 范围内。
 *
 * @param mA 目标模拟输出电流，单位 mA。
 * @return 返回整机错误码；NO_ERROR 表示目标 mA 已换算并写入 DAC，其他值表示范围或 AD5421 通信失败。
 */
uint32_t AD5421_SetCurrent(float mA);
/**
 * @brief 按 0.01mA 单位设置 AD5421 输出电流。
 *
 * @details 调用场景：AO 服务使用参数原始单位写入电流。
 * @note 关键约束：内部转为 mA 后复用 AD5421_SetCurrent()。
 *
 * @param mA_x100 准备写入 AD5421 的目标电流，单位 0.01 mA。
 * @return 返回整机错误码；NO_ERROR 表示 0.01 mA 定点目标已成功写入，其他值透传范围或驱动错误。
 */
uint32_t AD5421_SetCurrentX100(uint32_t mA_x100);
/**
 * @brief 按0.001mA单位设置AD5421输出电流。
 *
 * @param mA_x1000 准备写入AD5421的目标电流，单位0.001mA。
 * @return NO_ERROR表示目标已换算并写入，其他值透传驱动错误。
 */
uint32_t AD5421_SetCurrentX1000(uint32_t mA_x1000);
/**
 * @brief 按指定目标电流恢复 AD5421 输出。
 *
 * @details 调用场景：AO 运行期 READFAULT 异常后的自动恢复。
 * @note 关键约束：复位芯片后直接写回目标电流，保留控制回读和故障回读诊断。
 *
 * @param target_mA_x100 目标电流定点值，单位 0.01 mA。
 * @return 返回整机错误码；NO_ERROR 表示 AD5421 已恢复目标电流，其他值标识复位、初始化或写入失败。
 */
uint32_t AD5421_RecoverCurrentX100(uint32_t target_mA_x100);
uint32_t AD5421_RecoverCurrentX1000(uint32_t target_mA_x1000);
/**
 * @brief 轮询 AD5421 故障寄存器。
 *
 * @details 调用场景：AO 初始化和周期刷新时确认电流环/芯片状态。
 * @note 关键约束：会访问 SPI，不应在中断中调用；当前 PCB 未接 AD5421 FAULT 引脚。
 *
 * @return NO_ERROR 表示轮询 AD5421 故障寄存器已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t AD5421_PollDiagnostics(void);
/**
 * @brief 读取 AD5421 驱动累计故障标志。
 *
 * @details 调用场景：AO 运行态打包和故障分析。
 * @note 关键约束：只读内存状态，不访问外设。
 *
 * @return 返回 AD5421 驱动累计故障标志对应的位掩码；各位含义由相邻枚举或宏定义。
 */
uint32_t AD5421_GetFaultFlags(void);
/**
 * @brief 读取最近一次 AD5421 READFAULT 原始值。
 *
 * @details 调用场景：AO 运行态打包和故障分析。
 * @note 关键约束：只读缓存值，不主动刷新 AD5421。
 *
 * @return 返回最近一次成功读取并缓存的 AD5421 READFAULT 原始寄存器位图。
 */
uint32_t AD5421_GetFaultRegister(void);
/**
 * @brief 复制最后一次有效 AD5421 故障现场。
 *
 * @details 调用场景：AO 主循环延后日志和故障注入测试读取。
 * @note 关键约束：使用短临界区保证快照字段来自同一次故障，不访问 SPI。
 *
 * @param snapshot AD5421 最近一次有效故障现场输出对象；写入序号、错误码、故障寄存器、HAL 状态、阶段、方向以及期望值和实测值。
 */
void AD5421_GetDiagnosticSnapshot(AD5421DiagnosticSnapshot *snapshot);
/**
 * @brief 切换 AD5421 调试打印抑制状态并返回旧状态。
 *
 * @details 调用场景：TIM4 中断刷新 AO 前抑制 printf，退出中断前恢复。
 * @note 关键约束：只影响本驱动内部诊断打印，不改变错误码和故障标志。
 *
 * @param suppress true 表示暂时抑制 AD5421 逐次跟踪日志，false 表示恢复打印。
 * @return 返回切换前的调试打印抑制状态，供调用方退出临时静默区时恢复。
 */
uint8_t AD5421_SetTraceSuppressed(uint8_t suppress);
/**
 * @brief 返回 AD5421 当前是否正在访问 SPI。
 *
 * @details 调用场景：TIM4 AO 刷新前判断是否需要跳过本次中断刷新。
 * @note 关键约束：只返回软件访问保护状态，不读取 SPI 外设寄存器。
 *
 * @return 1 表示寄存器访问或分阶段输出序列正在占用 AD5421 SPI；0 表示两类访问均处于空闲状态。
 */
uint8_t AD5421_IsAccessBusy(void);

#endif

