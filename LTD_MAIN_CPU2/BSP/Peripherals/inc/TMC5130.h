
#ifndef TMC_IC_TMC5130_H_
/* TMC_IC_TMC5130_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define TMC_IC_TMC5130_H_

#include "main.h"
#include <stdbool.h>

#include "TMC5130_Constants.h"
#include "TMC5130_Pins.h"
#include "TMC5130_Register.h"

/* TMC5130 通信诊断阶段。 */
#define TMC5130_DIAG_STAGE_NONE               0U
/* TMC5130 诊断阶段编码：调用参数或寄存器地址校验失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_PARAMETER          1U
/* TMC5130 诊断阶段编码：SPI 访问仲裁忙，未取得总线所有权；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_ACCESS_BUSY        2U
/* TMC5130 诊断阶段编码：SPI 读流水线触发阶段失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_SPI_READ_TRIGGER   3U
/* TMC5130 诊断阶段编码：SPI 读数据返回阶段失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_SPI_READ_DATA      4U
/* TMC5130 诊断阶段编码：SPI 寄存器写入阶段失败；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_SPI_WRITE          5U
/* TMC5130 诊断阶段编码：XACTUAL 连续采样不稳定；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_XACTUAL_UNSTABLE   6U
/* TMC5130 诊断阶段编码：驱动器配置回读与期望值不一致；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_CONFIGURATION_LOST 7U
/* TMC5130 诊断阶段编码：检测到 TMC5130 芯片复位状态；该编码写入结构化诊断信息，用于区分同一错误码发生在哪个硬件访问阶段。 */
#define TMC5130_DIAG_STAGE_CHIP_RESET         8U

/* TMC5130 诊断访问方向。 */
#define TMC5130_DIAG_DIRECTION_NONE  0U
/* TMC5130 诊断访问方向编码：写寄存器；与寄存器地址和诊断阶段一起记录，不能单独解释为故障原因。 */
#define TMC5130_DIAG_DIRECTION_WRITE 1U
/* TMC5130 诊断访问方向编码：读寄存器；与寄存器地址和诊断阶段一起记录，不能单独解释为故障原因。 */
#define TMC5130_DIAG_DIRECTION_READ  2U

/* 保存最后一次有效 TMC5130 故障现场，供最终错误日志和测试读取。 */
typedef struct {
    /* TMC5130 驱动诊断快照；保存 SPI/HAL 状态、寄存器期望/实测值、连续样本和失败阶段。 */
    uint32_t sequence; /* 诊断快照发布序号；每次完整更新后递增，供读取方检测变化。 */
    uint32_t error_code; /* 本次 TMC5130 诊断对应的完整统一故障码。 */
    uint32_t hal_status; /* 失败操作对应的 HAL 返回状态。 */
    uint32_t expected_value; /* 驱动读写校验期望得到的寄存器值。 */
    uint32_t actual_value; /* 驱动实际读回的寄存器值。 */
    int32_t sample_first; /* 诊断窗口内第一次位置或寄存器采样值。 */
    int32_t sample_second; /* 诊断窗口内第二次位置或寄存器采样值。 */
    int32_t sample_third; /* 诊断窗口内第三次位置或寄存器采样值。 */
    uint8_t stage; /* 诊断记录对应的驱动操作阶段。 */
    uint8_t direction; /* 诊断发生时的读写方向。 */
    uint8_t address; /* TMC5130 诊断涉及的寄存器地址。 */
    uint8_t response_status; /* TMC5130 SPI 响应状态字节。 */
} TMC5130DiagnosticSnapshot;

/* TMC5130 单电机驱动句柄：只保留 SPI、片选和使能脚等真实硬件资源。 */
typedef struct
{
    SPI_HandleTypeDef *spi;      /* SPI总线句柄 */

    GPIO_TypeDef *cs_port;       /* 片选（CS）GPIO端口 */
    uint16_t cs_pin;             /* 片选（CS）GPIO引脚 */

    GPIO_TypeDef *en_port;       /* 使能（EN）GPIO端口 */
    uint16_t en_pin;             /* 使能（EN）GPIO引脚 */
} TMC5130TypeDef;

extern TMC5130TypeDef stepper;
/* ===================== TMC5130 底层驱动接口 =====================
 * 约定：
 *  - 本文件只暴露 motor_ctrl.c 需要调用的底层能力。
 *  - SPI 帧收发、速度模式方向选择等细节留在 TMC5130.c 内部。
 *  - 业务层优先使用 motor_ctrl.h 的上层电机接口。
 */

/* ---------- 初始化 / 使能 ---------- */

/**
 * @brief 初始化 TMC5130 步进电机驱动器的寄存器参数。
 *
 * 包括斩波配置、加减速、速度、stealthChop、StallGuard 等。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param spi SPI 句柄。
 * @param cs_port 片选 GPIO 端口。
 * @param cs_pin 片选 GPIO 引脚。
 * @param dir 方向配置（如需从 GCONF 中配置方向，可使用）。
 * @param current 电流设置（IRUN）。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
uint32_t stpr_initStepper(TMC5130TypeDef *tmc5130,
                      SPI_HandleTypeDef *spi,
                      GPIO_TypeDef *cs_port,
                      uint16_t cs_pin,
                      uint8_t dir,
                      uint8_t current);

/**
 * @brief  禁止驱动（EN 引脚拉高或拉低视硬件设计，一般为高电平关闭）
 *
 * 关闭驱动输出。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 */
void stpr_disableDriver(TMC5130TypeDef *tmc5130);

/**
 * @brief  使能驱动（EN 引脚）
 *
 * 使能驱动输出。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 */
void stpr_enableDriver(TMC5130TypeDef *tmc5130);

/* ---------- 寄存器访问 ---------- */

/**
 * @brief 向 TMC5130 指定寄存器写入 32 位整数。
 *
 * 向 TMC5130 指定寄存器写入 32bit 值，返回写入是否成功。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param address 寄存器地址（不带写标志位）。诊断和底层收发均保存该 TMC5130 原始寄存器地址。
 * @param value 要写入的 32 位数。
 * @return true 表示 32 位 value 已按高字节在前拆分并写入指定 TMC5130 寄存器；false 表示底层 5 字节寄存器写报文发送失败。
 */
bool stpr_writeInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t value);


/**
 * @brief  从 TMC5130 指定寄存器读取 32 位整数。
 *
 * XACTUAL 直接影响电机尺带长度和运动判断，单次错位值风险最高，因此使用稳定读取；
 * 其它寄存器保持原来的两帧流水线读取流程，避免改变状态类寄存器的实时语义。
 *
 * 带成功/失败返回值的寄存器读取接口，避免 SPI 失败时把寄存器值误判为 0。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param address 寄存器地址（不带写标志位）。诊断和底层收发均保存该 TMC5130 原始寄存器地址。
 * @param value 待原地读取或更新的数值对象。该输出指针在 TMC5130 寄存器读取成功时写入按补码解释的 32 位值，失败时保持调用方原值。
 * @return true 表示指定寄存器已成功读取，其中 XACTUAL 还通过了稳定读策略；false 表示设备或输出参数无效，或相应稳定读或单次读事务失败。
 */
bool stpr_tryReadInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value);

/**
 * @brief 复制最后一次有效 TMC5130 故障现场。
 *
 * @details 调用场景：故障管理最终出口和故障注入测试读取。
 * @note 关键约束：使用短临界区保证所有字段来自同一次故障，不访问 SPI。
 *
 * @param snapshot TMC5130 最近一次有效故障现场输出对象；写入序号、错误码、阶段、方向、寄存器地址、状态字和连续采样值。
 */
void TMC5130_GetDiagnosticSnapshot(TMC5130DiagnosticSnapshot *snapshot);

/* ---------- 运动控制 ---------- */

/**
 * @brief  停止电机并回到安全的位置模式。
 *
 * 速度模式下只把 VMAX 写成 0 虽然能停住当前动作，但 RAMPMODE 会停留在速度模式。
 * 后续如果只是恢复速度参数并写 VMAX，驱动可能在没有新位置命令的情况下重新运动。
 * 因此停止时把当前位置同步为目标位置，再切回位置模式，保证空闲态写 VMAX 不会启动电机。
 *
 * 按速度斜坡停止电机。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @return NO_ERROR 表示停止电机并回到安全的位置模式已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t stpr_stop(TMC5130TypeDef *tmc5130);

/**
 * @brief 以给定速度持续旋转（速度模式）。
 *
 * Velocity mode: keep rotating at signed VMAX until caller stops or changes speed.
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param velocity 目标速度（正：正向，负：反向）。
 * @return NO_ERROR 表示以给定速度持续旋转（速度模式）已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t stpr_rotate(TMC5130TypeDef *tmc5130, int32_t velocity);

/**
 * @brief 以给定最大速度，运行到指定位置（位置模式）。
 *
 * 位置模式：移动到指定绝对 ticks 位置。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param position 目标位置（步数）。
 * @param velocityMax 最大速度（VMAX）。
 * @return NO_ERROR 表示以给定最大速度，运行到指定位置（位置模式）已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t stpr_moveTo(TMC5130TypeDef *tmc5130, int32_t position, uint32_t velocityMax);

/**
 * @brief 在当前位置基础上“相对移动”一定步数。
 *
 * 位置模式：以当前位置为基准相对移动 ticks，成功后 *ticks 会被改写为绝对目标位置。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param ticks [入/出] 相对位移 / 计算后的绝对目标位置。
 * @param velocityMax 最大速度。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行。
 */
uint32_t stpr_moveBy(TMC5130TypeDef *tmc5130, int32_t *ticks, uint32_t velocityMax);

/**
 * @brief  同时设置“实际位置”和“目标位置”，相当于软件重置坐标
 *
 * 同步设置 XACTUAL 和 XTARGET，常用于标定零点时重建驱动坐标。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param position 位置。
 * @return NO_ERROR 表示同时设置“实际位置”和“目标位置”，相当于软件重置坐标已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t stpr_setPos(TMC5130TypeDef *tmc5130, int32_t position);

/**
 * @brief 等待目标运动结束，循环处理命令切换、扭力碰撞、驱动和功率状态及运行位置刷新。
 *
 * 阻塞等待当前运动结束，同时检查驱动基础故障。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @return NO_ERROR 表示等待目标运动结束，循环处理命令切换、扭力碰撞、驱动和功率状态及运行位置刷新已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t stpr_waitMove(TMC5130TypeDef *tmc5130);

/**
 * @brief 检查并解析 TMC5130 驱动异常状态。
 *
 * 该函数只解析 GSTAT/DRV_STATUS 故障位，不检查 CS_ACTUAL 功率级。
 * - GSTAT 非法高位只打印并丢弃，避免把 SPI 错帧误判为真实故障；
 * - drv_err 会继续读取 DRV_STATUS，细分过温、短路、开路、StallGuard；
 * - uv_cp 和 reset 会清除 GSTAT 后返回对应错误。
 * 功率级是否建立统一由 stpr_checkDriverPowerReady() 或 MotorDriver_CheckHealth() 追加检查。
 *
 * 检查并解析 TMC5130 GSTAT/DRV_STATUS 驱动异常，不检查 CS_ACTUAL 功率级。
 *
 * @param tmc5130 TMC5130 设备对象。该实例提供 SPI、片选和使能 GPIO，用于读取驱动状态寄存器并判断故障。
 * @return NO_ERROR 或对应电机故障错误码。
 */
uint32_t stpr_checkDriverStatus(TMC5130TypeDef *tmc5130);

/**
 * @brief 检查驱动功率级是否已经建立实际电流。
 *
 * 用于初始化和运动前/运动中的健康检查；会读取 DRV_STATUS 并可能打印错误，
 * 因此只允许在任务上下文调用。
 */
uint32_t stpr_checkDriverPowerReady(TMC5130TypeDef *tmc5130);

/* ---------- 参数即时更新 ---------- */

/**
 * @brief 设置驱动电流（IHOLD_IRUN 寄存器）。
 *
 * 设置运行电流 IRUN，保持 IHOLD/IHOLDDELAY 为统一默认值。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param current 运行电流 IRUN 的编码值（0~31）。
 * @return NO_ERROR 表示设置驱动电流（IHOLD_IRUN 寄存器）已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t stpr_setCurrent(TMC5130TypeDef *tmc5130, uint8_t current);

/**
 * @brief  设置最大速度（VMAX）
 *
 * 运行中刷新 VMAX。
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param velocity 待写入 TMC5130 的速度寄存器值。
 * @return NO_ERROR 表示设置最大速度（VMAX）已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t stpr_setVelocity(TMC5130TypeDef *tmc5130, uint32_t velocity);

#endif /* TMC_IC_TMC5130_H_ */
