
#ifndef TMC_IC_TMC5130_H_
#define TMC_IC_TMC5130_H_

#include "main.h"
#include <stdbool.h>

#include "TMC5130_Constants.h"
#include "TMC5130_Pins.h"
#include "TMC5130_Register.h"

/* TMC5130 通信诊断阶段。 */
#define TMC5130_DIAG_STAGE_NONE               0U
#define TMC5130_DIAG_STAGE_PARAMETER          1U
#define TMC5130_DIAG_STAGE_ACCESS_BUSY        2U
#define TMC5130_DIAG_STAGE_SPI_READ_TRIGGER   3U
#define TMC5130_DIAG_STAGE_SPI_READ_DATA      4U
#define TMC5130_DIAG_STAGE_SPI_WRITE          5U
#define TMC5130_DIAG_STAGE_XACTUAL_UNSTABLE   6U
#define TMC5130_DIAG_STAGE_CONFIGURATION_LOST 7U
#define TMC5130_DIAG_STAGE_CHIP_RESET         8U

/* TMC5130 诊断访问方向。 */
#define TMC5130_DIAG_DIRECTION_NONE  0U
#define TMC5130_DIAG_DIRECTION_WRITE 1U
#define TMC5130_DIAG_DIRECTION_READ  2U

/* 保存最后一次有效 TMC5130 故障现场，供最终错误日志和测试读取。 */
typedef struct {
    uint32_t sequence;
    uint32_t error_code;
    uint32_t hal_status;
    uint32_t expected_value;
    uint32_t actual_value;
    int32_t sample_first;
    int32_t sample_second;
    int32_t sample_third;
    uint8_t stage;
    uint8_t direction;
    uint8_t address;
    uint8_t response_status;
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

/* * 初始化 TMC5130 寄存器、SPI 句柄、片选脚和运行电流。 */
uint32_t stpr_initStepper(TMC5130TypeDef *tmc5130,
                      SPI_HandleTypeDef *spi,
                      GPIO_TypeDef *cs_port,
                      uint16_t cs_pin,
                      uint8_t dir,
                      uint8_t current);

/* * 关闭驱动输出。 */
void stpr_disableDriver(TMC5130TypeDef *tmc5130);

/* * 使能驱动输出。 */
void stpr_enableDriver(TMC5130TypeDef *tmc5130);

/* ---------- 寄存器访问 ---------- */

/* * 向 TMC5130 指定寄存器写入 32bit 值，返回写入是否成功。 */
bool stpr_writeInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t value);


/* * 带成功/失败返回值的寄存器读取接口，避免 SPI 失败时把寄存器值误判为 0。 */
bool stpr_tryReadInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value);

/* 复制最后一次有效 TMC5130 故障现场，不访问 SPI。 */
void TMC5130_GetDiagnosticSnapshot(TMC5130DiagnosticSnapshot *snapshot);

/* ---------- 运动控制 ---------- */

/* * 按速度斜坡停止电机。 */
uint32_t stpr_stop(TMC5130TypeDef *tmc5130);

/* * Velocity mode: keep rotating at signed VMAX until caller stops or changes speed. */
uint32_t stpr_rotate(TMC5130TypeDef *tmc5130, int32_t velocity);

/* * 位置模式：移动到指定绝对 ticks 位置。 */
uint32_t stpr_moveTo(TMC5130TypeDef *tmc5130, int32_t position, uint32_t velocityMax);

/* * 位置模式：以当前位置为基准相对移动 ticks，成功后 *ticks 会被改写为绝对目标位置。 */
uint32_t stpr_moveBy(TMC5130TypeDef *tmc5130, int32_t *ticks, uint32_t velocityMax);

/* * 同步设置 XACTUAL 和 XTARGET，常用于标定零点时重建驱动坐标。 */
uint32_t stpr_setPos(TMC5130TypeDef *tmc5130, int32_t position);

/* * 阻塞等待当前运动结束，同时检查驱动基础故障。 */
uint32_t stpr_waitMove(TMC5130TypeDef *tmc5130);

/* * 检查并解析 TMC5130 GSTAT/DRV_STATUS 驱动异常，不检查 CS_ACTUAL 功率级。 */
uint32_t stpr_checkDriverStatus(TMC5130TypeDef *tmc5130);

/**
 * @brief 检查驱动功率级是否已经建立实际电流。
 *
 * 用于初始化和运动前/运动中的健康检查；会读取 DRV_STATUS 并可能打印错误，
 * 因此只允许在任务上下文调用。
 */
uint32_t stpr_checkDriverPowerReady(TMC5130TypeDef *tmc5130);

/* ---------- 参数即时更新 ---------- */

/* * 设置运行电流 IRUN，保持 IHOLD/IHOLDDELAY 为统一默认值。 */
uint32_t stpr_setCurrent(TMC5130TypeDef *tmc5130, uint8_t current);

/* * 运行中刷新 VMAX。 */
uint32_t stpr_setVelocity(TMC5130TypeDef *tmc5130, uint32_t velocity);

#endif /* TMC_IC_TMC5130_H_ */
