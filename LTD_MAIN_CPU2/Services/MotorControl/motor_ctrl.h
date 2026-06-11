/*
 * motor_ctrl.h
 *
 * 步进电机上层控制模块（TMC5130）。
 *
 * 模块职责：
 *  - 统一电机方向、速度、电流和运动等待口径
 *  - 负责 mm <-> TMC5130 ticks 的卷筒模型换算
 *  - 维护编码轮记步 / 电机记步两套位置源
 *  - 处理运动过程中的碰撞、驱动异常、丢步检测和位置持久化
 *
 * 全工程必须保持一致的单位：
 *  - speed_x100：0.01m/min，100 表示 1.00m/min
 *  - cable_length / sensor_position / motor_distance：0.1mm
 *  - first_loop_circumference_mm：0.1mm
 *  - motor_count_first_loop_circumference_mm：0.001mm
 *  - tape_thickness_mm：0.001mm
 */

#ifndef INC_MOTOR_CTRL_H_
#define INC_MOTOR_CTRL_H_

#include <stdint.h>
#include <stdbool.h>
#include "TMC5130.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 方向和基础类型 ===================== */

/* * 下行放带：cable_length 增加。 */
#define MOTOR_DIRECTION_DOWN   0

/* * 上行收带：cable_length 减少。 */
#define MOTOR_DIRECTION_UP     1

/* * 当前换算后的 TMC5130 VMAX，主要供调试观察；业务层速度由参数和测量流程统一维护。 */
extern uint32_t velocity;


/**
 * @brief 电机侧卷筒状态快照
 *
 * 注意：motor_distance_01mm 是根据电机模型预测的长度，不等同于编码轮物理长度。
 */
typedef struct
{
    int32_t motor_step;          /* /< TMC5130_XACTUAL，单位 ticks */
    double  turns_total;         /* /< 输出轴总圈数，含小数 */
    int32_t turns_int;           /* /< 整圈数 */
    double  angle_deg;           /* /< 当前圈内角度，范围 [0, 360) */
    int32_t motor_distance_01mm; /* /< 电机模型预测长度，单位 0.1mm */
} MotorDrumState;

/* ===================== 初始化 / 停止 / 状态 ===================== */

/* * 初始化 TMC5130、恢复电机位置持久化数据并使能驱动。 */
uint32_t MotorCtrl_Init(void);

/* * 上电早期安全停机：清除 TMC5130 旧运动状态，必须在 App_Init() 前调用。 */
uint32_t MotorCtrl_BootSafeStop(void);

/* * 标记 TMC5130 需要重新完整初始化，用于驱动复位/通信异常后的恢复。 */
void MotorCtrl_InvalidateDriverInit(void);

/* * 判断 TMC5130 初始化和上电安全停机状态是否仍有效，供故障恢复判断是否需要先初始化。 */
bool MotorCtrl_IsDriverInitValid(void);

/* * 急停：立即停机，短暂关闭驱动后重新使能。 */
uint32_t MotorCtrl_QuickStop(void);

/* * 慢停：只下发 TMC5130 停止命令，等待驱动按斜坡减速。 */
uint32_t MotorCtrl_SlowStop(void);

/* * 获取当前显示用电机状态：0 停止，1 上行，2 下行。 */
uint32_t MotorCtrl_GetDisplayState(void);

/* * 将业务方向转换为统一中文文本，避免日志各处手写方向导致口径相反。 */
const char *MotorCtrl_DirectionText(int dir);

/* * 将显示状态转换为统一中文文本：0 静止，1 上行，2 下行。 */
const char *MotorCtrl_DisplayStateText(uint32_t display_state);

/**
 * @brief 运行期位置轮询
 *
 * 运动中按固定周期读取 XACTUAL，刷新 motor_step/motor_distance；
 * 当位置源为电机记步时，同时刷新 cable_length/sensor_position。
 * 可在主循环和等待循环中高频调用，函数内部自带节流。
 */
uint32_t MotorCtrl_PollRuntimePosition(void);

/* * 强制读取 XACTUAL 并刷新调试用卷筒状态。 */
void MotorCtrl_RefreshDebugDrumState(void);

/* * 按当前记步源强制刷新业务位置：电机记步读 XACTUAL，编码轮记步按编码轮刷新。 */
void MotorCtrl_RefreshPositionFromActiveSource(void);

/* * 打印当前编码轮/电机位置参考，供现场排查位置源差异。 */
void MotorCtrl_PrintPositionRefs(void);

/* * 打印编码轮位置、电机推算位置和差值。 */
void MotorCtrl_PrintPositionCompare(void);

/* * 电机记步专用诊断：打印基准、局部周长、XACTUAL/XTARGET、编码轮差值和运动状态。 */
void MotorCtrl_PrintMotorCountStatus(void);

/* * 底层驱动目标位置变化后调用，用于保存 XACTUAL/XTARGET 和电机记步基准。 */
void MotorCtrl_PersistRegistersFromDriver(void);

/* ===================== 位置源 / 零点基准 ===================== */

/**
 * @brief 上位机或 CPU3 写入设备参数后调用
 *
 * 修正非法 position_count_mode / motor_count_first_loop_circumference_mm；
 * 如果当前为电机记步模式，同步刷新电机推算位置。
 */
void MotorCtrl_ApplyPositionSourceParams(void);

/**
 * @brief 回零成功后的电机记步基准清理
 *
 * 清零 XACTUAL/XTARGET、清除电机记步切换基准和旧局部周长。
 * 普通回零和标定零点都会调用，随后统一切回编码轮记步。
 */
uint32_t MotorCtrl_ResetDrumReferenceForZeroCalibration(void);

/* * 切换为编码轮记步，后续 cable_length/sensor_position 由外部编码器刷新。 */
uint32_t MotorCtrl_SwitchPositionSourceToEncoder(void);

/**
 * @brief 切换为电机记步
 *
 * 切换瞬间以当前编码轮长度作为基准；随后通过 XACTUAL 相对变化量推算位置。
 * 切换过程中会自动下行一圈测量当前位置局部周长，再回到切换前位置。
 */
uint32_t MotorCtrl_SwitchPositionSourceToMotor(void);

/* * 当前整机位置源是否为电机记步。 */
bool MotorCtrl_IsPositionSourceMotor(void);

/**
 * @brief 用编码轮差分校正当前位置局部尺带周长
 *
 * 前提：已经处于电机记步模式，编码器仍可信，并且电机已经走过足够步数。
 * 只更新 motor_count_first_loop_circumference_mm，不修改全局 first_loop_circumference_mm。
 */
uint32_t MotorCtrl_CalibrateCurrentTapeCircumference(void);

/* * 从 TMC5130_XACTUAL 计算圈数、圈内角度和电机模型预测长度。 */
void MotorCtrl_UpdateDrumStateFromXActual(TMC5130TypeDef *tmc5130,
                                       MotorDrumState *out);

/* ===================== 速度 / 电流参数 ===================== */

/**
 * @brief 持久修改速度设定
 *
 * 若电机空闲，只影响后续运动；若电机正在运行，会立即重算并写入 VMAX。
 */

/* * 设置 TMC5130 运行电流 IRUN；驱动已初始化时立即生效。 */
uint32_t MotorCtrl_SetCurrent(uint32_t current);

/* * 获取默认运动速度，当前取 g_deviceParams.max_motor_speed。 */
uint32_t MotorCtrl_GetDefaultSpeedX100(void);

/* ===================== 常规运动接口 ===================== */

/* * 非阻塞连续上行，返回后电机仍可能在运行。 */
uint32_t MotorCtrl_MoveUp(uint32_t speed_x100);

/* * 非阻塞连续下行，返回后电机仍可能在运行。 */
uint32_t MotorCtrl_MoveDown(uint32_t speed_x100);

/**
 * @brief 按距离下发运动命令（非阻塞）
 *
 * mm 为正距离，dir 为 MOTOR_DIRECTION_UP/DOWN；函数只负责下发目标，
 * 上层需要自行等待或监控停止。
 */
uint32_t MotorCtrl_MoveNoWait(float mm, int dir, uint32_t speed_x100);

/* * 按距离运动并阻塞等待停止；推荐业务流程优先使用该接口。 */
uint32_t MotorCtrl_MoveAndWait(float mm, int dir, uint32_t speed_x100);

/* * 按 TMC5130 ticks 相对运动并等待停止，主要用于标定和内部换算后的步数控制。 */
uint32_t MotorCtrl_MoveByTicksAndWait(int32_t ticks, uint32_t speed_x100);

/* * 移动到绝对尺带位置 target_mm，并阻塞等待停止。 */
uint32_t MotorCtrl_MoveToPosition(float target_mm, uint32_t speed_x100);

/* * Jog mode: move relative distance using velocity mode and active position feedback. */
uint32_t MotorCtrl_JogMoveAndWait(float mm, int dir, uint32_t speed_x100);

/* * Jog mode: move to absolute position using velocity mode and active position feedback. */
uint32_t MotorCtrl_JogMoveToPosition(float target_mm, uint32_t speed_x100);

/* * 无检测阻塞运动，调试/维护用，不建议用于关键测量流程。 */
uint32_t MotorCtrl_MoveBlockingNoDetect(float mm, int dir, uint32_t speed_x100);

/* * 强制调试无检测运动：只绕过编码器首帧门控，仍保留驱动初始化和上电安全检查。 */
uint32_t MotorCtrl_MoveBlockingNoDetectForceDebug(float mm, int dir, uint32_t speed_x100);

/* ===================== 驱动状态 / 故障检测 ===================== */

/* * Read driver moving state; is_moving is valid only when NO_ERROR is returned. */
uint32_t MotorCtrl_IsDriverMoving(TMC5130TypeDef *tmc5130, bool *is_moving);

/* * 兼容旧命名的驱动健康检查：实际检查 GSTAT/DRV_STATUS、配置和功率级。 */
uint32_t MotorCtrl_CheckDriverGstat(void);

/* * 每次新的运动阶段开始前初始化丢步检测状态。 */
void MotorCtrl_LostStepInit(void);

/* * 自动定时丢步检测；currentPos 单位 0.1mm，通常传 sensor_position。 */
uint32_t MotorCtrl_CheckLostStepAutoTiming(int32_t currentPos);

/* * 读取当前 sensor_position 快照，输出单位 mm。 */
void MotorCtrl_SnapshotSensorPositionMm(float *pos_mm);

/* ===================== 尺带参数标定 / TFIT ===================== */

/* * 开始全局 TFIT 采样，样本用于拟合首圈周长 C0 和尺带厚度 t。 */
void MotorCtrl_TapeFitStart(void);

/* * 开始局部 TFIT 采样，以当前位置作为局部 0 圈起点。 */
void MotorCtrl_TapeFitStartLocalOrigin(void);

/* * 停止 TFIT 自动采样。 */
void MotorCtrl_TapeFitStop(void);

/* * 手动加入当前电机/编码轮位置作为一条 TFIT 样本。 */
void MotorCtrl_TapeFitAddCurrentSample(void);

/* * 打印 TFIT 样本数、状态和最近求解结果。 */
void MotorCtrl_TapeFitPrintStatus(void);

/* * 使用全局 TFIT 样本求解 C0 和 t。 */
uint32_t MotorCtrl_TapeFitSolve(void);

/* * 使用局部 TFIT 样本求解当前位置附近的局部周长/厚度。 */
uint32_t MotorCtrl_TapeFitSolveLocalOrigin(void);

/* * 将最近一次 TFIT 求解结果写入参数区；apply_c0/apply_t 控制写入项。 */
uint32_t MotorCtrl_TapeFitApply(bool apply_c0, bool apply_t);

/**
 * @brief 标定零点处第一圈周长
 *
 * 电机位于零点时下行一圈，读取放带长度 dL。
 * 当前实现模型：dL = C0 - pi * t，因此 C0 = dL + pi * t。
 * 标定成功后会保存 first_loop_circumference_mm，并同步刷新电机局部周长。
 */
uint32_t MotorCtrl_CalibrateFirstLoopCircumferenceAtZero(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_CTRL_H_ */
