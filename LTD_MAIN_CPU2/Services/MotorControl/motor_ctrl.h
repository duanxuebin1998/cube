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
 *  - fine_speed_m_min：液位近界面专用浮点速度，单位 m/min
 *  - cable_length / sensor_position / motor_distance：0.1mm
 *  - first_loop_circumference_mm：0.1mm
 *  - motor_count_first_loop_circumference_mm：0.001mm
 *  - tape_thickness_mm：0.001mm
 */

#ifndef INC_MOTOR_CTRL_H_
/* INC_MOTOR_CTRL_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define INC_MOTOR_CTRL_H_

#include <stdint.h>
#include <stdbool.h>
#include "TMC5130.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 方向和基础类型 ===================== */

/* * 下行放带：cable_length 增加。 */
#define MOTOR_DIRECTION_DOWN   0 /* 电机方向枚举：向下。 */

/* * 上行收带：cable_length 减少。 */
#define MOTOR_DIRECTION_UP     1 /* 电机方向枚举：向上。 */

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

/**
 * @brief 初始化 TMC5130 电机驱动和电机控制运行态。
 *
 * 首次初始化会恢复持久化位置和位置源；非首次初始化用于恢复/退出调试，
 * 只重写 TMC5130 寄存器配置，不恢复 FRAM 位置和位置源。
 *
 * 初始化 TMC5130、恢复电机位置持久化数据并使能驱动。
 *
 * @return 成功返回 NO_ERROR，否则返回驱动通信或状态错误码。
 */
uint32_t MotorCtrl_Init(void);

/**
 * @brief 上电早期清除 TMC5130 残留运动状态。
 *
 * 该函数在外设初始化完成后、App_Init() 前调用：先保持驱动输出关闭，
 * 再把 VMAX 清零、XTARGET 对齐当前 XACTUAL，并切回位置模式。
 * 这样即使设备在运动中复位，TMC5130 也不会沿用旧速度或旧目标继续跑。
 * 本函数会访问 SPI 和 GPIO，只能在任务上下文调用，不能在中断中调用。
 *
 * 上电早期安全停机：清除 TMC5130 旧运动状态，必须在 App_Init() 前调用。
 *
 * @return NO_ERROR 表示上电早期清除 TMC5130 残留运动状态已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
uint32_t MotorCtrl_BootSafeStop(void);

/**
 * @brief 使 TMC5130 初始化状态失效，强制下次运动前重新校验。
 *
 * 标记 TMC5130 需要重新完整初始化，用于驱动复位/通信异常后的恢复。
 */
void MotorCtrl_InvalidateDriverInit(void);

/**
 * @brief 判断当前 TMC5130 初始化状态是否仍可信。
 *
 * 判断 TMC5130 初始化和上电安全停机状态是否仍有效，供故障恢复判断是否需要先初始化。
 *
 * @return true 表示当前 TMC5130 初始化状态仍可信；false 表示当前 TMC5130 初始化状态已不再可信。
 */
bool MotorCtrl_IsDriverInitValid(void);

/**
 * @brief 急停电机并刷新状态。
 *
 * 急停会立即下发停止，并短暂关闭再重新使能驱动。
 *
 * 急停：立即停机，短暂关闭驱动后重新使能。
 *
 * @return 成功返回 NO_ERROR，否则返回停止或等待错误码。
 */
uint32_t MotorCtrl_QuickStop(void);

/**
 * @brief 慢停电机并等待斜坡减速结束。
 *
 * 只下发停止命令，保留驱动斜坡减速行为。
 *
 * 慢停：只下发 TMC5130 停止命令，等待驱动按斜坡减速。
 *
 * @return 成功返回 NO_ERROR，否则返回停止或等待错误码。
 */
uint32_t MotorCtrl_SlowStop(void);

/**
 * @brief 获取上层显示用电机运动状态。
 *
 * 获取当前显示用电机状态：0 停止，1 上行，2 下行。
 *
 * @return 0 表示停止，1 表示上行，2 表示下行。
 */
uint32_t MotorCtrl_GetDisplayState(void);

/**
 * @brief 返回电机运动方向对应的现场日志文字。
 *
 * 将业务方向转换为统一中文文本，避免日志各处手写方向导致口径相反。
 *
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @return 返回电机运动方向对应的现场日志文字对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *MotorCtrl_DirectionText(int dir);

/**
 * @brief 把电机控制状态转换为现场可读文字。
 *
 * 将显示状态转换为统一中文文本：0 静止，1 上行，2 下行。
 *
 * @param display_state 待转换为文字或运动方向的上层电机显示状态。
 * @return 返回现场可读文字对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
const char *MotorCtrl_DisplayStateText(uint32_t display_state);

/**
 * @brief 运行期位置轮询
 *
 * 运动中按固定周期读取 XACTUAL，刷新 motor_step/motor_distance；
 * 当位置源为电机记步时，同时刷新 cable_length/sensor_position。
 * 可在主循环和等待循环中高频调用，函数内部自带节流。
 */
uint32_t MotorCtrl_PollRuntimePosition(void);

/**
 * @brief 强制刷新调试用电机卷筒状态。
 *
 * 读取 XACTUAL 并更新 debug_data 中的 motor_step 和 motor_distance。
 *
 * 强制读取 XACTUAL 并刷新调试用卷筒状态。
 */
void MotorCtrl_RefreshDebugDrumState(void);

/**
 * @brief 按当前记步源强制刷新业务位置。
 *
 * 电机记步模式下读取 TMC5130 XACTUAL 并刷新 cable_length/sensor_position；
 * 编码轮记步模式下按编码轮重新计算当前位置。
 */
void MotorCtrl_RefreshPositionFromActiveSource(void);

/**
 * @brief 打印当前编码轮/电机位置参考信息。
 *
 * 用于现场快速确认记步模式、编码轮长度、电机模型长度和局部周长。
 *
 * 打印当前编码轮/电机位置参考，供现场排查位置源差异。
 */
void MotorCtrl_PrintPositionRefs(void);

/**
 * @brief 打印编码轮位置与电机推算位置的对比。
 *
 * 用于排查两套位置源是否发生偏差。
 *
 * 打印编码轮位置、电机推算位置和差值。
 */
void MotorCtrl_PrintPositionCompare(void);

/**
 * @brief 打印电机记步专用诊断状态。
 *
 * 统一输出基准、局部周长、XACTUAL/XTARGET、编码轮差值和运动状态。
 *
 * 电机记步专用诊断：打印基准、局部周长、XACTUAL/XTARGET、编码轮差值和运动状态。
 */
void MotorCtrl_PrintMotorCountStatus(void);

/**
 * @brief 从当前驱动状态强制保存电机位置寄存器。
 *
 * 供底层目标位置变化后调用，确保 XACTUAL/XTARGET 和基准能断电恢复。
 *
 * 底层驱动目标位置变化后调用，用于保存 XACTUAL/XTARGET 和电机记步基准。
 */
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

/**
 * @brief 切换为编码轮记步位置源。
 *
 * 切换后业务位置由外部编码器刷新，电机基准仍保留用于诊断和后续切回。
 *
 * 切换为编码轮记步，后续 cable_length/sensor_position 由外部编码器刷新。
 *
 * @return NO_ERROR表示切换完成；编码器未就绪、位置不可信或已有锁存故障时返回具体错误码。
 */
uint32_t MotorCtrl_SwitchPositionSourceToEncoder(void);

/**
 * @brief 切换为电机记步
 *
 * 切换瞬间以当前编码轮长度作为基准；随后通过 XACTUAL 相对变化量推算位置。
 * 切换过程中会自动下行一圈测量当前位置局部周长，再回到切换前位置。
 */
uint32_t MotorCtrl_SwitchPositionSourceToMotor(void);

/**
 * @brief 判断当前是否使用电机记步作为位置源。
 *
 * 当前整机位置源是否为电机记步。
 *
 * @return 电机记步返回 true，编码轮记步返回 false。
 */
bool MotorCtrl_IsPositionSourceMotor(void);

/**
 * @brief 用编码轮差分校正当前位置局部尺带周长
 *
 * 前提：已经处于电机记步模式，编码器仍可信，并且电机已经走过足够步数。
 * 只更新 motor_count_first_loop_circumference_mm，不修改全局 first_loop_circumference_mm。
 */
uint32_t MotorCtrl_CalibrateCurrentTapeCircumference(void);

/**
 * @brief 从 TMC5130_XACTUAL 计算并输出卷筒状态。
 *
 * 兼容旧接口：读取失败时使用最近缓存状态填充输出。
 *
 * 从 TMC5130_XACTUAL 计算圈数、圈内角度和电机模型预测长度。
 *
 * @param tmc5130 TMC5130 设备对象。该实例用于读取 XACTUAL，并按当前位置模型更新卷筒圈数、角度和尺带长度。
 * @param out 输出卷筒状态。
 */
void MotorCtrl_UpdateDrumStateFromXActual(TMC5130TypeDef *tmc5130,
                                       MotorDrumState *out);

/* ===================== 速度 / 电流参数 ===================== */

/**
 * @brief 持久修改速度设定
 *
 * 若电机空闲，只影响后续运动；若电机正在运行，会立即重算并写入 VMAX。
 */

/**
 * @brief 持久设置 TMC5130 运行电流。
 *
 * 驱动初始化后会立即写 IHOLD_IRUN，使运行电流无需重启即可生效。
 *
 * 设置 TMC5130 运行电流 IRUN；驱动已初始化时立即生效。
 *
 * @param current 请求运行电流档位。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_SetCurrent(uint32_t current);

/**
 * @brief 获取对外公开的默认电机速度。
 *
 * 获取默认运动速度，当前取 g_deviceParams.max_motor_speed。
 *
 * @return 默认速度，单位 0.01m/min。
 */
uint32_t MotorCtrl_GetDefaultSpeedX100(void);

/* ===================== 常规运动接口 ===================== */

/**
 * @brief 以指定速度连续上行收带。
 *
 * 非阻塞连续上行，返回后电机仍可能在运行。
 *
 * @param speed_x100 运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_MoveUp(uint32_t speed_x100);

/**
 * @brief 以指定速度连续下行放带。
 *
 * 非阻塞连续下行，返回后电机仍可能在运行。
 *
 * @param speed_x100 运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_MoveDown(uint32_t speed_x100);

/**
 * @brief 检查命令切换、方向、运动门禁和驱动健康后启动连续速度模式。
 *
 * Start continuous velocity mode in the given direction.
 *
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return PARAM_RANGE_ERROR 表示参数超出允许范围；NO_ERROR 表示操作成功。
 */
uint32_t MotorCtrl_StartVelocity(int dir, uint32_t speed_x100);

/**
 * @brief 以低于常规0.01m/min分辨率的速度启动液位近界面连续运动。
 *
 * @param dir 运动方向，必须为 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN。
 * @param speed_m_min 精细线速度，单位 m/min，必须大于0且不超过常规最大速度。
 * @return NO_ERROR表示启动成功，其他值为参数、驱动或通信错误。
 */
uint32_t MotorCtrl_StartFineVelocity(int dir, float speed_m_min);

/**
 * @brief 按指定距离和方向下发非阻塞电机运动命令。
 *
 * 函数只完成目标位置换算和命令下发，返回后电机可能仍在运行；调用方必须自行等待到位、监控停止条件或处理中途故障。
 *
 * @param mm 正向移动距离，单位 mm；负值返回参数范围错误，0 表示无需运动。
 * @param dir 运动方向；必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN。
 * @param speed_x100 本次运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return NO_ERROR 表示运动命令已成功下发；其他值为参数、驱动就绪、位置换算或通信错误码。
 */
uint32_t MotorCtrl_MoveNoWait(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 按距离运动并阻塞等待停止。
 *
 * 该接口是业务流程推荐的距离运动入口，内部包含目标规划、到位等待和保护检测。
 * @param mm 移动距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、保护或打断错误码。
 */
uint32_t MotorCtrl_MoveAndWait(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 按相对 ticks 移动并等待停止。
 *
 * 主要用于标定、位置源切换和内部精确步数控制。
 *
 * 按 TMC5130 ticks 相对运动并等待停止，主要用于标定和内部换算后的步数控制。
 *
 * @param ticks 相对移动步数，正负号表示方向。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、通信或打断错误码。
 */
uint32_t MotorCtrl_MoveByTicksAndWait(int32_t ticks, uint32_t speed_x100);

/**
 * @brief 移动到指定绝对位置并等待停止。
 *
 * 函数根据当前位置与目标位置自动判断方向和距离。
 *
 * 移动到绝对尺带位置 target_mm，并阻塞等待停止。
 *
 * @param target_mm 目标绝对位置，单位 mm。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、打断或到位错误码。
 */
uint32_t MotorCtrl_MoveToPosition(float target_mm, uint32_t speed_x100);

/**
 * @brief 使用速度点动模式按相对距离运动并等待到位。
 *
 * 该接口不把目标距离一次性换算成 XTARGET，而是先下发方向和速度，运行中持续读取
 * 当前有效位置源；接近目标时切换到低速，达到或越过保护边界立即慢停。
 *
 * Jog mode: move relative distance using velocity mode and active position feedback.
 *
 * @param mm 移动距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次运动速度，0 表示使用当前默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、保护或打断错误码。
 */
uint32_t MotorCtrl_JogMoveAndWait(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 使用速度点动模式运动到绝对位置并等待到位。
 *
 * 该接口用于长距离运动的提前减速场景：启动前只计算目标方向，运行中按有效位置源
 * 判断剩余距离、减速距离、越界和异常停止，不依赖拟合距离一次性生成 XTARGET。
 *
 * Jog mode: move to absolute position using velocity mode and active position feedback.
 *
 * @param target_mm 目标位置，单位 mm。
 * @param speed_x100 本次运动速度，0 表示使用当前默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、保护或打断错误码。
 */
uint32_t MotorCtrl_JogMoveToPosition(float target_mm, uint32_t speed_x100);

/**
 * @brief 调试/维护用无检测阻塞运动。
 *
 * 该接口不做撞底和丢步保护，只用于现场调试或受控维护流程。
 *
 * 无检测阻塞运动，调试/维护用，不建议用于关键测量流程。
 *
 * @param mm 移动距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 *
 * @return 返回无撞底和丢步检测阻塞运动结果；NO_ERROR 表示到位，其他值保留命令切换、驱动或超时错误。
 */
uint32_t MotorCtrl_MoveBlockingNoDetect(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 兼容入口：执行不带撞底和丢步检测、且不打印过程日志的阻塞运动。
 *
 * 无检测阻塞运动静默版，仅供内部短距离退让使用。
 *
 * @param mm 本次电机运动的距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return 返回静默无检测阻塞运动结果；NO_ERROR 表示到位，其他值保留命令切换、驱动或超时错误。
 */
uint32_t MotorCtrl_MoveBlockingNoDetectQuiet(float mm, int dir, uint32_t speed_x100);

/**
 * @brief 强制调试无检测阻塞运动。
 *
 * 只绕过编码器首帧门控，仍保留上电安全停机、驱动初始化和 TMC 健康检查。
 * 仅供人工强制上/下行等调试命令使用，正常测量流程不能调用。
 *
 * 强制调试无检测运动：只绕过编码器首帧门控，仍保留驱动初始化和上电安全检查。
 *
 * @param mm 本次电机运动的距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return 返回强制调试无检测阻塞运动结果；NO_ERROR 表示到位，不可绕过的驱动或超时错误原样返回。
 */
uint32_t MotorCtrl_MoveBlockingNoDetectForceDebug(float mm, int dir, uint32_t speed_x100);

/* ===================== 驱动状态 / 故障检测 ===================== */

/**
 * @brief 读取 TMC5130 实时运动状态并通过输出参数返回。
 *
 * Read driver moving state; is_moving is valid only when NO_ERROR is returned.
 *
 * @param tmc5130 目标 TMC5130 驱动实例。对象保存 SPI 句柄、片选 GPIO 和使能 GPIO，底层寄存器访问与驱动使能操作均通过该实例定位硬件。
 * @param is_moving 运动状态输出指针；函数返回 NO_ERROR 时，true 表示仍在运动，false 表示已经确认停止。
 * @return 返回 TMC5130 运动状态读取结果码；NO_ERROR 时通过 is_moving 返回当前是否运动。
 */
uint32_t MotorCtrl_IsDriverMoving(TMC5130TypeDef *tmc5130, bool *is_moving);

/**
 * @brief 对外兼容的 TMC5130 驱动健康检查入口。
 *
 * 历史接口名保留为 Gstat，但当前系统只有一个全局 stepper，
 * 实际会统一检查 GSTAT/DRV_STATUS、配置寄存器和功率级状态。
 *
 * 兼容旧命名的驱动健康检查：实际检查 GSTAT/DRV_STATUS、配置和功率级。
 *
 * @return NO_ERROR 或对应驱动故障错误码。
 */
uint32_t MotorCtrl_CheckDriverGstat(void);

/**
 * @brief 初始化丢步检测窗口。
 *
 * 每次新的运动阶段开始前调用，清空位置采样、时间戳和报警抑制计时。
 *
 * 每次新的运动阶段开始前初始化丢步检测状态。
 */
void MotorCtrl_LostStepInit(void);

/**
 * @brief 按固定周期执行丢步检测。
 *
 * 函数内部自带采样节流，比较最近窗口平均速度与设定速度。
 *
 * 自动定时丢步检测；currentPos 单位 0.1mm，通常传 sensor_position。
 *
 * @param currentPos 当前业务位置，单位 0.1mm。
 * @return NO_ERROR 或疑似丢步错误码。
 */
uint32_t MotorCtrl_CheckLostStepAutoTiming(int32_t currentPos);

/**
 * @brief 获取当前传感器位置快照。
 *
 * 读取当前 sensor_position 快照，输出单位 mm。
 *
 * @param pos_mm 输出当前位置，单位 mm；传入 NULL 时不执行操作。
 */
void MotorCtrl_SnapshotSensorPositionMm(float *pos_mm);

/* ===================== 尺带参数标定 / TFIT ===================== */

/**
 * @brief 开始全局 TFIT 自动采样。
 *
 * 清空旧样本和旧结果，后续运动过程中会自动记录样本。
 *
 * 开始全局 TFIT 采样，样本用于拟合首圈周长 C0 和尺带厚度 t。
 */
void MotorCtrl_TapeFitStart(void);

/**
 * @brief 以当前位置作为原点开始局部 TFIT 采样。
 *
 * 局部拟合用于求当前位置附近的局部周长和厚度，不直接改变全局原点。
 *
 * 开始局部 TFIT 采样，以当前位置作为局部 0 圈起点。
 */
void MotorCtrl_TapeFitStartLocalOrigin(void);

/**
 * @brief 停止 TFIT 自动采样并保留已有样本。
 */
void MotorCtrl_TapeFitStop(void);

/**
 * @brief 手动加入当前电机/编码轮状态作为 TFIT 样本。
 */
void MotorCtrl_TapeFitAddCurrentSample(void);

/**
 * @brief 打印 TFIT 当前采样状态和最近求解结果。
 */
void MotorCtrl_TapeFitPrintStatus(void);

/**
 * @brief 使用全局 TFIT 样本求解 C0、厚度和零点偏移。
 *
 * 使用全局 TFIT 样本求解 C0 和 t。
 *
 * @return 成功返回 NO_ERROR，否则返回样本不足或求解失败错误码。
 */
uint32_t MotorCtrl_TapeFitSolve(void);

/**
 * @brief 使用局部原点样本求解当前位置附近的周长和厚度。
 *
 * 使用局部 TFIT 样本求解当前位置附近的局部周长/厚度。
 *
 * @return 成功返回 NO_ERROR，否则返回样本不足或求解失败错误码。
 */
uint32_t MotorCtrl_TapeFitSolveLocalOrigin(void);

/**
 * @brief 将最近一次 TFIT 求解结果写入设备参数。
 *
 * 将最近一次 TFIT 求解结果写入参数区；apply_c0/apply_t 控制写入项。
 *
 * @param apply_c0 为 true 时应用首圈周长或局部周长。
 * @param apply_t 为 true 时应用尺带厚度。
 * @return 成功返回 NO_ERROR，否则返回参数或状态错误码。
 */
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
