/**
 * @file motor_ctrl_internal.h
 * @brief MotorControl 模块内部共享声明。
 *
 * 该头文件只允许同目录 motor_ctrl_*.c 使用，用于集中保存内部常量、共享类型、
 * 跨文件状态和跨文件函数声明。公共业务接口必须放在 motor_ctrl.h，避免内部实现
 * 细节泄漏到其它模块。
 */

#ifndef MOTOR_CTRL_INTERNAL_H_
#define MOTOR_CTRL_INTERNAL_H_

#include "motor_ctrl.h"
#include "fault_manager.h"
#include "spi.h"
#include "measure_tank_height.h"
#include "sensor.h"
#include "encoder.h"
#include "mb85rs2m.h"
#include "my_crc.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 内部常量/配置 ===================== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EPS_MM                   0.20f    /* 到位公差(±mm) */

#define LOST_STEP_WINDOW                 6       /* 6个采样点 -> 覆盖最近5秒 */
#define LOST_STEP_INTERVAL_MS            1000    /* 每1秒采样一次 */
#define LOST_STEP_SUPPRESS_MS            1000    /* 报警抑制1秒 */
#define LOST_STEP_SPEED_RATIO_PERCENT    20U     /* 平均速度低于设定速度20%判定疑似丢步 */
#define LOST_STEP_MIN_AVG_SPEED_MM_S     0.5f    /* 低速档丢步判定绝对下限 */

#ifndef C0_MIN_MM
#define C0_MIN_MM (50.0)
#endif
#ifndef C0_MAX_MM
#define C0_MAX_MM (5000.0)
#endif

#define MOTOR_VELOCITY_BASE     (10000UL)

#ifndef MOTOR_LINEAR_SPEED_MIN_X100
#define MOTOR_LINEAR_SPEED_MIN_X100   10U
#endif
#ifndef MOTOR_LINEAR_SPEED_MAX_X100
#define MOTOR_LINEAR_SPEED_MAX_X100   600U
#endif

#ifndef TMC5130_FCLK_HZ
#define TMC5130_FCLK_HZ               (12000000.0)
#endif

#ifndef MOTOR_LINEAR_SPEED_COMP_ENABLE
#define MOTOR_LINEAR_SPEED_COMP_ENABLE       1
#endif
#ifndef MOTOR_LINEAR_COMP_UPDATE_MS
#define MOTOR_LINEAR_COMP_UPDATE_MS          200U
#endif
#ifndef MOTOR_LINEAR_COMP_APPLY_DELTA_X100
#define MOTOR_LINEAR_COMP_APPLY_DELTA_X100   3U
#endif

#ifndef TAPE_MIN_RADIUS_MM
#define TAPE_MIN_RADIUS_MM   (40.0)
#endif

#define MOTOR_MOVE_RETRY_MAX 3

#ifndef MOTOR_STOP_WAIT_TIMEOUT_MS
#define MOTOR_STOP_WAIT_TIMEOUT_MS          15000U
#endif

#ifndef COMMAND_SWITCH_ABORT
#define COMMAND_SWITCH_ABORT   STATE_SWITCH
#endif

#define MOTOR_STORE_MAGIC            (0x4D4F544Fu) /* 'MOTO' */
#define MOTOR_STORE_VERSION          (1u)
#define FRAM_MOTOR_A_ADDRESS         (FRAM_ANGLE_ADDRESS + 0x80u)
#define FRAM_MOTOR_SLOT_SIZE         (0x40u)
#define FRAM_MOTOR_B_ADDRESS         (FRAM_MOTOR_A_ADDRESS + FRAM_MOTOR_SLOT_SIZE)
#define MOTOR_PERSIST_DELTA_MM       (1.0)

#define MOTOR_TAPE_FIT_MAX_SAMPLES          (256)
#define MOTOR_TAPE_FIT_AUTO_DELTA_TICKS     (1536000 / 4)

/* 非 void 函数：检测命令切换 -> 停止电机；停止失败返回实际错误码，停止成功返回 retcode。 */
#define CHECK_COMMAND_SWITCH_AND_STOP(retcode)                                    \
    do {                                                                          \
        if (HasEffectiveCommandSwitchRequest()) {                                 \
            uint32_t stop_ret;                                                    \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            stop_ret = MotorDriver_StopAndMarkStopped();                          \
            if (stop_ret != NO_ERROR) {                                           \
                return stop_ret;                                                  \
            }                                                                     \
            return (retcode);                                                     \
        }                                                                         \
    } while (0)

/* void 函数：检测命令切换 -> 停止电机 -> return。 */
#define CHECK_COMMAND_SWITCH_AND_STOP_NO_RETURN()                                 \
    do {                                                                          \
        if (HasEffectiveCommandSwitchRequest()) {                                 \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            (void)MotorDriver_StopAndMarkStopped();                               \
            return;                                                               \
        }                                                                         \
    } while (0)

/* 电机位置持久化记录。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t xactual;
    int32_t base_length_01mm;
    int32_t base_step;
    uint32_t crc;
} MotorPersistRecord;

/* TFIT 采样点。 */
typedef struct {
    int32_t motor_step;
    int32_t encoder_length_01mm;
} MotorTapeFitSample;

/* TFIT 拟合结果。 */
typedef struct {
    bool valid;
    double offset_mm;
    double first_loop_circ_mm;
    double tape_thickness_mm;
    double rmse_mm;
    double max_abs_err_mm;
} MotorTapeFitResult;

/* ===================== 内部共享状态 ===================== */

/* 驱动运行态：只保存跨文件需要共享的驱动侧状态。 */
typedef struct {
    uint32_t applied_velocity;  /* 最近一次写入/准备写入 TMC5130 的 VMAX。 */
    bool initialized;           /* TMC5130 驱动是否已经完成初始化。 */
} MotorDriverRuntime;

/* 位置运行态：保存电机记步模式的切换基准。 */
typedef struct {
    int32_t count_base_step;        /* 切换到电机记步时的 XACTUAL。 */
    int32_t count_base_length_01mm; /* 切换瞬间采用的尺带长度，单位 0.1mm。 */
    double count_base_turns;        /* 基准长度对应的模型圈数。 */
} MotorPositionRuntime;

extern MotorDriverRuntime s_motor_driver;
extern MotorPositionRuntime s_motor_position;
/* ===================== 内部跨文件函数 ===================== */

/**
 * @brief 读取当前线速度设定并转换为 m/min。
 *
 * @return 当前线速度，单位 m/min。
 */
float MotorDriver_GetSpeedSetpointMMin(void);

/**
 * @brief 根据当前速度参数刷新全局 VMAX 缓存。
 *
 * 该函数只更新软件侧 velocity 和已应用速度缓存，不主动判断运动状态。
 * 调用方如果需要立即生效，应继续写入 TMC5130 VMAX。
 */
void MotorDriver_UpdateVelocityFromParams(void);

/**
 * @brief 判断业务方向参数是否合法。
 *
 * @param dir 业务方向，必须是 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN。
 * @return 方向合法返回 1，否则返回 0。
 */
int MotorDriver_IsDirValid(int dir);

/**
 * @brief 下发停止命令并刷新显示状态。
 *
 * 该函数用于命令切换、异常保护和主动停止场景。停止命令只表示开始减速，
 * 最终显示状态仍会结合驱动 vzero / rampstat 判断。
 * @return 停止命令和等待停稳成功返回 NO_ERROR，否则返回实际错误码。
 */
uint32_t MotorDriver_StopAndMarkStopped(void);

/**
 * @brief 检查是否有有效命令切换请求，并在需要时停止电机。
 *
 * @return 无命令切换返回 NO_ERROR；命令切换且停止成功返回 STATE_SWITCH；停止失败返回实际错误码。
 */
uint32_t MotorDriver_StopIfCommandSwitchRequested(void);

/**
 * @brief 从 TMC5130 读取当前运动状态。
 *
 * 通过 RAMPSTAT/VACTUAL 等驱动状态推断电机是否仍在运动。
 * @param tmc5130 TMC5130 设备对象。
 * @param is_moving 输出运动状态。
 * @return 读取成功返回 true，通信失败返回 false。
 */
bool MotorDriver_TryReadMovingState(TMC5130TypeDef *tmc5130, bool *is_moving);

/**
 * @brief 根据驱动状态推断上层显示用运动状态。
 *
 * 显示状态只区分停止、上行、下行，读取失败时会尽量使用最近缓存状态兜底。
 * @param tmc5130 TMC5130 设备对象。
 * @return 0 表示停止，1 表示上行，2 表示下行。
 */
uint32_t MotorDriver_InferDisplayStateFromDriver(TMC5130TypeDef *tmc5130);

/**
 * @brief 按指定尺带长度计算恒线速度对应的 VMAX。
 *
 * 用于运行中速度补偿和下发运动前的速度计算。
 * @param L_mm 当前或目标参考尺带长度，单位 mm。
 * @return 对应 TMC5130 VMAX。
 */
uint32_t MotorDriver_ComputeUniformVelocityFromLength(double L_mm);

/**
 * @brief 运动过程中按卷径变化刷新 TMC5130 VMAX。
 *
 * 函数内部带刷新周期和变化阈值，避免高频重复写寄存器。
 * @param tmc5130 TMC5130 设备对象。
 * @param last_refresh_tick 上次刷新 tick，函数会在成功检查后更新。
 */
void MotorDriver_RefreshVelocityDuringRun(TMC5130TypeDef *tmc5130,
                                                  uint32_t *last_refresh_tick);

/**
 * @brief 处理运动接口传入的可选速度参数。
 *
 * speed_x100 为 0 时使用默认速度；非 0 时先限幅再用于本次运动。
 * @param speed_x100 请求速度，单位 0.01m/min。
 * @return 本次运动应使用的速度设定。
 */
uint32_t MotorDriver_ApplyOptionalSpeed(uint32_t speed_x100);

/**
 * @brief 为单次运动临时切换速度设定。
 *
 * 如果请求速度与当前速度不同，函数会保存原速度并写入临时速度。
 * @param speed_x100 请求速度，单位 0.01m/min。
 * @param restore_needed 输出是否需要结束时恢复速度。
 * @param restore_speed_x100 输出原速度。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorDriver_BeginTemporarySpeed(uint32_t speed_x100,
                                                 bool *restore_needed,
                                                 uint32_t *restore_speed_x100);

/**
 * @brief 单次运动结束后恢复临时速度。
 *
 * @param restore_needed 是否需要恢复。
 * @param restore_speed_x100 需要恢复的速度，单位 0.01m/min。
 * @return 成功返回 NO_ERROR，否则返回通信错误码。
 */
uint32_t MotorDriver_EndTemporarySpeed(bool restore_needed,
                                               uint32_t restore_speed_x100);

/**
 * @brief 无检测运动过程中的周期性调试日志。
 *
 * 用于维护模式下观察位置、陀螺仪和密度数据，不参与安全停机判断。
 */
void MotorLostStep_NoDetectRuntimeLogUpdate(void);

/**
 * @brief 获取当前电机记步局部周长参数。
 *
 * @return 合法局部周长，单位 mm；未配置时返回 0。
 */
double MotorPosition_GetLocalCircumferenceFromParams(void);

/**
 * @brief 将局部周长写回设备参数结构。
 *
 * 只更新 RAM 中的参数结构，是否保存由调用方决定。
 * @param circumference_mm 局部周长，单位 mm。
 * @return 写入成功返回 true，参数非法返回 false。
 */
bool MotorPosition_SetLocalCircumferenceToParams(double circumference_mm);

/**
 * @brief 获取卷筒输出轴每圈对应的电机 ticks。
 *
 * @return 每圈 ticks，异常配置时返回兜底值。
 */
int32_t MotorPosition_TapeTicksPerRev(void);

/**
 * @brief 获取尺带模型首圈周长 C0。
 *
 * @return 首圈周长，单位 mm。
 */
double MotorPosition_TapeC0Mm(void);

/**
 * @brief 获取尺带厚度参数。
 *
 * @return 尺带厚度，单位 mm。
 */
double MotorPosition_TapeThicknessMm(void);

/**
 * @brief 根据带符号尺带长度反算卷筒圈数。
 *
 * @param L_mm 尺带长度，单位 mm。
 * @param C0_mm 首圈周长，单位 mm。
 * @param t_mm 尺带厚度，单位 mm。
 * @return 反算得到的卷筒圈数。
 */
double MotorPosition_TapeTurnsFromSignedLength(double L_mm,
                                                   double C0_mm,
                                                   double t_mm);

/**
 * @brief 根据当前位置长度估算当前卷层瞬时周长。
 *
 * @param L_mm 当前尺带长度，单位 mm。
 * @return 当前卷层周长，单位 mm。
 */
double MotorPosition_TapeInstantCircumferenceFromLength(double L_mm);

/**
 * @brief 安全地将 double 四舍五入为 int64。
 *
 * 避免部分嵌入式库环境下直接依赖 llround 的兼容问题。
 * @param x 输入浮点数。
 * @return 四舍五入后的整数。
 */
int64_t MotorPosition_RoundToInt64(double x);

/**
 * @brief 获取当前尺带长度用于卷筒模型换算。
 *
 * 位置源为电机记步时优先使用电机推算长度，否则使用编码轮长度。
 * @return 当前尺带长度，单位 mm。
 */
double MotorPosition_GetCurrentTapeLengthMm(void);

/**
 * @brief 读取用于标定/拟合的编码轮长度。
 *
 * @return 编码轮长度，单位 0.1mm；读取失败时返回当前测量缓存。
 */
int32_t MotorPosition_ReadEncoderLengthForFit(void);

/**
 * @brief 上电初始化时从 FRAM 恢复电机位置寄存器。
 *
 * 恢复 XACTUAL/XTARGET，并缓存电机记步基准供位置源恢复使用。
 * @param tmc5130 TMC5130 设备对象。
 */
uint32_t MotorPosition_RestorePersistedRegisters(TMC5130TypeDef *tmc5130);

/**
 * @brief 同步调试区中的卷筒步数和模型长度。
 *
 * @param tmc5130 TMC5130 设备对象。
 */
void MotorPosition_SyncDebugDrumState(TMC5130TypeDef *tmc5130);

/**
 * @brief 在电机记步模式下用电机模型刷新业务位置。
 *
 * @param drum 当前卷筒状态快照。
 */
void MotorPosition_UpdatePositionFromMotorSource(const MotorDrumState *drum);

/**
 * @brief 根据设备参数和 FRAM 记录恢复位置源运行态。
 *
 * 电机记步模式下优先恢复持久化基准；编码轮模式下保留基准用于诊断。
 */
void MotorPosition_RestorePositionSourceFromParams(void);

/**
 * @brief 运动过程中按电机步数间隔自动采样 TFIT。
 *
 * 只有 TFIT 采样使能时才会记录，采样间隔约为 1/4 卷筒圈。
 */
void MotorTapeFit_AutoSample(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CTRL_INTERNAL_H_ */
