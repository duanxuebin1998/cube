/*
 * motor_ctrl.c
 *
 *  Created on: Jan 18, 2025
 *      Author: Duan Xuebin
 *
 * 说明：
 *  - 本文件用于尺带机构的步进电机控制（TMC5130）
 *  - 支持：精确长度换算下发、阻塞等待、碰撞/异常检测、丢步检测、卷筒圈数/角度计算、第一圈周长标定
 *  - 支持：命令切换打断（运行中检测 HasEffectiveCommandSwitchRequest() 时立即停机并返回）
 *
 * 关键口径（必须统一）：
 *  1) 方向：
 *     - MOTOR_DIRECTION_DOWN：下行放带，cable_length 增加；sensor_position 减小（按你现场口径）
 *     - MOTOR_DIRECTION_UP  ：上行收带，cable_length 减少；sensor_position 增大（按你现场口径）
 *  2) 单位：
 *     - cable_length：0.1mm
 *     - sensor_position：0.1mm
 *     - first_loop_circumference_mm：0.1mm
 *     - tape_thickness_mm：0.001mm
 *
 * 3) 尺带模型（以最外层为基准，允许 L<0 表示缠带）：
 *    - C0(mm)=first_loop_circumference_mm*0.1
 *    - t(mm) =tape_thickness_mm*0.001
 *    - 每放/缠 1 圈，半径变化 t（不是 2t）
 *    - 放带（n>=0）：L = C0*n - pi*t*n^2        // 每圈半径减 t
 *    - 缠带（n<0）：L = -(C0*|n| + pi*t*|n|^2)  // 每圈半径增 t
 *
 * 4) minR 截断（仅放带 L>=0 需要）：
 *    - 当放带模型算到半径 < minR 时，不再继续减半径
 *    - 之后按固定半径 minR（固定周长 Cmin=2*pi*minR）线性展开
 *
 *    放带分段：
 *      设 n1 为达到 minR 的圈数阈值，L1 为对应长度
 *      n <= n1：L = C0*n - pi*t*n^2
 *      n >  n1：L = L1 + (n-n1)*Cmin
 *
 *    反解分段：
 *      L <= L1：用二次反解（disc = C0^2 - 4*pi*t*L）
 *      L >  L1：n = n1 + (L-L1)/Cmin
 *
 * 5) 反解公式（用于长度->圈数）：
 *    - 放带（L>=0，且 L<=L1）：
 *        L = C0*n - pi*t*n^2
 *        disc = C0^2 - 4*pi*t*L
 *        n = (C0 - sqrt(disc)) / (2*pi*t)
 *    - 缠带（L<0）：
 *        |L| = C0*m + pi*t*m^2
 *        disc = C0^2 + 4*pi*t*|L|
 *        m = (-C0 + sqrt(disc)) / (2*pi*t)
 *        n = -m
 *
 * 命令切换打断策略：
 *  - 所有“阻塞等待/长循环”处均插入 CHECK_COMMAND_SWITCH... 宏
 *  - 一旦检测到命令切换请求：
 *      1) 打印提示
 *      2) stpr_stop(&stepper) 立即停止电机
 *      3) 返回 COMMAND_SWITCH_ABORT（或 void 直接 return）
 */

#include "motor_ctrl.h"
#include "fault_manager.h"
#include "spi.h"
#include "measure_tank_height.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <inttypes.h>
#include "sensor.h"
#include "encoder.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include <stddef.h>


/* ===================== 常量/配置 ===================== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EPS_MM                   0.20f    // 到位公差(±mm)

#define LOST_STEP_WINDOW                 6       // 6个采样点 -> 覆盖最近5秒
#define LOST_STEP_INTERVAL_MS            1000    // 每1秒采样一次
#define LOST_STEP_SUPPRESS_MS            1000    // 报警抑制1秒

/* 平均速度判定阈值：
 * 最近5秒平均速度 < 设定速度的 20% -> 判定疑似丢步
 */
#define LOST_STEP_SPEED_RATIO_PERCENT    20U

/* 额外绝对下限，避免低速档误判太苛刻（单位：mm/s） */
#define LOST_STEP_MIN_AVG_SPEED_MM_S     0.5f

/* 周长标定结果合理性范围（按机构实际可调整） */
#ifndef C0_MIN_MM
#define C0_MIN_MM (50.0)
#endif
#ifndef C0_MAX_MM
#define C0_MAX_MM (5000.0)
#endif

/* 速度基准：仅作上电默认兜底，真正速度由线速度参数反算 */
#define MOTOR_VELOCITY_BASE     (10000UL)

/* 速度参数单位：0.01m/min（例如 150 表示 1.50m/min） */
#ifndef MOTOR_LINEAR_SPEED_MIN_X100
#define MOTOR_LINEAR_SPEED_MIN_X100   10U    // 最小 0.10 m/min
#endif
#ifndef MOTOR_LINEAR_SPEED_MAX_X100
#define MOTOR_LINEAR_SPEED_MAX_X100   600U   // 最大 6.00 m/min
#endif

/* TMC5130 内部时钟（很关键！若你板上用了外部时钟，请改成实际值） */
#ifndef TMC5130_FCLK_HZ
#define TMC5130_FCLK_HZ               (12000000.0)   // 12 MHz
#endif

/* 尺带线速度补偿：建议先关掉，先把基础速度跑准 */
#ifndef MOTOR_LINEAR_SPEED_COMP_ENABLE
#define MOTOR_LINEAR_SPEED_COMP_ENABLE       1      // 先关，调通后再开
#endif
#ifndef MOTOR_LINEAR_COMP_UPDATE_MS
#define MOTOR_LINEAR_COMP_UPDATE_MS          200U
#endif
#ifndef MOTOR_LINEAR_COMP_APPLY_DELTA_X100
#define MOTOR_LINEAR_COMP_APPLY_DELTA_X100   3U
#endif

/* 最小有效半径（mm）：半径小于该值后，按该半径线性展开 */
#ifndef TAPE_MIN_RADIUS_MM
#define TAPE_MIN_RADIUS_MM   (40.0)   /* 尺带最内圈半径40mm */
#endif

#define MOTOR_MOVE_RETRY_MAX 3       /* 电机移动重试次数上限 */

#ifndef MOTOR_CONDITION_DEFAULT_POLL_MS
#define MOTOR_CONDITION_DEFAULT_POLL_MS      20U
#endif
#ifndef MOTOR_CONDITION_DEFAULT_TIMEOUT_MS
#define MOTOR_CONDITION_DEFAULT_TIMEOUT_MS   (60000U * 60U)
#endif
#ifndef MOTOR_STOP_WAIT_TIMEOUT_MS
#define MOTOR_STOP_WAIT_TIMEOUT_MS          15000U
#endif
/* ===================== 命令切换打断 ===================== */

/* 命令切换导致的中断错误码 */
#ifndef COMMAND_SWITCH_ABORT
#define COMMAND_SWITCH_ABORT   STATE_SWITCH
#endif

/**
 * @brief 非 void 函数：检测命令切换 -> 停止电机 -> 返回 retcode
 */
#define CHECK_COMMAND_SWITCH_AND_STOP(retcode)                                    \
    do {                                                                          \
        if (HasEffectiveCommandSwitchRequest()) {                                 \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            Motor_StopAndMarkStopped();                                                  \
            return (retcode);                                                     \
        }                                                                         \
    } while (0)

/**
 * @brief void 函数：检测命令切换 -> 停止电机 -> return
 */
#define CHECK_COMMAND_SWITCH_AND_STOP_NO_RETURN()                                 \
    do {                                                                          \
        if (HasEffectiveCommandSwitchRequest()) {                                 \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            Motor_StopAndMarkStopped();                                                  \
            return;                                                               \
        }                                                                         \
    } while (0)

/* 前置声明：线速度->VMAX 反算与运行中刷新 */
static inline uint32_t Motor_ComputeUniformVelocityFromLength(double L_mm);
static inline void Motor_RefreshVelocityDuringRun(TMC5130TypeDef *tmc5130,
                                                  uint32_t *last_refresh_tick);
static void Motor_SyncDebugDrumState(TMC5130TypeDef *tmc5130);
static void Motor_UpdatePositionFromMotorSource(const MotorDrumState *drum);
static void Motor_MaybePersistRegisters(TMC5130TypeDef *tmc5130, bool force);
static void Motor_RestorePersistedRegisters(TMC5130TypeDef *tmc5130);
static bool Motor_IsEncoderErrorCode(uint32_t error_code);
static int64_t Motor_PersistDeltaTicksForLength(int32_t xactual);
static int32_t Motor_ReadEncoderLengthForFit(void);
static bool Motor_TryUpdateDrumState_FromXACTUAL(TMC5130TypeDef *tmc5130, MotorDrumState *out);
static void Motor_TapeFitAutoSample(void);
static void Motor_TapeFitLocalRange(double *q_min, double *q_max);
static bool Motor_TryReadMovingState(TMC5130TypeDef *tmc5130, bool *is_moving);
static uint32_t Motor_InferDisplayStateFromDriver(TMC5130TypeDef *tmc5130);
/* ===================== 内部状态变量 ===================== */

/* 丢步检测环形缓冲区：记录最近 3 次位置（单位：0.1mm） */
static int32_t  pos_buf[LOST_STEP_WINDOW];
static uint32_t tick_buf[LOST_STEP_WINDOW];
static int write_idx = 0;
static int samples = 0;
static uint32_t last_check_tick = 0;
static uint32_t last_alarm_tick = 0;

/* 全局速度：由 Motor_UpdateVelocityFromParams() 根据参数动态更新 */
uint32_t velocity = MOTOR_VELOCITY_BASE;  // 先给一个默认值，真正值在 Init/运行中更新
static uint32_t s_motor_applied_velocity = 0;
static bool s_motor_initialized = false;

/* 电机记步基准只保存“切换瞬间”的参考点。
 * 当前记步模式和电机局部首圈周长直接使用 g_deviceParams，避免同一参数在运行态和参数区各存一份。 */
static int32_t s_motor_count_base_step = 0;
static int32_t s_motor_count_base_length_01mm = 0;
static double s_motor_count_base_turns = 0.0;
/* 把电机记步模式的局部首圈周长转成参数存储值，单位 0.001mm。 */
static uint32_t Motor_EncodeLocalCircumferenceParam(double circumference_mm)
{
    int32_t value;

    if ((circumference_mm < C0_MIN_MM) || (circumference_mm > C0_MAX_MM)) {
        return 0U;
    }

    value = (int32_t)(circumference_mm * 1000.0 + 0.5);
    if (value <= 0) {
        return 0U;
    }

    return (uint32_t)value;
}

/* 从参数区读取电机记步局部首圈周长；非法或未配置时返回 0，由调用方回退模型值。 */
static double Motor_DecodeLocalCircumferenceParam(void)
{
    const double circumference_mm =
        (double)g_deviceParams.motor_count_first_loop_circumference_mm * 0.001;

    if ((circumference_mm < C0_MIN_MM) || (circumference_mm > C0_MAX_MM)) {
        return 0.0;
    }

    return circumference_mm;
}
/* 当前电机记步局部周长直接来自 DeviceParameters；非法时由调用方回退到模型计算。 */
static double Motor_GetLocalCircumferenceFromParams(void)
{
    return Motor_DecodeLocalCircumferenceParam();
}

/* 标定得到的新局部周长直接写回 DeviceParameters，单位保持 0.001mm。 */
static bool Motor_SetLocalCircumferenceToParams(double circumference_mm)
{
    uint32_t value = Motor_EncodeLocalCircumferenceParam(circumference_mm);
    if (value == 0U) {
        return false;
    }
    g_deviceParams.motor_count_first_loop_circumference_mm = value;
    return true;
}
/* 上位机/CPU3 写入设备参数后调用。
 * 参数值直接落在 g_deviceParams 中，这里只做合法性修正，并在电机记步模式下刷新当前位置。 */
void motorApplyPositionSourceParamsFromDeviceParams(void)
{
    MotorDrumState drum;
    double local_circumference_mm;

    if (g_deviceParams.position_count_mode > POSITION_COUNT_MODE_MOTOR) {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
    }

    local_circumference_mm = Motor_DecodeLocalCircumferenceParam();
    if (local_circumference_mm <= 0.0) {
        g_deviceParams.motor_count_first_loop_circumference_mm =
            g_deviceParams.first_loop_circumference_mm * 100U;
        local_circumference_mm = Motor_DecodeLocalCircumferenceParam();
    }

    if ((g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
        s_motor_initialized) {
        Motor_UpdateDrumState_FromXACTUAL(&stepper, &drum);
        g_measurement.debug_data.motor_step = drum.motor_step;
        g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;
        Motor_UpdatePositionFromMotorSource(&drum);
    }

    printf("电机局部周长已按参数生效 | 局部周长=%.3fmm\r\n",
           local_circumference_mm);
}

/* 保存 DeviceParameters 中的记步模式和电机局部周长。
 * 这两个值本身就是设备参数，不再从额外运行态结构体回填。 */
static void Motor_SavePositionSourceParams(bool force)
{
    bool changed = false;

    if (g_deviceParams.position_count_mode > POSITION_COUNT_MODE_MOTOR) {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
        changed = true;
    }

    if (Motor_DecodeLocalCircumferenceParam() <= 0.0) {
        g_deviceParams.motor_count_first_loop_circumference_mm =
            g_deviceParams.first_loop_circumference_mm * 100U;
        changed = true;
    }

    if (changed || force) {
        save_device_params();
    }
}

#define MOTOR_STORE_MAGIC            (0x4D4F544Fu) /* 'MOTO' */
#define MOTOR_STORE_VERSION          (1u)
#define FRAM_MOTOR_A_ADDRESS         (FRAM_ANGLE_ADDRESS + 0x80u)
#define FRAM_MOTOR_SLOT_SIZE         (0x40u)
#define FRAM_MOTOR_B_ADDRESS         (FRAM_MOTOR_A_ADDRESS + FRAM_MOTOR_SLOT_SIZE)
#define MOTOR_PERSIST_DELTA_MM       (1.0)
/* 在 XACTUAL 后面增加电机记步源的切换基准。
 * base_length_01mm 是切换时采用的尺带长度，base_step 是切换时的 XACTUAL。
 * 这两个值配合 position_count_mode，才能在断电重启后继续保持“切换时编码轮长度 + 电机步进变化量”的口径。 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t xactual;
    int32_t base_length_01mm;
    int32_t base_step;
    uint32_t crc;
} MotorPersistRecord;
static int32_t s_motor_saved_xactual = 0;
static int32_t s_motor_restored_base_length_01mm = 0;
static int32_t s_motor_restored_base_step = 0;
static bool s_motor_restored_base_valid = false;
/* ===================== 电机侧卷筒拟合（TFIT） ===================== */
/* 样本仅保存在 RAM 中，断电后丢失。 */
#define MOTOR_TAPE_FIT_MAX_SAMPLES          (256)
/* 自动采样间隔：约 1/4 卷筒圈。 */
#define MOTOR_TAPE_FIT_AUTO_DELTA_TICKS     (1536000 / 4)
/* TFIT 采样点。
 *  - motor_step：电机侧 XACTUAL
 *  - encoder_length_01mm：编码器测得的尺带长度，单位 0.1mm
 */
typedef struct {
    int32_t motor_step;
    int32_t encoder_length_01mm;
} MotorTapeFitSample;
/* TFIT 拟合结果。
 * 模型：L = b + C0*n - pi*t*n^2
 *  - offset_mm           -> b，零点偏移
 *  - first_loop_circ_mm  -> C0，首圈周长
 *  - tape_thickness_mm   -> t，尺带厚度
 */
typedef struct {
    bool valid;
    double offset_mm;
    double first_loop_circ_mm;
    double tape_thickness_mm;
    double rmse_mm;
    double max_abs_err_mm;
} MotorTapeFitResult;
static MotorTapeFitSample s_motor_tape_fit_samples[MOTOR_TAPE_FIT_MAX_SAMPLES];
static uint16_t s_motor_tape_fit_count = 0;
static bool s_motor_tape_fit_enabled = false;
static int32_t s_motor_tape_fit_last_step = INT32_MIN;
static MotorTapeFitResult s_motor_tape_fit_result = {0};
static bool s_motor_tape_fit_result_is_local = false;
static bool s_motor_tape_fit_local_origin_valid = false;
static int32_t s_motor_tape_fit_origin_step = 0;
static int32_t s_motor_tape_fit_origin_length_01mm = 0;

/* ===================== 速度控制（由参数决定） ===================== */

static inline uint32_t Motor_ClampSpeedSetpointX100(uint32_t speed_x100)
{
    if (speed_x100 < MOTOR_LINEAR_SPEED_MIN_X100) {
        return MOTOR_LINEAR_SPEED_MIN_X100;
    }
    if (speed_x100 > MOTOR_LINEAR_SPEED_MAX_X100) {
        return MOTOR_LINEAR_SPEED_MAX_X100;
    }
    return speed_x100;
}

static inline uint32_t Motor_ClampCurrentSetting(uint32_t current)
{
    if ((current < MOTOR_CURRENT_MIN) || (current > MOTOR_CURRENT_MAX)) {
        return MOTOR_CURRENT_DEFAULT;
    }
    return current;
}

static inline int32_t Motor_GetSpeedSetpointX100(void)
{
    return (int32_t)Motor_ClampSpeedSetpointX100(g_measurement.debug_data.motor_speed);
}

static inline float Motor_GetSpeedSetpoint_m_min(void)
{
    return (float)Motor_GetSpeedSetpointX100() / 100.0f;
}
static inline double TMC5130_VMAX_To_UstepsPerSec(uint32_t vmax)
{
    return ((double)vmax * TMC5130_FCLK_HZ) / 16777216.0;
}

static inline void Motor_UpdateVelocityFromParams(void)
{
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = Motor_ComputeUniformVelocityFromLength(Lcur_mm);

    printf("速度初始化 | 尺带长度=%.1f mm | VMAX=%lu | 等效微步/s=%.1f\r\n",
           Lcur_mm,
           (unsigned long)velocity,
           TMC5130_VMAX_To_UstepsPerSec(velocity));
}

static inline uint32_t Motor_ClampVelocityU64(uint64_t v)
{
    if (v < 1ULL) {
        return 1U;
    }
    if (v > (uint64_t)TMC5130_MAX_VELOCITY) {
        return (uint32_t)TMC5130_MAX_VELOCITY;
    }
    return (uint32_t)v;
}

static inline uint32_t TMC5130_UstepsPerSec_To_VMAX(double usteps_per_s)
{
    /* TMC5130: usteps/s = VMAX * fCLK / 2^24
       => VMAX = usteps/s * 2^24 / fCLK */
    double vmax = usteps_per_s * 16777216.0 / TMC5130_FCLK_HZ;   /* 2^24 */

    if (vmax < 1.0) {
        vmax = 1.0;
    }

    return Motor_ClampVelocityU64((uint64_t)llround(vmax));
}

/* ===================== 方向统一工具 ===================== */

static inline int Motor_IsDirValid(int dir)
{
    return (dir == MOTOR_DIRECTION_UP) || (dir == MOTOR_DIRECTION_DOWN);
}

static inline void Motor_StopAndMarkStopped(void)
{
    bool is_moving = true;

    stpr_stop(&stepper);
    Motor_SyncDebugDrumState(&stepper);

    /* 停止命令只是开始减速，只有驱动确认 vzero 后才显示静止。 */
    if (Motor_TryReadMovingState(&stepper, &is_moving) && (!is_moving)) {
        g_measurement.debug_data.motor_state = 0U;
    }
}

static inline bool Motor_StopIfCommandSwitchRequested(void)
{
    if (HasEffectiveCommandSwitchRequest()) {
        printf("检测到命令切换请求，停止当前操作\r\n");
        Motor_StopAndMarkStopped();
        return true;
    }
    return false;
}
static bool Motor_TryReadMovingState(TMC5130TypeDef *tmc5130, bool *is_moving)
{
    int32_t rampstat = 0;

    if ((tmc5130 == NULL) || (is_moving == NULL)) {
        return false;
    }

    if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat)) {
        return false;
    }

    /* RAMPSTAT.bit10(vzero)=1 表示速度已经为 0，用它校正显示状态，避免软件缓存滞留。 */
    *is_moving = (((uint32_t)rampstat & 0x400U) != 0x400U);
    return true;
}

static uint32_t Motor_InferDisplayStateFromDriver(TMC5130TypeDef *tmc5130)
{
    int32_t vactual = 0;
    int32_t xactual = 0;
    int32_t xtarget = 0;

    if (tmc5130 == NULL) {
        return 0U;
    }

    /* TMC5130_VACTUAL 为有符号速度；本工程正向 ticks 对应下行放带。 */
    if (stpr_tryReadInt(tmc5130, TMC5130_VACTUAL, &vactual)) {
        if (vactual > 0) {
            return 2U;
        }
        if (vactual < 0) {
            return 1U;
        }
    }

    /* 低速或刚启动时 VACTUAL 可能暂为 0，退回用目标位置差判断方向。 */
    if (stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual) &&
        stpr_tryReadInt(tmc5130, TMC5130_XTARGET, &xtarget)) {
        if (xtarget > xactual) {
            return 2U;
        }
        if (xtarget < xactual) {
            return 1U;
        }
    }

    return 0U;
}

/* ===================== 尺带/卷筒模型工具（每圈半径变化=t） ===================== */

/**
 * @brief 卷筒输出轴每转对应的 ticks（微步数）
 */
static inline int32_t tape_ticks_per_rev(void)
{
    return (int32_t)(30L * 1600L * 32L);  /* 1,536,000 */
}

/**
 * @brief 最外层周长 C0（单位：mm）
 */
static inline double tape_C0_mm(void)
{
    return (double)g_deviceParams.first_loop_circumference_mm * 0.1;
}

/**
 * @brief 尺带厚度 t（单位：mm）
 */
static inline double tape_t_mm(void)
{
    return (double)g_deviceParams.tape_thickness_mm * 0.001;
}

/**
 * @brief 计算放带进入“最小半径 minR”后的分段阈值（新模型）
 */
static inline void tape_minR_threshold(double C0_mm, double t_mm,
                                       double *n1, double *L1, double *Cmin)
{
    const double Cmin_local = 2.0 * M_PI * (double)TAPE_MIN_RADIUS_MM; /* mm */

    if (Cmin) *Cmin = Cmin_local;
    if (n1)   *n1   = 0.0;
    if (L1)   *L1   = 0.0;

    if (C0_mm <= 0.0 || t_mm <= 0.0) {
        return;
    }

    if (C0_mm <= Cmin_local) {
        return;
    }

    /* 达到 minR 时：C(n)=C0-2*pi*t*n = Cmin  =>  n1=(C0-Cmin)/(2*pi*t) */
    const double n1_local = (C0_mm - Cmin_local) / (2.0 * M_PI * t_mm);

    if (n1) {
        *n1 = (n1_local > 0.0) ? n1_local : 0.0;
    }

    if (L1) {
        /* L1 = C0*n1 - pi*t*n1^2 */
        double L1_local = (C0_mm * n1_local) - (1.0 * M_PI * t_mm * n1_local * n1_local);
        if (L1_local < 0.0) L1_local = 0.0;
        *L1 = L1_local;
    }
}

/**
 * @brief 有符号圈数 n -> 有符号长度 L（mm）（新模型）
 */
static inline double tape_signed_length_from_turns(double n,
                                                   double C0_mm,
                                                   double t_mm)
{
    if (C0_mm <= 0.0) return 0.0;

    /* 厚度忽略：线性 */
    if (t_mm <= 0.0) {
        return C0_mm * n;
    }

    if (n >= 0.0) {
        /* 放带：带 minR 截断 */
        double n1, L1, Cmin;
        tape_minR_threshold(C0_mm, t_mm, &n1, &L1, &Cmin);

        if (n <= n1) {
            /* L = C0*n - pi*t*n^2 */
            return (C0_mm * n) - (1.0 * M_PI * t_mm * n * n);
        } else {
            /* 线性展开区间 */
            return L1 + (n - n1) * Cmin;
        }
    } else {
        /* 缠带：|L| = C0*m + pi*t*m^2 */
        const double m = -n; /* m >= 0 */
        return -((C0_mm * m) + (1.0 * M_PI * t_mm * m * m));
    }
}

/**
 * @brief 有符号长度 L（mm） -> 有符号圈数 n（新模型）
 */
static inline double tape_turns_from_signed_length(double L_mm,
                                                   double C0_mm,
                                                   double t_mm)
{
    if (C0_mm <= 0.0) return 0.0;

    /* 厚度忽略：线性 */
    if (t_mm <= 0.0) {
        return L_mm / C0_mm;
    }

    if (L_mm >= 0.0) {
        /* 放带：带 minR 截断 */
        double n1, L1, Cmin;
        tape_minR_threshold(C0_mm, t_mm, &n1, &L1, &Cmin);

        if (L_mm <= L1) {
            /* L = C0*n - pi*t*n^2
             * => pi*t*n^2 - C0*n + L = 0
             * disc = C0^2 - 4*pi*t*L
             * n = (C0 - sqrt(disc)) / (2*pi*t)
             */
            const double disc = (C0_mm * C0_mm) - (4.0 * M_PI * t_mm * L_mm);

            if (disc <= 0.0) {
                return n1;
            }

            return (C0_mm - sqrt(disc)) / (2.0 * M_PI * t_mm);
        } else {
            /* 线性展开区间 */
            return n1 + (L_mm - L1) / Cmin;
        }
    } else {
        /* 缠带：|L| = C0*m + pi*t*m^2
         * => pi*t*m^2 + C0*m - |L| = 0
         * disc = C0^2 + 4*pi*t*|L|
         * m = (-C0 + sqrt(disc)) / (2*pi*t)
         * n = -m
         */
        const double absL = -L_mm;
        const double disc = (C0_mm * C0_mm) + (4.0 * M_PI * t_mm * absL);

        if (disc <= 0.0) return 0.0;

        const double m = (-C0_mm + sqrt(disc)) / (2.0 * M_PI * t_mm);
        return -m;
    }
}

/**
 * @brief 根据当前有符号长度 L 计算当前层周长 C(L)（单位：mm）
 */
static inline double tape_instant_circumference_mm_from_length(double L_mm)
{
    const double C0 = tape_C0_mm();
    const double t  = tape_t_mm();

    if (C0 <= 0.0) {
        return 0.0;
    }
    if (t <= 0.0) {
        return C0;
    }

    const double n = tape_turns_from_signed_length(L_mm, C0, t);

    if (n >= 0.0) {
        double n1, L1, Cmin;
        tape_minR_threshold(C0, t, &n1, &L1, &Cmin);
        if (n <= n1) {
            const double C = C0 - (2.0 * M_PI * t * n);
            return (C > 1e-6) ? C : 1e-6;
        }
        return (Cmin > 1e-6) ? Cmin : 1e-6;
    }

    /* 上行收带：每圈周长增加 */
    const double C = C0 + (2.0 * M_PI * t * (-n));
    return (C > 1e-6) ? C : 1e-6;
}

/**
 * @brief 基于首圈周长做兜底反算（当当前周长不可用时）
 *        返回值：TMC5130 VMAX寄存器值
 */
static inline uint32_t Motor_ComputeBaseVelocityFromParams(void)
{
    const int32_t speed_x100 = Motor_GetSpeedSetpointX100();
    const double C0_mm = tape_C0_mm();
    const double C_ref_mm = (C0_mm > 1e-6) ? C0_mm : (2.0 * M_PI * (double)TAPE_MIN_RADIUS_MM);
    const double ticks_per_rev = (double)tape_ticks_per_rev();

    /* 先算目标输出轴微步速度：usteps/s */
    const double usteps_per_s = ((double)speed_x100 * ticks_per_rev) / (6.0 * C_ref_mm);

    /* 再换成 TMC5130 VMAX */
    return TMC5130_UstepsPerSec_To_VMAX(usteps_per_s);
}

/**
 * @brief 按当前层周长直接反算 VMAX（目标线速度恒定）
 *        speed_x100 单位：0.01m/min
 *        返回值：TMC5130 VMAX寄存器值
 */
static inline uint32_t Motor_ComputeUniformVelocityFromLength(double L_mm)
{
    const int32_t speed_x100 = Motor_GetSpeedSetpointX100();
    double C_cur = tape_instant_circumference_mm_from_length(L_mm);
    double local_circumference_mm = Motor_GetLocalCircumferenceFromParams();

    if ((g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
        (local_circumference_mm > 1e-6)) {
        C_cur = local_circumference_mm;
    }

    if (C_cur > 1e-6) {
        const double ticks_per_rev = (double)tape_ticks_per_rev();

        /* 目标输出轴微步速度：usteps/s
           线速度(mm/s) = speed_x100 / 6
           rev/s       = (speed_x100 / 6) / C_cur
           usteps/s    = rev/s * ticks_per_rev
                       = speed_x100 * ticks_per_rev / (6*C_cur) */
        const double usteps_per_s =
            ((double)speed_x100 * ticks_per_rev) / (6.0 * C_cur);

        return TMC5130_UstepsPerSec_To_VMAX(usteps_per_s);
    }

    return Motor_ComputeBaseVelocityFromParams();
}
static inline void Motor_RefreshVelocityDuringRun(TMC5130TypeDef *tmc5130,
                                                  uint32_t *last_refresh_tick)
{
#if MOTOR_LINEAR_SPEED_COMP_ENABLE
    if ((tmc5130 == NULL) || (last_refresh_tick == NULL)) {
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    if ((now_tick - *last_refresh_tick) < MOTOR_LINEAR_COMP_UPDATE_MS) {
        return;
    }

    /* 优先用 XACTUAL 推当前长度，避免依赖外部变量刷新滞后 */
    MotorDrumState drum;
    Motor_UpdateDrumState_FromXACTUAL(tmc5130, &drum);
    Motor_UpdatePositionFromMotorSource(&drum);
    const double Lcur_mm = (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) ?
                           ((double)g_measurement.debug_data.cable_length * 0.1) :
                           ((double)drum.motor_distance_01mm * 0.1);

    const uint32_t new_v = Motor_ComputeUniformVelocityFromLength(Lcur_mm);
    const uint32_t old_v = (s_motor_applied_velocity != 0U) ? s_motor_applied_velocity : velocity;

    uint32_t delta = (new_v >= old_v) ? (new_v - old_v) : (old_v - new_v);
    uint32_t threshold = (old_v * MOTOR_LINEAR_COMP_APPLY_DELTA_X100 + 99U) / 100U;

    if (threshold == 0U) {
        threshold = 1U;
    }

    if (delta >= threshold) {
        stpr_setVelocity(tmc5130, new_v);
        s_motor_applied_velocity = new_v;
        velocity = new_v;

        printf("速度刷新 | 尺带长度=%.1f mm | 旧VMAX=%lu | 新VMAX=%lu | 旧微步/s=%.1f | 新微步/s=%.1f\r\n",
               Lcur_mm,
               (unsigned long)old_v,
               (unsigned long)new_v,
               TMC5130_VMAX_To_UstepsPerSec(old_v),
               TMC5130_VMAX_To_UstepsPerSec(new_v));
    }

    *last_refresh_tick = now_tick;
#else
    (void)tmc5130;
    (void)last_refresh_tick;
#endif
}

static inline uint32_t Motor_ApplyOptionalSpeed(uint32_t speed_x100)
{
    /* 约定：
     * speed_x100 == 0 表示“调用方不想改速度，只沿用当前设定值”。
     * 这样老接口和新接口可以共用一套实现，而不会强制改写速度状态。 */
    if (speed_x100 == 0U) {
        return NO_ERROR;
    }
    return motorSetSpeed(speed_x100);
}

static inline uint32_t Motor_GetDefaultSpeedSetpointX100(void)
{
    /* “默认恢复速度”统一取设备参数中的最大速度。
     * 后续如果要把“恢复默认速度”改成别的策略，只需要改这里。 */
    return Motor_ClampSpeedSetpointX100(g_deviceParams.max_motor_speed);
}

uint32_t motorGetDefaultSpeedX100(void)
{
    return Motor_GetDefaultSpeedSetpointX100();
}

static inline uint32_t Motor_BeginTemporarySpeed(uint32_t speed_x100,
                                                 bool *restore_needed,
                                                 uint32_t *restore_speed_x100)
{
    /* 这组 Begin/End helper 专门服务“阻塞型命令的临时速度”：
     * - Begin: 命令开始前切到临时速度，并记录结束后要恢复的默认速度
     * - End  : 命令结束时恢复默认速度
     *
     * 注意：
     * speed_x100 == 0 时，不做速度切换，也就不需要恢复。 */
    if (restore_needed) {
        *restore_needed = false;
    }
    if (restore_speed_x100) {
        *restore_speed_x100 = Motor_GetDefaultSpeedSetpointX100();
    }

    if (speed_x100 == 0U) {
        return NO_ERROR;
    }

    if (restore_needed) {
        *restore_needed = true;
    }

    return motorSetSpeed(speed_x100);
}

static inline uint32_t Motor_EndTemporarySpeed(bool restore_needed,
                                               uint32_t restore_speed_x100)
{
    /* 只有 Begin 确认本次命令确实切换过速度，这里才执行恢复。 */
    if (!restore_needed) {
        return NO_ERROR;
    }
    return motorSetSpeed(restore_speed_x100);
}

uint32_t motorSetSpeed(uint32_t speed_x100)
{
    const uint32_t clamped_speed = Motor_ClampSpeedSetpointX100(speed_x100);
    bool is_running = false;
    double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;

    /* 先更新“设定速度”本身。
     * 后续所有速度换算都以 debug_data.motor_speed 作为源头。 */
    g_measurement.debug_data.motor_speed = clamped_speed;

    if (s_motor_initialized) {
        is_running = stpr_isMoving(&stepper);
    }

    if (is_running) {
        /* 电机正在跑时，优先用 XACTUAL 推当前卷筒长度。
         * 这样可以避免外部 cable_length 刷新滞后导致本次改速不准。 */
        MotorDrumState drum;
        Motor_UpdateDrumState_FromXACTUAL(&stepper, &drum);
        Motor_UpdatePositionFromMotorSource(&drum);
        Lcur_mm = (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) ?
                  ((double)g_measurement.debug_data.cable_length * 0.1) :
                  ((double)drum.motor_distance_01mm * 0.1);
    }

    /* 把“线速度设定”转换成当前卷径下的 TMC5130 VMAX。 */
    velocity = Motor_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_applied_velocity = velocity;

    if (is_running) {
        /* 运行中改速：立即写入驱动，让本次运动立刻生效。 */
        stpr_setVelocity(&stepper, velocity);
//        printf("SpeedSet | running | req=%.2f m/min | set=%.2f m/min | L=%.1f mm | VMAX=%lu | eq_usteps/s=%.1f\r\n",
//               requested_speed / 100.0,
//               clamped_speed / 100.0,
//               Lcur_mm,
//               (unsigned long)velocity,
//               TMC5130_VMAX_To_UstepsPerSec(velocity));
    } else {
        /* 未运行时只更新内部速度状态，供下一次运动使用。 */
//        printf("SpeedSet | idle | req=%.2f m/min | set=%.2f m/min | baseL=%.1f mm | nextVMAX=%lu | eq_usteps/s=%.1f\r\n",
//               requested_speed / 100.0,
//               clamped_speed / 100.0,
//               Lcur_mm,
//               (unsigned long)velocity,
//               TMC5130_VMAX_To_UstepsPerSec(velocity));
    }
    if (is_running) {
        uint32_t inferred_state = Motor_InferDisplayStateFromDriver(&stepper);
        if (((g_measurement.debug_data.motor_state != 1U) &&
             (g_measurement.debug_data.motor_state != 2U)) &&
            ((inferred_state == 1U) || (inferred_state == 2U))) {
            g_measurement.debug_data.motor_state = inferred_state;
        }
    } else {
        g_measurement.debug_data.motor_state = 0U;
    }
    return NO_ERROR;
}

uint32_t motorSetCurrent(uint32_t current)
{
    const uint32_t clamped_current = Motor_ClampCurrentSetting(current);

    g_deviceParams.motor_current = clamped_current;

    if (s_motor_initialized) {
        /* 电流参数写入后立即更新 TMC5130，避免必须重启才生效。 */
        stpr_setCurrent(&stepper, (uint8_t)clamped_current);
    }

    return NO_ERROR;
}

/**
 * @brief 四舍五入到 int64，避免 double->int 直接截断导致系统误差
 */
static inline int64_t llround_safe(double x)
{
    return (int64_t)llround(x);
}

/**
 * @brief 读取当前放带长度（单位：mm）
 */
static inline double get_current_tape_length_mm(void)
{
    return (double)g_measurement.debug_data.cable_length * 0.1;
}

static bool Motor_IsEncoderErrorCode(uint32_t error_code)
{
    return (error_code >= ENCODER_TIMEOUT) && (error_code <= ENCODER_OCF_INCOMPLETE);
}

bool motorIsPositionSourceMotor(void)
{
    return g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR;
}

static int64_t Motor_PersistDeltaTicksForLength(int32_t xactual)
{
    const double turns = (double)xactual / (double)tape_ticks_per_rev();
    const double length_mm = tape_signed_length_from_turns(turns, tape_C0_mm(), tape_t_mm());
    const double circumference_mm = tape_instant_circumference_mm_from_length(length_mm);
    double ticks;

    if (circumference_mm <= 1e-6) {
        return 1;
    }

    ticks = ((double)tape_ticks_per_rev() * MOTOR_PERSIST_DELTA_MM) / circumference_mm;
    if (ticks < 1.0) {
        return 1;
    }
    if (ticks > (double)INT32_MAX) {
        return (int64_t)INT32_MAX;
    }
    return (int64_t)llround(ticks);
}

static int32_t Motor_ReadEncoderLengthForFit(void)
{
    const double encoder_value = -(double)g_encoder_count;
    const double revolutions = encoder_value / 4096.0;
    const double length_01mm = ((double)g_deviceParams.encoder_wheel_circumference_mm * revolutions) / 100.0;

    if (length_01mm > (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (length_01mm < (double)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)llround(length_01mm);
}

static void Motor_BuildDrumStateFromStep(int32_t motor_step, MotorDrumState *out)
{
    double turns;
    int32_t turns_int;
    double frac;
    double angle_deg;
    double C0;
    double t;
    double L_mm;

    if (out == NULL) {
        return;
    }

    /* ticks -> 输出轴圈数（含小数） */
    turns = (double)motor_step / (double)tape_ticks_per_rev();

    /* 圈内角度归一到 [0,1)，负向位置也能得到正常角度。 */
    turns_int = (int32_t)floor(turns);
    frac = turns - (double)turns_int;
    if (frac < 0.0) frac += 1.0;
    if (frac >= 1.0) frac -= floor(frac);
    angle_deg = frac * 360.0;

    /* 由电机卷筒模型预测长度，单位 mm。 */
    C0 = tape_C0_mm();
    t = tape_t_mm();
    if (t > 0.0) {
        L_mm = tape_signed_length_from_turns(turns, C0, t);
    } else {
        L_mm = C0 * turns;
    }

    out->motor_step          = motor_step;
    out->turns_total         = turns;
    out->turns_int           = turns_int;
    out->angle_deg           = angle_deg;
    out->motor_distance_01mm = (int32_t)llround(L_mm * 10.0); // mm -> 0.1mm
}

/* 带错误状态的 XACTUAL 读取。SPI 读失败时返回 false，调用方不得覆盖旧位置。 */
static bool Motor_TryUpdateDrumState_FromXACTUAL(TMC5130TypeDef *tmc5130,
                                                 MotorDrumState *out)
{
    int32_t motor_step;

    if ((out == NULL) || (tmc5130 == NULL)) {
        return false;
    }
    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &motor_step)) {
        return false;
    }
    if ((motor_step == 0) &&
        (g_measurement.debug_data.motor_step != 0)) {
        int32_t chopconf = 0;

        /* MISO 悬空或 SPI 帧错位时，HAL 可能仍返回 OK 但数据全 0。
         * CHOPCONF 初始化后不应为 0；若 XACTUAL 和 CHOPCONF 同时读 0，
         * 按无效读数处理，避免把非零位置误覆盖/误持久化为 0。 */
        if ((!stpr_tryReadInt(tmc5130, TMC5130_CHOPCONF, &chopconf)) ||
            (chopconf == 0)) {
            return false;
        }
    }

    Motor_BuildDrumStateFromStep(motor_step, out);
    return true;
}

/**
 * @brief 从 TMC5130_XACTUAL 计算卷筒总圈数、圈内角度、预测长度（新模型）
 */
void Motor_UpdateDrumState_FromXACTUAL(TMC5130TypeDef *tmc5130,
                                      MotorDrumState *out)
{
    if (!out) return;

    if (!Motor_TryUpdateDrumState_FromXACTUAL(tmc5130, out)) {
        /* 兼容旧接口：读取失败时用上一次缓存值填充，但不代表本次读到了真实 0。 */
        Motor_BuildDrumStateFromStep(g_measurement.debug_data.motor_step, out);
        out->motor_distance_01mm = g_measurement.debug_data.motor_distance;
    }
}

/* 计算并校验，用于确认电机位置记录有效。 */
static uint32_t MotorPersistRecordCRC(const MotorPersistRecord *record)
{
    const uint8_t *base = (const uint8_t *)&record->version;
    const uint32_t len = (uint32_t)(offsetof(MotorPersistRecord, crc) - offsetof(MotorPersistRecord, version));
    return CRC32_HAL(base, len);
}
/* 读取 FRAM 槽位，校验电机位置记录。 */
static int Motor_ReadPersistFromSlot(uint32_t base_addr,
                                     int32_t *xactual,
                                     int32_t *base_length_01mm,
                                     int32_t *base_step,
                                     const char *slot_name)
{
    MotorPersistRecord rec;
    ReadMultiData((uint8_t *)&rec, (int)base_addr, sizeof(rec));
    if (rec.magic != MOTOR_STORE_MAGIC) {
        printf("电机持久化[%s]魔术字错误: 0x%08lX\r\n", slot_name, (unsigned long)rec.magic);
        return 0;
    }

    if (rec.version == MOTOR_STORE_VERSION) {
        if (MotorPersistRecordCRC(&rec) != rec.crc) {
            printf("电机持久化[%s] CRC校验失败\r\n", slot_name);
            return 0;
        }
        *xactual = rec.xactual;
        *base_length_01mm = rec.base_length_01mm;
        *base_step = rec.base_step;
        return 1;
    }
    printf("电机持久化[%s]版本错误: %lu\r\n", slot_name, (unsigned long)rec.version);
    return 0;
}
/* 同步写入 FRAM A/B 双备份，提升断电恢复可靠性。 */
static void Motor_WritePersistAB(int32_t xactual,
                                  int32_t base_length_01mm,
                                  int32_t base_step)
{
    MotorPersistRecord rec;
    rec.magic = MOTOR_STORE_MAGIC;
    rec.version = MOTOR_STORE_VERSION;
    rec.xactual = xactual;
    rec.base_length_01mm = base_length_01mm;
    rec.base_step = base_step;
    rec.crc = MotorPersistRecordCRC(&rec);
    WriteMultiData((const uint8_t *)&rec, (int)FRAM_MOTOR_A_ADDRESS, sizeof(rec));
    WriteMultiData((const uint8_t *)&rec, (int)FRAM_MOTOR_B_ADDRESS, sizeof(rec));
}
/* 优先读取 A 槽，失败时使用 B 槽兜底并修复备份。 */
static int Motor_ReadPersistAB(int32_t *xactual,
                               int32_t *base_length_01mm,
                               int32_t *base_step)
{
    if (Motor_ReadPersistFromSlot(FRAM_MOTOR_A_ADDRESS,
                                  xactual,
                                  base_length_01mm,
                                  base_step,
                                  "A")) {
        return 1;
    }
    if (Motor_ReadPersistFromSlot(FRAM_MOTOR_B_ADDRESS,
                                  xactual,
                                  base_length_01mm,
                                  base_step,
                                  "B")) {
        printf("电机持久化: A槽无效，回退到B槽\r\n");
        Motor_WritePersistAB(*xactual, *base_length_01mm, *base_step);
        return 1;
    }
    return 0;
}
/* 更新已保存快照，用于控制后续 FRAM 写入频率。 */
static void Motor_StorePersistSnapshot(int32_t xactual)
{
    Motor_WritePersistAB(xactual,
                         s_motor_count_base_length_01mm,
                         s_motor_count_base_step);
    s_motor_saved_xactual = xactual;
}
/* 条件保存 XACTUAL，强制模式立即写入，普通模式按 1mm 位移限频。 */
static void Motor_MaybePersistRegisters(TMC5130TypeDef *tmc5130, bool force)
{
    MotorDrumState drum;
    int64_t delta_ticks;
    int64_t delta_threshold_ticks;

    if ((tmc5130 == NULL) || (!s_motor_initialized)) {
        return;
    }

    /* 持久化必须使用和实时刷新相同的防误读路径。
     * 如果 SPI 全 0 或读失败，这里直接跳过，避免把 FRAM 中的非零 XACTUAL 覆盖成 0。 */
    if (!Motor_TryUpdateDrumState_FromXACTUAL(tmc5130, &drum)) {
        return;
    }

    delta_ticks = (int64_t)drum.motor_step - (int64_t)s_motor_saved_xactual;
    delta_threshold_ticks = Motor_PersistDeltaTicksForLength(drum.motor_step);
    if ((!force) && (llabs(delta_ticks) < delta_threshold_ticks)) {
        return;
    }
    Motor_StorePersistSnapshot(drum.motor_step);
}
/* 上电恢复寄存器前先进入 HOLD，并让 XTARGET 跟随 XACTUAL，避免旧目标导致自动运行。 */
static void Motor_RestorePersistedRegisters(TMC5130TypeDef *tmc5130)
{
    int32_t xactual = 0;
    int32_t base_length_01mm = 0;
    int32_t base_step = 0;
    if ((tmc5130 == NULL) || (!s_motor_initialized)) {
        return;
    }
    if (Motor_ReadPersistAB(&xactual, &base_length_01mm, &base_step)) {
        stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_HOLD);
        stpr_writeInt(tmc5130, TMC5130_XACTUAL, xactual);
        stpr_writeInt(tmc5130, TMC5130_XTARGET, xactual);
        s_motor_restored_base_length_01mm = base_length_01mm;
        s_motor_restored_base_step = base_step;
        s_motor_restored_base_valid = true;
        Motor_WritePersistAB(xactual, base_length_01mm, base_step);
        s_motor_saved_xactual = xactual;
        printf("电机持久化已恢复: XACTUAL=%ld, 基准长度=%.1fmm, 基准步数=%ld\r\n",
               (long)xactual,
               (double)base_length_01mm * 0.1,
               (long)base_step);
        return;
    }
    s_motor_restored_base_length_01mm = 0;
    s_motor_restored_base_step = 0;
    s_motor_restored_base_valid = false;
    Motor_StorePersistSnapshot(0);
    printf("电机持久化: A/B槽均无效，复位为零\r\n");
}
/* 重置 TFIT 状态；clear_samples=1 时同时清空已采集样本。 */
static void Motor_TapeFitResetState(bool clear_samples)
{
    s_motor_tape_fit_enabled = false;
    s_motor_tape_fit_last_step = INT32_MIN;
    s_motor_tape_fit_result.valid = false;
    s_motor_tape_fit_result.offset_mm = 0.0;
    s_motor_tape_fit_result.first_loop_circ_mm = 0.0;
    s_motor_tape_fit_result.tape_thickness_mm = 0.0;
    s_motor_tape_fit_result.rmse_mm = 0.0;
    s_motor_tape_fit_result.max_abs_err_mm = 0.0;
    s_motor_tape_fit_result_is_local = false;
    if (clear_samples) {
        s_motor_tape_fit_count = 0;
        s_motor_tape_fit_local_origin_valid = false;
        s_motor_tape_fit_origin_step = 0;
        s_motor_tape_fit_origin_length_01mm = 0;
    }
}
/* 保存一个 TFIT 样本。
 * force=0 时按最小 tick 间隔过滤自动样本。
 * force=1 时强制保存当前样本。 */
static int Motor_TapeFitStoreSample(int32_t motor_step,
                                    int32_t encoder_length_01mm,
                                    bool force)
{
    int64_t delta_ticks;
    if (s_motor_tape_fit_count > 0U) {
        delta_ticks = (int64_t)motor_step - (int64_t)s_motor_tape_fit_last_step;
        if ((!force) && (llabs(delta_ticks) < (int64_t)MOTOR_TAPE_FIT_AUTO_DELTA_TICKS)) {
            return 0;
        }
    }
    if (s_motor_tape_fit_count >= MOTOR_TAPE_FIT_MAX_SAMPLES) {
        if (s_motor_tape_fit_enabled) {
            s_motor_tape_fit_enabled = false;
            printf("TFIT采样缓冲已满，自动停止采样\r\n");
        }
        return -1;
    }
    s_motor_tape_fit_samples[s_motor_tape_fit_count].motor_step = motor_step;
    s_motor_tape_fit_samples[s_motor_tape_fit_count].encoder_length_01mm = encoder_length_01mm;
    s_motor_tape_fit_count++;
    s_motor_tape_fit_last_step = motor_step;
    s_motor_tape_fit_result.valid = false;
    s_motor_tape_fit_result_is_local = false;
    return 1;
}
/* 采集当前 TFIT 样本。
 * XACTUAL 作为电机侧位置，编码器尺带长度作为实测值。 */
static int Motor_TapeFitCaptureCurrentSample(bool force)
{
    MotorDrumState drum;
    int32_t encoder_length_01mm;
    int ret;
    if (!s_motor_initialized) {
        printf("TFIT不可用: 电机未初始化\r\n");
        return -1;
    }
    Motor_UpdateDrumState_FromXACTUAL(&stepper, &drum);
    encoder_length_01mm = Motor_ReadEncoderLengthForFit();
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;
    ret = Motor_TapeFitStoreSample(drum.motor_step,
                                   encoder_length_01mm,
                                   force);
    if (ret > 0) {
        printf("TFIT采样[%u] 步数=%ld, 编码轮=%.1f mm\r\n",
               (unsigned)s_motor_tape_fit_count,
               (long)drum.motor_step,
               encoder_length_01mm / 10.0);
    }
    return ret;
}
/* 停止 TFIT 自动采样，保留已采集样本。 */
static void Motor_TapeFitAutoSample(void)
{
    if (!s_motor_tape_fit_enabled) {
        return;
    }
    (void)Motor_TapeFitCaptureCurrentSample(false);
}
/* 求解 3x3 线性方程组。
 * 矩阵奇异或求解失败时返回 0。 */
static int Motor_TapeFitSolveLinear3x3(double m[3][4], double out[3])
{
    int row;
    int col;
    for (col = 0; col < 3; col++) {
        int pivot = col;
        double max_abs = fabs(m[col][col]);
        for (row = col + 1; row < 3; row++) {
            const double cur_abs = fabs(m[row][col]);
            if (cur_abs > max_abs) {
                max_abs = cur_abs;
                pivot = row;
            }
        }
        if (max_abs < 1e-12) {
            return 0;
        }
        if (pivot != col) {
            int k;
            for (k = col; k < 4; k++) {
                const double tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
            }
        }
        for (row = col + 1; row < 3; row++) {
            int k;
            const double factor = m[row][col] / m[col][col];
            for (k = col; k < 4; k++) {
                m[row][k] -= factor * m[col][k];
            }
        }
    }
    for (row = 2; row >= 0; row--) {
        double sum = m[row][3];
        for (col = row + 1; col < 3; col++) {
            sum -= m[row][col] * out[col];
        }
        if (fabs(m[row][row]) < 1e-12) {
            return 0;
        }
        out[row] = sum / m[row][row];
    }
    return 1;
}
/* 计算当前样本覆盖的圈数范围。
 * 用于拒绝跨度不足的样本集。 */
static void Motor_TapeFitRange(double *n_min, double *n_max)
{
    uint16_t i;
    if (n_min) {
        *n_min = 0.0;
    }
    if (n_max) {
        *n_max = 0.0;
    }
    if (s_motor_tape_fit_count == 0U) {
        return;
    }
    if (n_min) {
        *n_min = (double)s_motor_tape_fit_samples[0].motor_step / (double)tape_ticks_per_rev();
    }
    if (n_max) {
        *n_max = (double)s_motor_tape_fit_samples[0].motor_step / (double)tape_ticks_per_rev();
    }
    for (i = 1; i < s_motor_tape_fit_count; i++) {
        const double n = (double)s_motor_tape_fit_samples[i].motor_step / (double)tape_ticks_per_rev();
        if ((n_min != NULL) && (n < *n_min)) {
            *n_min = n;
        }
        if ((n_max != NULL) && (n > *n_max)) {
            *n_max = n;
        }
    }
}

/* 同步 XACTUAL 推算值，并处理 TFIT 采样和持久化。 */
/* Calculate sample span in relative turns with the current local origin as turn zero. */
static void Motor_TapeFitLocalRange(double *q_min, double *q_max)
{
    uint16_t i;

    if (q_min) {
        *q_min = 0.0;
    }
    if (q_max) {
        *q_max = 0.0;
    }
    if ((!s_motor_tape_fit_local_origin_valid) ||
        (s_motor_tape_fit_count == 0U)) {
        return;
    }

    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double q = ((double)s_motor_tape_fit_samples[i].motor_step -
                          (double)s_motor_tape_fit_origin_step) /
                         (double)tape_ticks_per_rev();
        if (i == 0U) {
            if (q_min) {
                *q_min = q;
            }
            if (q_max) {
                *q_max = q;
            }
        } else {
            if ((q_min != NULL) && (q < *q_min)) {
                *q_min = q;
            }
            if ((q_max != NULL) && (q > *q_max)) {
                *q_max = q;
            }
        }
    }
}

static void Motor_SyncDebugDrumState(TMC5130TypeDef *tmc5130)
{
    MotorDrumState drum;

    if ((tmc5130 == NULL) || (!s_motor_initialized)) {
        return;
    }

    if (!Motor_TryUpdateDrumState_FromXACTUAL(tmc5130, &drum)) {
        return;
    }
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;
    Motor_UpdatePositionFromMotorSource(&drum);
    /* 同步电机位置时顺带尝试 TFIT 自动采样；内部会判断是否启用和步进间隔。 */
    Motor_TapeFitAutoSample();
    Motor_MaybePersistRegisters(tmc5130, false);
}

static void Motor_UpdatePositionFromMotorSource(const MotorDrumState *drum)
{
    int64_t delta_step;
    double delta_turns;
    double turns;
    double length_mm;
    int32_t length_01mm;
    double local_circumference_mm;

    if ((drum == NULL) ||
        (g_deviceParams.position_count_mode != POSITION_COUNT_MODE_MOTOR)) {
        return;
    }

    delta_step = (int64_t)drum->motor_step -
                 (int64_t)s_motor_count_base_step;
    delta_turns = (double)delta_step / (double)tape_ticks_per_rev();
    local_circumference_mm = Motor_GetLocalCircumferenceFromParams();

    if (local_circumference_mm > 1e-6) {
        length_mm = ((double)s_motor_count_base_length_01mm * 0.1) +
                    (delta_turns * local_circumference_mm);
    } else {
        double base_model_mm;
        double current_model_mm;

        turns = s_motor_count_base_turns + delta_turns;
        base_model_mm = tape_signed_length_from_turns(s_motor_count_base_turns,
                                                      tape_C0_mm(),
                                                      tape_t_mm());
        current_model_mm = tape_signed_length_from_turns(turns,
                                                         tape_C0_mm(),
                                                         tape_t_mm());
        length_mm = ((double)s_motor_count_base_length_01mm * 0.1) +
                    (current_model_mm - base_model_mm);
    }
    length_01mm = (int32_t)llround(length_mm * 10.0);

    g_measurement.debug_data.cable_length = length_01mm;
    g_measurement.debug_data.sensor_position =
        (int32_t)g_deviceParams.tankHeight - length_01mm;
}

/* 对外刷新电机调试状态，避免返回旧缓存。 */
void motorRefreshDebugDrumState(void)
{
    Motor_SyncDebugDrumState(&stepper);
}


void motorPrintPositionRefs(void)
{
    double motor_cable_mm;
    double encoder_cable_mm;

    motorRefreshDebugDrumState();
    /* 打印口径按当前记步源切换：
     * - 电机记步：打印业务实际使用的电机源长度，即“切换时编码轮尺带长度 + XACTUAL 相对变化量”。
     * - 编码轮记步：打印纯 XACTUAL 按全局卷筒模型推算的长度，用于和编码轮尺带对比。 */
    if (motorIsPositionSourceMotor()) {
        motor_cable_mm = (double)g_measurement.debug_data.cable_length / 10.0;
    } else {
        motor_cable_mm = (double)g_measurement.debug_data.motor_distance / 10.0;
    }
    encoder_cable_mm = (double)encoder_get_cable_length_01mm() / 10.0;

    printf("\t{编码模式}%s\t{电机尺带}%.1f\t{编码轮尺带}%.1f",
           motorIsPositionSourceMotor() ? "电机记步" : "编码轮记步",
           motor_cable_mm,
           encoder_cable_mm);
}


void motorPrintPositionCompare(void)
{
    MotorDrumState drum;
    int32_t xactual;
    int32_t motor_source_length_01mm;
    int32_t motor_source_position_01mm;
    int32_t encoder_length_01mm;
    int32_t encoder_position_01mm;
    int32_t length_diff_01mm;
    int32_t position_diff_01mm;
    double local_circumference_mm;
    double delta_turns;
    bool xactual_ok;

    local_circumference_mm = Motor_GetLocalCircumferenceFromParams();
    encoder_length_01mm = encoder_get_cable_length_01mm();
    encoder_position_01mm = encoder_get_sensor_position_01mm();
    xactual_ok = Motor_TryUpdateDrumState_FromXACTUAL(&stepper, &drum);

    printf("电机/编码位置对比 | 模式=%lu(%s) | 局部周长原始值=%lu(0.001mm) | 局部周长=%.3fmm\r\n",
           (unsigned long)g_deviceParams.position_count_mode,
           motorIsPositionSourceMotor() ? "电机记步" : "编码轮记步",
           (unsigned long)g_deviceParams.motor_count_first_loop_circumference_mm,
           local_circumference_mm);
    printf("电机/编码基准 | 基准步数=%ld | 基准长度=%.1fmm | 基准圈数=%.6f\r\n",
           (long)s_motor_count_base_step,
           (double)s_motor_count_base_length_01mm * 0.1,
           s_motor_count_base_turns);

    if (!xactual_ok) {
        printf("电机/编码位置对比 | XACTUAL读取失败，保留旧电机位置缓存 | 编码轮长度=%.1fmm | 编码轮位置=%.1fmm\r\n",
               (double)encoder_length_01mm * 0.1,
               (double)encoder_position_01mm * 0.1);
        return;
    }

    xactual = drum.motor_step;
    delta_turns = (double)((int64_t)xactual - (int64_t)s_motor_count_base_step) /
                  (double)tape_ticks_per_rev();

    /* 电机源位置按“切换时编码轮尺带长度 + XACTUAL 相对变化量”计算。
     * 这样即使电机模型长度和编码轮长度存在偏差，切换瞬间也不会跳变。 */
    if (local_circumference_mm > 1e-6) {
        const double motor_length_mm = ((double)s_motor_count_base_length_01mm * 0.1) +
                                       (delta_turns * local_circumference_mm);
        motor_source_length_01mm = (int32_t)llround(motor_length_mm * 10.0);
    } else {
        double base_model_mm;
        double current_model_mm;
        const double turns = s_motor_count_base_turns + delta_turns;

        base_model_mm = tape_signed_length_from_turns(s_motor_count_base_turns,
                                                      tape_C0_mm(),
                                                      tape_t_mm());
        current_model_mm = tape_signed_length_from_turns(turns,
                                                         tape_C0_mm(),
                                                         tape_t_mm());
        motor_source_length_01mm = s_motor_count_base_length_01mm +
            (int32_t)llround((current_model_mm - base_model_mm) * 10.0);
    }
    motor_source_position_01mm = (int32_t)g_deviceParams.tankHeight - motor_source_length_01mm;
    length_diff_01mm = motor_source_length_01mm - encoder_length_01mm;
    position_diff_01mm = motor_source_position_01mm - encoder_position_01mm;

    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    printf("电机寄存器 | XACTUAL=%ld | 模型长度=%.1fmm | 圈数=%.6f | 角度=%.2f度\r\n",
           (long)xactual,
           (double)drum.motor_distance_01mm * 0.1,
           drum.turns_total,
           drum.angle_deg);
    printf("位置计算 | 电机长度=%.1fmm | 编码轮长度=%.1fmm | 差值=%.1fmm\r\n",
           (double)motor_source_length_01mm * 0.1,
           (double)encoder_length_01mm * 0.1,
           (double)length_diff_01mm * 0.1);
    printf("位置计算 | 电机位置=%.1fmm | 编码轮位置=%.1fmm | 差值=%.1fmm\r\n",
           (double)motor_source_position_01mm * 0.1,
           (double)encoder_position_01mm * 0.1,
           (double)position_diff_01mm * 0.1);
}
void motorPollRuntimePosition(void)
{
    static uint32_t s_last_runtime_poll_tick = 0U;
    const uint32_t now = HAL_GetTick();
    bool is_moving = false;

    if (!s_motor_initialized) {
        return;
    }
    if ((now - s_last_runtime_poll_tick) < 50U) {
        return;
    }
    s_last_runtime_poll_tick = now;

    if (!Motor_TryReadMovingState(&stepper, &is_moving)) {
        uint32_t inferred_state = Motor_InferDisplayStateFromDriver(&stepper);
        if ((inferred_state == 1U) || (inferred_state == 2U)) {
            g_measurement.debug_data.motor_state = inferred_state;
            Motor_SyncDebugDrumState(&stepper);
        }
        return;
    }

    if (!is_moving) {
        if ((g_measurement.debug_data.motor_state == 1U) ||
            (g_measurement.debug_data.motor_state == 2U)) {
            g_measurement.debug_data.motor_state = 0U;
            Motor_SyncDebugDrumState(&stepper);
        }
        return;
    }

    if ((g_measurement.debug_data.motor_state != 1U) &&
        (g_measurement.debug_data.motor_state != 2U)) {
        uint32_t inferred_state = Motor_InferDisplayStateFromDriver(&stepper);
        if ((inferred_state == 1U) || (inferred_state == 2U)) {
            g_measurement.debug_data.motor_state = inferred_state;
        }
    }
    Motor_SyncDebugDrumState(&stepper);
}

uint32_t motorSwitchPositionSourceToEncoder(void)
{
    g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
    update_sensor_height_from_encoder();
    Motor_SyncDebugDrumState(&stepper);
    Motor_SavePositionSourceParams(true);
    printf("位置来源已切换为编码轮\r\n");
    return NO_ERROR;
}

static uint32_t Motor_RollbackPositionSourceSwitch(uint32_t ret,
                                                   uint32_t old_mode,
                                                   uint32_t old_local_circ_param,
                                                   int32_t old_base_step,
                                                   int32_t old_base_length_01mm,
                                                   double old_base_turns,
                                                   uint32_t old_error_code)
{
    MotorDrumState drum;

    g_deviceParams.position_count_mode = old_mode;
    g_deviceParams.motor_count_first_loop_circumference_mm = old_local_circ_param;
    s_motor_count_base_step = old_base_step;
    s_motor_count_base_length_01mm = old_base_length_01mm;
    s_motor_count_base_turns = old_base_turns;

    if (Motor_IsEncoderErrorCode(g_measurement.device_status.error_code)) {
        g_measurement.device_status.error_code = old_error_code;
    }

    if (old_mode == POSITION_COUNT_MODE_MOTOR) {
        if (Motor_TryUpdateDrumState_FromXACTUAL(&stepper, &drum)) {
            Motor_UpdatePositionFromMotorSource(&drum);
        }
    } else {
        update_sensor_height_from_encoder();
    }

    printf("位置来源切换到电机记步失败，已回滚 | 错误码=0x%08lX\r\n", (unsigned long)ret);
    return ret;
}
/**
 * @brief 将位置计数源从编码器切换到电机步进模式，并执行当前位置局部周长标定
 *
 * 该函数执行以下主要操作：
 * 1. 保存切换前的系统参数和状态，以便失败时回滚
 * 2. 获取切换时的基准编码器计数和尺带长度
 * 3. 临时切换到电机源模式，屏蔽切换窗口内的编码器 SSI 错误
 * 4. 控制电机下行一周，通过编码轮真实长度变化标定当前位置局部周长
 * 5. 验证标定结果的有效性（必须在 C0_MIN_MM 和 C0_MAX_MM 范围内）
 * 6. 返回到切换瞬间的 XACTUAL 位置
 * 7. 保存切换基准和当前位置参数到持久化存储
 *
 * @return uint32_t 返回状态码：
 *             - NO_ERROR: 切换和标定成功
 *             - MOTOR_DISABLED: 电机未初始化
 *             - MOTOR_TMC_COMM_ERROR: TMC 通信错误
 *             - PARAM_ERROR: 参数错误（如每转刻度数无效）
 *             - ENCODER_LOST_STEP: 编码器丢步（标定周长小于最小值）
 *             - ENCODER_DIFF_EXCESS: 编码器差异过大（标定周长超过最大值）
 *             - 其他错误码: 电机移动或操作失败
 *
 * @note 函数内部会调用以下辅助函数：
 *       - Motor_TryUpdateDrumState_FromXACTUAL(): 从 XACTUAL 更新鼓状态
 *       - tape_ticks_per_rev(): 获取每转刻度数
 *       - encoder_get_cable_length_01mm(): 获取尺带长度
 *       - tape_turns_from_signed_length(): 计算卷绕圈数
 *       - Motor_GetLocalCircumferenceFromParams(): 获取局部周长参数
 *       - Motor_SetLocalCircumferenceToParams(): 设置局部周长参数
 *       - motorMoveWaitByTicksWithSpeed(): 按刻度数和速度移动电机
 *       - Motor_RollbackPositionSourceSwitch(): 回滚位置源切换
 *       - Motor_SavePositionSourceParams(): 保存位置源参数
 *       - Motor_StorePersistSnapshot(): 保存持久化快照
 *
 * @note 切换逻辑：
 *       - 切换后电机源长度 = 切换时编码轮尺带长度 + XACTUAL 相对变化量
 *       - 标定过程中临时切到电机源，用于屏蔽切换窗口内的编码器 SSI 错误
 *       - 成功前不保存参数；任何失败都会回滚到进入函数前的模式和基准
 *
 * @note 输出信息包括：
 *       - 周长标定开始时的基准参数
 *       - 周长标定采样的测量数据
 *       - 周长标定失败时的错误信息
 *       - 切换成功后的最终参数和状态
 */
uint32_t motorSwitchPositionSourceToMotor(void)
{
    MotorDrumState drum;
    double base_length_mm;
    double measured_length_mm;
    double measured_circumference_mm = 0.0;
    double local_circumference_mm;
    int32_t one_rev_ticks;
    int32_t return_ticks;
    int32_t base_encoder_count;
    int32_t measured_encoder_count;
    int32_t delta_encoder_count;
    bool measured_circumference_valid = false;
    uint32_t calibration_ret = NO_ERROR;
    uint32_t ret;
    uint32_t old_mode;
    uint32_t old_local_circ_param;
    int32_t old_base_step;
    int32_t old_base_length_01mm;
    double old_base_turns;
    uint32_t old_error_code;

    if (!s_motor_initialized) {
        return MOTOR_DISABLED;
    }

    old_mode = g_deviceParams.position_count_mode;
    old_local_circ_param = g_deviceParams.motor_count_first_loop_circumference_mm;
    old_base_step = s_motor_count_base_step;
    old_base_length_01mm = s_motor_count_base_length_01mm;
    old_base_turns = s_motor_count_base_turns;
    old_error_code = g_measurement.device_status.error_code;

    if (!Motor_TryUpdateDrumState_FromXACTUAL(&stepper, &drum)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    one_rev_ticks = tape_ticks_per_rev();
    if (one_rev_ticks <= 0) {
        return PARAM_ERROR;
    }

    /* 切换到电机记步时，基准尺带长度必须直接来自编码轮。
     * 后续电机源长度 = 切换时编码轮尺带长度 + XACTUAL 相对变化量。 */
    base_encoder_count = g_encoder_count;
    g_measurement.debug_data.current_encoder_value = -g_encoder_count;
    s_motor_count_base_length_01mm = encoder_get_cable_length_01mm();
    base_length_mm = (double)s_motor_count_base_length_01mm * 0.1;
    s_motor_count_base_step = drum.motor_step;
    s_motor_count_base_turns = tape_turns_from_signed_length(base_length_mm,
                                                             tape_C0_mm(),
                                                             tape_t_mm());

    local_circumference_mm = Motor_GetLocalCircumferenceFromParams();
    if (local_circumference_mm <= 0.0) {
        local_circumference_mm = tape_instant_circumference_mm_from_length(base_length_mm);
        (void)Motor_SetLocalCircumferenceToParams(local_circumference_mm);
    }

    /* 标定过程中临时切到电机源，用于屏蔽切换窗口内的编码器 SSI 错误。
     * 成功前不保存参数；任何失败都会回滚到进入函数前的模式和基准。 */
    g_deviceParams.position_count_mode = POSITION_COUNT_MODE_MOTOR;
    if (Motor_IsEncoderErrorCode(g_measurement.device_status.error_code)) {
        g_measurement.device_status.error_code = NO_ERROR;
    }
    Motor_UpdatePositionFromMotorSource(&drum);

    /* 切换后自动下行一周，用编码轮真实长度变化标定当前位置局部周长。 */
    printf("当前位置周长标定开始 | 基准编码值=%ld | 基准长度=%.1fmm | 一圈步数=%ld | 原局部周长=%.3fmm\r\n",
           (long)base_encoder_count,
           base_length_mm,
           (long)one_rev_ticks,
           local_circumference_mm);
    ret = motorMoveWaitByTicksWithSpeed(one_rev_ticks, motorGetDefaultSpeedX100());
    if (ret != NO_ERROR) {
        return Motor_RollbackPositionSourceSwitch(ret, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }

    measured_encoder_count = g_encoder_count;
    delta_encoder_count = measured_encoder_count - base_encoder_count;
    measured_length_mm = (double)encoder_get_cable_length_01mm() * 0.1;
    measured_circumference_mm = measured_length_mm - base_length_mm;
    if (measured_circumference_mm < 0.0) {
        measured_circumference_mm = -measured_circumference_mm;
    }
    printf("当前位置周长标定采样 | 基准编码值=%ld | 当前编码值=%ld | 编码差值=%ld | 基准长度=%.1fmm | 实测长度=%.1fmm | 实测周长=%.3fmm\r\n",
           (long)base_encoder_count,
           (long)measured_encoder_count,
           (long)delta_encoder_count,
           base_length_mm,
           measured_length_mm,
           measured_circumference_mm);
    if ((measured_circumference_mm >= C0_MIN_MM) &&
        (measured_circumference_mm <= C0_MAX_MM)) {
        (void)Motor_SetLocalCircumferenceToParams(measured_circumference_mm);
        local_circumference_mm = measured_circumference_mm;
        measured_circumference_valid = true;
    } else {
        calibration_ret = (measured_circumference_mm < C0_MIN_MM) ? ENCODER_LOST_STEP : ENCODER_DIFF_EXCESS;
        printf("当前位置周长标定失败 | 测得周长=%.3fmm | 范围=(%.1f, %.1f) | 错误码=0x%08lX\r\n",
               measured_circumference_mm,
               (double)C0_MIN_MM,
               (double)C0_MAX_MM,
               (unsigned long)calibration_ret);
    }

    /* 无论标定值是否有效，都回到切换瞬间的 XACTUAL 位置。 */
    return_ticks = -one_rev_ticks;
    ret = motorMoveWaitByTicksWithSpeed(return_ticks, motorGetDefaultSpeedX100());
    if (ret != NO_ERROR) {
        return Motor_RollbackPositionSourceSwitch(ret, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }

    if (!Motor_TryUpdateDrumState_FromXACTUAL(&stepper, &drum)) {
        return Motor_RollbackPositionSourceSwitch(MOTOR_TMC_COMM_ERROR, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }
    Motor_UpdatePositionFromMotorSource(&drum);

    if (calibration_ret != NO_ERROR) {
        return Motor_RollbackPositionSourceSwitch(calibration_ret, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }

    Motor_SavePositionSourceParams(true);
    /* 此处已经通过 Motor_TryUpdateDrumState_FromXACTUAL() 得到可信 XACTUAL，
     * 直接保存切换基准和当前位置，避免再次 SPI 读取失败时静默跳过电机 FRAM 保存。 */
    Motor_StorePersistSnapshot(drum.motor_step);

    printf("编码切换到电机步进模式 | 切换时尺带长度=%.1fmm | 切换时电机步进XACTUAL=%ld | 切换时的周长=%.3fmm | 实测周长=%.3fmm | 来源=%s\r\n",
           (double)s_motor_count_base_length_01mm * 0.1,
           (long)s_motor_count_base_step,
           Motor_GetLocalCircumferenceFromParams(),
           measured_circumference_mm,
           measured_circumference_valid ? "measured" : "model");
    return NO_ERROR;

}
uint32_t motorCalibrateCurrentTapeCircumference(void)
{
    MotorDrumState drum;
    int64_t delta_step;
    double delta_turns;
    double encoder_length_mm;
    double delta_length_mm;
    double local_circumference_mm;

    if (!s_motor_initialized) {
        return MOTOR_DISABLED;
    }
    if (g_deviceParams.position_count_mode != POSITION_COUNT_MODE_MOTOR) {
        return PARAM_ERROR;
    }

    update_sensor_height_from_encoder_force();
    Motor_UpdateDrumState_FromXACTUAL(&stepper, &drum);

    delta_step = (int64_t)drum.motor_step -
                 (int64_t)s_motor_count_base_step;
    if (llabs(delta_step) < ((int64_t)tape_ticks_per_rev() / 16)) {
        Motor_UpdatePositionFromMotorSource(&drum);
        return PARAM_ERROR;
    }

    delta_turns = (double)delta_step / (double)tape_ticks_per_rev();
    encoder_length_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    delta_length_mm = encoder_length_mm -
                      ((double)s_motor_count_base_length_01mm * 0.1);
    if (((delta_length_mm > 0.0) && (delta_turns < 0.0)) ||
        ((delta_length_mm < 0.0) && (delta_turns > 0.0))) {
        Motor_UpdatePositionFromMotorSource(&drum);
        return PARAM_ERROR;
    }

    local_circumference_mm = delta_length_mm / delta_turns;
    if ((local_circumference_mm < C0_MIN_MM) ||
        (local_circumference_mm > C0_MAX_MM)) {
        Motor_UpdatePositionFromMotorSource(&drum);
        return PARAM_ERROR;
    }

    (void)Motor_SetLocalCircumferenceToParams(local_circumference_mm);
    Motor_UpdatePositionFromMotorSource(&drum);

    Motor_SavePositionSourceParams(true);
    Motor_MaybePersistRegisters(&stepper, true);

    printf("电机局部周长已校准 | 周长=%.3fmm | 长度差=%.1fmm | 圈数差=%.6f\r\n",
           Motor_GetLocalCircumferenceFromParams(),
           delta_length_mm,
           delta_turns);
    return NO_ERROR;
}

/* 供 TMC5130 底层驱动在位置寄存器变化后调用。 */
/* 底层改写位置寄存器后调用，强制保存到 FRAM。 */
void motorPersistRegistersFromDriver(void)
{
    Motor_MaybePersistRegisters(&stepper, true);
}
/* 启动 TFIT：清空旧样本并立即采集当前点。 */
void motorTapeFitStart(void)
{
    Motor_TapeFitResetState(true);
    s_motor_tape_fit_enabled = true;
    printf("TFIT开始: 已启用自动采样, C0=%.3f mm, 尺带厚度=%.4f mm\r\n",
           tape_C0_mm(),
           tape_t_mm());
    (void)Motor_TapeFitCaptureCurrentSample(true);
}
/* Start TFIT with the current position as local turn zero. */
void motorTapeFitStartLocalOrigin(void)
{
    MotorDrumState drum;
    int32_t encoder_length_01mm;

    if (!s_motor_initialized) {
        printf("TFIT局部不可用: 电机未初始化\r\n");
        return;
    }

    Motor_UpdateDrumState_FromXACTUAL(&stepper, &drum);
    encoder_length_01mm = Motor_ReadEncoderLengthForFit();
    Motor_TapeFitResetState(true);
    s_motor_tape_fit_origin_step = drum.motor_step;
    s_motor_tape_fit_origin_length_01mm = encoder_length_01mm;
    s_motor_tape_fit_local_origin_valid = true;
    s_motor_tape_fit_enabled = true;
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    printf("TFIT局部开始: 原点步数=%ld, 原点长度=%.1f mm\r\n",
           (long)s_motor_tape_fit_origin_step,
           (double)s_motor_tape_fit_origin_length_01mm * 0.1);
    (void)Motor_TapeFitCaptureCurrentSample(true);
}
/* Stop TFIT auto sampling and keep captured samples. */
void motorTapeFitStop(void)
{
    s_motor_tape_fit_enabled = false;
    printf("TFIT停止: 已关闭自动采样, 采样数=%u\r\n",
           (unsigned)s_motor_tape_fit_count);
}
/* 在当前位置强制采集一个 TFIT 样本。 */
void motorTapeFitAddCurrentSample(void)
{
    (void)Motor_TapeFitCaptureCurrentSample(true);
}
/* 打印 TFIT 样本数量、覆盖范围和求解结果。 */
void motorTapeFitPrintStatus(void)
{
    double n_min = 0.0;
    double n_max = 0.0;
    Motor_TapeFitRange(&n_min, &n_max);
    printf("TFIT状态: 使能=%u, 采样数=%u, 圈数范围=[%.3f, %.3f], 当前C0=%.3f mm, 当前尺带厚度=%.4f mm\r\n",
           s_motor_tape_fit_enabled ? 1U : 0U,
           (unsigned)s_motor_tape_fit_count,
           n_min,
           n_max,
           tape_C0_mm(),
           tape_t_mm());
    if (s_motor_tape_fit_result.valid) {
        printf("TFIT结果: 偏移=%.3f mm, C0=%.3f mm, 尺带厚度=%.4f mm, 均方根误差=%.3f mm, 最大误差=%.3f mm\r\n",
               s_motor_tape_fit_result.offset_mm,
               s_motor_tape_fit_result.first_loop_circ_mm,
               s_motor_tape_fit_result.tape_thickness_mm,
               s_motor_tape_fit_result.rmse_mm,
               s_motor_tape_fit_result.max_abs_err_mm);
    }
}
/* 执行二次最小二乘拟合。
 * 模型：L = b + C0*n - pi*t*n^2，其中 n = motor_step / tape_ticks_per_rev()。 */
uint32_t motorTapeFitSolve(void)
{
    double normal[3][4] = { 0 };
    double coeff[3] = { 0 };
    double sum_sq = 0.0;
    double max_abs_err = 0.0;
    double n_min = 0.0;
    double n_max = 0.0;
    uint16_t i;
    (void)Motor_TapeFitCaptureCurrentSample(true);
    if (s_motor_tape_fit_count < 6U) {
        printf("TFIT求解失败: 至少需要6个采样点, 当前=%u\r\n",
               (unsigned)s_motor_tape_fit_count);
        return PARAM_ERROR;
    }
    Motor_TapeFitRange(&n_min, &n_max);
    if (fabs(n_max - n_min) < 1.0) {
        printf("TFIT求解失败: 电机步数跨度过小, 圈数范围=[%.3f, %.3f]\r\n",
               n_min,
               n_max);
        return PARAM_ERROR;
    }
    /* 拟合形式：
     *   y = a0 + a1*n + a2*n^2
     * 参数换算：
     *   offset=b=a0, C0=a1, t=-a2/pi
     */
    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double n = (double)s_motor_tape_fit_samples[i].motor_step / (double)tape_ticks_per_rev();
        const double x1 = n;
        const double x2 = n * n;
        const double y = (double)s_motor_tape_fit_samples[i].encoder_length_01mm * 0.1;
        normal[0][0] += 1.0;
        normal[0][1] += x1;
        normal[0][2] += x2;
        normal[0][3] += y;
        normal[1][0] += x1;
        normal[1][1] += x1 * x1;
        normal[1][2] += x1 * x2;
        normal[1][3] += x1 * y;
        normal[2][0] += x2;
        normal[2][1] += x1 * x2;
        normal[2][2] += x2 * x2;
        normal[2][3] += x2 * y;
    }
    if (!Motor_TapeFitSolveLinear3x3(normal, coeff)) {
        printf("TFIT求解失败: 矩阵奇异\r\n");
        return PARAM_ERROR;
    }
    s_motor_tape_fit_result.offset_mm = coeff[0];
    s_motor_tape_fit_result.first_loop_circ_mm = coeff[1];
    s_motor_tape_fit_result.tape_thickness_mm = -coeff[2] / M_PI;
    if ((s_motor_tape_fit_result.first_loop_circ_mm <= 0.0) ||
        (s_motor_tape_fit_result.tape_thickness_mm <= 0.0)) {
        printf("TFIT求解失败: 结果无效 偏移=%.3f, C0=%.3f, 尺带厚度=%.4f\r\n",
               s_motor_tape_fit_result.offset_mm,
               s_motor_tape_fit_result.first_loop_circ_mm,
               s_motor_tape_fit_result.tape_thickness_mm);
        s_motor_tape_fit_result.valid = false;
        return PARAM_ERROR;
    }
    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double n = (double)s_motor_tape_fit_samples[i].motor_step / (double)tape_ticks_per_rev();
        const double y = (double)s_motor_tape_fit_samples[i].encoder_length_01mm * 0.1;
        const double pred = coeff[0] + coeff[1] * n - M_PI * s_motor_tape_fit_result.tape_thickness_mm * n * n;
        const double err = pred - y;
        const double abs_err = fabs(err);
        sum_sq += err * err;
        if (abs_err > max_abs_err) {
            max_abs_err = abs_err;
        }
    }
    s_motor_tape_fit_result.rmse_mm = sqrt(sum_sq / (double)s_motor_tape_fit_count);
    s_motor_tape_fit_result.max_abs_err_mm = max_abs_err;
    s_motor_tape_fit_result.valid = true;
    s_motor_tape_fit_result_is_local = false;
    printf("TFIT求解完成: 偏移=%.3f mm, C0=%.3f mm, 尺带厚度=%.4f mm, 均方根误差=%.3f mm, 最大误差=%.3f mm, 采样数=%u\r\n",
           s_motor_tape_fit_result.offset_mm,
           s_motor_tape_fit_result.first_loop_circ_mm,
           s_motor_tape_fit_result.tape_thickness_mm,
           s_motor_tape_fit_result.rmse_mm,
           s_motor_tape_fit_result.max_abs_err_mm,
           (unsigned)s_motor_tape_fit_count);
    return NO_ERROR;
}
/* 将求解出的 TFIT 结果应用到系统参数。
 * 调用方可选择只应用厚度，或同时应用 C0 和厚度。 */
/* 以当前位置作为局部 0 圈求解尺带厚度。
 * 局部模型：dL = C_local*q - pi*t*q^2，q 为相对当前位置起点的电机圈数。 */
uint32_t motorTapeFitSolveLocalOrigin(void)
{
    double m00 = 0.0;
    double m01 = 0.0;
    double m11 = 0.0;
    double b0 = 0.0;
    double b1 = 0.0;
    double det;
    double a1;
    double a2;
    double sum_sq = 0.0;
    double max_abs_err = 0.0;
    double q_min = 0.0;
    double q_max = 0.0;
    uint16_t i;

    if (!s_motor_tape_fit_local_origin_valid) {
        printf("TFIT局部求解失败: 未设置局部原点\r\n");
        return PARAM_ERROR;
    }

    (void)Motor_TapeFitCaptureCurrentSample(true);
    if (s_motor_tape_fit_count < 6U) {
        printf("TFIT局部求解失败: 至少需要6个采样点, 当前=%u\r\n",
               (unsigned)s_motor_tape_fit_count);
        return PARAM_ERROR;
    }

    Motor_TapeFitLocalRange(&q_min, &q_max);
    if (fabs(q_max - q_min) < 1.0) {
        printf("TFIT局部求解失败: 相对圈数跨度过小, q范围=[%.3f, %.3f]\r\n",
               q_min,
               q_max);
        return PARAM_ERROR;
    }

    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double q = ((double)s_motor_tape_fit_samples[i].motor_step -
                          (double)s_motor_tape_fit_origin_step) /
                         (double)tape_ticks_per_rev();
        const double x1 = q;
        const double x2 = q * q;
        const double y = ((double)s_motor_tape_fit_samples[i].encoder_length_01mm -
                          (double)s_motor_tape_fit_origin_length_01mm) * 0.1;
        m00 += x1 * x1;
        m01 += x1 * x2;
        m11 += x2 * x2;
        b0 += x1 * y;
        b1 += x2 * y;
    }

    det = (m00 * m11) - (m01 * m01);
    if (fabs(det) < 1e-12) {
        printf("TFIT局部求解失败: 矩阵奇异\r\n");
        return PARAM_ERROR;
    }

    a1 = ((b0 * m11) - (b1 * m01)) / det;
    a2 = ((m00 * b1) - (m01 * b0)) / det;

    s_motor_tape_fit_result.offset_mm = 0.0;
    s_motor_tape_fit_result.first_loop_circ_mm = a1;
    s_motor_tape_fit_result.tape_thickness_mm = -a2 / M_PI;
    if ((s_motor_tape_fit_result.first_loop_circ_mm <= 0.0) ||
        (s_motor_tape_fit_result.tape_thickness_mm <= 0.0)) {
        printf("TFIT局部求解失败: 结果无效 局部周长=%.3f, 尺带厚度=%.4f\r\n",
               s_motor_tape_fit_result.first_loop_circ_mm,
               s_motor_tape_fit_result.tape_thickness_mm);
        s_motor_tape_fit_result.valid = false;
        return PARAM_ERROR;
    }

    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double q = ((double)s_motor_tape_fit_samples[i].motor_step -
                          (double)s_motor_tape_fit_origin_step) /
                         (double)tape_ticks_per_rev();
        const double y = ((double)s_motor_tape_fit_samples[i].encoder_length_01mm -
                          (double)s_motor_tape_fit_origin_length_01mm) * 0.1;
        const double pred = a1 * q + a2 * q * q;
        const double err = pred - y;
        const double abs_err = fabs(err);
        sum_sq += err * err;
        if (abs_err > max_abs_err) {
            max_abs_err = abs_err;
        }
    }

    s_motor_tape_fit_result.rmse_mm = sqrt(sum_sq / (double)s_motor_tape_fit_count);
    s_motor_tape_fit_result.max_abs_err_mm = max_abs_err;
    s_motor_tape_fit_result.valid = true;
    s_motor_tape_fit_result_is_local = true;

    printf("TFIT局部求解完成: 局部周长=%.3f mm, 尺带厚度=%.4f mm, 均方根误差=%.3f mm, 最大误差=%.3f mm, 采样数=%u\r\n",
           s_motor_tape_fit_result.first_loop_circ_mm,
           s_motor_tape_fit_result.tape_thickness_mm,
           s_motor_tape_fit_result.rmse_mm,
           s_motor_tape_fit_result.max_abs_err_mm,
           (unsigned)s_motor_tape_fit_count);
    return NO_ERROR;
}

uint32_t motorTapeFitApply(bool apply_c0, bool apply_t)
{
    uint32_t old_c0 = g_deviceParams.first_loop_circumference_mm;
    uint32_t old_t = g_deviceParams.tape_thickness_mm;
    bool changed = false;
    if (!s_motor_tape_fit_result.valid) {
        uint32_t ret;
        if (s_motor_tape_fit_local_origin_valid) {
            if (apply_c0) {
                printf("TFIT应用失败: 局部原点采样不能更新全局C0\r\n");
                return PARAM_ERROR;
            }
            ret = motorTapeFitSolveLocalOrigin();
        } else {
            ret = motorTapeFitSolve();
        }
        if (ret != NO_ERROR) {
            return ret;
        }
    }
    if (apply_c0 && s_motor_tape_fit_result_is_local) {
        printf("TFIT应用失败: 局部结果不能更新全局C0\r\n");
        return PARAM_ERROR;
    }
    if (apply_c0) {
        const int32_t new_c0 = (int32_t)llround(s_motor_tape_fit_result.first_loop_circ_mm * 10.0);
        if (new_c0 <= 0) {
            return PARAM_ERROR;
        }
        g_deviceParams.first_loop_circumference_mm = (uint32_t)new_c0;
        changed = true;
    }
    if (apply_t) {
        const int32_t new_t = (int32_t)llround(s_motor_tape_fit_result.tape_thickness_mm * 1000.0);
        if (new_t <= 0) {
            return PARAM_ERROR;
        }
        g_deviceParams.tape_thickness_mm = (uint32_t)new_t;
        changed = true;
    }
    if (!changed) {
        return PARAM_ERROR;
    }
    save_device_params();
    printf("TFIT已应用: C0 %lu -> %lu (0.1mm), t %lu -> %lu (0.001mm)\r\n",
           (unsigned long)old_c0,
           (unsigned long)g_deviceParams.first_loop_circumference_mm,
           (unsigned long)old_t,
           (unsigned long)g_deviceParams.tape_thickness_mm);
    return NO_ERROR;
}
/* 回零后清零 XACTUAL/XTARGET，保持电机和编码器基准一致。 */
void motorResetDrumReferenceToZero(void)
{
    /* 回零只重建两套位置零点，不改变 g_deviceParams.position_count_mode。 */
    s_motor_count_base_step = 0;
    s_motor_count_base_length_01mm = 0;
    s_motor_count_base_turns = 0.0;
    (void)Motor_SetLocalCircumferenceToParams(tape_C0_mm());

    if (Motor_IsEncoderErrorCode(g_measurement.device_status.error_code)) {
        g_measurement.device_status.error_code = NO_ERROR;
    }

    if (!s_motor_initialized) {
        g_measurement.debug_data.motor_step = 0;
        g_measurement.debug_data.motor_distance = 0;
        return;
    }

    stpr_setPos(&stepper, 0);
    Motor_SyncDebugDrumState(&stepper);
    Motor_SavePositionSourceParams(true);
    Motor_MaybePersistRegisters(&stepper, true);
}


/* 根据参数区保存的记步模式恢复运行态。
 * 电机模式下优先使用电机 FRAM 记录中的 base_length/base_step，
 * 保持“切换时编码轮长度 + 电机步进变化量”的连续口径。 */
static void Motor_RestorePositionSourceFromParams(void)
{
    MotorDrumState drum;
    double local_circumference_mm;

    if (!s_motor_initialized) {
        return;
    }

    if (!Motor_TryUpdateDrumState_FromXACTUAL(&stepper, &drum)) {
        printf("位置来源恢复跳过: XACTUAL读取失败\r\n");
        return;
    }
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    local_circumference_mm = Motor_GetLocalCircumferenceFromParams();
    if (local_circumference_mm <= 0.0) {
        local_circumference_mm = tape_instant_circumference_mm_from_length(
            (double)drum.motor_distance_01mm * 0.1);
        (void)Motor_SetLocalCircumferenceToParams(local_circumference_mm);
    }

    if (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) {
        if (s_motor_restored_base_valid) {
            s_motor_count_base_step = s_motor_restored_base_step;
            s_motor_count_base_length_01mm = s_motor_restored_base_length_01mm;
        } else {
            s_motor_count_base_step = drum.motor_step;
            s_motor_count_base_length_01mm = drum.motor_distance_01mm;
        }
        s_motor_count_base_turns = tape_turns_from_signed_length(
            (double)s_motor_count_base_length_01mm * 0.1,
            tape_C0_mm(),
            tape_t_mm());
        Motor_UpdatePositionFromMotorSource(&drum);
        printf("位置来源已恢复为电机记步 | 基准长度=%.1fmm | XACTUAL=%ld | 局部周长=%.3fmm\r\n",
               (double)s_motor_count_base_length_01mm * 0.1,
               (long)s_motor_count_base_step,
               Motor_GetLocalCircumferenceFromParams());
        Motor_MaybePersistRegisters(&stepper, true);
    } else {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
        if (s_motor_restored_base_valid) {
            s_motor_count_base_step = s_motor_restored_base_step;
            s_motor_count_base_length_01mm = s_motor_restored_base_length_01mm;
            s_motor_count_base_turns = tape_turns_from_signed_length(
                (double)s_motor_count_base_length_01mm * 0.1,
                tape_C0_mm(),
                tape_t_mm());
        } else {
            s_motor_count_base_step = 0;
            s_motor_count_base_length_01mm = 0;
            s_motor_count_base_turns = 0.0;
        }
        update_sensor_height_from_encoder();
        printf("位置来源已恢复为编码轮 | 保存的电机基准长度=%.1fmm | 保存的电机基准步数=%ld\r\n",
               (double)s_motor_count_base_length_01mm * 0.1,
               (long)s_motor_count_base_step);
    }

}

/* ===================== 初始化 ===================== */

uint32_t motor_Init(void)
{
    uint32_t motor_current = Motor_ClampCurrentSetting(g_deviceParams.motor_current);

    /* 规范化后的电流回写到参数区，保证显示、保存和驱动寄存器一致。 */
    g_deviceParams.motor_current = motor_current;

    if (g_measurement.debug_data.motor_speed == 0U) {
        g_measurement.debug_data.motor_speed = g_deviceParams.max_motor_speed;
    } else {
        g_measurement.debug_data.motor_speed =
            Motor_ClampSpeedSetpointX100(g_measurement.debug_data.motor_speed);
    }

    Motor_UpdateVelocityFromParams();
    s_motor_applied_velocity = velocity;

    if (!s_motor_initialized) {
        stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, (uint8_t)motor_current);
//      stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 16);
        stpr_enableDriver(&stepper);
        s_motor_initialized = true;
        Motor_RestorePersistedRegisters(&stepper);
        Motor_RestorePositionSourceFromParams();
        /* 上电读取 DeviceParameters 和电机 FRAM 记录后，立即打印一次电机/编码轮位置对比。
         * 用于确认 XACTUAL、记步模式、局部周长、切换基准和编码轮位置是否一致。 */
        motorPrintPositionCompare();
    } else {
        stpr_enableDriver(&stepper);
        (void)motorSetCurrent(motor_current);
    }

    Motor_SyncDebugDrumState(&stepper);

    printf("电机初始化 | 设定速度=%.2f m/min | 尺带长度=%.1f mm | VMAX=%lu | 等效微步/s=%.1f\r\n",
           g_deviceParams.max_motor_speed / 100.0,
           g_measurement.debug_data.cable_length / 10.0,
           (unsigned long)velocity,
           TMC5130_VMAX_To_UstepsPerSec(velocity));
    return NO_ERROR;
}

/* ===================== 运动状态判断/芯片异常检查 ===================== */

bool stpr_isMoving(TMC5130TypeDef *tmc5130)
{
    uint32_t rampstat = stpr_readInt(tmc5130, TMC5130_RAMPSTAT);
    return ((rampstat & 0x400) != 0x400); // bit10=1 表示停止
}

uint32_t stpr_checkGstat(TMC5130TypeDef *tmc5130)
{
    uint32_t gstat = stpr_readInt(tmc5130, TMC5130_GSTAT);
    uint32_t ret = NO_ERROR;

    if (gstat == 0)
        return NO_ERROR;

    if (gstat & (1 << 0)) {
        printf("TMC5130: 芯片复位检测到（bit0=1）\r\n");
    }
    if (gstat & (1 << 1)) {
        printf("TMC5130: 驱动器因过热或短路被关闭\r\n");
        ret = MOTOR_OVERTEMPERATURE;
    }
    if (gstat & (1 << 2)) {
        printf("TMC5130: 充电泵欠压\r\n");
        ret = MOTOR_CHARGE_PUMP_UNDER_VOLTAGE;
    }

    /* 写 1 清除状态位 */
    stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07);

    if (ret != NO_ERROR)
        CHECK_ERROR(ret);

    return ret;
}

/* ===================== 可打断等待：通用等待封装（新增） ===================== */

static uint32_t motor_wait_stop_abortable(uint32_t poll_ms)
{
    uint32_t last_vel_refresh_tick = HAL_GetTick();

    while (stpr_isMoving(&stepper)) {
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
        Motor_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        Motor_SyncDebugDrumState(&stepper);
        HAL_Delay(poll_ms);
    }
    Motor_SyncDebugDrumState(&stepper);
    g_measurement.debug_data.motor_state = 0U;
    return NO_ERROR;
}

/* ===================== ticks 运动封装（阻塞，可打断） ===================== */

uint32_t motorMoveWaitByTicksWithSpeed(int32_t ticks, uint32_t speed_x100)
{
    uint32_t ret = NO_ERROR;
    uint32_t restore_ret = NO_ERROR;
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    int32_t xactual_before = 0;
    int32_t xactual_after = 0;
    int32_t xtarget_after = 0;
    int32_t vmax_after = 0;
    int32_t rampmode_after = 0;
    int32_t rampstat_after = 0;
    int32_t gstat_after = 0;
//    int32_t delta_ticks = ticks;

    ret = Motor_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    CHECK_ERROR(ret);

    /* ticks 运动虽然不走“长度->圈数->ticks”的路径，
     * 但仍然要基于当前卷径刷新 VMAX，保证速度口径一致。 */
    Motor_UpdateVelocityFromParams();

    if (Motor_StopIfCommandSwitchRequested()) {
        restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        (void)restore_ret;
        return COMMAND_SWITCH_ABORT;
    }

    /* 若方向相反，仅需在此统一翻转 */
    /* ticks = -ticks; */

    if (!stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual_before)) {
        restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        (void)restore_ret;
        printf("步进运动失败 | 下发前XACTUAL读取失败 | 增量=%ld\r\n", (long)ticks);
        return MOTOR_TMC_COMM_ERROR;
    }
//    printf("ticks运动下发 | 增量=%ld | XACTUAL_before=%ld | VMAX=%lu | speed=%.2fm/min\r\n",
//           (long)delta_ticks,
//           (long)xactual_before,
//           (unsigned long)velocity,
//           (double)Motor_GetSpeedSetpointX100() / 100.0);
    s_motor_applied_velocity = velocity;
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    if (ret != NO_ERROR) {
        restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (restore_ret != NO_ERROR) {
            return restore_ret;
        }
        return ret;
    }

    /* TMC5130 写入 XTARGET 后，不能只看 RAMPSTAT.vzero。
     * 这里同时读回关键寄存器，并要求 XACTUAL 在启动窗口内发生变化。
     * 如果目标写入了但 XACTUAL 不变，按“未真正启动”返回错误，避免首圈标定假成功。 */
    (void)stpr_tryReadInt(&stepper, TMC5130_XTARGET, &xtarget_after);
    (void)stpr_tryReadInt(&stepper, TMC5130_VMAX, &vmax_after);
    (void)stpr_tryReadInt(&stepper, TMC5130_RAMPMODE, &rampmode_after);
    (void)stpr_tryReadInt(&stepper, TMC5130_RAMPSTAT, &rampstat_after);
    (void)stpr_tryReadInt(&stepper, TMC5130_GSTAT, &gstat_after);
//    printf("ticks运动读回 | target=%ld | XACTUAL=%ld | XTARGET=%ld | VMAX=%ld | RAMPMODE=%ld | RAMPSTAT=0x%08lX | GSTAT=0x%08lX\r\n",
//           (long)ticks,
//           (long)xactual_before,
//           (long)xtarget_after,
//           (long)vmax_after,
//           (long)rampmode_after,
//           (unsigned long)rampstat_after,
//           (unsigned long)gstat_after);
    {
        uint32_t start_wait_tick = HAL_GetTick();
        uint32_t last_vel_refresh_tick = start_wait_tick;
        bool position_changed = false;
        do {
            if (Motor_StopIfCommandSwitchRequested()) {
                restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
                (void)restore_ret;
                return COMMAND_SWITCH_ABORT;
            }
            Motor_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
            Motor_SyncDebugDrumState(&stepper);
            if (stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual_after) &&
                (xactual_after != xactual_before)) {
                position_changed = true;
                break;
            }
            HAL_Delay(10);
        } while ((HAL_GetTick() - start_wait_tick) < 500U);

        if (!position_changed) {
            (void)stpr_tryReadInt(&stepper, TMC5130_XTARGET, &xtarget_after);
            (void)stpr_tryReadInt(&stepper, TMC5130_RAMPSTAT, &rampstat_after);
            (void)stpr_tryReadInt(&stepper, TMC5130_GSTAT, &gstat_after);
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            printf("步进运动失败 | 500ms内XACTUAL未变化 | 变化前=%ld | 变化后=%ld | 目标=%ld | XTARGET=%ld | RAMPSTAT=0x%08lX | GSTAT=0x%08lX\r\n",
                   (long)xactual_before,
                   (long)xactual_after,
                   (long)ticks,
                   (long)xtarget_after,
                   (unsigned long)rampstat_after,
                   (unsigned long)gstat_after);
            return MOTOR_STEP_ERROR;
        }
    }

    ret = motor_wait_stop_abortable(10);

    restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
    if (ret != NO_ERROR) {
        return ret;
    }
    CHECK_ERROR(restore_ret);
    return restore_ret;
}

/* ===================== 精确长度换算下发（不等待） ===================== */

uint32_t motorMoveNoWaitWithSpeed(float move_mm, int dir, uint32_t speed_x100)
{
    if (move_mm < 0.0f) return PARAM_ERROR;
    if (move_mm == 0.0f) return NO_ERROR;
    if (!Motor_IsDirValid(dir)) return PARAM_ERROR;

    uint32_t ret = Motor_ApplyOptionalSpeed(speed_x100);
    CHECK_ERROR(ret);

    g_measurement.debug_data.motor_state = (dir == MOTOR_DIRECTION_UP) ? 1U : 2U;
    Motor_SyncDebugDrumState(&stepper);

    /* 1) 当前“相对基准点”的有符号长度（mm，可正可负） */
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = Motor_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_applied_velocity = velocity;

    /* 2) 目标有符号长度（mm） */
    double dL_mm = (double)move_mm;
    if (dir == MOTOR_DIRECTION_UP) {
        dL_mm = -dL_mm;  // 上行：长度更小（可继续变负）
    }
    const double Ltar_mm = Lcur_mm + dL_mm;

    int64_t ticks64;

    {
        double local_circumference_mm = Motor_GetLocalCircumferenceFromParams();
        if ((g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
            (local_circumference_mm > 1e-6)) {
            ticks64 = llround_safe((dL_mm / local_circumference_mm) *
                                   (double)tape_ticks_per_rev());
        } else {
            const double C0 = tape_C0_mm();
            const double t  = tape_t_mm();
            if (C0 <= 0.0) return PARAM_ERROR;

            const double ncur = tape_turns_from_signed_length(Lcur_mm, C0, t);
            const double ntar = tape_turns_from_signed_length(Ltar_mm, C0, t);
            const double dn   = ntar - ncur;

            double ticks_d = dn * (double)tape_ticks_per_rev();
            ticks64 = llround_safe(ticks_d);
        }
    }

    if (ticks64 > (int64_t)INT32_MAX) ticks64 = (int64_t)INT32_MAX;
    if (ticks64 < (int64_t)INT32_MIN) ticks64 = (int64_t)INT32_MIN;

    int32_t ticks = (int32_t)ticks64;

    /* 6) 下发运动 */
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    if (ret != NO_ERROR) {
        return ret;
    }
    Motor_SyncDebugDrumState(&stepper);

    return NO_ERROR;
}

/* ===================== 第一圈周长标定（可打断，新模型） ===================== */

uint32_t CalibrateFirstLoopCircumference_OneTurnAtZero(void)
{
    const double t = tape_t_mm();
    if (t <= 0.0) {
        printf("首圈周长标定失败 | 尺带厚度非法 尺带厚度=%.4fmm\r\n", t);
        return PARAM_ERROR;
    }

    const int32_t one_rev_ticks = tape_ticks_per_rev();
    if (one_rev_ticks <= 0) {
        printf("首圈周长标定失败 | 每圈步数非法 步数=%ld\r\n", (long)one_rev_ticks);
        return PARAM_ERROR;
    }

    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 1) 记录零点长度 */
    const double L0 = get_current_tape_length_mm();

    /* 2) 下行一圈并等待停止（可打断） */
    uint32_t ret = motorMoveWaitByTicksWithSpeed(one_rev_ticks, motorGetDefaultSpeedX100());
    if (ret != NO_ERROR) {
        printf("首圈周长标定失败 | 下行一圈失败 错误码=0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    HAL_Delay(200);

    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 3) 读取一圈后的长度 */
    const double L1 = get_current_tape_length_mm();
    const double dL = L1 - L0;
    printf("首圈周长标定采样 | 起点长度=%.1fmm | 终点长度=%.1fmm | 长度差=%.1fmm | 步数=%ld\r\n",
           L0, L1, dL, (long)one_rev_ticks);
    if (dL <= 0.0) {
        printf("首圈周长标定失败 | 一圈后尺带长度未增加 长度差=%.1fmm\r\n", dL);
        return PARAM_ERROR;
    }

    /* 新模型：第一圈放出长度：dL = C0 - pi*t  =>  C0 = dL + pi*t */
    const double C0 = dL + (1.0 * M_PI * t);

    if (!(C0 > C0_MIN_MM && C0 < C0_MAX_MM)) {
        printf("首圈周长标定失败 | C0越界 C0=%.3fmm | 范围=(%.1f, %.1f)\r\n",
               C0, C0_MIN_MM, C0_MAX_MM);
        return PARAM_ERROR;
    }

    printf("首圈周长标定计算 | 长度差=%.1fmm | 尺带厚度=%.4fmm | C0=%.3fmm | 旧值=%lu(0.1mm) | 新值=%ld(0.1mm)\r\n",
           dL,
           t,
           C0,
           (unsigned long)g_deviceParams.first_loop_circumference_mm,
           (long)llround(C0 * 10.0));
    g_deviceParams.first_loop_circumference_mm = (int32_t)llround(C0 * 10.0);
    ret = motorMoveAndWaitUntilStopWithSpeed(dL, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100()); // 回到起点
    if (ret != NO_ERROR) {
        printf("首圈周长标定失败 | 回到起点失败 错误码=0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }
    /* 首圈周长标定直接修改了系统参数 first_loop_circumference_mm。
     * 这个值会影响后续的尺带长度换算，所以标定成功后必须立即保存，
     * 同时通过 parameter_update_flag 通知 CPU3 刷新参数缓存。 */
    save_device_params();
    printf("首圈周长标定成功 | 首圈周长=%lu(0.1mm)\r\n",
           (unsigned long)g_deviceParams.first_loop_circumference_mm);
    return NO_ERROR;
}

/* ===================== 带检测的等待停止（可打断，强制停机返回） ===================== */

static uint32_t stpr_wait_until_stop_with_target(TMC5130TypeDef *tmc5130,
                                                float target_mm,
                                                float eps_mm,
                                                int dir)
{
    uint32_t ret = NO_ERROR;
    uint32_t startTick = HAL_GetTick();
    uint32_t last_vel_refresh_tick = startTick;
    const uint32_t MAX_WAIT_MS = 60000 * 60;

    while (stpr_isMoving(tmc5130)) {

        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
        Motor_RefreshVelocityDuringRun(tmc5130, &last_vel_refresh_tick);
        Motor_SyncDebugDrumState(tmc5130);

        /* 1) 当前位置（mm） */
        float cur_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

        /* 2) 到位（容差）-> 立即停机并返回成功 */
        if (fabsf(cur_mm - target_mm) <= eps_mm) {
            Motor_StopAndMarkStopped();
            while (stpr_isMoving(tmc5130)) {
                CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
                HAL_Delay(5);
            }
            return NO_ERROR;
        }

        /* 3) 越过目标也应停（结合方向口径）：
         *    - 下行：cur 逐渐变小，越过目标意味着 cur <= target
         *    - 上行：cur 逐渐变大，越过目标意味着 cur >= target
         */
        if (dir == MOTOR_DIRECTION_DOWN) {
            if (cur_mm <= target_mm) {
                Motor_StopAndMarkStopped();
                while (stpr_isMoving(tmc5130)) {
                    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
                    HAL_Delay(5);
                }
                return NO_ERROR;
            }
        } else {
            if (cur_mm >= target_mm) {
                Motor_StopAndMarkStopped();
                while (stpr_isMoving(tmc5130)) {
                    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
                    HAL_Delay(5);
                }
                return NO_ERROR;
            }
        }

        /* 4) 碰撞/极限检测 */
        ret = CheckWeightCollision();
        CHECK_ERROR(ret);

        /* 5) 芯片异常检测 */
        ret = stpr_checkGstat(tmc5130);
        CHECK_ERROR(ret);

        /* 6) 超时保护 */
        if (HAL_GetTick() - startTick > MAX_WAIT_MS) {
            printf("TMC5130: 等待停止超时！\r\n");
            RETURN_ERROR(MOTOR_RUN_TIMEOUT);
        }

        HAL_Delay(50);
    }

    Motor_SyncDebugDrumState(tmc5130);
    return NO_ERROR;
}

/* ===================== 高层运动：分段+等待+保护（可打断） ===================== */

uint32_t motorMoveAndWaitUntilStopWithSpeed(float mm, int dir, uint32_t speed_x100)
{
    uint32_t ret = NO_ERROR;
    uint32_t restore_ret = NO_ERROR;
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    float startPos_mm;
    float currentPos_mm;
    float targetPos_mm;
    float total_cmd_mm = mm;
    float moved_mm;
    float remain_mm;
    uint8_t attempt = 0;

    if (mm <= 0.0f) {
        return NO_ERROR;
    }
    if (!Motor_IsDirValid(dir)) {
        return PARAM_ERROR;
    }

    /* 阻塞型接口支持“本次命令临时速度”：
     * 开始前切到 speed_x100，结束后恢复到默认最大速度。 */
    ret = Motor_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    CHECK_ERROR(ret);

    if (Motor_StopIfCommandSwitchRequested()) {
        restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        (void)restore_ret;
        return COMMAND_SWITCH_ABORT;
    }

    startPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

    /* 你现场口径：下行 pos 变小，上行 pos 变大 */
    targetPos_mm = startPos_mm +
                   ((dir == MOTOR_DIRECTION_UP) ? total_cmd_mm : -total_cmd_mm);

    remain_mm = total_cmd_mm;

    while (attempt < MOTOR_MOVE_RETRY_MAX && remain_mm > 0.0f) {

        attempt++;

        if (Motor_StopIfCommandSwitchRequested()) {
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            return COMMAND_SWITCH_ABORT;
        }

        /* 分段重试时不再重复切换速度，避免每一段都触发“恢复默认速度”。 */
        ret = motorMoveNoWaitWithSpeed(remain_mm, dir, 0U);
        CHECK_ERROR(ret);

        /* 下发后先进入一个短暂的“启动观察窗口”，
         * 期间持续做速度补偿，让起步阶段也尽快贴合目标线速度。 */
        uint32_t prewait_vel_refresh_tick = HAL_GetTick();
        for (int i = 0; i < 100; i++) {
            if (Motor_StopIfCommandSwitchRequested()) {
                restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
                (void)restore_ret;
                return COMMAND_SWITCH_ABORT;
            }
            Motor_RefreshVelocityDuringRun(&stepper, &prewait_vel_refresh_tick);
            motorPollRuntimePosition();
            HAL_Delay(10);
        }

        ret = stpr_wait_until_stop_with_target(&stepper, targetPos_mm, EPS_MM, dir);

        if (ret == COMMAND_SWITCH_ABORT) {
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            return ret;
        }
        if (Motor_StopIfCommandSwitchRequested()) {
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            return COMMAND_SWITCH_ABORT;
        }

        currentPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

        moved_mm = fabsf(currentPos_mm - startPos_mm);

        remain_mm = fabsf(targetPos_mm - currentPos_mm);
        if (remain_mm < EPS_MM) remain_mm = 0.0f;

        if (ret == NO_ERROR) {
            printf("电机段运行成功: 尝试次数=%d, 累计位移=%.2f mm, 剩余=%.2f mm\r\n",
                   attempt, moved_mm, remain_mm);

            MotorDrumState drum;
            Motor_UpdateDrumState_FromXACTUAL(&stepper, &drum);
//            printf("电机状态 | XACTUAL=%ld | 圈=%.4f | 角度=%.1f° | 预测长度=%.1fmm\r\n",
//                   (long)drum.motor_step,
//                   drum.turns_total,
//                   drum.angle_deg,
//                   drum.motor_distance_01mm / 10.0);

            break;
        } else {

            printf("警告：电机在第%d次运行过程中异常停止，错误0x%X\r\n",
                   attempt, (unsigned int)ret);

            if (attempt >= MOTOR_MOVE_RETRY_MAX) {
                restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
                if ((restore_ret != NO_ERROR) && (ret == NO_ERROR)) {
                    ret = restore_ret;
                }
                CHECK_ERROR(ret);
                return ret;
            }

            printf("尝试继续补偿剩余距离 %.2f mm (方向=%s)\r\n",
                   remain_mm,
                   (dir == MOTOR_DIRECTION_DOWN) ? "下行" : "上行");

            for (int i = 0; i < 20; i++) {
                if (Motor_StopIfCommandSwitchRequested()) {
                    restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
                    (void)restore_ret;
                    return COMMAND_SWITCH_ABORT;
                }
                HAL_Delay(10);
            }
        }
    }

    currentPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
    moved_mm = fabsf(currentPos_mm - startPos_mm);

    /* 用 abs 计算误差百分比，避免方向口径导致的负值 */
    float diff_pct = 100.0f * fabsf(moved_mm - total_cmd_mm) / total_cmd_mm;

    if ((diff_pct > 70.0f) && (total_cmd_mm > 5.0f)) {
        printf("警告：检测到电机可能丢步！\r\n");
        printf("目标=%.2f mm, 实际=%.2f mm, 误差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        if (motorIsPositionSourceMotor()) {
            printf("当前为电机步进位置源，忽略编码器丢步错误\r\n");
            ret = NO_ERROR;
        } else {
            ret = ENCODER_LOST_STEP;
        }
    } else {
        printf("电机移动完成。\t");
        printf("起点=%.2f mm, 目标=%.2f mm, 最终=%.2f mm\t",
               startPos_mm, targetPos_mm, currentPos_mm);
        printf("期望=%.2f mm, 实际=%.2f mm, 偏差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        ret = NO_ERROR;
    }

    Motor_SyncDebugDrumState(&stepper);

    /* 只有主命令整体结束后，才恢复默认速度。 */
    restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
    if (ret != NO_ERROR) {
        return ret;
    }
    CHECK_ERROR(restore_ret);
    return restore_ret;
}

/* ===================== 长距离运行（不阻塞，不做等待） ===================== */

uint32_t motorMove_upWithSpeed(uint32_t speed_x100)
{
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    uint32_t ret = motorMoveNoWaitWithSpeed(2000, MOTOR_DIRECTION_UP, speed_x100);
    CHECK_ERROR(ret);
    return NO_ERROR;
}

uint32_t motorMove_downWithSpeed(uint32_t speed_x100)
{
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    uint32_t ret = motorMoveNoWaitWithSpeed(200, MOTOR_DIRECTION_DOWN, speed_x100);
    CHECK_ERROR(ret);
    return NO_ERROR;
}

/* ===================== 运动辅助：位置快照与到位控制（可打断） ===================== */

void snapshot_sensor_pos_mm(float *pos_mm)
{
    uint32_t pos_01mm = (uint32_t)g_measurement.debug_data.sensor_position;
    *pos_mm = (float)pos_01mm / 10.0f;
}

uint32_t motorMoveToPositionOneShotWithSpeed(float target_mm, uint32_t speed_x100)
{
    uint32_t ret = NO_ERROR;
    uint32_t restore_ret = NO_ERROR;
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    float cur_mm;

    ret = Motor_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    CHECK_ERROR(ret);

    /* 绝对位置运动用“小步逼近”的方式做，
     * 每一轮都重新读取当前位置，降低位置更新滞后带来的影响。 */
    for (int i = 0; i < 10; i++) {

        if (Motor_StopIfCommandSwitchRequested()) {
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            return COMMAND_SWITCH_ABORT;
        }

        snapshot_sensor_pos_mm(&cur_mm);
        printf("运动到位置 | 当前=%.3fmm | 目标=%.3fmm\r\n", cur_mm, target_mm);

        float delta = target_mm - cur_mm;

        if (fabsf(delta) <= EPS_MM) {
            printf("到位 | 当前=%.3fmm ≈ 目标=%.3fmm (±%.2fmm)\r\n",
                   cur_mm, target_mm, EPS_MM);
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            CHECK_ERROR(restore_ret);
            return restore_ret;
        }

        int dir = (delta > 0.0f) ? MOTOR_DIRECTION_UP : MOTOR_DIRECTION_DOWN;

        float plan_mm = fabsf(delta);
        if (plan_mm < (EPS_MM * 2.0f)) {
            plan_mm = (EPS_MM * 2.0f);
        }
        ret = motorMoveAndWaitUntilStopWithSpeed(plan_mm, dir, 0U);
        if (ret != NO_ERROR) {
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            return ret; // 含 COMMAND_SWITCH_ABORT
        }
    }

    restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
    CHECK_ERROR(restore_ret);
    return restore_ret;
}

uint32_t motorMoveRelativeOneShotWithSpeed(float mm, uint32_t speed_x100)
{
    float cur_mm;
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
    snapshot_sensor_pos_mm(&cur_mm);
    return motorMoveToPositionOneShotWithSpeed(cur_mm + mm, speed_x100);
}

uint32_t motorMoveUntilCondition(float max_mm,
                                 int dir,
                                 uint32_t speed_x100,
                                 uint32_t poll_ms,
                                 uint32_t timeout_ms,
                                 MotorConditionCheckFn condition_fn,
                                 void *user_ctx,
                                 bool *condition_met)
{
    uint32_t ret = NO_ERROR;
    uint32_t restore_ret = NO_ERROR;
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    bool met = false;
    uint32_t start_tick;
    uint32_t last_vel_refresh_tick;

    /* 输出参数默认置 false，避免调用方读取到旧值。 */
    if (condition_met) {
        *condition_met = false;
    }

    if ((max_mm <= 0.0f) || (!Motor_IsDirValid(dir)) || (condition_fn == NULL)) {
        return PARAM_ERROR;
    }

    /* 若进入函数时条件已经满足，直接返回成功，不再额外运动。 */
    if (condition_fn(user_ctx)) {
        if (condition_met) {
            *condition_met = true;
        }
        return NO_ERROR;
    }

    if (poll_ms == 0U) {
        poll_ms = MOTOR_CONDITION_DEFAULT_POLL_MS;
    }
    if (timeout_ms == 0U) {
        timeout_ms = MOTOR_CONDITION_DEFAULT_TIMEOUT_MS;
    }

    ret = Motor_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    CHECK_ERROR(ret);

    if (Motor_StopIfCommandSwitchRequested()) {
        restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        (void)restore_ret;
        return COMMAND_SWITCH_ABORT;
    }

    /* 先按“最大允许距离”发一段运动命令，
     * 真正的停止时机由下面的条件检测和保护逻辑共同决定。 */
    ret = motorMoveNoWaitWithSpeed(max_mm, dir, 0U);
    if (ret != NO_ERROR) {
        restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        (void)restore_ret;
        return ret;
    }

    start_tick = HAL_GetTick();
    last_vel_refresh_tick = start_tick;

    while (stpr_isMoving(&stepper)) {
        if (Motor_StopIfCommandSwitchRequested()) {
            restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            return COMMAND_SWITCH_ABORT;
        }

        /* 运行中持续按当前卷径补偿速度，保证“线速度”尽量稳定。 */
        Motor_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        motorPollRuntimePosition();

        /* 用户条件优先级最高。
         * 一旦条件满足，立即停机并以“正常到达条件”返回。 */
        if (condition_fn(user_ctx)) {
            Motor_StopAndMarkStopped();
            while (stpr_isMoving(&stepper)) {
                motorPollRuntimePosition();
                HAL_Delay(5);
            }
            met = true;
            break;
        }

        /* 条件未满足时，仍要继续执行通用保护。 */
        ret = CheckWeightCollision();
        if (ret != NO_ERROR) {
            Motor_StopAndMarkStopped();
            while (stpr_isMoving(&stepper)) {
                motorPollRuntimePosition();
                HAL_Delay(5);
            }
            break;
        }

        ret = stpr_checkGstat(&stepper);
        if (ret != NO_ERROR) {
            Motor_StopAndMarkStopped();
            while (stpr_isMoving(&stepper)) {
                motorPollRuntimePosition();
                HAL_Delay(5);
            }
            break;
        }

        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            Motor_StopAndMarkStopped();
            while (stpr_isMoving(&stepper)) {
                motorPollRuntimePosition();
                HAL_Delay(5);
            }
            ret = MOTOR_RUN_TIMEOUT;
            break;
        }

        HAL_Delay(poll_ms);
    }

    g_measurement.debug_data.motor_state = 0U;

    if (condition_met) {
        *condition_met = met;
    }

    /* ret==NO_ERROR 但 met==false，说明：
     * 电机已经自然停下，但并不是因为条件回调触发。 */
    if ((ret == NO_ERROR) && (!met)) {
        printf("条件运动: 运动结束，但条件未满足 | 最大距离=%.2f | 方向=%d\r\n",
               max_mm, dir);
    }

    restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
    if (ret != NO_ERROR) {
        return ret;
    }
    CHECK_ERROR(restore_ret);
    return restore_ret;
}


static uint32_t Motor_WaitStoppedAfterStopCommand(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    int32_t rampstat = 0;

    if (timeout_ms == 0U) {
        timeout_ms = MOTOR_STOP_WAIT_TIMEOUT_MS;
    }

    while (1) {
        if (!stpr_tryReadInt(&stepper, TMC5130_RAMPSTAT, &rampstat)) {
            printf("电机停止等待失败：RAMPSTAT读取失败\r\n");
            return MOTOR_TMC_COMM_ERROR;
        }

        if ((rampstat & 0x400) == 0x400) {
            Motor_SyncDebugDrumState(&stepper);
            g_measurement.debug_data.motor_state = 0U;
            return NO_ERROR;
        }

        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            printf("电机停止等待超时：RAMPSTAT=0x%08lX\r\n", (unsigned long)rampstat);
            return MOTOR_RUN_TIMEOUT;
        }

        HAL_Delay(10);
    }
}
/* ===================== 急停/停机 ===================== */

uint32_t motorQuickStop(void)
{
    if (stpr_isMoving(&stepper)) {
        Motor_StopAndMarkStopped();
        stpr_disableDriver(&stepper);
        HAL_Delay(4000);
        stpr_enableDriver(&stepper);
    }
    Motor_SyncDebugDrumState(&stepper);
    g_measurement.debug_data.motor_state = 0U;
    return motorSetSpeed(g_deviceParams.max_motor_speed);
}

uint32_t motorSlowStop(void)
{
    uint32_t ret;

    Motor_StopAndMarkStopped();

    /* 慢停后必须等 RAMPSTAT.vzero 确认停稳，再恢复软件速度设定。
     * 否则 TMC5130 仍处于速度模式时写回 VMAX，可能导致电机继续运行。 */
    ret = Motor_WaitStoppedAfterStopCommand(MOTOR_STOP_WAIT_TIMEOUT_MS);
    if (ret != NO_ERROR) {
        return ret;
    }

    return motorSetSpeed(g_deviceParams.max_motor_speed);
}

uint32_t motorGetDisplayState(void)
{
    uint32_t motor_state = g_measurement.debug_data.motor_state;
    bool is_moving = false;

    if (!s_motor_initialized) {
        return ((motor_state == 1U) || (motor_state == 2U)) ? motor_state : 0U;
    }

    if (!Motor_TryReadMovingState(&stepper, &is_moving)) {
        uint32_t inferred_state = Motor_InferDisplayStateFromDriver(&stepper);
        if ((inferred_state == 1U) || (inferred_state == 2U)) {
            g_measurement.debug_data.motor_state = inferred_state;
            return inferred_state;
        }
        return ((motor_state == 1U) || (motor_state == 2U)) ? motor_state : 0U;
    }

    if (!is_moving) {
        if ((motor_state == 1U) || (motor_state == 2U)) {
            g_measurement.debug_data.motor_state = 0U;
            Motor_SyncDebugDrumState(&stepper);
        }
        return 0U;
    }

    if ((motor_state == 1U) || (motor_state == 2U)) {
        return motor_state;
    }

    motor_state = Motor_InferDisplayStateFromDriver(&stepper);
    if ((motor_state == 1U) || (motor_state == 2U)) {
        g_measurement.debug_data.motor_state = motor_state;
        return motor_state;
    }

    return 0U;
}

/* ===================== 丢步检测 ===================== */

void MotorLostStep_Init(void)
{
    write_idx = 0;
    samples = 0;
    last_check_tick = 0;
    last_alarm_tick = 0;

    for (int i = 0; i < LOST_STEP_WINDOW; i++) {
        pos_buf[i]  = 0;
        tick_buf[i] = 0;
    }
}

uint32_t Motor_CheckLostStep_AutoTiming(int32_t currentPos)
{
    uint32_t now = HAL_GetTick();

    /* 每1秒检查一次 */
    if ((now - last_check_tick) < LOST_STEP_INTERVAL_MS) {
        return NO_ERROR;
    }

    last_check_tick = now;

    /* 写入当前位置和时间戳 */
    pos_buf[write_idx]  = currentPos;   /* 单位：0.1mm */
    tick_buf[write_idx] = now;          /* 单位：ms */
    write_idx = (write_idx + 1) % LOST_STEP_WINDOW;

    if (samples < LOST_STEP_WINDOW) {
        samples++;
    }

    /* 样本不足，先不判定 */
    if (samples < LOST_STEP_WINDOW) {
        return NO_ERROR;
    }

    /* oldest_idx 指向最旧样本 */
    int oldest_idx = write_idx;
    int newest_idx = (write_idx + LOST_STEP_WINDOW - 1) % LOST_STEP_WINDOW;

    int32_t pos_old = pos_buf[oldest_idx];
    int32_t pos_new = pos_buf[newest_idx];
    uint32_t tick_old = tick_buf[oldest_idx];
    uint32_t tick_new = tick_buf[newest_idx];

    if (tick_new <= tick_old) {
        return NO_ERROR;
    }

    /* 最近5秒总位移（绝对值），单位：0.1mm */
    int32_t delta_pos_01mm = pos_new - pos_old;
    if (delta_pos_01mm < 0) {
        delta_pos_01mm = -delta_pos_01mm;
    }

    /* 时间差，单位：s */
    float dt_s = (float)(tick_new - tick_old) / 1000.0f;
    if (dt_s < 0.001f) {
        return NO_ERROR;
    }

    /* 平均速度，单位：mm/s */
    float avg_speed_mm_s = ((float)delta_pos_01mm / 10.0f) / dt_s;

    /* 设定速度，单位：mm/s
       motor_speed单位=0.01m/min
       => mm/s = x100 / 6
     */
    float set_speed_m_min = Motor_GetSpeedSetpoint_m_min();
    float set_speed_mm_s  = set_speed_m_min * 1000.0f / 60.0f;
    /* 速度阈值 = max(设定速度的20%, 绝对下限0.5mm/s) */
    float speed_threshold_mm_s =
        set_speed_mm_s * ((float)LOST_STEP_SPEED_RATIO_PERCENT / 100.0f);

    if (speed_threshold_mm_s < LOST_STEP_MIN_AVG_SPEED_MM_S) {
        speed_threshold_mm_s = LOST_STEP_MIN_AVG_SPEED_MM_S;
    }

    /* 中间区域才启用丢步判定，避免靠近零点/罐底误判 */
    if ((g_measurement.debug_data.cable_length > 1000) &&
        (g_measurement.debug_data.cable_length < bottom_value - 1000)) {

        if (avg_speed_mm_s < speed_threshold_mm_s) {
            if ((now - last_alarm_tick) >= LOST_STEP_SUPPRESS_MS) {
                last_alarm_tick = now;

                printf("电机丢步报警 | 当前位置=%.1f mm | 最近%.1f s平均速度=%.3f mm/s < 阈值=%.3f mm/s | 设定=%.3f mm/s\r\n",
                       (float)currentPos / 10.0f,
                       dt_s,
                       avg_speed_mm_s,
                       speed_threshold_mm_s,
                       set_speed_mm_s);

                if (motorIsPositionSourceMotor()) {
                    printf("当前为电机步进位置源，忽略编码器丢步错误\r\n");
                    return NO_ERROR;
                }
                return ENCODER_LOST_STEP;
            }
        } else {
            printf("丢步检测正常 | 当前位置=%.1f mm | 最近%.1f s平均速度=%.3f mm/s | 阈值=%.3f mm/s | 设定=%.3f mm/s\r\n",
                   (float)currentPos / 10.0f,
                   dt_s,
                   avg_speed_mm_s,
                   speed_threshold_mm_s,
                   set_speed_mm_s);
        }
    }

    return NO_ERROR;
}

/* ================== 无检测运行中：调试打印 ================== */

#ifndef NODETECT_LOG_PERIOD_MS
#define NODETECT_LOG_PERIOD_MS        100u   /* 打印周期：100ms */
#endif

#ifndef NODETECT_GYRO_PERIOD_MS
#define NODETECT_GYRO_PERIOD_MS       300u   /* 陀螺仪读取周期：300ms */
#endif

#ifndef NODETECT_DENS_PERIOD_MS
#define NODETECT_DENS_PERIOD_MS       500u   /* 密度读取周期：500ms */
#endif

static void NoDetect_RuntimeLogUpdate(void)
{
    static uint32_t last_log_tick  = 0;
    static uint32_t last_den_tick  = 0;

    uint32_t now = HAL_GetTick();

    /* 1) 位置/尺带/称重 */
    g_measurement.debug_data.current_weight = weight_parament.current_weight;
    g_measurement.debug_data.current_encoder_value = -g_encoder_count;

    /* 2) 陀螺仪 */

//    if ((now - last_gyro_tick) >= NODETECT_GYRO_PERIOD_MS) {
//        last_gyro_tick = now;
//        float ax = 0.0f, ay = 0.0f;
//        uint32_t ret = Read_Gyro_Angle(&ax, &ay);
//        if (ret == NO_ERROR) {
//            g_measurement.debug_data.angle_x = (int32_t)(ax * 100.0f);
//            g_measurement.debug_data.angle_y = (int32_t)(ay * 100.0f);
//        }
//    }

    /* 3) 密度/温度/频率 */
    if ((now - last_den_tick) >= NODETECT_DENS_PERIOD_MS) {
        last_den_tick = now;
        float f = 0.0f, d = 0.0f, t = 0.0f;
        uint32_t ret = Read_Density(&f, &d, &t);
        (void)f; (void)d; (void)t;
        if (ret == NO_ERROR) {
            /* Read_Density 内部写 debug_data.temperature/frequency 等 */
        }
    }

    /* 4) 节流打印 */
    if ((now - last_log_tick) < NODETECT_LOG_PERIOD_MS) {
        return;
    }
    last_log_tick = now;

    printf("无底运行 | 方向=%lu 称重=%lu 距离零点=%ld(0.1mm) 罐底距离=%ld "
           "| X轴（X100）=%ld Y轴（X100）=%ld | 密度=%lu 温度=%lu 频率=%lu\r\n",
           (unsigned long)g_measurement.debug_data.motor_state,
           (unsigned long)g_measurement.debug_data.current_weight,
           (long)g_measurement.debug_data.cable_length,
           (long)g_measurement.debug_data.sensor_position,
           (long)g_measurement.debug_data.angle_x,
           (long)g_measurement.debug_data.angle_y,
           (unsigned long)g_measurement.single_point_monitoring.density,
           (unsigned long)g_measurement.single_point_monitoring.temperature,
           (unsigned long)g_measurement.debug_data.frequency);
}

/* ===================== 无检测阻塞运动（可打断，void） ===================== */

void motorMoveBlocking_NoDetectWithSpeed(float mm, int dir, uint32_t speed_x100)
{
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    uint32_t ret;

    if (mm <= 0.0f) return;
    if (!Motor_IsDirValid(dir)) return;

    /* 该接口是“无检测”版本，只保留基础运动和日志。
     * 但为了让语义一致，临时速度恢复策略仍然与其他阻塞接口保持一致。 */
    ret = Motor_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：速度设置失败 错误码=0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    printf("无检测阻塞运动：距离=%.2f, 方向=%d\r\n", mm, dir);

    g_measurement.debug_data.motor_state = (dir == MOTOR_DIRECTION_UP) ? 1U : 2U;
    Motor_SyncDebugDrumState(&stepper);

    const double C0 = tape_C0_mm();
    const double t  = tape_t_mm();
    const double local_circumference_mm = Motor_GetLocalCircumferenceFromParams();
    const bool use_local_circ =
        (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
        (local_circumference_mm > 1e-6);
    if ((!use_local_circ) && (C0 <= 0.0)) {
        ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (ret != NO_ERROR) {
            printf("无检测阻塞运动：恢复默认速度失败 错误码=0x%08lX\r\n", (unsigned long)ret);
        }
        return;
    }

    /* 当前有符号长度 */
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = Motor_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_applied_velocity = velocity;

    /* 目标有符号长度 */
    double dL = (double)mm;
    if (dir == MOTOR_DIRECTION_UP) dL = -dL;
    const double Ltar_mm = Lcur_mm + dL;

    int64_t ticks64;

    if (use_local_circ) {
        ticks64 = llround_safe((dL / local_circumference_mm) *
                               (double)tape_ticks_per_rev());
    } else {
        const double ncur = tape_turns_from_signed_length(Lcur_mm, C0, t);
        const double ntar = tape_turns_from_signed_length(Ltar_mm, C0, t);
        const double dn   = ntar - ncur;
        ticks64 = llround_safe(dn * (double)tape_ticks_per_rev());
    }

    if (ticks64 > (int64_t)INT32_MAX) ticks64 = (int64_t)INT32_MAX;
    if (ticks64 < (int64_t)INT32_MIN) ticks64 = (int64_t)INT32_MIN;
    int32_t ticks = (int32_t)ticks64;

    if (Motor_StopIfCommandSwitchRequested()) {
        ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (ret != NO_ERROR) {
            printf("无检测阻塞运动：恢复默认速度失败 错误码=0x%08lX\r\n", (unsigned long)ret);
        }
        return;
    }
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：目标位置越界 错误码=0x%08lX\r\n", (unsigned long)ret);
        (void)Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        return;
    }
    Motor_SyncDebugDrumState(&stepper);
    HAL_Delay(100);

    uint32_t last_vel_refresh_tick = HAL_GetTick();
    while (stpr_isMoving(&stepper)) {
        if (Motor_StopIfCommandSwitchRequested()) {
            ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (ret != NO_ERROR) {
                printf("无检测阻塞运动：恢复默认速度失败 错误码=0x%08lX\r\n", (unsigned long)ret);
            }
            return;
        }
        Motor_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        Motor_SyncDebugDrumState(&stepper);
        NoDetect_RuntimeLogUpdate();
    }

    g_measurement.debug_data.motor_state = 0U;
    Motor_SyncDebugDrumState(&stepper);

    ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：恢复默认速度失败 错误码=0x%08lX\r\n", (unsigned long)ret);
    }
}

/* ===================== 预留接口 ===================== */

void motorRun(float distance_mm,
              int direction,
              bool enable_step_loss_detection,
              bool enable_weight_detection)
{
    (void)distance_mm;
    (void)direction;
    (void)enable_step_loss_detection;
    (void)enable_weight_detection;
    // TODO：实现综合运动逻辑
}
