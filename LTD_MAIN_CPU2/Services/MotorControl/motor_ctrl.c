/*
 * motor_ctrl.c
 *
 *  Created on: Jan 18, 2025
 *      Author: Duan Xuebin
 *
 * 说明：
 *  - 本文件用于尺带机构的步进电机控制（TMC5130）
 *  - 支持：精确长度换算下发、阻塞等待、碰撞/异常检测、丢步检测、卷筒圈数/角度计算、第一圈周长标定
 *  - 支持：命令切换打断（运行中检测 g_deviceParams.command != CMD_NONE 时立即停机并返回）
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
        if (g_deviceParams.command != CMD_NONE) {                                 \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            stpr_stop(&stepper);                                                  \
            return (retcode);                                                     \
        }                                                                         \
    } while (0)

/**
 * @brief void 函数：检测命令切换 -> 停止电机 -> return
 */
#define CHECK_COMMAND_SWITCH_AND_STOP_NO_RETURN()                                 \
    do {                                                                          \
        if (g_deviceParams.command != CMD_NONE) {                                 \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            stpr_stop(&stepper);                                                  \
            return;                                                               \
        }                                                                         \
    } while (0)

/* 前置声明：线速度->VMAX 反算与运行中刷新 */
static inline uint32_t Motor_ComputeUniformVelocityFromLength(double L_mm);
static inline void Motor_RefreshVelocityDuringRun(TMC5130TypeDef *tmc5130,
                                                  uint32_t *last_refresh_tick);

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

    printf("SpeedInit | cable=%.1f mm | VMAX=%lu | eq_usteps/s=%.1f\r\n",
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

static inline bool Motor_StopIfCommandSwitchRequested(void)
{
    if (g_deviceParams.command != CMD_NONE) {
        printf("检测到命令切换请求，停止当前操作\r\n");
        stpr_stop(&stepper);
        return true;
    }
    return false;
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
    const double C_cur = tape_instant_circumference_mm_from_length(L_mm);

    if (C_cur > 1e-6) {
        const double ticks_per_rev = (double)tape_ticks_per_rev();

        /* 目标输出轴微步速度：usteps/s
           线速度(mm/s) = speed_x100 / 6
           rev/s       = (speed_x100 / 6) / C_cur
           usteps/s    = rev/s * ticks_per_rev
                       = speed_x100 * ticks_per_rev / (6*C_cur) */
        const double usteps_per_s =
            ((double)speed_x100 * ticks_per_rev) / (6.0 * C_cur);

        const uint32_t vmax = TMC5130_UstepsPerSec_To_VMAX(usteps_per_s);

        printf("SpeedCalc | set=%.2f m/min | L=%.1f mm | C=%.2f mm | usteps/s=%.1f | VMAX=%lu\r\n",
               speed_x100 / 100.0,
               L_mm,
               C_cur,
               usteps_per_s,
               (unsigned long)vmax);

        return vmax;
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
    const double Lcur_mm = (double)drum.motor_distance_01mm * 0.1;

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

        printf("SpeedRefresh | L=%.1f mm | oldVMAX=%lu | newVMAX=%lu | old_usps=%.1f | new_usps=%.1f\r\n",
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
    const uint32_t requested_speed = speed_x100;
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
        Lcur_mm = (double)drum.motor_distance_01mm * 0.1;
    }

    /* 把“线速度设定”转换成当前卷径下的 TMC5130 VMAX。 */
    velocity = Motor_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_applied_velocity = velocity;

    if (is_running) {
        /* 运行中改速：立即写入驱动，让本次运动立刻生效。 */
        stpr_setVelocity(&stepper, velocity);
        printf("SpeedSet | running | req=%.2f m/min | set=%.2f m/min | L=%.1f mm | VMAX=%lu | eq_usteps/s=%.1f\r\n",
               requested_speed / 100.0,
               clamped_speed / 100.0,
               Lcur_mm,
               (unsigned long)velocity,
               TMC5130_VMAX_To_UstepsPerSec(velocity));
    } else {
        /* 未运行时只更新内部速度状态，供下一次运动使用。 */
        printf("SpeedSet | idle | req=%.2f m/min | set=%.2f m/min | baseL=%.1f mm | nextVMAX=%lu | eq_usteps/s=%.1f\r\n",
               requested_speed / 100.0,
               clamped_speed / 100.0,
               Lcur_mm,
               (unsigned long)velocity,
               TMC5130_VMAX_To_UstepsPerSec(velocity));
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

/**
 * @brief 从 TMC5130_XACTUAL 计算卷筒总圈数、圈内角度、预测长度（新模型）
 */
void Motor_UpdateDrumState_FromXACTUAL(TMC5130TypeDef *tmc5130,
                                      MotorDrumState *out)
{
    if (!out || !tmc5130) return;

    /* 1) 读取当前 XACTUAL（微步 ticks，含符号） */
    const int32_t motor_step = stpr_readInt(tmc5130, TMC5130_XACTUAL);

    /* 2) ticks -> 输出轴圈数（含小数） */
    const double turns = (double)motor_step / (double)tape_ticks_per_rev();

    /* 3) 圈内角度（归一到 [0,1)） */
    int32_t turns_int = (int32_t)floor(turns);
    double frac = turns - (double)turns_int;
    if (frac < 0.0) frac += 1.0;
    if (frac >= 1.0) frac -= floor(frac);

    const double angle_deg = frac * 360.0;

    /* 4) 由 turns 预测长度（单位：mm） */
    const double C0 = tape_C0_mm();
    const double t  = tape_t_mm();

    double L_mm;
    if (t > 0.0) {
        L_mm = tape_signed_length_from_turns(turns, C0, t);
    } else {
        L_mm = C0 * turns;
    }

    /* 5) 输出 */
    out->motor_step          = motor_step;
    out->turns_total         = turns;
    out->turns_int           = turns_int;
    out->angle_deg           = angle_deg;
    out->motor_distance_01mm = (int32_t)llround(L_mm * 10.0); // mm -> 0.1mm
}

/* ===================== 初始化 ===================== */

uint32_t motor_Init(void)
{
    if (g_measurement.debug_data.motor_speed == 0U) {
        g_measurement.debug_data.motor_speed = g_deviceParams.max_motor_speed;
    } else {
        g_measurement.debug_data.motor_speed =
            Motor_ClampSpeedSetpointX100(g_measurement.debug_data.motor_speed);
    }

    Motor_UpdateVelocityFromParams();
    s_motor_applied_velocity = velocity;

    stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 14);
//    stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 16);
    stpr_enableDriver(&stepper);
    s_motor_initialized = true;

    printf("电机初始化 | speed_set=%.2f m/min | cable=%.1f mm | VMAX=%lu | eq_usteps/s=%.1f\r\n",
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
        printf("TMC5130: Charge pump 欠压\r\n");
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
        HAL_Delay(poll_ms);
    }
    return NO_ERROR;
}

/* ===================== ticks 运动封装（阻塞，可打断） ===================== */

uint32_t motorMoveWaitByTicksWithSpeed(int32_t ticks, uint32_t speed_x100)
{
    uint32_t ret = NO_ERROR;
    uint32_t restore_ret = NO_ERROR;
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;

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

    s_motor_applied_velocity = velocity;
    stpr_moveBy(&stepper, &ticks, velocity);

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
    if (move_mm < 0.0f) return MOTOR_STEP_ERROR;
    if (move_mm == 0.0f) return NO_ERROR;
    if (!Motor_IsDirValid(dir)) return MOTOR_STEP_ERROR;

    uint32_t ret = Motor_ApplyOptionalSpeed(speed_x100);
    CHECK_ERROR(ret);

    g_measurement.debug_data.motor_state = dir;

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

    /* 3) 模型参数 */
    const double C0 = tape_C0_mm();
    const double t  = tape_t_mm();
    if (C0 <= 0.0) return MOTOR_STEP_ERROR;

    /* 4) 有符号长度 -> 有符号圈数（允许负值） */
    const double ncur = tape_turns_from_signed_length(Lcur_mm, C0, t);
    const double ntar = tape_turns_from_signed_length(Ltar_mm, C0, t);
    const double dn   = ntar - ncur;

    /* 5) 圈数 -> ticks */
    double ticks_d  = dn * (double)tape_ticks_per_rev();
    int64_t ticks64 = llround_safe(ticks_d);

    if (ticks64 > (int64_t)INT32_MAX) ticks64 = (int64_t)INT32_MAX;
    if (ticks64 < (int64_t)INT32_MIN) ticks64 = (int64_t)INT32_MIN;

    int32_t ticks = (int32_t)ticks64;

    /* 6) 下发运动 */
    stpr_moveBy(&stepper, &ticks, velocity);

    return NO_ERROR;
}

/* ===================== 第一圈周长标定（可打断，新模型） ===================== */

uint32_t CalibrateFirstLoopCircumference_OneTurnAtZero(void)
{
    const double t = tape_t_mm();
    if (t <= 0.0) {
        return PARAM_ERROR;
    }

    const int32_t one_rev_ticks = tape_ticks_per_rev();
    if (one_rev_ticks <= 0) {
        return PARAM_ERROR;
    }

    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 1) 记录零点长度 */
    const double L0 = get_current_tape_length_mm();

    /* 2) 下行一圈并等待停止（可打断） */
    uint32_t ret = motorMoveWaitByTicksWithSpeed(one_rev_ticks, motorGetDefaultSpeedX100());
    if (ret != NO_ERROR) {
        return ret;
    }

    HAL_Delay(200);

    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 3) 读取一圈后的长度 */
    const double L1 = get_current_tape_length_mm();
    const double dL = L1 - L0;
    if (dL <= 0.0) {
        return PARAM_ERROR;
    }

    /* 新模型：第一圈放出长度：dL = C0 - pi*t  =>  C0 = dL + pi*t */
    const double C0 = dL + (1.0 * M_PI * t);

    if (!(C0 > C0_MIN_MM && C0 < C0_MAX_MM)) {
        return PARAM_ERROR;
    }

    g_deviceParams.first_loop_circumference_mm = (int32_t)llround(C0 * 10.0);
    ret = motorMoveAndWaitUntilStopWithSpeed(dL, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100()); // 回到起点
    CHECK_ERROR(ret);
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

        /* 1) 当前位置（mm） */
        float cur_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

        /* 2) 到位（容差）-> 立即停机并返回成功 */
        if (fabsf(cur_mm - target_mm) <= eps_mm) {
            stpr_stop(&stepper);
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
                stpr_stop(&stepper);
                while (stpr_isMoving(tmc5130)) {
                    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
                    HAL_Delay(5);
                }
                return NO_ERROR;
            }
        } else {
            if (cur_mm >= target_mm) {
                stpr_stop(&stepper);
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
        return MOTOR_STEP_ERROR;
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
            printf("电机段运行成功: attempt=%d, 累计位移=%.2f mm, 剩余=%.2f mm\r\n",
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
        ret = ENCODER_LOST_STEP;
    } else {
        printf("电机移动完成。\t");
        printf("起点=%.2f mm, 目标=%.2f mm, 最终=%.2f mm\t",
               startPos_mm, targetPos_mm, currentPos_mm);
        printf("期望=%.2f mm, 实际=%.2f mm, 偏差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        ret = NO_ERROR;
    }

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

        /* 用户条件优先级最高。
         * 一旦条件满足，立即停机并以“正常到达条件”返回。 */
        if (condition_fn(user_ctx)) {
            stpr_stop(&stepper);
            while (stpr_isMoving(&stepper)) {
                HAL_Delay(5);
            }
            met = true;
            break;
        }

        /* 条件未满足时，仍要继续执行通用保护。 */
        ret = CheckWeightCollision();
        if (ret != NO_ERROR) {
            stpr_stop(&stepper);
            while (stpr_isMoving(&stepper)) {
                HAL_Delay(5);
            }
            break;
        }

        ret = stpr_checkGstat(&stepper);
        if (ret != NO_ERROR) {
            stpr_stop(&stepper);
            while (stpr_isMoving(&stepper)) {
                HAL_Delay(5);
            }
            break;
        }

        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            stpr_stop(&stepper);
            while (stpr_isMoving(&stepper)) {
                HAL_Delay(5);
            }
            ret = MOTOR_RUN_TIMEOUT;
            break;
        }

        HAL_Delay(poll_ms);
    }

    if (condition_met) {
        *condition_met = met;
    }

    /* ret==NO_ERROR 但 met==false，说明：
     * 电机已经自然停下，但并不是因为条件回调触发。 */
    if ((ret == NO_ERROR) && (!met)) {
        printf("motorMoveUntilCondition: 运动结束，但条件未满足 | max_mm=%.2f | dir=%d\r\n",
               max_mm, dir);
    }

    restore_ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
    if (ret != NO_ERROR) {
        return ret;
    }
    CHECK_ERROR(restore_ret);
    return restore_ret;
}

/* ===================== 急停/停机 ===================== */

uint32_t motorQuickStop(void)
{
    if (stpr_isMoving(&stepper)) {
        stpr_stop(&stepper);
        stpr_disableDriver(&stepper);
        HAL_Delay(4000);
        stpr_enableDriver(&stepper);
    }
    return motorSetSpeed(g_deviceParams.max_motor_speed);
}

uint32_t motorSlowStop(void)
{
    stpr_stop(&stepper);
    return motorSetSpeed(g_deviceParams.max_motor_speed);
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
    static uint32_t last_gyro_tick = 0;
    static uint32_t last_den_tick  = 0;

    uint32_t now = HAL_GetTick();

    /* 1) 位置/尺带/称重 */
    g_measurement.debug_data.current_weight = weight_parament.current_weight;
    g_measurement.debug_data.current_encoder_value = g_encoder_count;

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

    printf("ND_RUN | 方向=%lu 称重=%lu 距离零点=%ld(0.1mm) 罐底距离=%ld "
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
        printf("无检测阻塞运动：速度设置失败 ret=0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    printf("无检测阻塞运动：mm=%.2f, dir=%d\r\n", mm, dir);

    g_measurement.debug_data.motor_state = dir;

    const double C0 = tape_C0_mm();
    const double t  = tape_t_mm();
    if (C0 <= 0.0) {
        ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (ret != NO_ERROR) {
            printf("无检测阻塞运动：恢复默认速度失败 ret=0x%08lX\r\n", (unsigned long)ret);
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

    /* 有符号长度->有符号圈数 */
    const double ncur = tape_turns_from_signed_length(Lcur_mm, C0, t);
    const double ntar = tape_turns_from_signed_length(Ltar_mm, C0, t);
    const double dn   = ntar - ncur;

    int64_t ticks64 = llround_safe(dn * (double)tape_ticks_per_rev());
    if (ticks64 > (int64_t)INT32_MAX) ticks64 = (int64_t)INT32_MAX;
    if (ticks64 < (int64_t)INT32_MIN) ticks64 = (int64_t)INT32_MIN;
    int32_t ticks = (int32_t)ticks64;

    if (Motor_StopIfCommandSwitchRequested()) {
        ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (ret != NO_ERROR) {
            printf("无检测阻塞运动：恢复默认速度失败 ret=0x%08lX\r\n", (unsigned long)ret);
        }
        return;
    }
    stpr_moveBy(&stepper, &ticks, velocity);
    HAL_Delay(100);

    uint32_t last_vel_refresh_tick = HAL_GetTick();
    while (stpr_isMoving(&stepper)) {
        if (Motor_StopIfCommandSwitchRequested()) {
            ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (ret != NO_ERROR) {
                printf("无检测阻塞运动：恢复默认速度失败 ret=0x%08lX\r\n", (unsigned long)ret);
            }
            return;
        }
        Motor_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        NoDetect_RuntimeLogUpdate();
    }

    ret = Motor_EndTemporarySpeed(restore_needed, restore_speed_x100);
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：恢复默认速度失败 ret=0x%08lX\r\n", (unsigned long)ret);
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
