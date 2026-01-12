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
 *     - MOTOR_DIRECTION_DOWN：下行放带，cable_length 增加
 *     - MOTOR_DIRECTION_UP  ：上行收带，cable_length 减少
 *  2) 单位：
 *     - cable_length：0.1mm
 *     - sensor_position：0.1mm
 *     - first_loop_circumference_mm：0.1mm
 *     - tape_thickness_mm：0.001mm
 *  3) 卷筒模型：
 *     - L(n) = C0*n + π*t*n^2
 *       C0(mm)=first_loop_circumference_mm*0.1
 *       t(mm) =tape_thickness_mm*0.001
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

/* ===================== 常量/配置 ===================== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EPS_MM                   0.20f    // 到位公差(±mm)

/* 丢步检测参数（单位见注释） */
#define LOST_STEP_WINDOW         3        // 检测窗口（3次）
#define LOST_STEP_THRESHOLD      2        // 3秒内总位移小于0.2mm判定丢步（单位：0.1mm）
#define LOST_STEP_INTERVAL_MS    1000     // 每1秒检测一次
#define LOST_STEP_SUPPRESS_MS    1000     // 报警后1秒内不重复报警
#define MOTOR_MOVE_RETRY_MAX     3        // 最大重试次数，包含初次+补偿重跑

/* 周长标定结果合理性范围（按机构实际可调整） */
#ifndef C0_MIN_MM
#define C0_MIN_MM (50.0)
#endif
#ifndef C0_MAX_MM
#define C0_MAX_MM (5000.0)
#endif

/* 速度基准：与原工程保持一致（1600 step/rev + 32 microstep） */
#define MOTOR_VELOCITY_BASE     (1600UL * 32UL)

/* 可选：给速度做上下限保护（按实际机构能力调整） */
#ifndef MOTOR_SPEED_RATIO_MIN_X100
#define MOTOR_SPEED_RATIO_MIN_X100   10     // 最小 0.10 倍（=10/100）
#endif
#ifndef MOTOR_SPEED_RATIO_MAX_X100
#define MOTOR_SPEED_RATIO_MAX_X100   600    // 最大 6.00 倍（=600/100）
#endif

/* ===================== 命令切换打断（新增） ===================== */

/* 命令切换导致的中断错误码 */
#ifndef COMMAND_SWITCH_ABORT
#define COMMAND_SWITCH_ABORT   STATE_SWITCH
#endif

/**
 * @brief 非 void 函数：检测命令切换 -> 停止电机 -> 返回 retcode
 *
 * 使用场景：
 *  - uint32_t 返回值函数（motorMoveAndWaitUntilStop / stpr_wait_until_stop 等）
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
 *
 * 使用场景：
 *  - void 返回值函数（motorMoveBlocking_NoDetect 等）
 */
#define CHECK_COMMAND_SWITCH_AND_STOP_NO_RETURN()                                 \
    do {                                                                          \
        if (g_deviceParams.command != CMD_NONE) {                                 \
            printf("检测到命令切换请求，停止当前操作\r\n");                       \
            stpr_stop(&stepper);                                                  \
            return;                                                               \
        }                                                                         \
    } while (0)

/* ===================== 内部状态变量 ===================== */

/* 丢步检测环形缓冲区：记录最近 3 次位置（单位：0.1mm） */
static int pos_buf[LOST_STEP_WINDOW];
static int write_idx = 0;
static int samples = 0;
static uint32_t last_check_tick = 0;
static uint32_t last_alarm_tick = 0;

/* 全局速度：由 Motor_UpdateVelocityFromParams() 根据参数动态更新 */
uint32_t velocity = MOTOR_VELOCITY_BASE;  // 先给一个默认值，真正值在 Init/运行中更新

/* 前置声明 */
static uint32_t stpr_wait_until_stop(TMC5130TypeDef *tmc5130);

/* ===================== 速度控制（由参数决定） ===================== */
/**
 * @brief 根据 g_deviceParams.max_motor_speed 更新全局 velocity
 *
 * 参数说明：
 *  - g_deviceParams.max_motor_speed 为 100 倍缩放：
 *      100 -> 1.00 倍
 *      200 -> 2.00 倍
 *      150 -> 1.50 倍
 *
 * 计算：
 *  - velocity = MOTOR_VELOCITY_BASE * max_motor_speed / 100
 *
 * 保护：
 *  - ratio 过小会导致几乎不动；ratio 过大可能超过驱动/机构能力
 *  - 这里做了 min/max 钳位，你可按现场测试调节
 */
static inline void Motor_UpdateVelocityFromParams(void)
{
    int32_t ratio_x100 = (int32_t)g_deviceParams.max_motor_speed;

    /* 合理性钳位 */
    if (ratio_x100 < MOTOR_SPEED_RATIO_MIN_X100) {
        ratio_x100 = MOTOR_SPEED_RATIO_MIN_X100;
    } else if (ratio_x100 > MOTOR_SPEED_RATIO_MAX_X100) {
        ratio_x100 = MOTOR_SPEED_RATIO_MAX_X100;
    }

    /* 四舍五入：+50 实现 /100 的四舍五入 */
    uint64_t v = (uint64_t)MOTOR_VELOCITY_BASE * (uint64_t)ratio_x100 + 50ULL;
    v /= 100ULL;

    /* 再做一次下限保护，避免 0 */
    if (v < 1ULL) v = 1ULL;

    velocity = (uint32_t)v;
}

/* ===================== 方向统一工具 ===================== */

/**
 * @brief 检查方向是否合法：只允许 UP/DOWN 两种方向
 */
static inline int Motor_IsDirValid(int dir)
{
    return (dir == MOTOR_DIRECTION_UP) || (dir == MOTOR_DIRECTION_DOWN);
}

/* ===================== 尺带/卷筒模型工具 ===================== */

/**
 * @brief 卷筒输出轴每转对应的 ticks（微步数）
 *
 * 计算来源：
 *  - 减速比：30
 *  - 电机每圈步数：1600
 *  - 细分：32
 *  => 输出轴每圈 ticks = 30 * 1600 * 32 = 1,536,000
 */
static inline int32_t tape_ticks_per_rev(void)
{
    return (int32_t)(30L * 1600L * 32L);  // 1,536,000
}

/**
 * @brief 第一圈周长 C0（单位：mm）
 *        参数存储单位为 0.1mm，换算为 mm
 */
static inline double tape_C0_mm(void)
{
    return (double)g_deviceParams.first_loop_circumference_mm * 0.1;
}

/**
 * @brief 尺带厚度 t（单位：mm）
 *        参数存储单位为 0.001mm，换算为 mm
 */
static inline double tape_t_mm(void)
{
    return (double)g_deviceParams.tape_thickness_mm * 0.001;
}

/**
 * @brief 卷筒模型：给定圈数 n（可为小数），计算放带长度 L（单位：mm）
 *        L(n) = C0*n + π*t*n^2
 *
 * 说明：
 *  - n=0 时 L=0
 *  - n 增大时，因卷筒直径逐渐变大，长度增长呈二次项
 */
static inline double tape_length_from_turns(double n, double C0_mm, double t_mm)
{
    return (C0_mm * n) + (M_PI * t_mm * n * n);
}

/**
 * @brief 模型反解：给定长度 L（mm），反解圈数 n
 *        n(L)=(-C0 + sqrt(C0^2 + 4*pi*t*L)) / (2*pi*t)
 *
 * 边界：
 *  - L<=0 返回 0
 *  - t<=0 退化为 L=C0*n => n=L/C0
 */
static inline double tape_turns_from_length(double L_mm, double C0_mm, double t_mm)
{
    if (L_mm <= 0.0) return 0.0;
    if (C0_mm <= 0.0) return 0.0;

    if (t_mm <= 0.0) {
        return L_mm / C0_mm;
    }

    const double a = M_PI * t_mm;
    const double b = C0_mm;
    const double disc = b*b + 4.0*a*L_mm;

    if (disc <= 0.0) return 0.0;

    return (-b + sqrt(disc)) / (2.0*a);
}
/**
 * @brief 有符号长度 L(mm) -> 有符号圈数 n
 *
 * 约定：
 *  - L > 0：下行放带（基准点以下）
 *  - L < 0：上行收带超过基准点（缠绕到零点以上）
 *
 * 做法：
 *  - 先对 |L| 用原模型反解得到 |n|
 *  - 再乘 sign(L) 得到 n 的符号
 */
static inline double tape_turns_from_signed_length(double L_mm,
                                                   double C0_mm,
                                                   double t_mm)
{
    if (L_mm == 0.0) {
        return 0.0;
    }

    const double s = (L_mm > 0.0) ? 1.0 : -1.0;
    const double n_abs = tape_turns_from_length(fabs(L_mm), C0_mm, t_mm);
    return s * n_abs;
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
 *        来源：g_measurement.debug_data.cable_length（0.1mm）
 *
 * 说明：
 *  - 此量建议来自编码器/长度计算，代表更接近物理真实的长度
 */
static inline double get_current_tape_length_mm(void)
{
    return (double)g_measurement.debug_data.cable_length * 0.1;
}

/* ===================== 卷筒状态结构体（你原来是“测试用”，保留） ===================== */

/**
 * @brief 从 TMC5130_XACTUAL 计算卷筒总圈数、圈内角度、预测长度
 *
 * 注意：
 *  - XACTUAL 是驱动器内部位置，无法感知机械打滑
 *  - 回零后建议 stpr_setPos(&stepper,0) 将 XACTUAL 清零，保证“相对零点”的含义成立
 */
void Motor_UpdateDrumState_FromXACTUAL(TMC5130TypeDef *tmc5130,
                                      MotorDrumState *out)
{
    if (!out || !tmc5130) return;

    /* 1) 读取当前 XACTUAL（微步 ticks，含符号） */
    const int32_t motor_step = stpr_readInt(tmc5130, TMC5130_XACTUAL);

    /* 2) ticks -> 输出轴圈数（含小数） */
    const double turns = (double)motor_step / (double)tape_ticks_per_rev();

    /* 3) 计算圈内角度（处理负数方向，归一到 [0,1)） */
    int32_t turns_int = (int32_t)floor(turns);
    double frac = turns - (double)turns_int;
    if (frac < 0.0) frac += 1.0;
    if (frac >= 1.0) frac -= floor(frac);

    const double angle_deg = frac * 360.0;

    /* 4) 由 turns 预测长度（单位：mm） */
    const double C0 = tape_C0_mm();
    const double t  = tape_t_mm();
    double L_mm = (t > 0.0) ? tape_length_from_turns(turns, C0, t)
                            : (C0 * turns);

    /* 5) 输出 */
    out->motor_step          = motor_step;
    out->turns_total         = turns;
    out->turns_int           = turns_int;
    out->angle_deg           = angle_deg;
    out->motor_distance_01mm = (int32_t)llround(L_mm * 10.0); // mm -> 0.1mm
}

/* ===================== 初始化 ===================== */

/**
 * @brief 电机初始化：初始化驱动、使能驱动
 */
uint32_t motor_Init(void)
{
    /* 先根据参数更新速度 */
    Motor_UpdateVelocityFromParams();

    stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 14);
    stpr_enableDriver(&stepper);

    printf("电机初始化 | max_motor_speed=%ld(×0.01) | velocity=%lu\r\n",
           (long)g_deviceParams.max_motor_speed, (unsigned long)velocity);

    return NO_ERROR;
}

/* ===================== 运动状态判断/芯片异常检查 ===================== */

/**
 * @brief 判断电机是否仍在运动
 * @return true 仍运动；false 已停止
 */
bool stpr_isMoving(TMC5130TypeDef *tmc5130)
{
    uint32_t rampstat = stpr_readInt(tmc5130, TMC5130_RAMPSTAT);
    return ((rampstat & 0x400) != 0x400); // bit10=1 表示停止
}

/**
 * @brief 检查 TMC5130 的 GSTAT 状态寄存器并处理错误
 */
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

/**
 * @brief 等待电机停止（可被命令切换打断）
 *
 * 说明：
 *  - 不做碰撞/异常检测，仅做：运动状态轮询 + 命令切换打断
 *  - 用于“无检测阻塞等待”或替代 stpr_waitMove() 的库等待
 *
 * @param poll_ms 轮询周期（ms），建议 5~20ms
 * @return NO_ERROR / COMMAND_SWITCH_ABORT
 */
static uint32_t motor_wait_stop_abortable(uint32_t poll_ms)
{
    while (stpr_isMoving(&stepper)) {
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
        HAL_Delay(poll_ms);
    }
    return NO_ERROR;
}

/* ===================== ticks 运动封装（阻塞，可打断） ===================== */

/**
 * @brief 按 ticks 运动并等待结束（可被命令切换打断）
 *
 * 重要说明：
 *  - ticks 正方向是否等于“下行放带”，需要你现场确认
 *  - 若确认方向相反，建议只在这里统一翻转 ticks（不要分散到各处）
 */
uint32_t motorMoveWaitByTicks(int32_t ticks)
{
    /* 运行时更新速度 */
    Motor_UpdateVelocityFromParams();

    /* 下发前先检查一次命令切换，避免刚切换仍下发运动 */
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 若方向相反，仅需在此统一翻转 */
    /* ticks = -ticks; */

    stpr_moveBy(&stepper, &ticks, velocity);

    /* 等待完成（可打断） */
    return motor_wait_stop_abortable(10);
}

/* ===================== 精确长度换算下发（不等待） ===================== */

/**
 * @brief 尺带机构：按“长度增量(mm)”精确换算 ticks 并下发（不等待）
 *
 * 注意：
 *  - 不等待、不检测
 *  - 仅负责换算与下发
 */
uint32_t motorMoveNoWait(float move_mm, int dir)
{
    Motor_UpdateVelocityFromParams();

    if (move_mm < 0.0f) return MOTOR_STEP_ERROR;
    if (move_mm == 0.0f) return NO_ERROR;
    if (!Motor_IsDirValid(dir)) return MOTOR_STEP_ERROR;

    g_measurement.debug_data.motor_state = dir;

    /* 1) 当前“相对基准点”的有符号长度（mm，可正可负） */
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;

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


/* ===================== 第一圈周长标定（可打断） ===================== */

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

    /* 标定开始前检查命令切换 */
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 1) 记录零点长度 */
    const double L0 = get_current_tape_length_mm();

    /* 2) 下行一圈并等待停止（可打断） */
    uint32_t ret = motorMoveWaitByTicks(one_rev_ticks);
    if (ret != NO_ERROR) {
        return ret;
    }

    HAL_Delay(200); // 让采样稳定

    /* 标定过程中也允许打断 */
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 3) 读取一圈后的长度 */
    const double L1 = get_current_tape_length_mm();
    const double dL = L1 - L0;
    if (dL <= 0.0) {
        return PARAM_ERROR;
    }

    /* 4) 计算 C0（mm）并做范围校验：dL = C0 + pi*t */
    const double C0 = dL - (M_PI * t);
    if (!(C0 > C0_MIN_MM && C0 < C0_MAX_MM)) {
        return PARAM_ERROR;
    }

    /* 5) 写入参数（mm -> 0.1mm） */
    g_deviceParams.first_loop_circumference_mm = (int32_t)llround(C0 * 10.0);

    return NO_ERROR;
}

/* ===================== 带检测的等待停止（可打断，强制停机返回） ===================== */

/**
 * @brief 等待电机停止（阻塞），期间检测碰撞与芯片异常，并支持命令切换打断
 */
static uint32_t stpr_wait_until_stop(TMC5130TypeDef *tmc5130)
{
    uint32_t ret = NO_ERROR;
    uint32_t startTick = HAL_GetTick();
    const uint32_t MAX_WAIT_MS = 60000 * 60; // 超时时间：60s*60（按需调整）

    while (stpr_isMoving(tmc5130)) {

        /* 0) 命令切换打断（最高优先级） */
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

        /* 1) 碰撞/极限检测 */
        ret = CheckWeightCollision();
        CHECK_ERROR(ret);

        /* 2) 芯片异常检测 */
        ret = stpr_checkGstat(tmc5130);
        CHECK_ERROR(ret);

        /* 3) 超时保护 */
        if (HAL_GetTick() - startTick > MAX_WAIT_MS) {
            printf("TMC5130: 等待停止超时！\r\n");
            RETURN_ERROR(MOTOR_RUN_TIMEOUT);
        }

        /* 4) 你原本保留的罐底检测 */
        check_bottom_status();

        HAL_Delay(50);
    }

    printf("TMC5130: 已停止。\r\n");
    return NO_ERROR;
}

/* ===================== 高层运动：分段+等待+保护（可打断） ===================== */

uint32_t motorMoveAndWaitUntilStop(float mm, int dir)
{
    uint32_t ret;
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

    /* 开始前检查命令切换 */
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    startPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

    targetPos_mm = startPos_mm +
                   ((dir == MOTOR_DIRECTION_DOWN) ? total_cmd_mm : -total_cmd_mm);

    remain_mm = total_cmd_mm;

    while (attempt < MOTOR_MOVE_RETRY_MAX && remain_mm > 0.0f) {

        attempt++;

        /* 每次下发前都允许打断 */
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

        motorMoveNoWait(remain_mm, dir);

        /* 起步延时期间也允许打断 */
        for (int i = 0; i < 100; i++) {   // 100 * 10ms = 1000ms
            CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
            HAL_Delay(10);
        }

        ret = stpr_wait_until_stop(&stepper);

        /* wait 返回后再次检查（避免继续补偿） */
        if (ret == COMMAND_SWITCH_ABORT) {
            return ret;
        }
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

        currentPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
        moved_mm = fabsf(currentPos_mm - startPos_mm);
        remain_mm = total_cmd_mm - moved_mm;
        if (remain_mm < 0.0f) {
            remain_mm = 0.0f;
        }

        if (ret == NO_ERROR) {
            printf("电机段运行成功: attempt=%d, 累计位移=%.2f mm, 剩余=%.2f mm\r\n",
                   attempt, moved_mm, remain_mm);

            MotorDrumState drum;
            Motor_UpdateDrumState_FromXACTUAL(&stepper, &drum);
            printf("电机状态 | XACTUAL=%ld | 圈=%.4f | 角度=%.1f° | 预测长度=%.1fmm\r\n",
                   (long)drum.motor_step,
                   drum.turns_total,
                   drum.angle_deg,
                   drum.motor_distance_01mm / 10.0);

            break;
        } else {

            printf("警告：电机在第%d次运行过程中异常停止，错误0x%X\r\n",
                   attempt, (unsigned int)ret);

            if (attempt >= MOTOR_MOVE_RETRY_MAX) {
                CHECK_ERROR(ret);
                return ret;
            }

            printf("尝试继续补偿剩余距离 %.2f mm (方向=%s)\r\n",
                   remain_mm,
                   (dir == MOTOR_DIRECTION_DOWN) ? "下行" : "上行");

            /* 补偿前延时也允许打断 */
            for (int i = 0; i < 20; i++) {  // 20*10ms=200ms
                CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
                HAL_Delay(10);
            }
        }
    }

    currentPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
    moved_mm = fabsf(currentPos_mm - startPos_mm);

    float diff_pct = 100.0f * (moved_mm - total_cmd_mm) / total_cmd_mm;

    if ((diff_pct < -70.0f) && (total_cmd_mm > 2.0f)) {
        printf("警告：检测到电机可能丢步！\r\n");
        printf("目标=%.2f mm, 实际=%.2f mm, 误差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        return ENCODER_LOST_STEP;
    } else {
        printf("电机移动完成。\t");
        printf("起点=%.2f mm, 目标=%.2f mm, 最终=%.2f mm\t",
               startPos_mm, targetPos_mm, currentPos_mm);
        printf("期望=%.2f mm, 实际=%.2f mm, 偏差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
    }

    return NO_ERROR;
}

/* ===================== 长距离运行（不阻塞，不做等待；需要你上层自己控制停止） ===================== */

uint32_t motorMove_up(void)
{
    /* 下发前检查是否命令已切换（避免误启动） */
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    motorMoveNoWait(200, MOTOR_DIRECTION_UP);
    return NO_ERROR;
}

uint32_t motorMove_down(void)
{
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    motorMoveNoWait(200, MOTOR_DIRECTION_DOWN);
    MotorLostStep_Init();
    return NO_ERROR;
}

/* ===================== 运动辅助：位置快照与到位控制（可打断） ===================== */

/**
 * @brief 读取当前位置快照（单位：mm）
 *        来源：sensor_position（0.1mm）
 */
void snapshot_sensor_pos_mm(float *pos_mm)
{
    uint32_t pos_01mm = (uint32_t)g_measurement.debug_data.sensor_position;
    *pos_mm = (float)pos_01mm / 10.0f;
}

/**
 * @brief 一次下发移动到绝对位置（阻塞，可打断）
 */
uint32_t motorMoveToPositionOneShot(float target_mm)
{
    uint32_t ret;
    float cur_mm;

    for (int i = 0; i < 10; i++) {

        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

        snapshot_sensor_pos_mm(&cur_mm);
        printf("运动到位置 | 当前=%.3fmm | 目标=%.3fmm\r\n", cur_mm, target_mm);

        float delta = target_mm - cur_mm;

        if (fabsf(delta) <= EPS_MM) {
            printf("到位 | 当前=%.3fmm ≈ 目标=%.3fmm (±%.2fmm)\r\n",
                   cur_mm, target_mm, EPS_MM);
            return NO_ERROR;
        }

        int dir = (delta > 0.0f) ? MOTOR_DIRECTION_DOWN : MOTOR_DIRECTION_UP;

        float plan_mm = fabsf(delta);
        if (plan_mm < (EPS_MM * 2.0f)) {
            plan_mm = (EPS_MM * 2.0f);
        }

        ret = motorMoveAndWaitUntilStop(plan_mm, dir);
        if (ret != NO_ERROR) {
            return ret; // 含 COMMAND_SWITCH_ABORT
        }
    }

    return NO_ERROR;
}

/**
 * @brief 相对位移一次下发（阻塞，可打断）
 *
 * @param mm 正数表示下行（长度增加），负数表示上行（长度减少）
 */
uint32_t motorMoveRelativeOneShot(float mm)
{
    float cur_mm;
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
    snapshot_sensor_pos_mm(&cur_mm);
    return motorMoveToPositionOneShot(cur_mm + mm);
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
    return NO_ERROR;
}

uint32_t motorSlowStop(void)
{
    stpr_stop(&stepper);
    return NO_ERROR;
}

/* ===================== 丢步检测 ===================== */

void MotorLostStep_Init(void)
{
    write_idx = 0;
    samples = 0;
    last_check_tick = 0;
    last_alarm_tick = 0;
    for (int i = 0; i < LOST_STEP_WINDOW; i++) {
        pos_buf[i] = 0;
    }
}

uint32_t Motor_CheckLostStep_AutoTiming(int32_t currentPos)
{
    uint32_t now = HAL_GetTick();

    if (now - last_check_tick < LOST_STEP_INTERVAL_MS)
        return NO_ERROR;

    last_check_tick = now;

    pos_buf[write_idx] = currentPos;
    write_idx = (write_idx + 1) % LOST_STEP_WINDOW;
    if (samples < LOST_STEP_WINDOW)
        samples++;

    if (samples < LOST_STEP_WINDOW)
        return NO_ERROR;

    int total_movement = 0;
    int oldest_idx = write_idx;

    for (int k = 0; k < LOST_STEP_WINDOW - 1; k++) {
        int idx_a = (oldest_idx + k) % LOST_STEP_WINDOW;
        int idx_b = (oldest_idx + k + 1) % LOST_STEP_WINDOW;
        total_movement += abs(pos_buf[idx_b] - pos_buf[idx_a]);
    }

    if ((total_movement < LOST_STEP_THRESHOLD) &&
        (g_measurement.debug_data.cable_length > 1000) &&
        (g_measurement.debug_data.cable_length < bottom_value - 1000)) {

        if (now - last_alarm_tick >= LOST_STEP_SUPPRESS_MS) {
            last_alarm_tick = now;
            printf("电机丢步报警：3秒内总移动 %.3f mm < %.3f mm\r\n",
                   (float)total_movement / 10.0f,
                   (float)LOST_STEP_THRESHOLD / 10.0f);
            return ENCODER_LOST_STEP;
        }
    } else {
        printf("丢步检测：电机运行正常 | 当前位置=%.1f mm | 3秒总移动=%.3f mm\r\n",
               (float)currentPos / 10.0f,
               (float)total_movement / 10.0f);
    }

    return NO_ERROR;
}

/* ===================== 无检测阻塞运动（可打断，void） ===================== */

/**
 * @brief 无任何检测的电机运动（阻塞，可被命令切换打断）
 *
 * 注意：
 *  - 该函数为 void，打断时仅 stop 并 return
 *  - 仅用于调试/标定等可控场景
 */
void motorMoveBlocking_NoDetect(float mm, int dir)
{
	printf("无检测阻塞运动：mm=%.2f, dir=%d\r\n", mm, dir);
    Motor_UpdateVelocityFromParams();

    if (mm <= 0.0f) return;
    if (!Motor_IsDirValid(dir)) return;

    g_measurement.debug_data.motor_state = dir;

    const double C0 = tape_C0_mm();
    const double t  = tape_t_mm();
    if (C0 <= 0.0) return;

    /* 当前有符号长度 */
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;

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

    CHECK_COMMAND_SWITCH_AND_STOP_NO_RETURN();
    stpr_moveBy(&stepper, &ticks, velocity);

    while (stpr_isMoving(&stepper)) {
        CHECK_COMMAND_SWITCH_AND_STOP_NO_RETURN();
        check_bottom_status();
        HAL_Delay(10);
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
