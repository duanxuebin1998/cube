#include "motor_ctrl_internal.h"
#include "error_log.h"

/**
 * @file motor_ctrl_motion_api.c
 * @brief 电机运动接口、等待保护、停止接口和调试运动。
 *
 * 本文件负责把业务层的距离、目标位置或 ticks 指令下发到 TMC5130，并在阻塞等待
 * 过程中处理命令切换、撞底、驱动异常、丢步检测和运行期位置刷新。
 */


#ifndef MOTOR_JOG_SLOWDOWN_DISTANCE_MM
#define MOTOR_JOG_SLOWDOWN_DISTANCE_MM     2.0f /* 电机点动控制参数：减速 距离 MM。 */
#endif
#ifndef MOTOR_JOG_CREEP_SPEED_X100
#define MOTOR_JOG_CREEP_SPEED_X100         10U /* 电机点动控制参数：低速爬行 SPEED 放大 100 倍。 */
#endif
#ifndef MOTOR_JOG_POSITION_EPS_MM
#define MOTOR_JOG_POSITION_EPS_MM          0.1f /* 电机点动控制参数：位置 允许误差 MM。 */
#endif
#ifndef MOTOR_JOG_FINAL_ERROR_LIMIT_MM
#define MOTOR_JOG_FINAL_ERROR_LIMIT_MM     10.0f /* 电机点动控制参数：最终 错误 限值 MM。 */
#endif
#ifndef MOTOR_JOG_STOP_TRIGGER_MM
#define MOTOR_JOG_STOP_TRIGGER_MM          0.45f /* 电机点动控制参数：停止 触发 MM。 */
#endif
#ifndef MOTOR_JOG_OVERSHOOT_LIMIT_MM
#define MOTOR_JOG_OVERSHOOT_LIMIT_MM       0.1f /* 电机点动控制参数：越界 限值 MM。 */
#endif
#ifndef MOTOR_JOG_POLL_MS
#define MOTOR_JOG_POLL_MS                  20U /* 电机点动控制参数：轮询 毫秒。 */
#endif
#ifndef MOTOR_JOG_MAX_RUN_MS
#define MOTOR_JOG_MAX_RUN_MS               3600000U /* 电机点动控制参数：最大值 运行 毫秒。 */
#endif
#ifndef MOTOR_JOG_START_GRACE_MS
#define MOTOR_JOG_START_GRACE_MS           500U /* 电机点动控制参数：启动 宽限 毫秒。 */
#endif
#ifndef MOTOR_JOG_BRAKE_MARGIN_X100
#define MOTOR_JOG_BRAKE_MARGIN_X100       130U /* 电机点动控制参数：制动 余量 放大 100 倍。 */
#endif
#ifndef MOTOR_JOG_BRAKE_MAX_DISTANCE_MM
#define MOTOR_JOG_BRAKE_MAX_DISTANCE_MM   1000.0f /* 电机点动控制参数：制动 最大值 距离 MM。 */
#endif
#ifndef MOTOR_JOG_DECEL_DEFAULT_REG
#define MOTOR_JOG_DECEL_DEFAULT_REG       (20U * 32U) /* 电机点动默认减速度寄存器值。 */
#endif
/* ===================== 私有函数声明 ===================== */

typedef struct
{
    bool restore_needed;
    uint32_t restore_speed_x100;
} MotorMotionSpeedScope;

typedef struct
{
    float start_mm;
    float target_mm;
    float distance_mm;
    int dir;
    bool already_reached;
} MotorMotionTargetPlan;

/**
 * @brief 开始单次运动临时速度作用域并记录恢复信息。
 */
static uint32_t MotorMotion_BeginSpeedScope(MotorMotionSpeedScope *scope,
                                            uint32_t speed_x100);
static uint32_t MotorMotion_EndSpeedScope(const MotorMotionSpeedScope *scope);
/**
 * @brief 按原始错误优先级恢复临时速度后返回。
 */
static uint32_t MotorMotion_ReturnWithSpeedScope(uint32_t ret,
                                                 const MotorMotionSpeedScope *scope);
/**
 * @brief 在临时速度作用域内执行 CHECK_ERROR 等价检查。
 */
static uint32_t MotorMotion_CheckErrorWithSpeedScope(uint32_t ret,
                                                     const MotorMotionSpeedScope *scope,
                                                     const char *file,
                                                     uint32_t line,
                                                     const char *func);
static uint32_t MotorMotion_CheckCommandAbortWithSpeedScope(const MotorMotionSpeedScope *scope);
static uint32_t MotorMotion_CheckReadyForProtectedMotion(bool ignore_encoder_ready,
                                                        bool check_health);
static uint32_t MotorMotion_WaitMoveStartObservation(uint32_t command_display_state,
                                                     const MotorMotionSpeedScope *speed_scope,
                                                     const char *file,
                                                     uint32_t line,
                                                     const char *func);
static uint32_t MotorMotion_CheckAbortRefreshAndHealth(TMC5130TypeDef *tmc5130,
                                                       uint32_t *last_vel_refresh_tick);
static uint32_t MotorMotion_CheckStoppedAndRefresh(TMC5130TypeDef *tmc5130);
static uint32_t MotorMotion_WaitStopAbortable(uint32_t poll_ms);
static uint32_t MotorMotion_WaitUntilStopWithTarget(TMC5130TypeDef *tmc5130,
                                                float target_mm,
                                                float eps_mm,
                                                int dir);
static uint32_t MotorMotion_WaitStoppedAfterStopCommand(uint32_t timeout_ms);
static uint32_t MotorMotion_MoveBlockingNoDetectInternal(float mm,
                                                         int dir,
                                                         uint32_t speed_x100,
                                                         bool ignore_encoder_ready,
                                                         bool verbose);
/**
 * @brief 等待 ticks 运动启动窗口内 XACTUAL 发生变化。
 */
static uint32_t MotorMotion_WaitTicksStartChanged(int32_t requested_ticks,
                                                  int32_t xactual_before,
                                                  const MotorMotionSpeedScope *speed_scope);
/**
 * @brief 等待 ticks 运动停稳后确认 XACTUAL 接近目标位置。
 */
static uint32_t MotorMotion_WaitTicksReachTarget(int32_t target_ticks,
                                                 char *detail,
                                                 size_t detail_size);
static uint32_t MotorMotion_DistanceToTicks(float move_mm,
                                            int dir,
                                            double current_length_mm,
                                            int32_t *ticks);
static uint32_t MotorMotion_DisplayStateFromDirection(int dir);
static uint32_t MotorMotion_DisplayStateFromTicks(int32_t ticks);
static bool MotorMotion_IsDisplayStateActive(uint32_t display_state);
static void MotorMotion_SetActiveState(uint32_t display_state, bool wait_active);
static void MotorMotion_ClearActiveState(void);
static bool MotorMotion_ShouldStopAtTarget(float current_mm,
                                           float target_mm,
                                           float eps_mm,
                                           int dir);
static uint32_t MotorMotion_StopAtTargetAndWait(TMC5130TypeDef *tmc5130);
static uint32_t MotorMotion_JogMoveToTargetInternal(float target_mm,
                                                    int dir,
                                                    uint32_t speed_x100);
static uint32_t MotorMotion_RefreshActivePositionMm(float *pos_mm);
static bool MotorMotion_IsPositionSnapshotValid(float pos_mm);
static uint32_t MotorMotion_BuildRelativeTargetPlan(float start_mm,
                                                    float move_mm,
                                                    int dir,
                                                    MotorMotionTargetPlan *plan);
static uint32_t MotorMotion_BuildAbsoluteTargetPlan(float current_mm,
                                                    float target_mm,
                                                    float eps_mm,
                                                    MotorMotionTargetPlan *plan);
static uint32_t MotorMotion_CheckAbsoluteTargetRange(float target_mm);
static float MotorMotion_RemainingDistanceToTarget(float current_mm,
                                                   float target_mm,
                                                   int dir);
static bool MotorMotion_IsOvershotPastTarget(float current_mm,
                                             float target_mm,
                                             int dir,
                                             float limit_mm);
static uint32_t MotorMotion_RefreshJogPositionChecked(float *cur_mm,
                                                      float target_mm,
                                                      int dir);
static uint32_t MotorMotion_CheckJogRuntimeGuards(uint32_t start_tick,
                                                  bool *is_moving);
static uint32_t MotorMotion_StartJogVelocity(int dir);
static uint32_t MotorMotion_StopJogAndRestore(uint32_t ret,
                                              const MotorMotionSpeedScope *speed_scope,
                                              float target_mm,
                                              int dir);
static uint32_t MotorMotion_CalcJogSlowdownDistanceMm(float current_mm,
                                                       uint32_t fast_velocity,
                                                       float *slowdown_mm);
static double MotorMotion_TmcVelocityToUstepsPerSec(uint32_t vmax);
static double MotorMotion_TmcAccelerationToUstepsPerSec2(uint32_t accel);
static uint32_t MotorMotion_CalcVelocityFromSpeedX100(uint32_t speed_x100,
                                                       float current_mm);
static uint32_t MotorMotion_ClampVelocityDouble(double vmax);

/* ===================== 对外接口 ===================== */

const char *MotorCtrl_DirectionText(int dir)
{
    if (dir == MOTOR_DIRECTION_UP) {
        return "上行";
    }
    if (dir == MOTOR_DIRECTION_DOWN) {
        return "下行";
    }
    return "未知";
}
/**
 * @brief 显示或打印电机控制中的 MotorCtrl_DisplayStateText 逻辑。
 *
 * @param display_state 状态值。
 * @return 返回业务对象或缓冲区指针，NULL 表示无有效对象。
 */
const char *MotorCtrl_DisplayStateText(uint32_t display_state)
{
    if (display_state == 1U) {
        return "上行";
    }
    if (display_state == 2U) {
        return "下行";
    }
    return "静止";
}

/**
 * @brief 按相对 ticks 移动并等待停止。
 *
 * 主要用于标定、位置源切换和内部精确步数控制。
 * @param ticks 相对移动步数，正负号表示方向。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、通信或打断错误码。
 */
uint32_t MotorCtrl_MoveByTicksAndWait(int32_t ticks, uint32_t speed_x100)
{
    uint32_t ret = NO_ERROR;
    uint32_t restore_ret = NO_ERROR;
    MotorMotionSpeedScope speed_scope = { false, 0U };
    int32_t xactual_before = 0;
    const int32_t requested_ticks = ticks;
    const uint32_t requested_display_state = MotorMotion_DisplayStateFromTicks(requested_ticks);
    char detail[128];
/* int32_t delta_ticks = ticks; */

    if (requested_ticks == 0) {
        return NO_ERROR;
    }

    /* 写 VMAX/XTARGET 前先确认驱动和位置源就绪，避免重启盲区继续运动。 */
    ret = MotorDriver_CheckMotionReady();
    CHECK_ERROR(ret);

    ret = MotorMotion_BeginSpeedScope(&speed_scope, speed_x100);
    CHECK_ERROR(ret);

    /* ticks 运动虽然不走“长度->圈数->ticks”的路径，
     * 但仍然要基于当前卷径刷新 VMAX，保证速度口径一致。 */
    MotorDriver_UpdateVelocityFromParams();

    ret = MotorDriver_StopIfCommandSwitchRequested();
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    /* 若方向相反，仅需在此统一翻转 */
    /* ticks = -ticks; */

    if (!stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual_before)) {
        restore_ret = MotorMotion_EndSpeedScope(&speed_scope);
        (void)restore_ret;
        snprintf(detail, sizeof(detail), "增量：%ld", (long)ticks);
        /* 错误 阶段：错误报警 模块：电机 操作：步进运动 原因：通信失败 处理：停止电机 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_STEP_MOTION,
                            ERROR_LOG_REASON_COMM_FAIL,
                            ERROR_LOG_ACTION_STOP_MOTOR,
                            detail);
        printf("步进运动失败 | 下发前XACTUAL读取失败 | 增量：%ld\r\n", (long)ticks);
        return MOTOR_TMC_COMM_ERROR;
    }
/* printf("ticks运动下发 | 增量：%ld | XACTUAL_before=%ld | VMAX=%lu | speed=%.2fm/min\r\n", */
/* (long)delta_ticks, */
/* (long)xactual_before, */
/* (unsigned long)velocity, */
/* (double)MotorDriver_GetSpeedSetpointX100() / 100.0); */
    s_motor_driver.applied_velocity = velocity;
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }
    MotorMotion_SetActiveState(requested_display_state, true);

    ret = MotorMotion_WaitTicksStartChanged(requested_ticks, xactual_before, &speed_scope);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorMotion_WaitStopAbortable(10);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret == NO_ERROR) {
        ret = MotorMotion_WaitTicksReachTarget(ticks, detail, sizeof(detail));
    }

    restore_ret = MotorMotion_EndSpeedScope(&speed_scope);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    CHECK_ERROR(restore_ret);
    return restore_ret;
}
/**
 * @brief 按距离下发非阻塞运动命令。
 *
 * 函数只负责换算目标并下发，不等待电机到位。
 * @param move_mm 移动距离，单位 mm。
 * @param dir 运动方向。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_MoveNoWait(float move_mm, int dir, uint32_t speed_x100)
{
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (move_mm < 0.0f) return PARAM_ERROR;
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (move_mm == 0.0f) return NO_ERROR;
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (!MotorDriver_IsDirValid(dir)) return PARAM_ERROR;

    /* 非阻塞入口返回后电机会继续跑，因此必须在下发目标前完成就绪门控。 */
    uint32_t ret = MotorDriver_CheckMotionReady();
    CHECK_ERROR(ret);

    /* 下发运动前先确认 TMC5130 配置和 24V 功率级，避免未上电仍开始规划运动。 */
    ret = MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_BEFORE_MOTION);
    CHECK_ERROR(ret);

    ret = MotorDriver_ApplyOptionalSpeed(speed_x100);
    CHECK_ERROR(ret);

    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    CHECK_ERROR(ret);

    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    int32_t ticks = 0;

    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_driver.applied_velocity = velocity;

    ret = MotorMotion_DistanceToTicks(move_mm, dir, Lcur_mm, &ticks);
    CHECK_ERROR(ret);

    /* 6) 下发运动 */
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    MotorMotion_SetActiveState(MotorMotion_DisplayStateFromDirection(dir), false);
    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    CHECK_ERROR(ret);

    return NO_ERROR;
}

/**
 * @brief 在零点处通过一圈运动标定第一圈周长。
 *
 * 电机下行一圈读取编码轮长度差，再结合尺带厚度反算首圈周长 C0。
 * @return 成功返回 NO_ERROR，否则返回运动、参数或测量错误码。
 */
uint32_t MotorCtrl_CalibrateFirstLoopCircumferenceAtZero(void)
{
    const double t = MotorPosition_TapeThicknessMm();
    if (t <= 0.0) {
        printf("首圈周长标定失败 | 尺带厚度非法 尺带厚度=%.4fmm\r\n", t);
        return PARAM_ERROR;
    }

    const int32_t one_rev_ticks = MotorPosition_TapeTicksPerRev();
    if (one_rev_ticks <= 0) {
        printf("首圈周长标定失败 | 每圈步数非法 步数=%ld\r\n", (long)one_rev_ticks);
        return PARAM_ERROR;
    }

    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 1) 记录零点长度 */
    const double L0 = MotorPosition_GetCurrentTapeLengthMm();

    /* 2) 下行一圈并等待停止（可打断） */
    uint32_t ret = MotorCtrl_MoveByTicksAndWait(one_rev_ticks, MotorCtrl_GetDefaultSpeedX100());
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("首圈周长标定失败 | 下行一圈失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    HAL_Delay(200);

    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    /* 3) 读取一圈后的长度 */
    const double L1 = MotorPosition_GetCurrentTapeLengthMm();
    const double dL = L1 - L0;
    printf("首圈周长标定采样 | 起点长度：%.1fmm | 终点长度：%.1fmm | 长度差=%.1fmm | 步数=%ld\r\n",
           L0, L1, dL, (long)one_rev_ticks);
    if (dL <= 0.0) {
        printf("首圈周长标定失败 | 一圈后尺带长度未增加 长度差=%.1fmm\r\n", dL);
        return PARAM_ERROR;
    }

    /* 新模型：第一圈放出长度：dL = C0 - pi*t => C0 = dL + pi*t */
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
    /* 标定零点流程会切回编码轮记步，此处同步清掉旧的电机局部周长，
     * 让后续再切回电机记步时从新的零点首圈周长开始。 */
    (void)MotorPosition_SetLocalCircumferenceToParams(C0);
    ret = MotorCtrl_MoveAndWait(dL, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100()); /* 回到起点 */
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("首圈周长标定失败 | 回到起点失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }
    /* 首圈周长标定直接修改了系统参数 first_loop_circumference_mm。
     * 这个值会影响后续的尺带长度换算，所以标定成功后必须立即保存，
     * 同时通过 parameter_update_flag 通知 CPU3 刷新参数缓存。 */
    save_device_params();
    MotorMotion_ClearActiveState();
    printf("首圈周长标定成功 | 首圈周长=%lu(0.1mm)\r\n",
           (unsigned long)g_deviceParams.first_loop_circumference_mm);
    return NO_ERROR;
}

/**
 * @brief 按距离运动并阻塞等待停止。
 *
 * 该接口是业务流程推荐的距离运动入口，内部包含目标规划、到位等待和保护检测。
 * @param mm 移动距离，单位 mm。
 * @param dir 运动方向。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、保护或打断错误码。
 */
uint32_t MotorCtrl_MoveAndWait(float mm, int dir, uint32_t speed_x100)
{
    uint32_t ret = NO_ERROR;
    uint32_t restore_ret = NO_ERROR;
    MotorMotionSpeedScope speed_scope = { false, 0U };
    MotorMotionTargetPlan target_plan;
    float startPos_mm;
    float currentPos_mm;
    float targetPos_mm;
    float total_cmd_mm;
    float moved_mm;
    float diff_pct;
    MotorDrumState drum;

    if (mm <= 0.0f) {
        return NO_ERROR;
    }
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }

    /* 阻塞运动虽然后续会轮询保护，但首帧位置不可用时不能先下发运动。 */
    ret = MotorMotion_CheckReadyForProtectedMotion(false, false);
    CHECK_ERROR(ret);

    const uint32_t command_display_state = MotorMotion_DisplayStateFromDirection(dir);

    /* 阻塞型接口支持“本次命令临时速度”：
     * 开始前切到 speed_x100，结束后恢复到默认最大速度。 */
    ret = MotorMotion_BeginSpeedScope(&speed_scope, speed_x100);
    CHECK_ERROR(ret);

    ret = MotorMotion_CheckCommandAbortWithSpeedScope(&speed_scope);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorMotion_RefreshActivePositionMm(&startPos_mm);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    ret = MotorMotion_BuildRelativeTargetPlan(startPos_mm, mm, dir, &target_plan);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }
    startPos_mm = target_plan.start_mm;
    targetPos_mm = target_plan.target_mm;
    total_cmd_mm = target_plan.distance_mm;

    ret = MotorMotion_CheckCommandAbortWithSpeedScope(&speed_scope);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorCtrl_MoveNoWait(total_cmd_mm, dir, 0U);
    ret = MotorMotion_CheckErrorWithSpeedScope(ret,
                                             &speed_scope,
                                             GetShortFilename(__FILE__),
                                             __LINE__,
                                             __func__);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    MotorMotion_SetActiveState(command_display_state, true);

    ret = MotorMotion_WaitMoveStartObservation(command_display_state,
                                               &speed_scope,
                                               GetShortFilename(__FILE__),
                                               __LINE__,
                                               __func__);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorMotion_WaitUntilStopWithTarget(&stepper, targetPos_mm, EPS_MM, dir);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    ret = MotorMotion_CheckCommandAbortWithSpeedScope(&speed_scope);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorMotion_RefreshActivePositionMm(&currentPos_mm);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    moved_mm = fabsf(currentPos_mm - startPos_mm);
    MotorCtrl_UpdateDrumStateFromXActual(&stepper, &drum);
/* printf("电机状态 | XACTUAL=%ld | 圈=%.4f | 角度=%.1f° | 预测长度：%.1fmm\r\n", */
/* (long)drum.motor_step, */
/* drum.turns_total, */
/* drum.angle_deg, */
/* drum.motor_distance_01mm / 10.0); */

    /* 用 abs 计算误差百分比，避免方向口径导致的负值 */
    diff_pct = 100.0f * fabsf(moved_mm - total_cmd_mm) / total_cmd_mm;

    if ((diff_pct > 70.0f) && (total_cmd_mm > 5.0f)) {
        printf("警告：检测到电机可能丢步！\r\n");
        printf("目标：%.2f mm, 实际=%.2f mm, 误差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        if (MotorCtrl_IsPositionSourceMotor()) {
            printf("当前为电机步进位置源，忽略编码器丢步异常\r\n");
            ret = NO_ERROR;
        } else {
            ret = ENCODER_LOST_STEP;
        }
    } else {
        printf("电机移动完成。\t");
        printf("起点=%.2f mm, 目标：%.2f mm, 最终=%.2f mm\t",
               startPos_mm, targetPos_mm, currentPos_mm);
        printf("期望：%.2f mm, 实际=%.2f mm, 偏差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        ret = NO_ERROR;
    }

    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret == NO_ERROR) {
        MotorMotion_ClearActiveState();
    }

    /* 只有主命令整体结束后，才恢复默认速度。 */
    restore_ret = MotorMotion_EndSpeedScope(&speed_scope);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    CHECK_ERROR(restore_ret);
    return restore_ret;
}
/**
 * @brief 以指定速度连续上行收带。
 *
 * @param speed_x100 运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_MoveUp(uint32_t speed_x100)
{
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    uint32_t ret = MotorCtrl_MoveNoWait(2000, MOTOR_DIRECTION_UP, speed_x100);
    CHECK_ERROR(ret);
    return NO_ERROR;
}

/**
 * @brief 以指定速度连续下行放带。
 *
 * @param speed_x100 运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_MoveDown(uint32_t speed_x100)
{
    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    uint32_t ret = MotorCtrl_MoveNoWait(200, MOTOR_DIRECTION_DOWN, speed_x100);
    CHECK_ERROR(ret);
    return NO_ERROR;
}

uint32_t MotorCtrl_StartVelocity(int dir, uint32_t speed_x100)
{
    uint32_t ret;

    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);

    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }

    ret = MotorDriver_CheckMotionReady();
    CHECK_ERROR(ret);

    ret = MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_BEFORE_MOTION);
    CHECK_ERROR(ret);

    ret = MotorDriver_ApplyOptionalSpeed(speed_x100);
    CHECK_ERROR(ret);

    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    CHECK_ERROR(ret);

    ret = MotorMotion_StartJogVelocity(dir);
    CHECK_ERROR(ret);

    MotorMotion_SetActiveState(MotorMotion_DisplayStateFromDirection(dir), false);
    return NO_ERROR;
}

/**
 * @brief 获取当前传感器位置快照。
 *
 * @param pos_mm 输出当前位置，单位 mm；传入 NULL 时不执行操作。
 */
void MotorCtrl_SnapshotSensorPositionMm(float *pos_mm)
{
    if (pos_mm == NULL) {
        return;
    }

    int32_t pos_01mm = g_measurement.debug_data.sensor_position;
    *pos_mm = (float)pos_01mm / 10.0f;
}

/**
 * @brief 移动到指定绝对位置并等待停止。
 *
 * 函数根据当前位置与目标位置自动判断方向和距离。
 * @param target_mm 目标绝对位置，单位 mm。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、打断或到位错误码。
 */
uint32_t MotorCtrl_MoveToPosition(float target_mm, uint32_t speed_x100)
{
    uint32_t ret = NO_ERROR;
    MotorMotionSpeedScope speed_scope = { false, 0U };
    MotorMotionTargetPlan target_plan;
    float cur_mm;

    ret = MotorMotion_CheckAbsoluteTargetRange(target_mm);
    CHECK_ERROR(ret);

    /* 绝对位置运动依赖当前位置快照，位置源未就绪时直接拦截。 */
    ret = MotorDriver_CheckMotionReady();
    CHECK_ERROR(ret);

    ret = MotorMotion_BeginSpeedScope(&speed_scope, speed_x100);
    CHECK_ERROR(ret);

    /* 绝对位置运动用“小步逼近”的方式做，
     * 每一轮都重新读取当前位置，降低位置更新滞后带来的影响。 */
    for (int i = 0; i < 10; i++) {

        ret = MotorDriver_StopIfCommandSwitchRequested();
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
        }

        ret = MotorMotion_RefreshActivePositionMm(&cur_mm);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
        }
        printf("运动到位置 | 当前：%.3fmm | 目标：%.3fmm\r\n", cur_mm, target_mm);

        ret = MotorMotion_BuildAbsoluteTargetPlan(cur_mm, target_mm, EPS_MM, &target_plan);
        if (ret == MEASUREMENT_POSITION_ERROR) {
            printf("运动到位置 | 当前快照异常：%.3fmm，取消移动\r\n", cur_mm);
        }
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
        }

        if (target_plan.already_reached) {
            printf("到位 | 当前：%.3fmm ≈ 目标：%.3fmm (±%.2fmm)\r\n",
                   cur_mm, target_mm, EPS_MM);
            return MotorMotion_ReturnWithSpeedScope(NO_ERROR, &speed_scope);
        }

        printf("运动到位置 | 方向：%s | 剩余距离：%.3fmm\r\n",
               MotorCtrl_DirectionText(target_plan.dir), target_plan.distance_mm);

        float plan_mm = target_plan.distance_mm;
        if (plan_mm < (EPS_MM * 2.0f)) {
            plan_mm = (EPS_MM * 2.0f);
        }
        ret = MotorCtrl_MoveAndWait(plan_mm, target_plan.dir, 0U);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope); /* 含 COMMAND_SWITCH_ABORT */
        }
    }

    return MotorMotion_ReturnWithSpeedScope(NO_ERROR, &speed_scope);
}
/**
 * @brief 使用速度点动模式按相对距离运动并等待到位。
 *
 * 该接口不把目标距离一次性换算成 XTARGET，而是先下发方向和速度，运行中持续读取
 * 当前有效位置源；接近目标时切换到低速，达到或越过保护边界立即慢停。
 * @param mm 移动距离，单位 mm。
 * @param dir 运动方向。
 * @param speed_x100 本次运动速度，0 表示使用当前默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、保护或打断错误码。
 */
uint32_t MotorCtrl_JogMoveAndWait(float mm, int dir, uint32_t speed_x100)
{
    float start_mm;
    MotorMotionTargetPlan target_plan;
    uint32_t ret;

    if (mm <= 0.0f) {
        return NO_ERROR;
    }
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }

    ret = MotorMotion_CheckReadyForProtectedMotion(false, true);
    CHECK_ERROR(ret);

    ret = MotorMotion_RefreshActivePositionMm(&start_mm);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorMotion_BuildRelativeTargetPlan(start_mm, mm, dir, &target_plan);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    return MotorMotion_JogMoveToTargetInternal(target_plan.target_mm, target_plan.dir, speed_x100);
}

/**
 * @brief 使用速度点动模式运动到绝对位置并等待到位。
 *
 * 该接口用于长距离运动的提前减速场景：启动前只计算目标方向，运行中按有效位置源
 * 判断剩余距离、减速距离、越界和异常停止，不依赖拟合距离一次性生成 XTARGET。
 * @param target_mm 目标位置，单位 mm。
 * @param speed_x100 本次运动速度，0 表示使用当前默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、保护或打断错误码。
 */
uint32_t MotorCtrl_JogMoveToPosition(float target_mm, uint32_t speed_x100)
{
    MotorMotionTargetPlan target_plan;
    float cur_mm;
    uint32_t ret;

    ret = MotorMotion_CheckAbsoluteTargetRange(target_mm);
    CHECK_ERROR(ret);

    ret = MotorMotion_CheckReadyForProtectedMotion(false, true);
    CHECK_ERROR(ret);

    ret = MotorMotion_RefreshActivePositionMm(&cur_mm);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorMotion_BuildAbsoluteTargetPlan(cur_mm,
                                              target_mm,
                                              MOTOR_JOG_POSITION_EPS_MM,
                                              &target_plan);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    if (target_plan.already_reached) {
        return NO_ERROR;
    }

    return MotorMotion_JogMoveToTargetInternal(target_plan.target_mm, target_plan.dir, speed_x100);
}
/**
 * @brief 急停电机并刷新状态。
 *
 * 急停会立即下发停止，并短暂关闭再重新使能驱动。
 * @return 成功返回 NO_ERROR，否则返回停止或等待错误码。
 */
uint32_t MotorCtrl_QuickStop(void)
{
    uint32_t ret;

    /* 急停必须主动尝试下发停止命令，避免驱动状态读取失败时跳过停机。 */
    ret = MotorDriver_StopAndMarkStopped();
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    stpr_disableDriver(&stepper);
    HAL_Delay(4000);
    stpr_enableDriver(&stepper);

    MotorPosition_SyncDebugDrumState(&stepper);
    MotorMotion_ClearActiveState();
    return MotorDriver_SetSpeedQuiet(g_deviceParams.max_motor_speed);
}
/**
 * @brief 慢停电机并等待斜坡减速结束。
 *
 * 只下发停止命令，保留驱动斜坡减速行为。
 * @return 成功返回 NO_ERROR，否则返回停止或等待错误码。
 */
uint32_t MotorCtrl_SlowStop(void)
{
    uint32_t ret;

    ret = MotorDriver_StopAndMarkStopped();
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    /* 慢停后必须等 RAMPSTAT.vzero 确认停稳，再恢复软件速度设定。
     * 否则 TMC5130 仍处于速度模式时写回 VMAX，可能导致电机继续运行。 */
    ret = MotorMotion_WaitStoppedAfterStopCommand(MOTOR_STOP_WAIT_TIMEOUT_MS);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    return MotorDriver_SetSpeedQuiet(g_deviceParams.max_motor_speed);
}

/**
 * @brief 获取上层显示用电机运动状态。
 *
 * @return 0 表示停止，1 表示上行，2 表示下行。
 */
uint32_t MotorCtrl_GetDisplayState(void)
{
    uint32_t motor_state = g_measurement.debug_data.motor_state;
    bool is_moving = false;
    uint32_t ret;

    if (!s_motor_driver.initialized) {
        return MotorMotion_IsDisplayStateActive(motor_state) ? motor_state : 0U;
    }

    if (!s_motor_driver.motion_command_active) {
        s_motor_driver.motion_wait_active = false;
        g_measurement.debug_data.motor_state = 0U;
        return 0U;
    }

    ret = MotorDriver_ReadMovingState(&stepper, &is_moving);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_IsDisplayStateActive(motor_state) ? motor_state : 0U;
    }

    if (!is_moving) {
        if (s_motor_driver.motion_wait_active) {
            return MotorMotion_IsDisplayStateActive(motor_state) ? motor_state : 0U;
        }
        if (MotorMotion_IsDisplayStateActive(motor_state)) {
            MotorMotion_ClearActiveState();
        }
        return 0U;
    }

    if (MotorMotion_IsDisplayStateActive(motor_state)) {
        return motor_state;
    }

    motor_state = MotorDriver_InferDisplayStateFromDriver(&stepper);
    if (MotorMotion_IsDisplayStateActive(motor_state)) {
        g_measurement.debug_data.motor_state = motor_state;
        return motor_state;
    }
    return 0U;
}

/**
 * @brief 调试/维护用无检测阻塞运动。
 *
 * 该接口不做撞底和丢步保护，只用于现场调试或受控维护流程。
 * @param mm 移动距离，单位 mm。
 * @param dir 运动方向。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 */
uint32_t MotorCtrl_MoveBlockingNoDetect(float mm, int dir, uint32_t speed_x100)
{
    return MotorMotion_MoveBlockingNoDetectInternal(mm, dir, speed_x100, false, false);
}

uint32_t MotorCtrl_MoveBlockingNoDetectQuiet(float mm, int dir, uint32_t speed_x100)
{
    return MotorMotion_MoveBlockingNoDetectInternal(mm, dir, speed_x100, false, false);
}

/**
 * @brief 强制调试无检测阻塞运动。
 *
 * 只绕过编码器首帧门控，仍保留上电安全停机、驱动初始化和 TMC 健康检查。
 * 仅供人工强制上/下行等调试命令使用，正常测量流程不能调用。
 */
uint32_t MotorCtrl_MoveBlockingNoDetectForceDebug(float mm, int dir, uint32_t speed_x100)
{
    return MotorMotion_MoveBlockingNoDetectInternal(mm, dir, speed_x100, true, true);
}

/**
 * @brief 执行电机控制中的 MotorMotion_MoveBlockingNoDetectInternal 逻辑。
 *
 * @param mm 业务参数。
 * @param dir 业务参数。
 * @param speed_x100 业务参数。
 * @param ignore_encoder_ready 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t MotorMotion_MoveBlockingNoDetectInternal(float mm,
                                                         int dir,
                                                         uint32_t speed_x100,
                                                         bool ignore_encoder_ready,
                                                         bool verbose)
{
    MotorMotionSpeedScope speed_scope = { false, 0U };
    uint32_t ret;

    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (mm <= 0.0f) return PARAM_ERROR;
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (!MotorDriver_IsDirValid(dir)) return PARAM_ERROR;

    /* 无检测只跳过撞底/丢步检测；强制调试入口才允许额外绕过编码器首帧。 */
    ret = MotorMotion_CheckReadyForProtectedMotion(ignore_encoder_ready, true);
    if (ret != NO_ERROR) {
        return ret;
    }

    /* 该接口是“无检测”版本，只保留基础运动和日志。
     * 但为了让语义一致，临时速度恢复策略仍然与其他阻塞接口保持一致。 */
    ret = MotorMotion_BeginSpeedScope(&speed_scope, speed_x100);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：速度设置失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    if (verbose) {
        printf("无检测阻塞运动：距离=%.2f, 方向：%s\r\n", mm, MotorCtrl_DirectionText(dir));
    }

    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    int32_t ticks = 0;

    ret = MotorMotion_DistanceToTicks(mm, dir, Lcur_mm, &ticks);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_driver.applied_velocity = velocity;

    ret = MotorDriver_StopIfCommandSwitchRequested();
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：目标位置越界 错误码：0x%08lX\r\n", (unsigned long)ret);
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }
    MotorMotion_SetActiveState(MotorMotion_DisplayStateFromDirection(dir), true);
    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }
    HAL_Delay(100);

    uint32_t last_vel_refresh_tick = HAL_GetTick();
    bool is_moving = true;
    while (1) {
        ret = MotorCtrl_IsDriverMoving(&stepper, &is_moving);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
        }
        if (!is_moving) {
            break;
        }
        ret = MotorMotion_CheckAbortRefreshAndHealth(&stepper, &last_vel_refresh_tick);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
        }
        if (verbose) {
            MotorLostStep_NoDetectRuntimeLogUpdate();
        }
    }

    MotorMotion_ClearActiveState();
    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    ret = MotorMotion_EndSpeedScope(&speed_scope);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：恢复默认速度失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    return NO_ERROR;
}

/**
 * @brief 执行电机控制中的 MotorMotion_WaitTicksStartChanged 逻辑。
 *
 * @param requested_ticks 业务参数。
 * @param xactual_before 业务参数。
 * @param speed_scope 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t MotorMotion_WaitTicksStartChanged(int32_t requested_ticks,
                                                  int32_t xactual_before,
                                                  const MotorMotionSpeedScope *speed_scope)
{
    uint32_t ret;
    uint32_t start_wait_tick = HAL_GetTick();
    uint32_t last_vel_refresh_tick = start_wait_tick;
    int32_t xactual_after = xactual_before;
    int32_t xtarget_after = 0;
    int32_t rampstat_after = 0;
    int32_t gstat_after = 0;
    char detail[128];

    do {
        ret = MotorMotion_CheckAbortRefreshAndHealth(&stepper, &last_vel_refresh_tick);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            /* 提前退出前先恢复临时速度，避免下一条命令沿用本次速度。 */
            return MotorMotion_ReturnWithSpeedScope(ret, speed_scope);
        }
        if (stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual_after) &&
            (xactual_after != xactual_before)) {
            return NO_ERROR;
        }
        HAL_Delay(10);
    } while ((HAL_GetTick() - start_wait_tick) < 500U);

    (void)stpr_tryReadInt(&stepper, TMC5130_XTARGET, &xtarget_after);
    (void)stpr_tryReadInt(&stepper, TMC5130_RAMPSTAT, &rampstat_after);
    (void)stpr_tryReadInt(&stepper, TMC5130_GSTAT, &gstat_after);
    (void)MotorMotion_EndSpeedScope(speed_scope);
    snprintf(detail, sizeof(detail),
             "变化前：%ld,变化后：%ld,目标：%ld,目标寄存器：%ld,斜坡状态：0x%08lX,全局状态：0x%08lX",
             (long)xactual_before,
             (long)xactual_after,
             (long)requested_ticks,
             (long)xtarget_after,
             (unsigned long)rampstat_after,
             (unsigned long)gstat_after);
    /* 错误 阶段：错误报警 模块：电机 操作：步进运动 原因：ErrorLog_GetReasonByCode(MOTOR_STEP_ERROR) 处理：停止电机 详情：detail */
    ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                        ERROR_LOG_OP_STEP_MOTION,
                        ErrorLog_GetReasonByCode(MOTOR_STEP_ERROR),
                        ERROR_LOG_ACTION_STOP_MOTOR,
                        detail);
    printf("步进运动失败 | 500ms内XACTUAL未变化 | 变化前：%ld | 变化后：%ld | 目标：%ld | XTARGET=%ld | RAMPSTAT=0x%08lX | GSTAT=0x%08lX\r\n",
           (long)xactual_before,
           (long)xactual_after,
           (long)requested_ticks,
           (long)xtarget_after,
           (unsigned long)rampstat_after,
           (unsigned long)gstat_after);
    MotorMotion_ClearActiveState();
    return MOTOR_STEP_ERROR;
}

/**
 * @brief 执行电机控制中的 MotorMotion_WaitTicksReachTarget 逻辑。
 *
 * @param target_ticks 业务参数。
 * @param detail 业务参数。
 * @param detail_size 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t MotorMotion_WaitTicksReachTarget(int32_t target_ticks,
                                                 char *detail,
                                                 size_t detail_size)
{
    uint32_t ret;
    uint32_t settle_start_tick = HAL_GetTick();
    uint32_t last_vel_refresh_tick = settle_start_tick;
    const int32_t target_tolerance_ticks = 1024;

    /* YM 局部周长标定依赖下行一圈后的编码轮长度。
     * 仅看 RAMPSTAT.vzero 可能过早返回，这里再确认 XACTUAL 已到 XTARGET 附近。 */
    while (1) {
        int32_t xactual_now = 0;
        int32_t diff_ticks;

        ret = MotorDriver_StopIfCommandSwitchRequested();
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        if (!stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual_now)) {
            snprintf(detail, detail_size,
                     "寄存器：XACTUAL,目标位置：%ld",
                     (long)target_ticks);
            /* 错误 阶段：错误报警 模块：电机 操作：等待电机停止 原因：通信失败 处理：停止电机 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_WAIT_STOP,
                                ERROR_LOG_REASON_COMM_FAIL,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            return MOTOR_TMC_COMM_ERROR;
        }

        diff_ticks = xactual_now - target_ticks;
        if (diff_ticks < 0) {
            diff_ticks = -diff_ticks;
        }
        if (diff_ticks <= target_tolerance_ticks) {
            return NO_ERROR;
        }
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if ((HAL_GetTick() - settle_start_tick) > MOTOR_STOP_WAIT_TIMEOUT_MS) {
            snprintf(detail, detail_size,
                     "实际位置：%ld,目标位置：%ld,差值：%ld,超时：%lums",
                     (long)xactual_now,
                     (long)target_ticks,
                     (long)diff_ticks,
                     (unsigned long)MOTOR_STOP_WAIT_TIMEOUT_MS);
            /* 错误 阶段：错误报警 模块：电机 操作：等待电机停止 原因：ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT) 处理：停止电机 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_WAIT_STOP,
                                ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT),
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            printf("ticks运动等待到位超时 | 实际位置：%ld | 目标位置：%ld | 差值：%ld\r\n",
                   (long)xactual_now,
                   (long)target_ticks,
                   (long)diff_ticks);
            return MOTOR_RUN_TIMEOUT;
        }
        MotorDriver_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        HAL_Delay(10U);
    }
}
/**
 * @brief 将业务距离换算为 TMC5130 相对 ticks。
 *
 * 上层方向只负责表达收带/放带；本函数统一处理有符号尺带长度、局部周长优先级、
 * 卷筒模型和 int32 目标范围钳位，避免多个运动入口各自维护一份换算逻辑。
 */
static uint32_t MotorMotion_DistanceToTicks(float move_mm,
                                            int dir,
                                            double current_length_mm,
                                            int32_t *ticks)
{
    double delta_length_mm;
    double target_length_mm;
    double local_circumference_mm;
    int64_t ticks64;

    if (ticks == NULL) {
        return PARAM_ERROR;
    }
    *ticks = 0;

    if (move_mm < 0.0f) {
        return PARAM_ERROR;
    }
    if (move_mm == 0.0f) {
        return NO_ERROR;
    }
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }

    delta_length_mm = (double)move_mm;
    if (dir == MOTOR_DIRECTION_UP) {
        delta_length_mm = -delta_length_mm;
    }
    target_length_mm = current_length_mm + delta_length_mm;

    local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();
    if ((g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
        (local_circumference_mm > 1e-6)) {
        ticks64 = MotorPosition_RoundToInt64((delta_length_mm / local_circumference_mm) *
                                            (double)MotorPosition_TapeTicksPerRev());
    } else {
        const double C0 = MotorPosition_TapeC0Mm();
        const double t  = MotorPosition_TapeThicknessMm();
        double ncur;
        double ntar;
        double dn;

        if (C0 <= 0.0) {
            return PARAM_ERROR;
        }
        ncur = MotorPosition_TapeTurnsFromSignedLength(current_length_mm, C0, t);
        ntar = MotorPosition_TapeTurnsFromSignedLength(target_length_mm, C0, t);
        dn = ntar - ncur;
        ticks64 = MotorPosition_RoundToInt64(dn * (double)MotorPosition_TapeTicksPerRev());
    }

    if (ticks64 > (int64_t)INT32_MAX) {
        ticks64 = (int64_t)INT32_MAX;
    }
    if (ticks64 < (int64_t)INT32_MIN) {
        ticks64 = (int64_t)INT32_MIN;
    }
    *ticks = (int32_t)ticks64;
    return NO_ERROR;
}

/**
 * @brief 执行电机控制中的 MotorMotion_BeginSpeedScope 逻辑。
 *
 * @param scope 业务参数。
 * @param speed_x100 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t MotorMotion_BeginSpeedScope(MotorMotionSpeedScope *scope,
                                            uint32_t speed_x100)
{
    if (scope == NULL) {
        return PARAM_ERROR;
    }

    return MotorDriver_BeginTemporarySpeed(speed_x100,
                                           &scope->restore_needed,
                                           &scope->restore_speed_x100);
}

/**
 * @brief 结束临时速度作用域，恢复进入运动前的速度。
 */
static uint32_t MotorMotion_EndSpeedScope(const MotorMotionSpeedScope *scope)
{
    if (scope == NULL) {
        return PARAM_ERROR;
    }

    return MotorDriver_EndTemporarySpeed(scope->restore_needed,
                                         scope->restore_speed_x100);
}

/**
 * @brief 执行电机控制中的 MotorMotion_ReturnWithSpeedScope 逻辑。
 *
 * @param ret 业务参数。
 * @param scope 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t MotorMotion_ReturnWithSpeedScope(uint32_t ret,
                                                 const MotorMotionSpeedScope *scope)
{
    if (scope == NULL) {
        return (ret != NO_ERROR) ? ret : PARAM_ERROR;
    }

    return MotorDriver_ReturnAfterTemporarySpeed(ret,
                                                 scope->restore_needed,
                                                 scope->restore_speed_x100);
}

/**
 * @brief 检查电机控制中的 MotorMotion_CheckErrorWithSpeedScope 逻辑。
 *
 * @param ret 业务参数。
 * @param scope 业务参数。
 * @param file 业务参数。
 * @param line 业务参数。
 * @param func 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t MotorMotion_CheckErrorWithSpeedScope(uint32_t ret,
                                                     const MotorMotionSpeedScope *scope,
                                                     const char *file,
                                                     uint32_t line,
                                                     const char *func)
{
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        const uint32_t handled_ret = FaultManager_HandleCheckError(ret, file, line, func);
        (void)MotorMotion_EndSpeedScope(scope);
        return handled_ret;
    }

    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (g_measurement.device_status.error_code != NO_ERROR) {
        const uint32_t handled_ret =
            FaultManager_HandleGlobalError(g_measurement.device_status.error_code,
                                          file,
                                          line,
                                          func);
        (void)MotorMotion_EndSpeedScope(scope);
        return handled_ret;
    }

    if (HasEffectiveCommandSwitchRequest()) {
        err.error_code = STATE_SWITCH;
        HandleError();
        (void)MotorMotion_EndSpeedScope(scope);
        return err.error_code;
    }

    return NO_ERROR;
}

/*
 * 函数用途：检查命令切换打断，并在打断时恢复本次临时速度。
 * 调用场景：阻塞运动入口已经进入 MotorMotionSpeedScope 后使用。
 * 关键约束：返回非 NO_ERROR 时已经完成速度恢复，调用方直接返回该错误码。
 */
static uint32_t MotorMotion_CheckCommandAbortWithSpeedScope(const MotorMotionSpeedScope *scope)
{
    uint32_t ret;

    ret = MotorDriver_StopIfCommandSwitchRequested();
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, scope);
    }

    return NO_ERROR;
}

/*
 * 函数用途：统一执行受保护运动的运行前门控。
 * 调用场景：正式运动、Jog 运动和无检测运动下发前。
 * 关键约束：只封装既有就绪/健康检查；是否绕过编码器首帧和是否检查健康由调用方显式选择。
 */
static uint32_t MotorMotion_CheckReadyForProtectedMotion(bool ignore_encoder_ready,
                                                        bool check_health)
{
    uint32_t ret;

    ret = ignore_encoder_ready ? MotorDriver_CheckMotionReadyForceDebug()
                               : MotorDriver_CheckMotionReady();
    if (ret != NO_ERROR) {
        return ret;
    }

    if (check_health) {
        ret = MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_BEFORE_MOTION);
        if (ret != NO_ERROR) {
            return ret;
        }
    }

    return NO_ERROR;
}

/*
 * 函数用途：等待正式位置运动下发后的启动观察窗口。
 * 调用场景：MotorCtrl_MoveAndWait() 下发 MoveNoWait 后，到位等待前。
 * 关键约束：只抽取原有 100 次、10ms 观察流程，不改变启动节奏和保护顺序。
 */
static uint32_t MotorMotion_WaitMoveStartObservation(uint32_t command_display_state,
                                                     const MotorMotionSpeedScope *speed_scope,
                                                     const char *file,
                                                     uint32_t line,
                                                     const char *func)
{
    uint32_t ret;
    uint32_t prewait_vel_refresh_tick;

    prewait_vel_refresh_tick = HAL_GetTick();
    for (int i = 0; i < 100; i++) {
        ret = MotorMotion_CheckCommandAbortWithSpeedScope(speed_scope);
        if (ret != NO_ERROR) {
            return ret;
        }

        MotorDriver_RefreshVelocityDuringRun(&stepper, &prewait_vel_refresh_tick);
        ret = MotorCtrl_PollRuntimePosition();
        ret = MotorMotion_CheckErrorWithSpeedScope(ret,
                                                  speed_scope,
                                                  file,
                                                  line,
                                                  func);
        if (ret != NO_ERROR) {
            return ret;
        }

        MotorMotion_SetActiveState(command_display_state, true);
        HAL_Delay(10);
    }

    return NO_ERROR;
}

/*
 * 函数用途：集中执行等待循环内的命令切换、速度刷新和 TMC 同步/健康检查。
 * 调用场景：阻塞等待循环已确认电机仍在运动后，每轮进入路径专属检查前。
 * 关键约束：不处理路径专属的速度恢复、撞底、丢步、目标判断或停机收尾。
 */
static uint32_t MotorMotion_CheckAbortRefreshAndHealth(TMC5130TypeDef *tmc5130,
                                                       uint32_t *last_vel_refresh_tick)
{
    uint32_t ret;

    if ((tmc5130 == NULL) || (last_vel_refresh_tick == NULL)) {
        return PARAM_ERROR;
    }

    ret = MotorDriver_StopIfCommandSwitchRequested();
    if (ret != NO_ERROR) {
        return ret;
    }

    MotorDriver_RefreshVelocityDuringRun(tmc5130, last_vel_refresh_tick);
    ret = MotorDriver_SyncPositionOrCheckHealth(tmc5130);
    if (ret != NO_ERROR) {
        return ret;
    }

    return NO_ERROR;
}

/*
 * 函数用途：电机确认停稳后同步最终位置并清理活动状态。
 * 调用场景：等待循环发现驱动不再运动后，返回 NO_ERROR 前使用。
 * 关键约束：只做最终同步和状态清理；同步失败时保留原始错误码且不清活动状态。
 */
static uint32_t MotorMotion_CheckStoppedAndRefresh(TMC5130TypeDef *tmc5130)
{
    uint32_t ret;

    if (tmc5130 == NULL) {
        return PARAM_ERROR;
    }

    ret = MotorDriver_SyncPositionOrCheckHealth(tmc5130);
    if (ret != NO_ERROR) {
        return ret;
    }

    MotorMotion_ClearActiveState();
    return NO_ERROR;
}
/**
 * @brief 将业务方向转换为上层显示状态。
 */
static uint32_t MotorMotion_DisplayStateFromDirection(int dir)
{
    if (dir == MOTOR_DIRECTION_UP) {
        return 1U;
    }
    if (dir == MOTOR_DIRECTION_DOWN) {
        return 2U;
    }
    return 0U;
}

/**
 * @brief 将相对 ticks 方向转换为上层显示状态。
 */
static uint32_t MotorMotion_DisplayStateFromTicks(int32_t ticks)
{
    if (ticks > 0) {
        return 2U;
    }
    if (ticks < 0) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 判断显示状态是否代表有效运动方向。
 */
static bool MotorMotion_IsDisplayStateActive(uint32_t display_state)
{
    return (display_state == 1U) || (display_state == 2U);
}

/**
 * @brief 统一置位当前运动状态。
 */
static void MotorMotion_SetActiveState(uint32_t display_state, bool wait_active)
{
    if (!MotorMotion_IsDisplayStateActive(display_state)) {
        MotorMotion_ClearActiveState();
        return;
    }

    s_motor_driver.motion_command_active = true;
    s_motor_driver.motion_wait_active = wait_active;
    g_measurement.debug_data.motor_state = display_state;
}

/**
 * @brief 统一清除当前运动状态。
 */
static void MotorMotion_ClearActiveState(void)
{
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    g_measurement.debug_data.motor_state = 0U;
}

/**
 * @brief 判断当前位置是否已经到达或越过目标。
 */
static bool MotorMotion_ShouldStopAtTarget(float current_mm,
                                           float target_mm,
                                           float eps_mm,
                                           int dir)
{
    if (fabsf(current_mm - target_mm) <= eps_mm) {
        return true;
    }
    if (dir == MOTOR_DIRECTION_DOWN) {
        return current_mm <= target_mm;
    }
    if (dir == MOTOR_DIRECTION_UP) {
        return current_mm >= target_mm;
    }
    return false;
}

/**
 * @brief 到位后下发停止并等待驱动确认停稳。
 */
static uint32_t MotorMotion_StopAtTargetAndWait(TMC5130TypeDef *tmc5130)
{
    uint32_t ret;
    bool is_moving = true;

    ret = MotorDriver_StopAndMarkStopped();
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    while (1) {
        ret = MotorCtrl_IsDriverMoving(tmc5130, &is_moving);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        if (!is_moving) {
            break;
        }
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
        ret = MotorDriver_SyncPositionOrCheckHealth(tmc5130);
        CHECK_ERROR(ret);
        HAL_Delay(5U);
    }

    MotorMotion_ClearActiveState();
    return NO_ERROR;
}
/* ===================== 私有函数实现 ===================== */

/**
 * @brief 点动模式运动到目标位置的内部实现。
 *
 * 进入前已经明确目标和方向；本函数负责临时速度、运行中保护、提前降速、慢停和速度恢复。
 */
static uint32_t MotorMotion_JogMoveToTargetInternal(float target_mm,
                                                    int dir,
                                                    uint32_t speed_x100)
{
    uint32_t ret;
    MotorMotionSpeedScope speed_scope = { false, 0U };
    bool slow_mode = false;
    uint32_t start_tick;
    uint32_t last_vel_refresh_tick;
    uint32_t fast_velocity;
    float cur_mm = 0.0f;
    float remaining_mm;
    float slowdown_distance_mm = MOTOR_JOG_SLOWDOWN_DISTANCE_MM;
    uint32_t display_state;

    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }

    ret = MotorMotion_CheckReadyForProtectedMotion(false, true);
    CHECK_ERROR(ret);

    ret = MotorMotion_BeginSpeedScope(&speed_scope, speed_x100);
    CHECK_ERROR(ret);

    MotorDriver_UpdateVelocityFromParams();
    fast_velocity = velocity;

    ret = MotorDriver_StopIfCommandSwitchRequested();
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    ret = MotorMotion_RefreshJogPositionChecked(&cur_mm, target_mm, dir);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    ret = MotorMotion_CalcJogSlowdownDistanceMm(cur_mm, fast_velocity, &slowdown_distance_mm);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    remaining_mm = MotorMotion_RemainingDistanceToTarget(cur_mm, target_mm, dir);
    if (remaining_mm <= MOTOR_JOG_POSITION_EPS_MM) {
        return MotorMotion_ReturnWithSpeedScope(NO_ERROR, &speed_scope);
    }

    if (remaining_mm <= slowdown_distance_mm) {
        ret = MotorDriver_SetSpeedQuiet(MOTOR_JOG_CREEP_SPEED_X100);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
        }
        MotorDriver_UpdateVelocityFromParams();
        printf("Jog start creep | target=%.3fmm | current=%.3fmm | remaining=%.3fmm | slowdown=%.3fmm | VMAX=%lu\r\n",
               target_mm,
               cur_mm,
               remaining_mm,
               slowdown_distance_mm,
               (unsigned long)velocity);
        slow_mode = true;
    }

    if (!slow_mode) {
        velocity = fast_velocity;
    }
    ret = MotorMotion_StartJogVelocity(dir);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
    }

    display_state = MotorMotion_DisplayStateFromDirection(dir);
    MotorMotion_SetActiveState(display_state, true);
    MotorCtrl_LostStepInit();
    start_tick = HAL_GetTick();
    last_vel_refresh_tick = start_tick;

    while (1) {
        bool is_moving = true;

        ret = MotorDriver_StopIfCommandSwitchRequested();
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return MotorMotion_ReturnWithSpeedScope(ret, &speed_scope);
        }

        ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return MotorMotion_StopJogAndRestore(ret, &speed_scope, target_mm, dir);
        }

        ret = MotorMotion_RefreshJogPositionChecked(&cur_mm, target_mm, dir);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return MotorMotion_StopJogAndRestore(ret, &speed_scope, target_mm, dir);
        }

        remaining_mm = MotorMotion_RemainingDistanceToTarget(cur_mm, target_mm, dir);
        if (remaining_mm <= MOTOR_JOG_STOP_TRIGGER_MM) {
            printf("Jog stop trigger | target=%.3fmm | current=%.3fmm | remaining=%.3fmm | trigger=%.3fmm | VMAX=%lu\r\n",
                   target_mm,
                   cur_mm,
                   remaining_mm,
                   MOTOR_JOG_STOP_TRIGGER_MM,
                   (unsigned long)((s_motor_driver.applied_velocity != 0U) ? s_motor_driver.applied_velocity : velocity));
            return MotorMotion_StopJogAndRestore(NO_ERROR, &speed_scope, target_mm, dir);
        }


        if ((!slow_mode) && (remaining_mm <= slowdown_distance_mm)) {
            /* 接近目标后改用低速 VMAX 继续速度模式，减少停止惯性造成的过冲。 */
            ret = MotorDriver_SetSpeedQuiet(MOTOR_JOG_CREEP_SPEED_X100);
            /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
            if (ret != NO_ERROR) {
                return MotorMotion_StopJogAndRestore(ret, &speed_scope, target_mm, dir);
            }
            MotorDriver_UpdateVelocityFromParams();
            printf("Jog switch creep | target=%.3fmm | current=%.3fmm | remaining=%.3fmm | slowdown=%.3fmm | VMAX=%lu\r\n",
                   target_mm,
                   cur_mm,
                   remaining_mm,
                   slowdown_distance_mm,
                   (unsigned long)velocity);
            ret = MotorMotion_StartJogVelocity(dir);
            /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
            if (ret != NO_ERROR) {
                return MotorMotion_StopJogAndRestore(ret, &speed_scope, target_mm, dir);
            }
            slow_mode = true;
            MotorMotion_SetActiveState(display_state, true);
        } else {
            MotorDriver_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        }

        ret = MotorMotion_CheckJogRuntimeGuards(start_tick, &is_moving);
        if (ret != NO_ERROR) {
            return MotorMotion_StopJogAndRestore(ret, &speed_scope, target_mm, dir);
        }

        HAL_Delay(MOTOR_JOG_POLL_MS);
    }
}

/**
 * @brief 按当前有效位置源刷新并获取当前位置快照。
 */
static uint32_t MotorMotion_RefreshActivePositionMm(float *pos_mm)
{
    if (pos_mm == NULL) {
        return PARAM_ERROR;
    }

    MotorCtrl_RefreshPositionFromActiveSource();
    MotorCtrl_SnapshotSensorPositionMm(pos_mm);
    return NO_ERROR;
}

/**
 * @brief 判断位置快照是否在罐高兜底范围内。
 */
static bool MotorMotion_IsPositionSnapshotValid(float pos_mm)
{
    float guard_tank_height_mm = (float)g_deviceParams.tankHeight / 10.0f;
    float min_valid_mm;
    float max_valid_mm;

    if ((guard_tank_height_mm <= 0.0f) || (guard_tank_height_mm > 500000.0f)) {
        guard_tank_height_mm = 500000.0f;
    }

    min_valid_mm = -guard_tank_height_mm - 1000.0f;
    max_valid_mm = guard_tank_height_mm + 1000.0f;
    return (pos_mm >= min_valid_mm) && (pos_mm <= max_valid_mm);
}

/*
 * 函数用途：按当前位置、相对距离和方向生成统一目标规划。
 * 调用场景：正式相对运动和 Jog 相对点动入口共用。
 * 关键约束：只做目标计算和边界检查，不决定后续使用位置目标模式还是速度点动模式。
 */
static uint32_t MotorMotion_BuildRelativeTargetPlan(float start_mm,
                                                    float move_mm,
                                                    int dir,
                                                    MotorMotionTargetPlan *plan)
{
    uint32_t ret;

    if (plan == NULL) {
        return PARAM_ERROR;
    }

    plan->start_mm = start_mm;
    plan->target_mm = start_mm;
    plan->distance_mm = 0.0f;
    plan->dir = dir;
    plan->already_reached = false;

    if (move_mm <= 0.0f) {
        plan->already_reached = true;
        return NO_ERROR;
    }
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }
    if (!MotorMotion_IsPositionSnapshotValid(start_mm)) {
        return MEASUREMENT_POSITION_ERROR;
    }

    plan->distance_mm = move_mm;
    plan->target_mm = start_mm + ((dir == MOTOR_DIRECTION_UP) ? move_mm : -move_mm);

    ret = MotorMotion_CheckAbsoluteTargetRange(plan->target_mm);
    if (ret != NO_ERROR) {
        return ret;
    }

    return NO_ERROR;
}

/*
 * 函数用途：按当前位置和绝对目标生成统一目标规划。
 * 调用场景：正式到绝对位置和 Jog 到绝对位置入口共用。
 * 关键约束：只输出方向、剩余距离和是否已经到位，调用方继续选择执行策略。
 */
static uint32_t MotorMotion_BuildAbsoluteTargetPlan(float current_mm,
                                                    float target_mm,
                                                    float eps_mm,
                                                    MotorMotionTargetPlan *plan)
{
    float delta_mm;
    uint32_t ret;

    if (plan == NULL) {
        return PARAM_ERROR;
    }

    plan->start_mm = current_mm;
    plan->target_mm = target_mm;
    plan->distance_mm = 0.0f;
    plan->dir = MOTOR_DIRECTION_UP;
    plan->already_reached = false;

    ret = MotorMotion_CheckAbsoluteTargetRange(target_mm);
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!MotorMotion_IsPositionSnapshotValid(current_mm)) {
        return MEASUREMENT_POSITION_ERROR;
    }

    if (eps_mm < 0.0f) {
        eps_mm = 0.0f;
    }
    delta_mm = target_mm - current_mm;
    if (fabsf(delta_mm) <= eps_mm) {
        plan->already_reached = true;
        return NO_ERROR;
    }

    plan->dir = (delta_mm > 0.0f) ? MOTOR_DIRECTION_UP : MOTOR_DIRECTION_DOWN;
    plan->distance_mm = fabsf(delta_mm);
    return NO_ERROR;
}

/**
 * @brief 检查电机控制中的 MotorMotion_CheckAbsoluteTargetRange 逻辑。
 *
 * @param target_mm 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t MotorMotion_CheckAbsoluteTargetRange(float target_mm)
{
    float tank_height_mm = (float)g_deviceParams.tankHeight / 10.0f;

    if (!isfinite(target_mm)) {
        printf("absolute target invalid | target=%.3fmm\r\n", target_mm);
        return PARAM_RANGE_ERROR;
    }
    if ((tank_height_mm <= 0.0f) || (tank_height_mm > 500000.0f)) {
        printf("absolute target invalid | target=%.3fmm | tankHeight=%.3fmm\r\n",
               target_mm,
               tank_height_mm);
        return PARAM_RANGE_ERROR;
    }
    if ((target_mm < 0.0f) || (target_mm > tank_height_mm)) {
        printf("absolute target out of range | target=%.3fmm | range=0.000~%.3fmm\r\n",
               target_mm,
               tank_height_mm);
        return PARAM_RANGE_ERROR;
    }

    return NO_ERROR;
}

/**
 * @brief 按方向计算当前位置到目标的剩余距离。
 */
static float MotorMotion_RemainingDistanceToTarget(float current_mm,
                                                   float target_mm,
                                                   int dir)
{
    float remaining_mm;

    if (dir == MOTOR_DIRECTION_UP) {
        remaining_mm = target_mm - current_mm;
    } else if (dir == MOTOR_DIRECTION_DOWN) {
        remaining_mm = current_mm - target_mm;
    } else {
        return 0.0f;
    }

    return (remaining_mm > 0.0f) ? remaining_mm : 0.0f;
}

/**
 * @brief 判断当前位置是否已经越过目标超过允许保护量。
 */
static bool MotorMotion_IsOvershotPastTarget(float current_mm,
                                             float target_mm,
                                             int dir,
                                             float limit_mm)
{
    if (limit_mm < 0.0f) {
        limit_mm = 0.0f;
    }

    if (dir == MOTOR_DIRECTION_UP) {
        return current_mm > (target_mm + limit_mm);
    }
    if (dir == MOTOR_DIRECTION_DOWN) {
        return current_mm < (target_mm - limit_mm);
    }
    return false;
}

/**
 * @brief 刷新 Jog 当前位置，并统一检查快照范围和越界保护。
 */
static uint32_t MotorMotion_RefreshJogPositionChecked(float *cur_mm,
                                                      float target_mm,
                                                      int dir)
{
    uint32_t ret;

    ret = MotorMotion_RefreshActivePositionMm(cur_mm);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!MotorMotion_IsPositionSnapshotValid(*cur_mm)) {
        return MEASUREMENT_POSITION_ERROR;
    }
    if (MotorMotion_IsOvershotPastTarget(*cur_mm,
                                         target_mm,
                                         dir,
                                         MOTOR_JOG_OVERSHOOT_LIMIT_MM)) {
        return MEASUREMENT_POSITION_ERROR;
    }

    return NO_ERROR;
}

/*
 * 函数用途：集中执行 Jog 循环每轮末尾的运行保护检查。
 * 调用场景：速度点动模式已经启动后，每轮位置、减速和停机判断完成之后。
 * 关键约束：不计算剩余距离、不切换速度、不下发停机；只返回应由上层慢停收尾的错误码。
 */
static uint32_t MotorMotion_CheckJogRuntimeGuards(uint32_t start_tick,
                                                  bool *is_moving)
{
    uint32_t ret;

    if (is_moving == NULL) {
        return PARAM_ERROR;
    }

    ret = MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_RUNNING);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = CheckWeightCollision();
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorCtrl_IsDriverMoving(&stepper, is_moving);
    if (ret != NO_ERROR) {
        return ret;
    }
    if ((!(*is_moving)) && ((HAL_GetTick() - start_tick) > MOTOR_JOG_START_GRACE_MS)) {
        return MOTOR_STEP_ERROR;
    }

    if ((HAL_GetTick() - start_tick) > MOTOR_JOG_MAX_RUN_MS) {
        return MOTOR_RUN_TIMEOUT;
    }

    return NO_ERROR;
}
/**
 * @brief 根据 TMC5130 当前斜坡参数估算点动模式提前减速距离。
 */
static uint32_t MotorMotion_CalcJogSlowdownDistanceMm(float current_mm,
                                                      uint32_t fast_velocity,
                                                      float *slowdown_mm)
{
    int32_t dmax_reg = 0;
    int32_t d1_reg = 0;
    int32_t amax_reg = 0;
    bool dmax_valid = false;
    bool d1_valid = false;
    bool amax_valid = false;
    uint32_t decel_reg = MOTOR_JOG_DECEL_DEFAULT_REG;
    uint32_t creep_velocity;
    double fast_usteps_s;
    double creep_usteps_s;
    double accel_usteps_s2;
    double ticks_per_rev;
    double circumference_mm;
    double brake_usteps;
    double brake_mm;
    double tape_length_mm;

    if (slowdown_mm == NULL) {
        return PARAM_ERROR;
    }

    *slowdown_mm = MOTOR_JOG_SLOWDOWN_DISTANCE_MM;
    if (fast_velocity == 0U) {
        return NO_ERROR;
    }

    /* current_mm 是传感器位置，卷筒速度换算必须使用当前尺带长度。 */
    tape_length_mm = ((double)g_deviceParams.tankHeight * 0.1) - (double)current_mm;
    if ((!isfinite(tape_length_mm)) || (tape_length_mm < 0.0)) {
        tape_length_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    }
    if (tape_length_mm < 0.0) {
        tape_length_mm = 0.0;
    }

    creep_velocity = MotorMotion_CalcVelocityFromSpeedX100(MOTOR_JOG_CREEP_SPEED_X100,
                                                           (float)tape_length_mm);
    if (creep_velocity == 0U) {
        return NO_ERROR;
    }

    dmax_valid = stpr_tryReadInt(&stepper, TMC5130_DMAX, &dmax_reg) &&
                 (dmax_reg > 0) &&
                 ((uint32_t)dmax_reg <= (uint32_t)TMC5130_MAX_ACCELERATION);
    d1_valid = stpr_tryReadInt(&stepper, TMC5130_D1, &d1_reg) &&
               (d1_reg > 0) &&
               ((uint32_t)d1_reg <= (uint32_t)TMC5130_MAX_ACCELERATION);
    amax_valid = stpr_tryReadInt(&stepper, TMC5130_AMAX, &amax_reg) &&
                 (amax_reg > 0) &&
                 ((uint32_t)amax_reg <= (uint32_t)TMC5130_MAX_ACCELERATION);

    if (dmax_valid) {
        decel_reg = (uint32_t)dmax_reg;
    } else if (d1_valid) {
        decel_reg = (uint32_t)d1_reg;
    } else if (amax_valid) {
        decel_reg = (uint32_t)amax_reg;
    }

    fast_usteps_s = MotorMotion_TmcVelocityToUstepsPerSec(fast_velocity);
    creep_usteps_s = MotorMotion_TmcVelocityToUstepsPerSec(creep_velocity);
    if (fast_usteps_s <= creep_usteps_s) {
        return NO_ERROR;
    }

    accel_usteps_s2 = MotorMotion_TmcAccelerationToUstepsPerSec2(decel_reg);
    ticks_per_rev = (double)MotorPosition_TapeTicksPerRev();
    circumference_mm = MotorPosition_TapeInstantCircumferenceFromLength(tape_length_mm);
    if ((accel_usteps_s2 <= 1e-6) || (ticks_per_rev <= 1e-6) || (circumference_mm <= 1e-6)) {
        return NO_ERROR;
    }

    brake_usteps = ((fast_usteps_s * fast_usteps_s) -
                    (creep_usteps_s * creep_usteps_s)) / (2.0 * accel_usteps_s2);
    brake_mm = (brake_usteps / ticks_per_rev) * circumference_mm;
    brake_mm = (brake_mm * (double)MOTOR_JOG_BRAKE_MARGIN_X100) / 100.0;

    if (brake_mm > (double)MOTOR_JOG_BRAKE_MAX_DISTANCE_MM) {
        brake_mm = (double)MOTOR_JOG_BRAKE_MAX_DISTANCE_MM;
    }
    if (brake_mm > (double)(*slowdown_mm)) {
        *slowdown_mm = (float)brake_mm;
    }

    printf("Jog slowdown calc | current=%.3fmm | tape=%.3fmm | fastVMAX=%lu | creepVMAX=%lu | decel=%lu | DMAX=%ld | D1=%ld | AMAX=%ld | slowdown=%.3fmm\r\n",
           current_mm,
           tape_length_mm,
           (unsigned long)fast_velocity,
           (unsigned long)creep_velocity,
           (unsigned long)decel_reg,
           (long)dmax_reg,
           (long)d1_reg,
           (long)amax_reg,
           *slowdown_mm);

    return NO_ERROR;
}
/**
 * @brief 将 TMC5130 VMAX 寄存器值换算为微步每秒。
 */
static double MotorMotion_TmcVelocityToUstepsPerSec(uint32_t vmax)
{
    return ((double)vmax * TMC5130_FCLK_HZ) / 16777216.0;
}

/**
 * @brief 将 TMC5130 加速度寄存器值换算为微步每秒平方。
 */
static double MotorMotion_TmcAccelerationToUstepsPerSec2(uint32_t accel)
{
    return ((double)accel * TMC5130_FCLK_HZ * TMC5130_FCLK_HZ) / 2199023255552.0;
}
/**
 * @brief 按给定线速度和当前位置直接计算 TMC5130 VMAX。
 */
static uint32_t MotorMotion_CalcVelocityFromSpeedX100(uint32_t speed_x100,
                                                      float current_mm)
{
    const double circumference_mm = MotorPosition_TapeInstantCircumferenceFromLength((double)current_mm);
    const double ticks_per_rev = (double)MotorPosition_TapeTicksPerRev();
    double usteps_per_s;
    double vmax;

    if ((circumference_mm <= 1e-6) || (ticks_per_rev <= 1e-6)) {
        return 0U;
    }

    usteps_per_s = ((double)speed_x100 * ticks_per_rev) / (6.0 * circumference_mm);
    vmax = usteps_per_s * 16777216.0 / TMC5130_FCLK_HZ;
    return MotorMotion_ClampVelocityDouble(vmax);
}

/**
 * @brief 将浮点 VMAX 限制到 TMC5130 速度寄存器范围。
 */
static uint32_t MotorMotion_ClampVelocityDouble(double vmax)
{
    if (vmax < 1.0) {
        return 1U;
    }
    if (vmax > (double)TMC5130_MAX_VELOCITY) {
        return (uint32_t)TMC5130_MAX_VELOCITY;
    }
    return (uint32_t)llround(vmax);
}
/**
 * @brief 按业务方向下发 TMC5130 速度模式点动命令。
 */
static uint32_t MotorMotion_StartJogVelocity(int dir)
{
    int32_t signed_velocity;

    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }
    if (velocity == 0U) {
        MotorDriver_UpdateVelocityFromParams();
    }
    if ((velocity == 0U) || (velocity > (uint32_t)INT32_MAX)) {
        return PARAM_ERROR;
    }

    signed_velocity = (int32_t)velocity;
    if (dir == MOTOR_DIRECTION_UP) {
        signed_velocity = -signed_velocity;
    }

    s_motor_driver.applied_velocity = velocity;
    return stpr_rotate(&stepper, signed_velocity);
}

/**
 * @brief 点动模式异常或到位退出时慢停并恢复进入前速度。
 */
static uint32_t MotorMotion_StopJogAndRestore(uint32_t ret,
                                              const MotorMotionSpeedScope *speed_scope,
                                              float target_mm,
                                              int dir)
{
    uint32_t stop_ret;
    uint32_t restore_ret;
    uint32_t verify_ret = NO_ERROR;
    float final_mm = 0.0f;

    stop_ret = MotorCtrl_SlowStop();
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if ((ret == NO_ERROR) && (stop_ret == NO_ERROR)) {
        verify_ret = MotorMotion_RefreshActivePositionMm(&final_mm);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if ((verify_ret == NO_ERROR) &&
            ((!MotorMotion_IsPositionSnapshotValid(final_mm)) ||
             (fabsf(final_mm - target_mm) > MOTOR_JOG_FINAL_ERROR_LIMIT_MM))) {
            printf("Jog final position error | target=%.3fmm | final=%.3fmm | error=%.3fmm | limit=%.3fmm\r\n",
                   target_mm,
                   final_mm,
                   fabsf(final_mm - target_mm),
                   MOTOR_JOG_FINAL_ERROR_LIMIT_MM);
            verify_ret = MEASUREMENT_POSITION_ERROR;
        }
    }

    restore_ret = MotorMotion_EndSpeedScope(speed_scope);

    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (stop_ret != NO_ERROR) {
        return stop_ret;
    }
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (verify_ret != NO_ERROR) {
        return verify_ret;
    }
    return restore_ret;
}
/**
 * @brief 等待电机停止，并允许命令切换打断。
 *
 * 等待过程中会轮询运动状态、刷新位置和检查命令切换。
 * @param poll_ms 轮询周期，单位 ms。
 * @return NO_ERROR、COMMAND_SWITCH_ABORT 或驱动错误码。
 */
static uint32_t MotorMotion_WaitStopAbortable(uint32_t poll_ms)
{
    uint32_t ret;
    uint32_t last_vel_refresh_tick = HAL_GetTick();
    bool is_moving = true;

    while (1) {
        ret = MotorCtrl_IsDriverMoving(&stepper, &is_moving);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        if (!is_moving) {
            break;
        }
        ret = MotorMotion_CheckAbortRefreshAndHealth(&stepper, &last_vel_refresh_tick);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        HAL_Delay(poll_ms);
    }
    return MotorMotion_CheckStoppedAndRefresh(&stepper);
}

/**
 * @brief 等待电机停止并校验目标位置。
 *
 * 等待过程中持续检查到位、命令切换、驱动异常和位置刷新。
 * @param tmc5130 TMC5130 设备对象。
 * @param target_mm 目标位置，单位 mm。
 * @param eps_mm 到位公差，单位 mm。
 * @param dir 运动方向。
 * @return 成功返回 NO_ERROR，否则返回超时、打断或驱动错误码。
 */
static uint32_t MotorMotion_WaitUntilStopWithTarget(TMC5130TypeDef *tmc5130,
                                                float target_mm,
                                                float eps_mm,
                                                int dir)
{
    uint32_t ret = NO_ERROR;
    uint32_t startTick = HAL_GetTick();
    uint32_t last_vel_refresh_tick = startTick;
    const uint32_t MAX_WAIT_MS = 60000 * 60;
    char detail[96];
    const uint32_t command_display_state = MotorMotion_DisplayStateFromDirection(dir);
    bool is_moving = true;

    while (1) {
        ret = MotorCtrl_IsDriverMoving(tmc5130, &is_moving);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        if (!is_moving) {
            break;
        }

        MotorMotion_SetActiveState(command_display_state, true);
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
        MotorDriver_RefreshVelocityDuringRun(tmc5130, &last_vel_refresh_tick);
        /* 每轮等待都刷新位置并检查驱动健康，覆盖运行中 24V 断电。 */
        ret = MotorDriver_SyncPositionOrCheckHealth(tmc5130);
        CHECK_ERROR(ret);

        /* 1) 当前位置（mm） */
        float cur_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

        /* 到位或越过目标都立即停机，停止收尾统一放在 helper 内。 */
        if (MotorMotion_ShouldStopAtTarget(cur_mm, target_mm, eps_mm, dir)) {
            return MotorMotion_StopAtTargetAndWait(tmc5130);
        }

        /* 4) 碰撞/极限检测 */
        ret = CheckWeightCollision();
        CHECK_ERROR(ret);

        /* 5) 驱动健康检测：统一检查 GSTAT/DRV_STATUS、配置和功率级状态。 */
        ret = MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_RUNNING);
        CHECK_ERROR(ret);

        /* 6) 超时保护 */
        if (HAL_GetTick() - startTick > MAX_WAIT_MS) {
            snprintf(detail, sizeof(detail),
                     "当前位置=%.2f,目标位置：%.2f,超时：%lums",
                     (double)cur_mm,
                     (double)target_mm,
                     (unsigned long)MAX_WAIT_MS);
            /* 错误 阶段：错误报警 模块：电机 操作：等待电机停止 原因：ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT) 处理：停止电机 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_WAIT_STOP,
                                ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT),
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            printf("TMC5130等待停止超时 | 当前位置=%.2fmm | 目标位置：%.2fmm | 超时：%lums\r\n",
                   (double)cur_mm,
                   (double)target_mm,
                   (unsigned long)MAX_WAIT_MS);
            RETURN_ERROR(MOTOR_RUN_TIMEOUT);
        }

        HAL_Delay(50);
    }

    ret = MotorMotion_CheckStoppedAndRefresh(tmc5130);
    CHECK_ERROR(ret);
    return NO_ERROR;
}

/**
 * @brief 停止命令下发后等待驱动真正进入停止状态。
 *
 * 用于急停和慢停后的状态收尾，避免刚下发停止就显示静止。
 * @param timeout_ms 最大等待时间，单位 ms。
 * @return 成功返回 NO_ERROR，否则返回超时或通信错误码。
 */
static uint32_t MotorMotion_WaitStoppedAfterStopCommand(uint32_t timeout_ms)
{
    uint32_t ret;
    uint32_t start_tick = HAL_GetTick();
    bool is_moving = true;
    char detail[80];

    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (timeout_ms == 0U) {
        timeout_ms = MOTOR_STOP_WAIT_TIMEOUT_MS;
    }

    while (1) {
        ret = MotorDriver_ReadStoppingState(&stepper, &is_moving);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            snprintf(detail, sizeof(detail), "moving state error=0x%08lX", (unsigned long)ret);
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_WAIT_STOP,
                                ERROR_LOG_REASON_COMM_FAIL,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            return ret;
        }

        if (!is_moving) {
            return MotorMotion_CheckStoppedAndRefresh(&stepper);
        }

        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            snprintf(detail, sizeof(detail),
                     "timeout=%lums",
                     (unsigned long)timeout_ms);
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_WAIT_STOP,
                                ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT),
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            printf("motor stop wait timeout | timeout=%lums\r\n", (unsigned long)timeout_ms);
            return MOTOR_RUN_TIMEOUT;
        }

        HAL_Delay(10U);
    }
}
