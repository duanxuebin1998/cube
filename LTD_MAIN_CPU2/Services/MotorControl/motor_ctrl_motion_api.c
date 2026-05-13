#include "motor_ctrl_internal.h"
#include "error_log.h"

/**
 * @file motor_ctrl_motion_api.c
 * @brief 电机运动接口、等待保护、停止接口和调试运动。
 *
 * 本文件负责把业务层的距离、目标位置或 ticks 指令下发到 TMC5130，并在阻塞等待
 * 过程中处理命令切换、撞底、驱动异常、丢步检测和运行期位置刷新。
 */

/* ===================== 私有类型/状态 ===================== */

static uint32_t MotorMotion_WaitStopAbortable(uint32_t poll_ms);
static uint32_t MotorMotion_WaitUntilStopWithTarget(TMC5130TypeDef *tmc5130,
                                                float target_mm,
                                                float eps_mm,
                                                int dir);
static uint32_t MotorMotion_WaitStoppedAfterStopCommand(uint32_t timeout_ms);

/* ===================== 私有函数声明 ===================== */

static uint32_t MotorMotion_WaitStopAbortable(uint32_t poll_ms);
static uint32_t MotorMotion_WaitUntilStopWithTarget(TMC5130TypeDef *tmc5130,
                                                float target_mm,
                                                float eps_mm,
                                                int dir);
static uint32_t MotorMotion_WaitStoppedAfterStopCommand(uint32_t timeout_ms);

/* ===================== 对外接口 ===================== */

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
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    int32_t xactual_before = 0;
    int32_t xactual_after = 0;
    int32_t xtarget_after = 0;
    int32_t vmax_after = 0;
    int32_t rampmode_after = 0;
    int32_t rampstat_after = 0;
    int32_t gstat_after = 0;
    char detail[128];
//    int32_t delta_ticks = ticks;

    ret = MotorDriver_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    CHECK_ERROR(ret);

    /* ticks 运动虽然不走“长度->圈数->ticks”的路径，
     * 但仍然要基于当前卷径刷新 VMAX，保证速度口径一致。 */
    MotorDriver_UpdateVelocityFromParams();

    ret = MotorDriver_StopIfCommandSwitchRequested();
    if (ret != NO_ERROR) {
        restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (restore_ret != NO_ERROR) {
            return restore_ret;
        }
        return ret;
    }

    /* 若方向相反，仅需在此统一翻转 */
    /* ticks = -ticks; */

    if (!stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual_before)) {
        restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
        (void)restore_ret;
        snprintf(detail, sizeof(detail), "增量：%ld", (long)ticks);
        // 错误	阶段：错误报警	模块：电机	操作：步进运动	原因：通信失败	处理：停止电机	详情：detail
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_STEP_MOTION,
                            ERROR_LOG_REASON_COMM_FAIL,
                            ERROR_LOG_ACTION_STOP_MOTOR,
                            detail);
        printf("步进运动失败 | 下发前XACTUAL读取失败 | 增量：%ld\r\n", (long)ticks);
        return MOTOR_TMC_COMM_ERROR;
    }
//    printf("ticks运动下发 | 增量：%ld | XACTUAL_before=%ld | VMAX=%lu | speed=%.2fm/min\r\n",
//           (long)delta_ticks,
//           (long)xactual_before,
//           (unsigned long)velocity,
//           (double)MotorDriver_GetSpeedSetpointX100() / 100.0);
    s_motor_driver.applied_velocity = velocity;
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    if (ret != NO_ERROR) {
        restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
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
//    printf("ticks运动读回 | 目标位置：%ld | 实际位置：%ld | 目标寄存器：%ld | 最大速度=%ld | 斜坡模式=%ld | 斜坡状态：0x%08lX | 全局状态：0x%08lX\r\n",
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
            ret = MotorDriver_StopIfCommandSwitchRequested();
            if (ret != NO_ERROR) {
                restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
                if (restore_ret != NO_ERROR) {
                    return restore_ret;
                }
                return ret;
            }
            MotorDriver_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
            MotorPosition_SyncDebugDrumState(&stepper);
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
            restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            (void)restore_ret;
            snprintf(detail, sizeof(detail),
                     "变化前：%ld,变化后：%ld,目标：%ld,目标寄存器：%ld,斜坡状态：0x%08lX,全局状态：0x%08lX",
                     (long)xactual_before,
                     (long)xactual_after,
                     (long)ticks,
                     (long)xtarget_after,
                     (unsigned long)rampstat_after,
                     (unsigned long)gstat_after);
            // 错误	阶段：错误报警	模块：电机	操作：步进运动	原因：ErrorLog_GetReasonByCode(MOTOR_STEP_ERROR)	处理：停止电机	详情：detail
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_STEP_MOTION,
                                ErrorLog_GetReasonByCode(MOTOR_STEP_ERROR),
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            printf("步进运动失败 | 500ms内XACTUAL未变化 | 变化前：%ld | 变化后：%ld | 目标：%ld | XTARGET=%ld | RAMPSTAT=0x%08lX | GSTAT=0x%08lX\r\n",
                   (long)xactual_before,
                   (long)xactual_after,
                   (long)ticks,
                   (long)xtarget_after,
                   (unsigned long)rampstat_after,
                   (unsigned long)gstat_after);
            return MOTOR_STEP_ERROR;
        }
    }

    ret = MotorMotion_WaitStopAbortable(10);
    if (ret == NO_ERROR) {
        uint32_t settle_start_tick = HAL_GetTick();
        uint32_t last_vel_refresh_tick = settle_start_tick;
        const int32_t target_ticks = ticks;
        const int32_t target_tolerance_ticks = 1024;

        /* YM 局部周长标定依赖下行一圈后的编码轮长度。
         * 仅看 RAMPSTAT.vzero 可能过早返回，这里再确认 XACTUAL 已到 XTARGET 附近。 */
        while (1) {
            int32_t xactual_now = 0;
            int32_t diff_ticks;

            CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
            if (!stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual_now)) {
                snprintf(detail, sizeof(detail),
                         "寄存器：XACTUAL,目标位置：%ld",
                         (long)target_ticks);
                // 错误	阶段：错误报警	模块：电机	操作：等待电机停止	原因：通信失败	处理：停止电机	详情：detail
                ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                    ERROR_LOG_OP_WAIT_STOP,
                                    ERROR_LOG_REASON_COMM_FAIL,
                                    ERROR_LOG_ACTION_STOP_MOTOR,
                                    detail);
                ret = MOTOR_TMC_COMM_ERROR;
                break;
            }

            diff_ticks = xactual_now - target_ticks;
            if (diff_ticks < 0) {
                diff_ticks = -diff_ticks;
            }
            if (diff_ticks <= target_tolerance_ticks) {
                break;
            }
            if ((HAL_GetTick() - settle_start_tick) > MOTOR_STOP_WAIT_TIMEOUT_MS) {
                snprintf(detail, sizeof(detail),
                         "实际位置：%ld,目标位置：%ld,差值：%ld,超时：%lums",
                         (long)xactual_now,
                         (long)target_ticks,
                         (long)diff_ticks,
                         (unsigned long)MOTOR_STOP_WAIT_TIMEOUT_MS);
                // 错误	阶段：错误报警	模块：电机	操作：等待电机停止	原因：ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT)	处理：停止电机	详情：detail
                ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                    ERROR_LOG_OP_WAIT_STOP,
                                    ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT),
                                    ERROR_LOG_ACTION_STOP_MOTOR,
                                    detail);
                printf("ticks运动等待到位超时 | 实际位置：%ld | 目标位置：%ld | 差值：%ld\r\n",
                       (long)xactual_now,
                       (long)target_ticks,
                       (long)diff_ticks);
                ret = MOTOR_RUN_TIMEOUT;
                break;
            }
            MotorDriver_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
            MotorPosition_SyncDebugDrumState(&stepper);
            HAL_Delay(10U);
        }
    }

    restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
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
    if (move_mm < 0.0f) return PARAM_ERROR;
    if (move_mm == 0.0f) return NO_ERROR;
    if (!MotorDriver_IsDirValid(dir)) return PARAM_ERROR;

    uint32_t ret = MotorDriver_ApplyOptionalSpeed(speed_x100);
    CHECK_ERROR(ret);

    g_measurement.debug_data.motor_state = (dir == MOTOR_DIRECTION_UP) ? 1U : 2U;
    MotorPosition_SyncDebugDrumState(&stepper);

    /* 1) 当前“相对基准点”的有符号长度（mm，可正可负） */
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_driver.applied_velocity = velocity;

    /* 2) 目标有符号长度（mm） */
    double dL_mm = (double)move_mm;
    if (dir == MOTOR_DIRECTION_UP) {
        dL_mm = -dL_mm;  // 上行：长度更小（可继续变负）
    }
    const double Ltar_mm = Lcur_mm + dL_mm;

    int64_t ticks64;

    {
        double local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();
        if ((g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
            (local_circumference_mm > 1e-6)) {
            ticks64 = MotorPosition_RoundToInt64((dL_mm / local_circumference_mm) *
                                   (double)MotorPosition_TapeTicksPerRev());
        } else {
            const double C0 = MotorPosition_TapeC0Mm();
            const double t  = MotorPosition_TapeThicknessMm();
            if (C0 <= 0.0) return PARAM_ERROR;

            const double ncur = MotorPosition_TapeTurnsFromSignedLength(Lcur_mm, C0, t);
            const double ntar = MotorPosition_TapeTurnsFromSignedLength(Ltar_mm, C0, t);
            const double dn   = ntar - ncur;

            double ticks_d = dn * (double)MotorPosition_TapeTicksPerRev();
            ticks64 = MotorPosition_RoundToInt64(ticks_d);
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
    MotorPosition_SyncDebugDrumState(&stepper);

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
    /* 标定零点流程会切回编码轮记步，此处同步清掉旧的电机局部周长，
     * 让后续再切回电机记步时从新的零点首圈周长开始。 */
    (void)MotorPosition_SetLocalCircumferenceToParams(C0);
    ret = MotorCtrl_MoveAndWait(dL, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100()); // 回到起点
    if (ret != NO_ERROR) {
        printf("首圈周长标定失败 | 回到起点失败 错误码：0x%08lX\r\n", (unsigned long)ret);
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

/**
 * @brief 按距离运动并阻塞等待停止。
 *
 * 该接口是业务流程推荐的距离运动入口，内部包含重试和保护检测。
 * @param mm 移动距离，单位 mm。
 * @param dir 运动方向。
 * @param speed_x100 本次运动速度，0 表示使用默认速度。
 * @return 成功返回 NO_ERROR，否则返回运动、保护或打断错误码。
 */
uint32_t MotorCtrl_MoveAndWait(float mm, int dir, uint32_t speed_x100)
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
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
    }

    /* 阻塞型接口支持“本次命令临时速度”：
     * 开始前切到 speed_x100，结束后恢复到默认最大速度。 */
    ret = MotorDriver_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    CHECK_ERROR(ret);

    ret = MotorDriver_StopIfCommandSwitchRequested();
    if (ret != NO_ERROR) {
        restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (restore_ret != NO_ERROR) {
            return restore_ret;
        }
        return ret;
    }

    startPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

    /* 你现场口径：下行 pos 变小，上行 pos 变大 */
    targetPos_mm = startPos_mm +
                   ((dir == MOTOR_DIRECTION_UP) ? total_cmd_mm : -total_cmd_mm);

    remain_mm = total_cmd_mm;

    while (attempt < MOTOR_MOVE_RETRY_MAX && remain_mm > 0.0f) {

        attempt++;

        ret = MotorDriver_StopIfCommandSwitchRequested();
        if (ret != NO_ERROR) {
            restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (restore_ret != NO_ERROR) {
                return restore_ret;
            }
            return ret;
        }

        /* 分段重试时不再重复切换速度，避免每一段都触发“恢复默认速度”。 */
        ret = MotorCtrl_MoveNoWait(remain_mm, dir, 0U);
        CHECK_ERROR(ret);

        /* 下发后先进入一个短暂的“启动观察窗口”，
         * 期间持续做速度补偿，让起步阶段也尽快贴合目标线速度。 */
        uint32_t prewait_vel_refresh_tick = HAL_GetTick();
        for (int i = 0; i < 100; i++) {
            ret = MotorDriver_StopIfCommandSwitchRequested();
            if (ret != NO_ERROR) {
                restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
                if (restore_ret != NO_ERROR) {
                    return restore_ret;
                }
                return ret;
            }
            MotorDriver_RefreshVelocityDuringRun(&stepper, &prewait_vel_refresh_tick);
            MotorCtrl_PollRuntimePosition();
            HAL_Delay(10);
        }

        ret = MotorMotion_WaitUntilStopWithTarget(&stepper, targetPos_mm, EPS_MM, dir);

        if (ret == COMMAND_SWITCH_ABORT) {
            restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (restore_ret != NO_ERROR) {
                return restore_ret;
            }
            return ret;
        }
        ret = MotorDriver_StopIfCommandSwitchRequested();
        if (ret != NO_ERROR) {
            restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (restore_ret != NO_ERROR) {
                return restore_ret;
            }
            return ret;
        }

        currentPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

        moved_mm = fabsf(currentPos_mm - startPos_mm);

        remain_mm = fabsf(targetPos_mm - currentPos_mm);
        if (remain_mm < EPS_MM) remain_mm = 0.0f;

        if (ret == NO_ERROR) {
            if (attempt > 1U) {
                // 错误	阶段：重试成功	模块：电机	操作：步进运动	原因：恢复成功	尝试：attempt/MOTOR_MOVE_RETRY_MAX
                ErrorLog_Recover(ERROR_LOG_MODULE_MOTOR,
                                 ERROR_LOG_OP_STEP_MOTION,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)attempt,
                                 MOTOR_MOVE_RETRY_MAX);
            }

            MotorDrumState drum;
            MotorCtrl_UpdateDrumStateFromXActual(&stepper, &drum);
//            printf("电机状态 | XACTUAL=%ld | 圈=%.4f | 角度=%.1f° | 预测长度：%.1fmm\r\n",
//                   (long)drum.motor_step,
//                   drum.turns_total,
//                   drum.angle_deg,
//                   drum.motor_distance_01mm / 10.0);

            break;
        } else {

            if (attempt >= MOTOR_MOVE_RETRY_MAX) {
                restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
                if ((restore_ret != NO_ERROR) && (ret == NO_ERROR)) {
                    ret = restore_ret;
                }
                CHECK_ERROR(ret);
                return ret;
            }

            // 错误	阶段：错误重试	模块：电机	操作：步进运动	原因：ErrorLog_GetReasonByCode(ret)	尝试：attempt/MOTOR_MOVE_RETRY_MAX	错误码：ret	错误名：ErrorLog_GetCodeName(ret)
            ErrorLog_Retry(ERROR_LOG_MODULE_MOTOR,
                           ERROR_LOG_OP_STEP_MOTION,
                           ErrorLog_GetReasonByCode(ret),
                           (uint32_t)attempt,
                           MOTOR_MOVE_RETRY_MAX,
                           ret);

            for (int i = 0; i < 20; i++) {
                ret = MotorDriver_StopIfCommandSwitchRequested();
                if (ret != NO_ERROR) {
                    restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
                    if (restore_ret != NO_ERROR) {
                        return restore_ret;
                    }
                    return ret;
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

    MotorPosition_SyncDebugDrumState(&stepper);

    /* 只有主命令整体结束后，才恢复默认速度。 */
    restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
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

/**
 * @brief 获取当前传感器位置快照。
 *
 * @param pos_mm 输出当前位置，单位 mm；传入 NULL 时不执行操作。
 */
void MotorCtrl_SnapshotSensorPositionMm(float *pos_mm)
{
    uint32_t pos_01mm = (uint32_t)g_measurement.debug_data.sensor_position;
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
    uint32_t restore_ret = NO_ERROR;
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    float cur_mm;

    ret = MotorDriver_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    CHECK_ERROR(ret);

    /* 绝对位置运动用“小步逼近”的方式做，
     * 每一轮都重新读取当前位置，降低位置更新滞后带来的影响。 */
    for (int i = 0; i < 10; i++) {

        ret = MotorDriver_StopIfCommandSwitchRequested();
        if (ret != NO_ERROR) {
            restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (restore_ret != NO_ERROR) {
                return restore_ret;
            }
            return ret;
        }

        MotorCtrl_SnapshotSensorPositionMm(&cur_mm);
        printf("运动到位置 | 当前：%.3fmm | 目标：%.3fmm\r\n", cur_mm, target_mm);

        float delta = target_mm - cur_mm;

        if (fabsf(delta) <= EPS_MM) {
            printf("到位 | 当前：%.3fmm ≈ 目标：%.3fmm (±%.2fmm)\r\n",
                   cur_mm, target_mm, EPS_MM);
            restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            CHECK_ERROR(restore_ret);
            return restore_ret;
        }

        int dir = (delta > 0.0f) ? MOTOR_DIRECTION_UP : MOTOR_DIRECTION_DOWN;

        float plan_mm = fabsf(delta);
        if (plan_mm < (EPS_MM * 2.0f)) {
            plan_mm = (EPS_MM * 2.0f);
        }
        ret = MotorCtrl_MoveAndWait(plan_mm, dir, 0U);
        if (ret != NO_ERROR) {
            restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (restore_ret != NO_ERROR) {
                return restore_ret;
            }
            return ret; // 含 COMMAND_SWITCH_ABORT
        }
    }

    restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
    CHECK_ERROR(restore_ret);
    return restore_ret;
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
    if (ret != NO_ERROR) {
        return ret;
    }

    stpr_disableDriver(&stepper);
    HAL_Delay(4000);
    stpr_enableDriver(&stepper);

    MotorPosition_SyncDebugDrumState(&stepper);
    g_measurement.debug_data.motor_state = 0U;
    return MotorCtrl_SetSpeed(g_deviceParams.max_motor_speed);
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
    if (ret != NO_ERROR) {
        return ret;
    }

    /* 慢停后必须等 RAMPSTAT.vzero 确认停稳，再恢复软件速度设定。
     * 否则 TMC5130 仍处于速度模式时写回 VMAX，可能导致电机继续运行。 */
    ret = MotorMotion_WaitStoppedAfterStopCommand(MOTOR_STOP_WAIT_TIMEOUT_MS);
    if (ret != NO_ERROR) {
        return ret;
    }

    return MotorCtrl_SetSpeed(g_deviceParams.max_motor_speed);
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

    if (!s_motor_driver.initialized) {
        return ((motor_state == 1U) || (motor_state == 2U)) ? motor_state : 0U;
    }

    if (!MotorDriver_TryReadMovingState(&stepper, &is_moving)) {
        uint32_t inferred_state = MotorDriver_InferDisplayStateFromDriver(&stepper);
        if ((inferred_state == 1U) || (inferred_state == 2U)) {
            g_measurement.debug_data.motor_state = inferred_state;
            return inferred_state;
        }
        return ((motor_state == 1U) || (motor_state == 2U)) ? motor_state : 0U;
    }

    if (!is_moving) {
        if ((motor_state == 1U) || (motor_state == 2U)) {
            g_measurement.debug_data.motor_state = 0U;
        }
        return 0U;
    }

    if ((motor_state == 1U) || (motor_state == 2U)) {
        return motor_state;
    }

    motor_state = MotorDriver_InferDisplayStateFromDriver(&stepper);
    if ((motor_state == 1U) || (motor_state == 2U)) {
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
    uint32_t restore_speed_x100 = 0U;
    bool restore_needed = false;
    uint32_t ret;

    if (mm <= 0.0f) return PARAM_ERROR;
    if (!MotorDriver_IsDirValid(dir)) return PARAM_ERROR;

    /* 该接口是“无检测”版本，只保留基础运动和日志。
     * 但为了让语义一致，临时速度恢复策略仍然与其他阻塞接口保持一致。 */
    ret = MotorDriver_BeginTemporarySpeed(speed_x100, &restore_needed, &restore_speed_x100);
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：速度设置失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    printf("无检测阻塞运动：距离=%.2f, 方向：%d\r\n", mm, dir);

    g_measurement.debug_data.motor_state = (dir == MOTOR_DIRECTION_UP) ? 1U : 2U;
    MotorPosition_SyncDebugDrumState(&stepper);

    const double C0 = MotorPosition_TapeC0Mm();
    const double t  = MotorPosition_TapeThicknessMm();
    const double local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();
    const bool use_local_circ =
        (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
        (local_circumference_mm > 1e-6);
    if ((!use_local_circ) && (C0 <= 0.0)) {
        ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (ret != NO_ERROR) {
            printf("无检测阻塞运动：恢复默认速度失败 错误码：0x%08lX\r\n", (unsigned long)ret);
            return ret;
        }
        return PARAM_ERROR;
    }

    /* 当前有符号长度 */
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_driver.applied_velocity = velocity;

    /* 目标有符号长度 */
    double dL = (double)mm;
    if (dir == MOTOR_DIRECTION_UP) dL = -dL;
    const double Ltar_mm = Lcur_mm + dL;

    int64_t ticks64;

    if (use_local_circ) {
        ticks64 = MotorPosition_RoundToInt64((dL / local_circumference_mm) *
                               (double)MotorPosition_TapeTicksPerRev());
    } else {
        const double ncur = MotorPosition_TapeTurnsFromSignedLength(Lcur_mm, C0, t);
        const double ntar = MotorPosition_TapeTurnsFromSignedLength(Ltar_mm, C0, t);
        const double dn   = ntar - ncur;
        ticks64 = MotorPosition_RoundToInt64(dn * (double)MotorPosition_TapeTicksPerRev());
    }

    if (ticks64 > (int64_t)INT32_MAX) ticks64 = (int64_t)INT32_MAX;
    if (ticks64 < (int64_t)INT32_MIN) ticks64 = (int64_t)INT32_MIN;
    int32_t ticks = (int32_t)ticks64;

    ret = MotorDriver_StopIfCommandSwitchRequested();
    if (ret != NO_ERROR) {
        uint32_t stop_ret = ret;
        ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
        if (ret != NO_ERROR) {
            printf("无检测阻塞运动：恢复默认速度失败 错误码：0x%08lX\r\n", (unsigned long)ret);
            return ret;
        }
        return stop_ret;
    }
    ret = stpr_moveBy(&stepper, &ticks, velocity);
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：目标位置越界 错误码：0x%08lX\r\n", (unsigned long)ret);
        (void)MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
        return ret;
    }
    MotorPosition_SyncDebugDrumState(&stepper);
    HAL_Delay(100);

    uint32_t last_vel_refresh_tick = HAL_GetTick();
    while (MotorCtrl_IsDriverMoving(&stepper)) {
        ret = MotorDriver_StopIfCommandSwitchRequested();
        if (ret != NO_ERROR) {
            uint32_t stop_ret = ret;
            ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
            if (ret != NO_ERROR) {
                printf("无检测阻塞运动：恢复默认速度失败 错误码：0x%08lX\r\n", (unsigned long)ret);
                return ret;
            }
            return stop_ret;
        }
        MotorDriver_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        MotorPosition_SyncDebugDrumState(&stepper);
        MotorLostStep_NoDetectRuntimeLogUpdate();
    }

    g_measurement.debug_data.motor_state = 0U;
    MotorPosition_SyncDebugDrumState(&stepper);

    ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);
    if (ret != NO_ERROR) {
        printf("无检测阻塞运动：恢复默认速度失败 错误码：0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    return NO_ERROR;
}

/* ===================== 私有函数实现 ===================== */

/**
 * @brief 等待电机停止，并允许命令切换打断。
 *
 * 等待过程中会轮询运动状态、刷新位置和检查命令切换。
 * @param poll_ms 轮询周期，单位 ms。
 * @return NO_ERROR、COMMAND_SWITCH_ABORT 或驱动错误码。
 */
static uint32_t MotorMotion_WaitStopAbortable(uint32_t poll_ms)
{
    uint32_t last_vel_refresh_tick = HAL_GetTick();

    while (MotorCtrl_IsDriverMoving(&stepper)) {
        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
        MotorDriver_RefreshVelocityDuringRun(&stepper, &last_vel_refresh_tick);
        MotorPosition_SyncDebugDrumState(&stepper);
        HAL_Delay(poll_ms);
    }
    MotorPosition_SyncDebugDrumState(&stepper);
    g_measurement.debug_data.motor_state = 0U;
    return NO_ERROR;
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

    while (MotorCtrl_IsDriverMoving(tmc5130)) {

        CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
        MotorDriver_RefreshVelocityDuringRun(tmc5130, &last_vel_refresh_tick);
        MotorPosition_SyncDebugDrumState(tmc5130);

        /* 1) 当前位置（mm） */
        float cur_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;

        /* 2) 到位（容差）-> 立即停机并返回成功 */
        if (fabsf(cur_mm - target_mm) <= eps_mm) {
            ret = MotorDriver_StopAndMarkStopped();
            if (ret != NO_ERROR) {
                return ret;
            }
            while (MotorCtrl_IsDriverMoving(tmc5130)) {
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
                ret = MotorDriver_StopAndMarkStopped();
                if (ret != NO_ERROR) {
                    return ret;
                }
                while (MotorCtrl_IsDriverMoving(tmc5130)) {
                    CHECK_COMMAND_SWITCH_AND_STOP(COMMAND_SWITCH_ABORT);
                    HAL_Delay(5);
                }
                return NO_ERROR;
            }
        } else {
            if (cur_mm >= target_mm) {
                ret = MotorDriver_StopAndMarkStopped();
                if (ret != NO_ERROR) {
                    return ret;
                }
                while (MotorCtrl_IsDriverMoving(tmc5130)) {
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
       ret = MotorCtrl_CheckDriverGstat(tmc5130);
       CHECK_ERROR(ret);

        /* 6) 超时保护 */
        if (HAL_GetTick() - startTick > MAX_WAIT_MS) {
            snprintf(detail, sizeof(detail),
                     "当前位置=%.2f,目标位置：%.2f,超时：%lums",
                     (double)cur_mm,
                     (double)target_mm,
                     (unsigned long)MAX_WAIT_MS);
            // 错误	阶段：错误报警	模块：电机	操作：等待电机停止	原因：ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT)	处理：停止电机	详情：detail
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

    MotorPosition_SyncDebugDrumState(tmc5130);
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
    uint32_t start_tick = HAL_GetTick();
    int32_t rampstat = 0;
    char detail[80];

    if (timeout_ms == 0U) {
        timeout_ms = MOTOR_STOP_WAIT_TIMEOUT_MS;
    }

    while (1) {
        if (!stpr_tryReadInt(&stepper, TMC5130_RAMPSTAT, &rampstat)) {
            snprintf(detail, sizeof(detail), "寄存器：RAMPSTAT");
            // 错误	阶段：错误报警	模块：电机	操作：等待电机停止	原因：通信失败	处理：停止电机	详情：detail
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_WAIT_STOP,
                                ERROR_LOG_REASON_COMM_FAIL,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            printf("电机停止等待失败：RAMPSTAT读取失败\r\n");
            return MOTOR_TMC_COMM_ERROR;
        }

        if ((rampstat & 0x400) == 0x400) {
            MotorPosition_SyncDebugDrumState(&stepper);
            g_measurement.debug_data.motor_state = 0U;
            return NO_ERROR;
        }

        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            snprintf(detail, sizeof(detail),
                     "斜坡状态：0x%08lX,超时：%lums",
                     (unsigned long)rampstat,
                     (unsigned long)timeout_ms);
            // 错误	阶段：错误报警	模块：电机	操作：等待电机停止	原因：ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT)	处理：停止电机	详情：detail
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_WAIT_STOP,
                                ErrorLog_GetReasonByCode(MOTOR_RUN_TIMEOUT),
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            printf("电机停止等待超时：RAMPSTAT=0x%08lX\r\n", (unsigned long)rampstat);
            return MOTOR_RUN_TIMEOUT;
        }

        HAL_Delay(10);
    }
}
