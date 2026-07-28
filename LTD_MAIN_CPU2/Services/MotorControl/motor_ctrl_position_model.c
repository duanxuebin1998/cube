#include "motor_ctrl_internal.h"

/**
 * @file motor_ctrl_position_model.c
 * @brief 尺带数学模型、卷筒状态、位置源切换和位置持久化。
 *
 * 本文件统一维护 XACTUAL 与尺带长度之间的换算关系，处理编码轮记步和电机记步
 * 两种位置源的切换、恢复、诊断打印和 FRAM 持久化。这里的单位口径必须与
 * motor_ctrl.h 中的公开说明保持一致。
 */

/* ===================== 私有类型/状态 ===================== */

/* 电机位置持久化恢复缓存，由 MotorPosition_RestorePersistedRegisters() 写入。 */
static int32_t s_motor_saved_xactual = 0; /* 电机控制模块级变量，保存跨函数共享的业务状态。 */
static int32_t s_motor_restored_base_length_01mm = 0; /* 电机控制模块级变量，保存跨函数共享的业务状态。 */
static int32_t s_motor_restored_base_step = 0; /* 电机控制模块级变量，保存跨函数共享的业务状态。 */
static bool s_motor_restored_base_valid = false; /* 电机控制模块级变量，保存跨函数共享的业务状态。 */

/* XACTUAL 单次读取可能因为 SPI 帧错位出现 0 或极大跳变。
 * 这里用很宽的阈值只拦截明显不可能的单帧异常，真实大位移会通过二次读取确认。 */
#define MOTOR_XACTUAL_SUSPECT_JUMP_TICKS   (MotorPosition_TapeTicksPerRev() * 2L) /* XACTUAL 可疑跳变判定阈值，单位 tick。 */

/* ===================== 私有函数声明 ===================== */

static void MotorPosition_SavePositionSourceParams(bool force);
static bool MotorPosition_IsEncoderErrorCode(uint32_t error_code);
static bool MotorPosition_TryUpdateDrumStateFromXactual(TMC5130TypeDef *tmc5130,
                                                 MotorDrumState *out);
static bool MotorPosition_FilterXactualGlitch(TMC5130TypeDef *tmc5130,
                                              int32_t *motor_step);
static void MotorPosition_MaybePersistRegisters(TMC5130TypeDef *tmc5130, bool force);
static uint32_t MotorPosition_EncodeLocalCircumferenceParam(double circumference_mm);
static double MotorPosition_DecodeLocalCircumferenceParam(void);
static void MotorPosition_TapeMinRadiusThreshold(double C0_mm, double t_mm,
                                       double *n1, double *L1, double *Cmin);
static double MotorPosition_TapeSignedLengthFromTurns(double n,
                                                   double C0_mm,
                                                   double t_mm);
static int64_t MotorPosition_PersistDeltaTicksForLength(int32_t xactual);
static void MotorPosition_BuildDrumStateFromStep(int32_t motor_step, MotorDrumState *out);
static uint32_t MotorPosition_PersistRecordCrc(const MotorPersistRecord *record);
static int MotorPosition_ReadPersistFromSlot(uint32_t base_addr,
                                     int32_t *xactual,
                                     int32_t *base_length_01mm,
                                     int32_t *base_step,
                                     const char *slot_name);
static bool MotorPosition_VerifyPersistSlot(uint32_t base_addr,
                                            const MotorPersistRecord *expected);static bool MotorPosition_WritePersistAB(int32_t xactual,
                                  int32_t base_length_01mm,
                                  int32_t base_step);
static int MotorPosition_ReadPersistAB(int32_t *xactual,
                               int32_t *base_length_01mm,
                               int32_t *base_step);
static void MotorPosition_StorePersistSnapshot(int32_t xactual);
static uint32_t MotorPosition_RollbackPositionSourceSwitch(uint32_t ret,
                                                   uint32_t old_mode,
                                                   uint32_t old_local_circ_param,
                                                   int32_t old_base_step,
                                                   int32_t old_base_length_01mm,
                                                   double old_base_turns,
                                                   uint32_t old_error_code);

/* ===================== 对外接口 ===================== */

/**
 * @brief 应用上位机或 CPU3 写入的位置源相关参数。
 *
 * 函数会修正非法记步模式和局部周长；若当前使用电机记步，会立即刷新推算位置。
 */
void MotorCtrl_ApplyPositionSourceParams(void)
{
    MotorDrumState drum;
    double local_circumference_mm;

    if (g_deviceParams.position_count_mode > POSITION_COUNT_MODE_MOTOR) {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
    }

    local_circumference_mm = MotorPosition_DecodeLocalCircumferenceParam();
    if (local_circumference_mm <= 0.0) {
        g_deviceParams.motor_count_first_loop_circumference_mm =
            g_deviceParams.first_loop_circumference_mm * 100U;
        local_circumference_mm = MotorPosition_DecodeLocalCircumferenceParam();
    }

    if ((g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
        s_motor_driver.initialized) {
        MotorCtrl_UpdateDrumStateFromXActual(&stepper, &drum);
        g_measurement.debug_data.motor_step = drum.motor_step;
        g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;
        MotorPosition_UpdatePositionFromMotorSource(&drum);
    }
}

/**
 * @brief 判断当前是否使用电机记步作为位置源。
 *
 * @return 电机记步返回 true，编码轮记步返回 false。
 */
bool MotorCtrl_IsPositionSourceMotor(void)
{
    return g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR;
}

/**
 * @brief 从 TMC5130_XACTUAL 计算并输出卷筒状态。
 *
 * 兼容旧接口：读取失败时使用最近缓存状态填充输出。
 * @param tmc5130 TMC5130 设备对象。
 * @param out 输出卷筒状态。
 */
void MotorCtrl_UpdateDrumStateFromXActual(TMC5130TypeDef *tmc5130,
                                      MotorDrumState *out)
{
    if (!out) return;

    if (!MotorPosition_TryUpdateDrumStateFromXactual(tmc5130, out)) {
        /* 兼容旧接口：读取失败时用上一次缓存值填充，但不代表本次读到了真实 0。 */
        MotorPosition_BuildDrumStateFromStep(g_measurement.debug_data.motor_step, out);
        out->motor_distance_01mm = g_measurement.debug_data.motor_distance;
    }
}

/**
 * @brief 强制刷新调试用电机卷筒状态。
 *
 * 读取 XACTUAL 并更新 debug_data 中的 motor_step 和 motor_distance。
 */
void MotorCtrl_RefreshDebugDrumState(void)
{
    MotorPosition_SyncDebugDrumState(&stepper);
}

/**
 * @brief 按当前记步源强制刷新业务位置。
 *
 * 电机记步模式下读取 TMC5130 XACTUAL 并刷新 cable_length/sensor_position；
 * 编码轮记步模式下按编码轮重新计算当前位置。
 */
void MotorCtrl_RefreshPositionFromActiveSource(void)
{
    if (MotorCtrl_IsPositionSourceMotor()) {
        (void)MotorPosition_SyncDebugDrumState(&stepper);
    } else {
        update_sensor_height_from_encoder_force();
    }
}

/**
 * @brief 打印当前编码轮/电机位置参考信息。
 *
 * 用于现场快速确认记步模式、编码轮长度、电机模型长度和局部周长。
 */
void MotorCtrl_PrintPositionRefs(void)
{
    double motor_cable_mm;
    double encoder_cable_mm;

    MotorCtrl_RefreshDebugDrumState();
    /* 打印口径按当前记步源切换：
     * - 电机记步：打印业务实际使用的电机源长度，即“切换时编码轮尺带长度 + XACTUAL 相对变化量”。
     * - 编码轮记步：打印纯 XACTUAL 按全局卷筒模型推算的长度，用于和编码轮尺带对比。 */
    if (MotorCtrl_IsPositionSourceMotor()) {
        motor_cable_mm = (double)g_measurement.debug_data.cable_length / 10.0;
    } else {
        motor_cable_mm = (double)g_measurement.debug_data.motor_distance / 10.0;
    }
    encoder_cable_mm = (double)encoder_get_cable_length_01mm() / 10.0;

    printf("\t{编码模式}%s\t{电机尺带}\t%.1f\t{编码轮尺带}\t%.1f",
           MotorCtrl_IsPositionSourceMotor() ? "电机记步" : "编码轮记步",
           motor_cable_mm,
           encoder_cable_mm);
}

/**
 * @brief 打印编码轮位置与电机推算位置的对比。
 *
 * 用于排查两套位置源是否发生偏差。
 */
void MotorCtrl_PrintPositionCompare(void)
{
    MotorDrumState drum;
    int32_t xactual;
    int32_t motor_source_length_01mm;
    int32_t motor_source_position_01mm;
    int32_t encoder_length_01mm;
    int32_t encoder_position_01mm;
    int32_t length_diff_01mm;
    int32_t position_diff_01mm;
    double delta_turns;
    double local_circumference_mm;
    bool xactual_ok;

    encoder_length_01mm = encoder_get_cable_length_01mm();
    encoder_position_01mm = encoder_get_sensor_position_01mm();
    xactual_ok = MotorPosition_TryUpdateDrumStateFromXactual(&stepper, &drum);
    local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();

    printf("电机/编码位置对比 | 模式=%lu(%s) | 局部周长原始值=%lu(0.001mm) | 局部周长=%.3fmm\r\n",
           (unsigned long)g_deviceParams.position_count_mode,
           MotorCtrl_IsPositionSourceMotor() ? "电机记步" : "编码轮记步",
           (unsigned long)g_deviceParams.motor_count_first_loop_circumference_mm,
           local_circumference_mm);
    printf("电机/编码基准 | 基准步数=%ld | 基准长度：%.1fmm | 基准圈数=%.6f\r\n",
           (long)s_motor_position.count_base_step,
           (double)s_motor_position.count_base_length_01mm * 0.1,
           s_motor_position.count_base_turns);

    if (!xactual_ok) {
        printf("电机/编码位置对比 | XACTUAL读取失败，保留旧电机位置缓存 | 编码轮长度：%.1fmm | 编码轮位置=%.1fmm\r\n",
               (double)encoder_length_01mm * 0.1,
               (double)encoder_position_01mm * 0.1);
        return;
    }

    xactual = drum.motor_step;
    delta_turns = (double)((int64_t)xactual - (int64_t)s_motor_position.count_base_step) /
                  (double)MotorPosition_TapeTicksPerRev();

    /* 电机源位置按“切换时编码轮尺带长度 + XACTUAL 相对变化量”计算。
     * 有有效局部周长时沿用电机记步局部线性换算；否则回退到卷径模型。 */
    if (local_circumference_mm > 1e-6) {
        const double motor_length_mm = ((double)s_motor_position.count_base_length_01mm * 0.1) +
                                       (delta_turns * local_circumference_mm);
        motor_source_length_01mm = (int32_t)llround(motor_length_mm * 10.0);
    } else {
        double base_model_mm;
        double current_model_mm;
        const double turns = s_motor_position.count_base_turns + delta_turns;

        base_model_mm = MotorPosition_TapeSignedLengthFromTurns(s_motor_position.count_base_turns,
                                                      MotorPosition_TapeC0Mm(),
                                                      MotorPosition_TapeThicknessMm());
        current_model_mm = MotorPosition_TapeSignedLengthFromTurns(turns,
                                                         MotorPosition_TapeC0Mm(),
                                                         MotorPosition_TapeThicknessMm());
        motor_source_length_01mm = s_motor_position.count_base_length_01mm +
            (int32_t)llround((current_model_mm - base_model_mm) * 10.0);
    }
    motor_source_position_01mm = (int32_t)g_deviceParams.tankHeight - motor_source_length_01mm;
    length_diff_01mm = motor_source_length_01mm - encoder_length_01mm;
    position_diff_01mm = motor_source_position_01mm - encoder_position_01mm;

    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    printf("电机寄存器 | XACTUAL=%ld | 模型长度：%.1fmm | 圈数=%.6f | 角度=%.2f度\r\n",
           (long)xactual,
           (double)drum.motor_distance_01mm * 0.1,
           drum.turns_total,
           drum.angle_deg);
    printf("位置计算 | 电机长度：%.1fmm | 编码轮长度：%.1fmm | 差值：%.1fmm\r\n",
           (double)motor_source_length_01mm * 0.1,
           (double)encoder_length_01mm * 0.1,
           (double)length_diff_01mm * 0.1);
    printf("位置计算 | 电机位置=%.1fmm | 编码轮位置=%.1fmm | 差值：%.1fmm\r\n",
           (double)motor_source_position_01mm * 0.1,
           (double)encoder_position_01mm * 0.1,
           (double)position_diff_01mm * 0.1);
}

/**
 * @brief 打印电机记步专用诊断状态。
 *
 * 统一输出基准、局部周长、XACTUAL/XTARGET、编码轮差值和运动状态。
 */
void MotorCtrl_PrintMotorCountStatus(void)
{
    int32_t xtarget = 0;
    int32_t vactual = 0;
    int32_t rampstat = 0;
    int32_t gstat = 0;
    bool is_moving = false;
    uint32_t moving_ret;
    const uint32_t display_state = MotorCtrl_GetDisplayState();
    const bool display_moving = ((display_state == 1U) || (display_state == 2U));
    const char *display_text = (display_state == 1U) ? "上行" :
                               (display_state == 2U) ? "下行" : "静止";

    MotorCtrl_PrintPositionCompare();

    if (stpr_tryReadInt(&stepper, TMC5130_XTARGET, &xtarget) &&
        stpr_tryReadInt(&stepper, TMC5130_VACTUAL, &vactual) &&
        stpr_tryReadInt(&stepper, TMC5130_RAMPSTAT, &rampstat) &&
        stpr_tryReadInt(&stepper, TMC5130_GSTAT, &gstat)) {
        printf("电机记步诊断寄存器 | XTARGET=%ld | VACTUAL=%ld | RAMPSTAT=0x%08lX | GSTAT=0x%08lX | vzero=%s\r\n",
               (long)xtarget,
               (long)vactual,
               (unsigned long)rampstat,
               (unsigned long)gstat,
               ((rampstat & 0x400) != 0) ? "是" : "否");
    } else {
        printf("电机记步诊断寄存器 | 读取失败\r\n");
    }

    moving_ret = MotorDriver_ReadMovingState(&stepper, &is_moving);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (moving_ret == NO_ERROR) {
        printf("电机记步诊断状态 | 显示状态=%lu(%s) | 驱动运动=%s | 校验=%s\r\n",
               (unsigned long)display_state,
               display_text,
               is_moving ? "是" : "否",
               (display_moving == is_moving) ? "一致" : "不一致");
    } else {
        printf("电机记步诊断状态 | 显示状态=%lu(%s) | 驱动运动读取失败\r\n",
               (unsigned long)display_state,
               display_text);
    }
}

/**
 * @brief 运行期轮询刷新电机位置。
 *
 * 运动中定期读取 XACTUAL，刷新调试状态，并在电机记步模式下刷新业务位置。
 */
uint32_t MotorCtrl_PollRuntimePosition(void)
{
    static uint32_t s_last_runtime_poll_tick = 0U;
    const uint32_t now = HAL_GetTick();
    bool is_moving = false;
    uint32_t ret;

    /* 首次通信故障会使安全停机有效性失效，后台轮询随即停止，避免每50ms重复访问和刷屏。
     * 正式运动或测量重试仍由运动入口重新初始化驱动。 */
    if (!MotorCtrl_IsDriverInitValid()) {
        return NO_ERROR;
    }
    if ((now - s_last_runtime_poll_tick) < 50U) {
        return NO_ERROR;
    }
    s_last_runtime_poll_tick = now;

    if (!s_motor_driver.motion_command_active) {
        s_motor_driver.motion_command_active = false;
        s_motor_driver.motion_wait_active = false;
        g_measurement.debug_data.motor_state = 0U;
        ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        return NO_ERROR;
    }

    ret = MotorDriver_ReadMovingState(&stepper, &is_moving);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        uint32_t sync_ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (sync_ret != NO_ERROR) {
            return sync_ret;
        }
        return ret;
    }

    if (!is_moving) {
        if (s_motor_driver.motion_wait_active) {
            ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
            /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
            if (ret != NO_ERROR) {
                return ret;
            }
            return NO_ERROR;
        }
        if ((g_measurement.debug_data.motor_state == 1U) ||
            (g_measurement.debug_data.motor_state == 2U)) {
            s_motor_driver.motion_command_active = false;
            s_motor_driver.motion_wait_active = false;
            g_measurement.debug_data.motor_state = 0U;
            ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
            /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
            if (ret != NO_ERROR) {
                return ret;
            }
        }
        return NO_ERROR;
    }

    if ((g_measurement.debug_data.motor_state != 1U) &&
        (g_measurement.debug_data.motor_state != 2U)) {
        uint32_t inferred_state = MotorDriver_InferDisplayStateFromDriver(&stepper);
        if ((inferred_state == 1U) || (inferred_state == 2U)) {
            g_measurement.debug_data.motor_state = inferred_state;
        }
    }

    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    return NO_ERROR;
}

/**
 * @brief 切换为编码轮记步位置源。
 *
 * 切换后业务位置由外部编码器刷新，电机基准仍保留用于诊断和后续切回。
 * @return 成功返回 NO_ERROR，否则返回保存参数错误码。
 */
uint32_t MotorCtrl_SwitchPositionSourceToEncoder(void)
{
    g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
    update_sensor_height_from_encoder();
    MotorPosition_SyncDebugDrumState(&stepper);
    MotorPosition_SavePositionSourceParams(true);
    printf("位置来源已切换为编码轮\r\n");
    return NO_ERROR;
}

/**
 * @brief 切换为电机记步位置源。
 *
 * 切换瞬间记录编码轮长度和 XACTUAL 作为基准，并尝试标定当前位置局部周长。
 * @return 成功返回 NO_ERROR，否则返回运动、参数或保存错误码。
 */
uint32_t MotorCtrl_SwitchPositionSourceToMotor(void)
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

    if (!s_motor_driver.initialized) {
        return MOTOR_DRIVER_NOT_INITIALIZED;
    }

    old_mode = g_deviceParams.position_count_mode;
    old_local_circ_param = g_deviceParams.motor_count_first_loop_circumference_mm;
    old_base_step = s_motor_position.count_base_step;
    old_base_length_01mm = s_motor_position.count_base_length_01mm;
    old_base_turns = s_motor_position.count_base_turns;
    old_error_code = g_measurement.device_status.error_code;

    if (!MotorPosition_TryUpdateDrumStateFromXactual(&stepper, &drum)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    one_rev_ticks = MotorPosition_TapeTicksPerRev();
    if (one_rev_ticks <= 0) {
        return ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR;
    }

    /* 切换到电机记步时，基准尺带长度必须直接来自编码轮。
     * 后续电机源长度 = 切换时编码轮尺带长度 + XACTUAL 相对变化量。 */
    base_encoder_count = g_encoder_count;
    g_measurement.debug_data.current_encoder_value = -g_encoder_count;
    s_motor_position.count_base_length_01mm = encoder_get_cable_length_01mm();
    base_length_mm = (double)s_motor_position.count_base_length_01mm * 0.1;
    s_motor_position.count_base_step = drum.motor_step;
    s_motor_position.count_base_turns = MotorPosition_TapeTurnsFromSignedLength(base_length_mm,
                                                             MotorPosition_TapeC0Mm(),
                                                             MotorPosition_TapeThicknessMm());

    local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();
    if (local_circumference_mm <= 0.0) {
        local_circumference_mm = MotorPosition_TapeInstantCircumferenceFromLength(base_length_mm);
        (void)MotorPosition_SetLocalCircumferenceToParams(local_circumference_mm);
    }

    /* 标定过程中临时切到电机源，用于屏蔽切换窗口内的编码器 SSI 错误。
     * 成功前不保存参数；任何失败都会回滚到进入函数前的模式和基准。 */
    g_deviceParams.position_count_mode = POSITION_COUNT_MODE_MOTOR;
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (MotorPosition_IsEncoderErrorCode(g_measurement.device_status.error_code)) {
        g_measurement.device_status.error_code = NO_ERROR;
    }
    MotorPosition_UpdatePositionFromMotorSource(&drum);

    /* 切换后自动下行一周，用编码轮真实长度变化标定当前位置局部周长。 */
    printf("当前位置周长标定开始 | 基准编码值=%ld | 基准长度：%.1fmm | 一圈步数=%ld | 原局部周长=%.3fmm\r\n",
           (long)base_encoder_count,
           base_length_mm,
           (long)one_rev_ticks,
           local_circumference_mm);
    ret = MotorCtrl_MoveByTicksAndWait(one_rev_ticks, MotorCtrl_GetDefaultSpeedX100());
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorPosition_RollbackPositionSourceSwitch(ret, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }

    measured_encoder_count = g_encoder_count;
    delta_encoder_count = measured_encoder_count - base_encoder_count;
    measured_length_mm = (double)encoder_get_cable_length_01mm() * 0.1;
    measured_circumference_mm = measured_length_mm - base_length_mm;
    if (measured_circumference_mm < 0.0) {
        measured_circumference_mm = -measured_circumference_mm;
    }
    printf("当前位置周长标定采样 | 基准编码值=%ld | 当前编码值=%ld | 编码差值：%ld | 基准长度：%.1fmm | 实测长度：%.1fmm | 实测周长=%.3fmm\r\n",
           (long)base_encoder_count,
           (long)measured_encoder_count,
           (long)delta_encoder_count,
           base_length_mm,
           measured_length_mm,
           measured_circumference_mm);
    if ((measured_circumference_mm >= C0_MIN_MM) &&
        (measured_circumference_mm <= C0_MAX_MM)) {
        (void)MotorPosition_SetLocalCircumferenceToParams(measured_circumference_mm);
        local_circumference_mm = measured_circumference_mm;
        measured_circumference_valid = true;
    } else {
        calibration_ret = (measured_circumference_mm < C0_MIN_MM) ? ENCODER_LOST_STEP : ENCODER_DIFF_EXCESS;
        printf("当前位置周长标定失败 | 测得周长=%.3fmm | 范围=(%.1f, %.1f) | 错误码：0x%08lX\r\n",
               measured_circumference_mm,
               (double)C0_MIN_MM,
               (double)C0_MAX_MM,
               (unsigned long)calibration_ret);
    }

    /* 无论标定值是否有效，都回到切换瞬间的 XACTUAL 位置。 */
    return_ticks = -one_rev_ticks;
    ret = MotorCtrl_MoveByTicksAndWait(return_ticks, MotorCtrl_GetDefaultSpeedX100());
    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return MotorPosition_RollbackPositionSourceSwitch(ret, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }

    if (!MotorPosition_TryUpdateDrumStateFromXactual(&stepper, &drum)) {
        return MotorPosition_RollbackPositionSourceSwitch(MOTOR_TMC_COMM_ERROR, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }
    MotorPosition_UpdatePositionFromMotorSource(&drum);

    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (calibration_ret != NO_ERROR) {
        return MotorPosition_RollbackPositionSourceSwitch(calibration_ret, old_mode, old_local_circ_param, old_base_step, old_base_length_01mm, old_base_turns, old_error_code);
    }

    MotorPosition_SavePositionSourceParams(true);
    /* 此处已经通过 MotorPosition_TryUpdateDrumStateFromXactual() 得到可信 XACTUAL，
     * 直接保存切换基准和当前位置，避免再次 SPI 读取失败时静默跳过电机 FRAM 保存。 */
    MotorPosition_StorePersistSnapshot(drum.motor_step);

    printf("编码切换到电机步进模式 | 切换时尺带长度：%.1fmm | 切换时电机步进XACTUAL=%ld | 切换时的周长=%.3fmm | 实测周长=%.3fmm | 来源=%s\r\n",
           (double)s_motor_position.count_base_length_01mm * 0.1,
           (long)s_motor_position.count_base_step,
           MotorPosition_GetLocalCircumferenceFromParams(),
           measured_circumference_mm,
           measured_circumference_valid ? "measured" : "model");
    return NO_ERROR;

}

/**
 * @brief 用编码轮差分校准当前电机记步局部周长。
 *
 * 只更新 motor_count_first_loop_circumference_mm，不改变全局首圈周长。
 * @return 成功返回 NO_ERROR，否则返回参数或测量错误码。
 */
uint32_t MotorCtrl_CalibrateCurrentTapeCircumference(void)
{
    MotorDrumState drum;
    int64_t delta_step;
    double delta_turns;
    double encoder_length_mm;
    double delta_length_mm;
    double local_circumference_mm;

    if (!s_motor_driver.initialized) {
        return MOTOR_DRIVER_NOT_INITIALIZED;
    }
    if (g_deviceParams.position_count_mode != POSITION_COUNT_MODE_MOTOR) {
        return ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR;
    }

    /* 校准只需要编码轮长度作为参考，电机记步模式下不能覆盖业务 sensor_position。 */
    update_sensor_height_from_encoder();
    encoder_length_mm = (double)encoder_get_cable_length_01mm() * 0.1;
    MotorCtrl_UpdateDrumStateFromXActual(&stepper, &drum);

    delta_step = (int64_t)drum.motor_step -
                 (int64_t)s_motor_position.count_base_step;
    if (llabs(delta_step) < ((int64_t)MotorPosition_TapeTicksPerRev() / 16)) {
        MotorPosition_UpdatePositionFromMotorSource(&drum);
        return ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR;
    }

    delta_turns = (double)delta_step / (double)MotorPosition_TapeTicksPerRev();
    delta_length_mm = encoder_length_mm -
                      ((double)s_motor_position.count_base_length_01mm * 0.1);
    if (((delta_length_mm > 0.0) && (delta_turns < 0.0)) ||
        ((delta_length_mm < 0.0) && (delta_turns > 0.0))) {
        MotorPosition_UpdatePositionFromMotorSource(&drum);
        return ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR;
    }

    local_circumference_mm = delta_length_mm / delta_turns;
    if ((local_circumference_mm < C0_MIN_MM) ||
        (local_circumference_mm > C0_MAX_MM)) {
        MotorPosition_UpdatePositionFromMotorSource(&drum);
        return ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR;
    }

    (void)MotorPosition_SetLocalCircumferenceToParams(local_circumference_mm);
    MotorPosition_UpdatePositionFromMotorSource(&drum);

    MotorPosition_SavePositionSourceParams(true);
    MotorPosition_MaybePersistRegisters(&stepper, true);

    printf("电机局部周长已校准 | 周长=%.3fmm | 长度差=%.1fmm | 圈数差=%.6f\r\n",
           MotorPosition_GetLocalCircumferenceFromParams(),
           delta_length_mm,
           delta_turns);
    return NO_ERROR;
}

/**
 * @brief 从当前驱动状态强制保存电机位置寄存器。
 *
 * 供底层目标位置变化后调用，确保 XACTUAL/XTARGET 和基准能断电恢复。
 */
void MotorCtrl_PersistRegistersFromDriver(void)
{
    MotorPosition_MaybePersistRegisters(&stepper, true);
}

/**
 * @brief 清除或复位电机控制中的 MotorCtrl_ResetDrumReferenceForZeroCalibration 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t MotorCtrl_ResetDrumReferenceForZeroCalibration(void)
{
    /* 回零成功后统一切回编码轮记步，因此这里清除电机记步坐标、基准和旧局部周长。 */
    s_motor_position.count_base_step = 0;
    s_motor_position.count_base_length_01mm = 0;
    s_motor_position.count_base_turns = 0.0;
    if (!MotorPosition_SetLocalCircumferenceToParams(MotorPosition_TapeC0Mm())) {
        return ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR;
    }

    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (MotorPosition_IsEncoderErrorCode(g_measurement.device_status.error_code)) {
        g_measurement.device_status.error_code = NO_ERROR;
    }

    if (!s_motor_driver.initialized) {
        g_measurement.debug_data.motor_step = 0;
        g_measurement.debug_data.motor_distance = 0;
        return NO_ERROR;
    }
    {
        uint32_t ret = stpr_setPos(&stepper, 0);
        /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            printf("标定零点清除电机坐标失败，错误码：0x%08lX\r\n", (unsigned long)ret);
            return ret;
        }
    }
    MotorPosition_SyncDebugDrumState(&stepper);
    MotorPosition_SavePositionSourceParams(true);
    MotorPosition_MaybePersistRegisters(&stepper, true);
    return NO_ERROR;
}

/* ===================== 内部跨文件接口 ===================== */

/**
 * @brief 获取当前电机记步局部周长参数。
 *
 * @return 合法局部周长，单位 mm；未配置时返回 0。
 */
double MotorPosition_GetLocalCircumferenceFromParams(void)
{
    return MotorPosition_DecodeLocalCircumferenceParam();
}

/**
 * @brief 将局部周长写回设备参数结构。
 *
 * 只更新 RAM 中的参数结构，是否保存由调用方决定。
 * @param circumference_mm 局部周长，单位 mm。
 * @return 写入成功返回 true，参数非法返回 false。
 */
bool MotorPosition_SetLocalCircumferenceToParams(double circumference_mm)
{
    uint32_t value = MotorPosition_EncodeLocalCircumferenceParam(circumference_mm);
    if (value == 0U) {
        return false;
    }
    g_deviceParams.motor_count_first_loop_circumference_mm = value;
    return true;
}

/**
 * @brief 获取卷筒输出轴每圈对应的电机 ticks。
 *
 * @return 每圈 ticks，异常配置时返回兜底值。
 */
int32_t MotorPosition_TapeTicksPerRev(void)
{
    return (int32_t)(30L * 1600L * 32L);  /* 1,536,000 */
}

/**
 * @brief 获取尺带模型首圈周长 C0。
 *
 * @return 首圈周长，单位 mm。
 */
double MotorPosition_TapeC0Mm(void)
{
    return (double)g_deviceParams.first_loop_circumference_mm * 0.1;
}

/**
 * @brief 获取尺带厚度参数。
 *
 * @return 尺带厚度，单位 mm。
 */
double MotorPosition_TapeThicknessMm(void)
{
    return (double)g_deviceParams.tape_thickness_mm * 0.001;
}

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
        MotorPosition_TapeMinRadiusThreshold(C0_mm, t_mm, &n1, &L1, &Cmin);

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
 * @brief 根据当前位置长度估算当前卷层瞬时周长。
 *
 * @param L_mm 当前尺带长度，单位 mm。
 * @return 当前卷层周长，单位 mm。
 */
double MotorPosition_TapeInstantCircumferenceFromLength(double L_mm)
{
    const double C0 = MotorPosition_TapeC0Mm();
    const double t  = MotorPosition_TapeThicknessMm();

    if (C0 <= 0.0) {
        return 0.0;
    }
    if (t <= 0.0) {
        return C0;
    }

    const double n = MotorPosition_TapeTurnsFromSignedLength(L_mm, C0, t);

    if (n >= 0.0) {
        double n1, L1, Cmin;
        MotorPosition_TapeMinRadiusThreshold(C0, t, &n1, &L1, &Cmin);
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
 * @brief 安全地将 double 四舍五入为 int64。
 *
 * 避免部分嵌入式库环境下直接依赖 llround 的兼容问题。
 * @param x 输入浮点数。
 * @return 四舍五入后的整数。
 */
int64_t MotorPosition_RoundToInt64(double x)
{
    return (int64_t)llround(x);
}

/**
 * @brief 获取当前尺带长度用于卷筒模型换算。
 *
 * 位置源为电机记步时优先使用电机推算长度，否则使用编码轮长度。
 * @return 当前尺带长度，单位 mm。
 */
double MotorPosition_GetCurrentTapeLengthMm(void)
{
    return (double)g_measurement.debug_data.cable_length * 0.1;
}

/**
 * @brief 读取用于标定/拟合的编码轮长度。
 *
 * @return 编码轮长度，单位 0.1mm；读取失败时返回当前测量缓存。
 */
int32_t MotorPosition_ReadEncoderLengthForFit(void)
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

/**
 * @brief 上电初始化时从 FRAM 恢复电机位置寄存器。
 *
 * 恢复 XACTUAL/XTARGET，并缓存电机记步基准供位置源恢复使用。
 * @param tmc5130 TMC5130 设备对象。
 */
uint32_t MotorPosition_RestorePersistedRegisters(TMC5130TypeDef *tmc5130)
{
    int32_t xactual = 0;
    int32_t base_length_01mm = 0;
    int32_t base_step = 0;
    if (tmc5130 == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (!s_motor_driver.initialized) {
        return MOTOR_DRIVER_NOT_INITIALIZED;
    }
    if (MotorPosition_ReadPersistAB(&xactual, &base_length_01mm, &base_step)) {
        if ((!stpr_writeInt(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_HOLD)) ||
            (!stpr_writeInt(tmc5130, TMC5130_XACTUAL, xactual)) ||
            (!stpr_writeInt(tmc5130, TMC5130_XTARGET, xactual))) {
            s_motor_restored_base_length_01mm = 0;
            s_motor_restored_base_step = 0;
            s_motor_restored_base_valid = false;
            printf("[电机][初始化][失败] 持久化位置恢复失败：TMC5130寄存器写入失败\r\n");
            return MOTOR_TMC_COMM_ERROR;
        }
        s_motor_restored_base_length_01mm = base_length_01mm;
        s_motor_restored_base_step = base_step;
        s_motor_restored_base_valid = true;
        MotorPosition_WritePersistAB(xactual, base_length_01mm, base_step);
        s_motor_saved_xactual = xactual;
        printf("[电机][初始化][恢复] 持久化位置已恢复 | XACTUAL=%ld | 基准长度=%.1f mm | 基准步数=%ld\r\n",
               (long)xactual,
               (double)base_length_01mm * 0.1,
               (long)base_step);
        return NO_ERROR;
    }
    s_motor_restored_base_length_01mm = 0;
    s_motor_restored_base_step = 0;
    s_motor_restored_base_valid = false;
    MotorPosition_StorePersistSnapshot(0);
    printf("[电机][初始化][恢复] 未找到有效持久化位置，已复位为零\r\n");
    return NO_ERROR;
}

/**
 * @brief 同步调试区中的卷筒步数和模型长度。
 *
 * @param tmc5130 TMC5130 设备对象。
 */
bool MotorPosition_SyncDebugDrumState(TMC5130TypeDef *tmc5130)
{
    MotorDrumState drum;

    if ((tmc5130 == NULL) || (!s_motor_driver.initialized)) {
        return false;
    }

    if (!MotorPosition_TryUpdateDrumStateFromXactual(tmc5130, &drum)) {
        /* 返回 false 交给调用方继续检查驱动健康，避免在模型层直接打印错误。 */
        return false;
    }
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;
    MotorPosition_UpdatePositionFromMotorSource(&drum);
    /* 同步电机位置时顺带尝试 TFIT 自动采样；内部会判断是否启用和步进间隔。 */
    MotorTapeFit_AutoSample();
    MotorPosition_MaybePersistRegisters(tmc5130, false);
    return true;
}

/**
 * @brief 在电机记步模式下用电机模型刷新业务位置。
 *
 * @param drum 当前卷筒状态快照。
 */
void MotorPosition_UpdatePositionFromMotorSource(const MotorDrumState *drum)
{
    int64_t delta_step;
    double delta_turns;
    double turns;
    double length_mm;
    double local_circumference_mm;
    int32_t length_01mm;

    if ((drum == NULL) ||
        (g_deviceParams.position_count_mode != POSITION_COUNT_MODE_MOTOR)) {
        return;
    }

    delta_step = (int64_t)drum->motor_step -
                 (int64_t)s_motor_position.count_base_step;
    delta_turns = (double)delta_step / (double)MotorPosition_TapeTicksPerRev();
    local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();

    if (local_circumference_mm > 1e-6) {
        length_mm = ((double)s_motor_position.count_base_length_01mm * 0.1) +
                    (delta_turns * local_circumference_mm);
    } else {
        double base_model_mm;
        double current_model_mm;

        turns = s_motor_position.count_base_turns + delta_turns;
        base_model_mm = MotorPosition_TapeSignedLengthFromTurns(s_motor_position.count_base_turns,
                                                      MotorPosition_TapeC0Mm(),
                                                      MotorPosition_TapeThicknessMm());
        current_model_mm = MotorPosition_TapeSignedLengthFromTurns(turns,
                                                         MotorPosition_TapeC0Mm(),
                                                         MotorPosition_TapeThicknessMm());
        length_mm = ((double)s_motor_position.count_base_length_01mm * 0.1) +
                    (current_model_mm - base_model_mm);
    }
    length_01mm = (int32_t)llround(length_mm * 10.0);

    g_measurement.debug_data.cable_length = length_01mm;
    g_measurement.debug_data.sensor_position =
        (int32_t)g_deviceParams.tankHeight - length_01mm;
}

/**
 * @brief 根据设备参数和 FRAM 记录恢复位置源运行态。
 *
 * 电机记步模式下优先恢复持久化基准；编码轮模式下保留基准用于诊断。
 */
void MotorPosition_RestorePositionSourceFromParams(void)
{
    MotorDrumState drum;
    double local_circumference_mm;

    if (!s_motor_driver.initialized) {
        return;
    }

    if (!MotorPosition_TryUpdateDrumStateFromXactual(&stepper, &drum)) {
        printf("[电机][初始化][跳过] 位置源恢复跳过：XACTUAL读取失败\r\n");
        return;
    }
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();
    if (local_circumference_mm <= 0.0) {
        local_circumference_mm = MotorPosition_TapeInstantCircumferenceFromLength(
            (double)drum.motor_distance_01mm * 0.1);
        (void)MotorPosition_SetLocalCircumferenceToParams(local_circumference_mm);
    }

    if (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) {
        if (s_motor_restored_base_valid) {
            s_motor_position.count_base_step = s_motor_restored_base_step;
            s_motor_position.count_base_length_01mm = s_motor_restored_base_length_01mm;
        } else {
            s_motor_position.count_base_step = drum.motor_step;
            s_motor_position.count_base_length_01mm = drum.motor_distance_01mm;
        }
        s_motor_position.count_base_turns = MotorPosition_TapeTurnsFromSignedLength(
            (double)s_motor_position.count_base_length_01mm * 0.1,
            MotorPosition_TapeC0Mm(),
            MotorPosition_TapeThicknessMm());
        MotorPosition_UpdatePositionFromMotorSource(&drum);
        printf("[电机][初始化][位置源] 已恢复为电机记步 | 基准长度=%.1f mm | XACTUAL=%ld | 局部周长=%.3f mm\r\n",
               (double)s_motor_position.count_base_length_01mm * 0.1,
               (long)s_motor_position.count_base_step,
               MotorPosition_GetLocalCircumferenceFromParams());
        MotorPosition_MaybePersistRegisters(&stepper, true);
    } else {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
        if (s_motor_restored_base_valid) {
            s_motor_position.count_base_step = s_motor_restored_base_step;
            s_motor_position.count_base_length_01mm = s_motor_restored_base_length_01mm;
            s_motor_position.count_base_turns = MotorPosition_TapeTurnsFromSignedLength(
                (double)s_motor_position.count_base_length_01mm * 0.1,
                MotorPosition_TapeC0Mm(),
                MotorPosition_TapeThicknessMm());
        } else {
            s_motor_position.count_base_step = 0;
            s_motor_position.count_base_length_01mm = 0;
            s_motor_position.count_base_turns = 0.0;
        }
        update_sensor_height_from_encoder();
        printf("[电机][初始化][位置源] 已恢复为编码轮 | 电机基准长度=%.1f mm | 电机基准步数=%ld\r\n",
               (double)s_motor_position.count_base_length_01mm * 0.1,
               (long)s_motor_position.count_base_step);
    }

}

/* ===================== 私有函数实现 ===================== */

/**
 * @brief 保存位置源模式和电机局部周长参数。
 *
 * @param force 为 true 时即使未检测到修正也强制保存设备参数。
 */
static void MotorPosition_SavePositionSourceParams(bool force)
{
    bool changed = false;

    if (g_deviceParams.position_count_mode > POSITION_COUNT_MODE_MOTOR) {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
        changed = true;
    }

    if (MotorPosition_DecodeLocalCircumferenceParam() <= 0.0) {
        g_deviceParams.motor_count_first_loop_circumference_mm =
            g_deviceParams.first_loop_circumference_mm * 100U;
        changed = true;
    }

    if (changed || force) {
        save_device_params();
    }
}

/**
 * @brief 判断错误码是否属于编码轮/位置源相关错误。
 *
 * @param error_code 当前系统错误码。
 * @return 属于编码器相关错误返回 true，否则返回 false。
 */
static bool MotorPosition_IsEncoderErrorCode(uint32_t error_code)
{
    return (error_code >= ENCODER_TIMEOUT) && (error_code <= ENCODER_FIRST_SAMPLE_TIMEOUT);
}

/**
 * @brief 尝试读取 XACTUAL 并刷新卷筒状态。
 *
 * @param tmc5130 TMC5130 设备对象。
 * @param out 输出卷筒状态。
 * @return 读取和换算成功返回 true，通信失败返回 false。
 */
static bool MotorPosition_TryUpdateDrumStateFromXactual(TMC5130TypeDef *tmc5130,
                                                 MotorDrumState *out)
{
    int32_t motor_step;

    if ((out == NULL) || (tmc5130 == NULL)) {
        return false;
    }
    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &motor_step)) {
        return false;
    }
    if (!MotorPosition_FilterXactualGlitch(tmc5130, &motor_step)) {
        return false;
    }

    MotorPosition_BuildDrumStateFromStep(motor_step, out);
    return true;
}

/**
 * @brief 过滤 XACTUAL 的单帧异常读数。
 *
 * 现场日志中曾出现电机尺带从 -80mm 单次跳到 0 或 90m 后立刻恢复的情况。
 * 这类值来自 TMC5130 XACTUAL 的瞬时错误读数；如果直接更新缓存，会污染调试显示、
 * 电机记步位置和持久化记录。可疑值必须二次读取确认，确认失败则保留旧缓存。
 */
static bool MotorPosition_FilterXactualGlitch(TMC5130TypeDef *tmc5130,
                                              int32_t *motor_step)
{
    const int32_t cached_step = g_measurement.debug_data.motor_step;
    int32_t reread_step = 0;
    int64_t delta;
    int64_t reread_delta;

    if ((tmc5130 == NULL) || (motor_step == NULL)) {
        return false;
    }

    /* 非零位置偶发读到 0 时，先重读一次。若重读恢复正常，直接采用重读值；
     * 若两次都是 0，再用 CHOPCONF 判断是否为 SPI 全 0。 */
    if ((*motor_step == 0) && (cached_step != 0)) {
        int32_t chopconf = 0;

        if (stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &reread_step) &&
            (reread_step != 0)) {
            *motor_step = reread_step;
            return true;
        }

        if ((!stpr_tryReadInt(tmc5130, TMC5130_CHOPCONF, &chopconf)) ||
            (chopconf == 0)) {
            printf("XACTUAL读数丢弃 | 原因:疑似SPI全0 | 缓存=%ld\r\n",
                   (long)cached_step);
            return false;
        }
        return true;
    }

    if (cached_step == 0) {
        return true;
    }

    delta = (int64_t)(*motor_step) - (int64_t)cached_step;
    if (llabs(delta) <= (int64_t)MOTOR_XACTUAL_SUSPECT_JUMP_TICKS) {
        return true;
    }

    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &reread_step)) {
        printf("XACTUAL读数丢弃 | 原因:跳变后重读失败 | 缓存=%ld | 首读=%ld\r\n",
               (long)cached_step,
               (long)(*motor_step));
        return false;
    }

    reread_delta = (int64_t)reread_step - (int64_t)cached_step;
    if (llabs(reread_delta) <= (int64_t)MOTOR_XACTUAL_SUSPECT_JUMP_TICKS) {
        printf("XACTUAL读数修正 | 缓存=%ld | 首读=%ld | 重读=%ld\r\n",
               (long)cached_step,
               (long)(*motor_step),
               (long)reread_step);
        *motor_step = reread_step;
        return true;
    }

    if (llabs((int64_t)reread_step - (int64_t)(*motor_step)) <=
        ((int64_t)MotorPosition_TapeTicksPerRev() / 16)) {
        return true;
    }

    printf("XACTUAL读数丢弃 | 原因:连续读数不一致 | 缓存=%ld | 首读=%ld | 重读=%ld\r\n",
           (long)cached_step,
           (long)(*motor_step),
           (long)reread_step);
    return false;
}

/**
 * @brief 按变化阈值决定是否保存电机位置寄存器。
 *
 * @param tmc5130 TMC5130 设备对象。
 * @param force 为 true 时强制保存，不检查变化阈值。
 */
static void MotorPosition_MaybePersistRegisters(TMC5130TypeDef *tmc5130, bool force)
{
    MotorDrumState drum;
    int64_t delta_ticks;
    int64_t delta_threshold_ticks;

    if ((tmc5130 == NULL) || (!s_motor_driver.initialized)) {
        return;
    }

    /* 持久化必须使用和实时刷新相同的防误读路径。
     * 如果 SPI 全 0 或读失败，这里直接跳过，避免把 FRAM 中的非零 XACTUAL 覆盖成 0。 */
    if (!MotorPosition_TryUpdateDrumStateFromXactual(tmc5130, &drum)) {
        return;
    }

    delta_ticks = (int64_t)drum.motor_step - (int64_t)s_motor_saved_xactual;
    delta_threshold_ticks = MotorPosition_PersistDeltaTicksForLength(drum.motor_step);
    if ((!force) && (llabs(delta_ticks) < delta_threshold_ticks)) {
        return;
    }
    MotorPosition_StorePersistSnapshot(drum.motor_step);
}

/**
 * @brief 将局部首圈周长编码为设备参数存储值。
 *
 * @param circumference_mm 局部首圈周长，单位 mm。
 * @return 编码后的 0.001mm 参数值；非法时返回 0。
 */
static uint32_t MotorPosition_EncodeLocalCircumferenceParam(double circumference_mm)
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

/**
 * @brief 从设备参数解码电机记步局部首圈周长。
 *
 * @return 合法周长返回 mm；未配置或越界时返回 0。
 */
static double MotorPosition_DecodeLocalCircumferenceParam(void)
{
    const double circumference_mm =
        (double)g_deviceParams.motor_count_first_loop_circumference_mm * 0.001;

    if ((circumference_mm < C0_MIN_MM) || (circumference_mm > C0_MAX_MM)) {
        return 0.0;
    }

    return circumference_mm;
}

/**
 * @brief 计算尺带放带模型进入最小半径线性段的阈值。
 *
 * @param C0_mm 首圈周长，单位 mm。
 * @param t_mm 尺带厚度，单位 mm。
 * @param n1 输出进入最小半径前的圈数阈值。
 * @param L1 输出阈值处长度，单位 mm。
 * @param Cmin 输出最小半径对应周长，单位 mm。
 */
static void MotorPosition_TapeMinRadiusThreshold(double C0_mm, double t_mm,
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

    /* 达到 minR 时：C(n)=C0-2*pi*t*n = Cmin => n1=(C0-Cmin)/(2*pi*t) */
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
 * @brief 根据卷筒圈数计算带符号尺带长度。
 *
 * 正圈数表示放带，负圈数表示缠带。
 * @param n 卷筒圈数。
 * @param C0_mm 首圈周长，单位 mm。
 * @param t_mm 尺带厚度，单位 mm。
 * @return 模型计算长度，单位 mm。
 */
static double MotorPosition_TapeSignedLengthFromTurns(double n,
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
        MotorPosition_TapeMinRadiusThreshold(C0_mm, t_mm, &n1, &L1, &Cmin);

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
 * @brief 根据 XACTUAL 计算相对持久化基准的步数变化。
 *
 * @param xactual 当前 TMC5130 XACTUAL。
 * @return 相对最近保存基准的 ticks 差值。
 */
static int64_t MotorPosition_PersistDeltaTicksForLength(int32_t xactual)
{
    const double turns = (double)xactual / (double)MotorPosition_TapeTicksPerRev();
    const double length_mm = MotorPosition_TapeSignedLengthFromTurns(turns, MotorPosition_TapeC0Mm(), MotorPosition_TapeThicknessMm());
    const double circumference_mm = MotorPosition_TapeInstantCircumferenceFromLength(length_mm);
    double ticks;

    if (circumference_mm <= 1e-6) {
        return 1;
    }

    ticks = ((double)MotorPosition_TapeTicksPerRev() * MOTOR_PERSIST_DELTA_MM) / circumference_mm;
    if (ticks < 1.0) {
        return 1;
    }
    if (ticks > (double)INT32_MAX) {
        return (int64_t)INT32_MAX;
    }
    return (int64_t)llround(ticks);
}

/**
 * @brief 根据电机步数构造卷筒状态快照。
 *
 * @param motor_step TMC5130 XACTUAL，单位 ticks。
 * @param out 输出卷筒状态，传入 NULL 时不执行。
 */
static void MotorPosition_BuildDrumStateFromStep(int32_t motor_step, MotorDrumState *out)
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
    turns = (double)motor_step / (double)MotorPosition_TapeTicksPerRev();

    /* 圈内角度归一到 [0,1)，负向位置也能得到正常角度。 */
    turns_int = (int32_t)floor(turns);
    frac = turns - (double)turns_int;
    if (frac < 0.0) frac += 1.0;
    if (frac >= 1.0) frac -= floor(frac);
    angle_deg = frac * 360.0;

    /* 由电机卷筒模型预测长度，单位 mm。 */
    C0 = MotorPosition_TapeC0Mm();
    t = MotorPosition_TapeThicknessMm();
    if (t > 0.0) {
        L_mm = MotorPosition_TapeSignedLengthFromTurns(turns, C0, t);
    } else {
        L_mm = C0 * turns;
    }

    out->motor_step          = motor_step;
    out->turns_total         = turns;
    out->turns_int           = turns_int;
    out->angle_deg           = angle_deg;
    out->motor_distance_01mm = (int32_t)llround(L_mm * 10.0); /* mm -> 0.1mm */
}

/**
 * @brief 计算电机位置持久化记录 CRC。
 *
 * @param record 持久化记录。
 * @return CRC32 校验值。
 */
static uint32_t MotorPosition_PersistRecordCrc(const MotorPersistRecord *record)
{
    const uint8_t *base = (const uint8_t *)&record->version;
    const uint32_t len = (uint32_t)(offsetof(MotorPersistRecord, crc) - offsetof(MotorPersistRecord, version));
    return CRC32_HAL(base, len);
}

/**
 * @brief 从指定 FRAM 槽位读取并校验电机位置记录。
 *
 * @param base_addr FRAM 槽位起始地址。
 * @param xactual 输出保存的 XACTUAL。
 * @param base_length_01mm 输出电机记步基准长度。
 * @param base_step 输出电机记步基准步数。
 * @param slot_name 槽位名称，仅用于日志。
 * @return 读取成功返回 1，失败返回 0。
 */
static int MotorPosition_ReadPersistFromSlot(uint32_t base_addr,
                                     int32_t *xactual,
                                     int32_t *base_length_01mm,
                                     int32_t *base_step,
                                     const char *slot_name)
{
    MotorPersistRecord rec = {0};

    if (FRAM_Read((uint8_t *)&rec, base_addr, sizeof(rec)) != FRAM_STATUS_OK) {
        printf("电机持久化[%s]FRAM读取失败\r\n", slot_name);
        return 0;
    }
    if (rec.magic != MOTOR_STORE_MAGIC) {
        printf("电机持久化[%s]魔术字异常: 0x%08lX\r\n", slot_name, (unsigned long)rec.magic);
        return 0;
    }

    if (rec.version == MOTOR_STORE_VERSION) {
        if (MotorPosition_PersistRecordCrc(&rec) != rec.crc) {
            printf("电机持久化[%s] CRC校验失败\r\n", slot_name);
            return 0;
        }
        *xactual = rec.xactual;
        *base_length_01mm = rec.base_length_01mm;
        *base_step = rec.base_step;
        return 1;
    }
    printf("电机持久化[%s]版本异常: %lu\r\n", slot_name, (unsigned long)rec.version);
    return 0;
}

/**
 * @brief 读回并完整校验一个电机位置持久化槽位。
 *
 * @param base_addr FRAM槽位起始地址。
 * @param expected 本次期望写入的完整记录。
 * @return 魔术字、版本、CRC和全部业务字段一致时返回true。
 */
static bool MotorPosition_VerifyPersistSlot(
    uint32_t base_addr,
    const MotorPersistRecord *expected)
{
    MotorPersistRecord verify = {0};

    if ((expected == NULL) ||
        (FRAM_Read((uint8_t *)&verify,
                   base_addr,
                   sizeof(verify)) != FRAM_STATUS_OK)) {
        return false;
    }
    return (verify.magic == MOTOR_STORE_MAGIC) &&
           (verify.version == MOTOR_STORE_VERSION) &&
           (MotorPosition_PersistRecordCrc(&verify) == verify.crc) &&
           (verify.magic == expected->magic) &&
           (verify.version == expected->version) &&
           (verify.xactual == expected->xactual) &&
           (verify.base_length_01mm == expected->base_length_01mm) &&
           (verify.base_step == expected->base_step) &&
           (verify.crc == expected->crc);
}
/**
 * @brief 将电机位置记录同时写入 A/B 两个 FRAM 槽位。
 *
 * 双槽写入用于断电或写入中断后的冗余恢复。
 * @param xactual 当前 XACTUAL。
 * @param base_length_01mm 电机记步基准长度。
 * @param base_step 电机记步基准步数。
 */
static bool MotorPosition_WritePersistAB(int32_t xactual,
                                  int32_t base_length_01mm,
                                  int32_t base_step)
{
    MotorPersistRecord rec;
    FRAM_Status status_a;
    FRAM_Status status_b;
    bool verified_a = false;
    bool verified_b = false;

    rec.magic = MOTOR_STORE_MAGIC;
    rec.version = MOTOR_STORE_VERSION;
    rec.xactual = xactual;
    rec.base_length_01mm = base_length_01mm;
    rec.base_step = base_step;
    rec.crc = MotorPosition_PersistRecordCrc(&rec);
    status_a = FRAM_Write((const uint8_t *)&rec,
                          FRAM_MOTOR_A_ADDRESS,
                          sizeof(rec));
    if (status_a == FRAM_STATUS_OK) {
        /* FRAM_Write成功只表示SPI事务完成，必须完整读回后才接受A槽。 */
        verified_a = MotorPosition_VerifyPersistSlot(
            FRAM_MOTOR_A_ADDRESS, &rec);
    }
    status_b = FRAM_Write((const uint8_t *)&rec,
                          FRAM_MOTOR_B_ADDRESS,
                          sizeof(rec));
    if (status_b == FRAM_STATUS_OK) {
        /* B槽独立写入和校验，A槽失败不会阻止保留一份新的有效记录。 */
        verified_b = MotorPosition_VerifyPersistSlot(
            FRAM_MOTOR_B_ADDRESS, &rec);
    }
    /*
     * 最小可用边界为至少一槽写后回读一致；双槽都失败时调用方不得推进已保存基线。
     */
    return verified_a || verified_b;
}

/**
 * @brief 从 A/B 冗余槽位恢复电机位置记录。
 *
 * 优先使用 A 槽，A 槽无效时尝试 B 槽并回写修复。
 * @param xactual 输出保存的 XACTUAL。
 * @param base_length_01mm 输出电机记步基准长度。
 * @param base_step 输出电机记步基准步数。
 * @return 读取成功返回 1，失败返回 0。
 */
static int MotorPosition_ReadPersistAB(int32_t *xactual,
                               int32_t *base_length_01mm,
                               int32_t *base_step)
{
    if (MotorPosition_ReadPersistFromSlot(FRAM_MOTOR_A_ADDRESS,
                                  xactual,
                                  base_length_01mm,
                                  base_step,
                                  "A")) {
        return 1;
    }
    if (MotorPosition_ReadPersistFromSlot(FRAM_MOTOR_B_ADDRESS,
                                  xactual,
                                  base_length_01mm,
                                  base_step,
                                  "B")) {
        printf("电机持久化: A槽无效，回退到B槽\r\n");
        MotorPosition_WritePersistAB(*xactual, *base_length_01mm, *base_step);
        return 1;
    }
    return 0;
}

/**
 * @brief 保存当前电机位置和电机记步基准快照。
 *
 * @param xactual 当前 TMC5130 XACTUAL。
 */
static void MotorPosition_StorePersistSnapshot(int32_t xactual)
{
    if (MotorPosition_WritePersistAB(
            xactual,
            s_motor_position.count_base_length_01mm,
            s_motor_position.count_base_step)) {
        s_motor_saved_xactual = xactual;
    }
}

/**
 * @brief 位置源切换失败时回滚旧状态。
 *
 * 恢复旧记步模式、局部周长、电机基准和错误码。
 * @return 传入的 ret 错误码，便于调用方直接返回。
 */
static uint32_t MotorPosition_RollbackPositionSourceSwitch(uint32_t ret,
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
    s_motor_position.count_base_step = old_base_step;
    s_motor_position.count_base_length_01mm = old_base_length_01mm;
    s_motor_position.count_base_turns = old_base_turns;

    /* 先处理异常边界，避免电机控制状态机带故障继续运行。 */
    if (MotorPosition_IsEncoderErrorCode(g_measurement.device_status.error_code)) {
        g_measurement.device_status.error_code = old_error_code;
    }

    if (old_mode == POSITION_COUNT_MODE_MOTOR) {
        if (MotorPosition_TryUpdateDrumStateFromXactual(&stepper, &drum)) {
            MotorPosition_UpdatePositionFromMotorSource(&drum);
        }
    } else {
        update_sensor_height_from_encoder();
    }

    printf("位置来源切换到电机记步失败，已回滚 | 错误码：0x%08lX\r\n", (unsigned long)ret);
    return ret;
}
