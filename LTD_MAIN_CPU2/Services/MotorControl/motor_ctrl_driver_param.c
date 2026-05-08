#include "motor_ctrl_internal.h"

/**
 * @file motor_ctrl_driver_param.c
 * @brief 电机驱动初始化、速度/电流参数和底层状态判断。
 *
 * 本文件负责把设备参数转换成 TMC5130 可接受的速度、电流和运动状态口径。
 * 对外提供初始化、速度设置、电流设置和驱动异常检查；对内提供运行中速度刷新、
 * 停止命令和显示状态推断等基础能力。
 */

/* ===================== 私有类型/状态 ===================== */

static uint32_t MotorDriver_ClampSpeedSetpointX100(uint32_t speed_x100);
static uint32_t MotorDriver_ClampCurrentSetting(uint32_t current);
static double MotorDriver_VmaxToUstepsPerSec(uint32_t vmax);
static double MotorDriver_VmaxToOutputRevPerSec(uint32_t vmax);
static double MotorDriver_VmaxToMotorRevPerSec(uint32_t vmax);
static uint32_t MotorDriver_ClampVelocityU64(uint64_t v);
static uint32_t MotorDriver_UstepsPerSecToVmax(double usteps_per_s);
static uint32_t MotorDriver_ComputeBaseVelocityFromParams(void);
static uint32_t MotorDriver_GetDefaultSpeedSetpointX100(void);

/* ===================== 私有函数声明 ===================== */

static int32_t MotorDriver_GetSpeedSetpointX100(void);
static uint32_t MotorDriver_ClampSpeedSetpointX100(uint32_t speed_x100);
static uint32_t MotorDriver_ClampCurrentSetting(uint32_t current);
static double MotorDriver_VmaxToUstepsPerSec(uint32_t vmax);
static double MotorDriver_VmaxToOutputRevPerSec(uint32_t vmax);
static double MotorDriver_VmaxToMotorRevPerSec(uint32_t vmax);
static uint32_t MotorDriver_ClampVelocityU64(uint64_t v);
static uint32_t MotorDriver_UstepsPerSecToVmax(double usteps_per_s);
static uint32_t MotorDriver_ComputeBaseVelocityFromParams(void);
static uint32_t MotorDriver_GetDefaultSpeedSetpointX100(void);

/* ===================== 对外接口 ===================== */

/**
 * @brief 获取对外公开的默认电机速度。
 *
 * @return 默认速度，单位 0.01m/min。
 */
uint32_t MotorCtrl_GetDefaultSpeedX100(void)
{
    return MotorDriver_GetDefaultSpeedSetpointX100();
}

/**
 * @brief 持久设置电机速度参数。
 *
 * 空闲时只更新设备参数；运行中会立即重算 VMAX 并写入驱动。
 * @param speed_x100 请求速度，单位 0.01m/min。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_SetSpeed(uint32_t speed_x100)
{
    const uint32_t clamped_speed = MotorDriver_ClampSpeedSetpointX100(speed_x100);
    bool is_running = false;
    double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;

    /* 先更新“设定速度”本身。
     * 后续所有速度换算都以 debug_data.motor_speed 作为源头。 */
    g_measurement.debug_data.motor_speed = clamped_speed;

    if (s_motor_driver.initialized) {
        is_running = MotorCtrl_IsDriverMoving(&stepper);
    }

    if (is_running) {
        /* 电机正在跑时，优先用 XACTUAL 推当前卷筒长度。
         * 这样可以避免外部 cable_length 刷新滞后导致本次改速不准。 */
        MotorDrumState drum;
        MotorCtrl_UpdateDrumStateFromXActual(&stepper, &drum);
        MotorPosition_UpdatePositionFromMotorSource(&drum);
        Lcur_mm = (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) ?
                  ((double)g_measurement.debug_data.cable_length * 0.1) :
                  ((double)drum.motor_distance_01mm * 0.1);
    }

    /* 把“线速度设定”转换成当前卷径下的 TMC5130 VMAX。 */
    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_driver.applied_velocity = velocity;

    if (is_running) {
        uint32_t ret;
        /* 运行中改速：立即写入驱动，让本次运动立刻生效。 */
        ret = stpr_setVelocity(&stepper, velocity);
        if (ret != NO_ERROR) {
            return ret;
        }
//        printf("SpeedSet | running | req=%.2f m/min | set=%.2f m/min | L=%.1f mm | VMAX=%lu | eq_usteps/s=%.1f\r\n",
//               requested_speed / 100.0,
//               clamped_speed / 100.0,
//               Lcur_mm,
//               (unsigned long)velocity,
//               MotorDriver_VmaxToUstepsPerSec(velocity));
    } else {
        /* 未运行时只更新内部速度状态，供下一次运动使用。 */
//        printf("SpeedSet | idle | req=%.2f m/min | set=%.2f m/min | baseL=%.1f mm | nextVMAX=%lu | eq_usteps/s=%.1f\r\n",
//               requested_speed / 100.0,
//               clamped_speed / 100.0,
//               Lcur_mm,
//               (unsigned long)velocity,
//               MotorDriver_VmaxToUstepsPerSec(velocity));
    }
    if (is_running) {
        uint32_t inferred_state = MotorDriver_InferDisplayStateFromDriver(&stepper);
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

/**
 * @brief 持久设置 TMC5130 运行电流。
 *
 * 驱动初始化后会立即写 IHOLD_IRUN，使运行电流无需重启即可生效。
 * @param current 请求运行电流档位。
 * @return 成功返回 NO_ERROR，否则返回参数或通信错误码。
 */
uint32_t MotorCtrl_SetCurrent(uint32_t current)
{
    const uint32_t clamped_current = MotorDriver_ClampCurrentSetting(current);

    g_deviceParams.motor_current = clamped_current;

    if (s_motor_driver.initialized) {
        uint32_t ret;
        /* 电流参数写入后立即更新 TMC5130，避免必须重启才生效。 */
        ret = stpr_setCurrent(&stepper, (uint8_t)clamped_current);
        if (ret != NO_ERROR) {
            return ret;
        }
    }

    return NO_ERROR;
}

/**
 * @brief 初始化 TMC5130 电机驱动和电机控制运行态。
 *
 * 初始化会配置速度、电流、斜坡、位置持久化，并恢复位置源运行状态。
 * @return 成功返回 NO_ERROR，否则返回驱动通信或状态错误码。
 */
uint32_t MotorCtrl_Init(void)
{
    uint32_t motor_current = MotorDriver_ClampCurrentSetting(g_deviceParams.motor_current);

    /* 规范化后的电流回写到参数区，保证显示、保存和驱动寄存器一致。 */
    g_deviceParams.motor_current = motor_current;

    if (g_measurement.debug_data.motor_speed == 0U) {
        g_measurement.debug_data.motor_speed = g_deviceParams.max_motor_speed;
    } else {
        g_measurement.debug_data.motor_speed =
            MotorDriver_ClampSpeedSetpointX100(g_measurement.debug_data.motor_speed);
    }

    MotorDriver_UpdateVelocityFromParams();
    s_motor_driver.applied_velocity = velocity;

    if (!s_motor_driver.initialized) {
        uint32_t ret = stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, (uint8_t)motor_current);
        CHECK_ERROR(ret);
//      stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 16);
        stpr_enableDriver(&stepper);
        s_motor_driver.initialized = true;
        ret = MotorPosition_RestorePersistedRegisters(&stepper);
        if (ret != NO_ERROR) {
            s_motor_driver.initialized = false;
            return ret;
        }
        MotorPosition_RestorePositionSourceFromParams();

        /* stpr_initStepper() 只写基础斜坡参数，不写当前业务速度对应的 VMAX。
         * 初始化完成后必须把本次计算出的 velocity 下发到 TMC5130，否则驱动会沿用旧 VMAX。 */
        ret = stpr_setVelocity(&stepper, velocity);
        if (ret != NO_ERROR) {
            s_motor_driver.initialized = false;
            stpr_disableDriver(&stepper);
            return ret;
        }
        s_motor_driver.applied_velocity = velocity;

        /* 上电读取 DeviceParameters 和电机 FRAM 记录后，立即打印一次电机/编码轮位置对比。
         * 用于确认 XACTUAL、记步模式、局部周长、切换基准和编码轮位置是否一致。 */
        MotorCtrl_PrintPositionCompare();
    } else {
        uint32_t ret;

        stpr_enableDriver(&stepper);
        (void)MotorCtrl_SetCurrent(motor_current);

        /* 已初始化路径也要刷新 VMAX，确保参数修改后重新初始化能实时生效。 */
        ret = stpr_setVelocity(&stepper, velocity);
        CHECK_ERROR(ret);
        s_motor_driver.applied_velocity = velocity;
    }
    MotorPosition_SyncDebugDrumState(&stepper);

    printf("电机初始化 | 设定速度=%.2f m/min | 尺带长度=%.1f mm | VMAX=%lu | 等效微步/s=%.1f\r\n",
           g_deviceParams.max_motor_speed / 100.0,
           g_measurement.debug_data.cable_length / 10.0,
           (unsigned long)velocity,
           MotorDriver_VmaxToUstepsPerSec(velocity));
    return NO_ERROR;
}

/**
 * @brief 判断 TMC5130 当前是否仍在运动。
 *
 * @param tmc5130 TMC5130 设备对象。
 * @return 正在运动返回 true，停止或读取失败返回 false。
 */
bool MotorCtrl_IsDriverMoving(TMC5130TypeDef *tmc5130)
{
    int32_t rampstat = 0;
    int32_t rampstat_confirm = 0;
    int32_t vactual = 0;

    if (tmc5130 == NULL) {
        return false;
    }

    if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat)) {
        return false;
    }

    /* bit10(vzero)=0 表示斜坡发生器仍有速度，直接判定为运动中。 */
    if (((uint32_t)rampstat & 0x400U) != 0x400U) {
        return true;
    }

    /* 换向过程中速度会短暂过零，RAMPSTAT.vzero 可能瞬间置位。
     * 首次读到停止候选时延时后再确认一次，避免把换向过零误判成运动完成。 */
    HAL_Delay(5U);
    if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat_confirm)) {
        return false;
    }
    if (((uint32_t)rampstat_confirm & 0x400U) != 0x400U) {
        return true;
    }

    /* 若状态位连续显示 vzero，但实际速度寄存器仍非 0，仍按运动中处理。 */
    if (stpr_tryReadInt(tmc5130, TMC5130_VACTUAL, &vactual) && (vactual != 0)) {
        return true;
    }

    return false;
}

/**
 * @brief 检查 TMC5130 GSTAT/DRV_STATUS 驱动状态并转换为系统错误码。
 *
 * @param tmc5130 TMC5130 设备对象。
 * @return NO_ERROR 或对应驱动故障错误码。
 */
uint32_t MotorCtrl_CheckDriverGstat(TMC5130TypeDef *tmc5130)
{
    uint32_t ret = stpr_checkDriverStatus(tmc5130);
    CHECK_ERROR(ret);

    return ret;
}

/* ===================== 内部跨文件接口 ===================== */

/**
 * @brief 读取当前线速度设定并转换为 m/min。
 *
 * @return 当前线速度，单位 m/min。
 */
float MotorDriver_GetSpeedSetpointMMin(void)
{
    return (float)MotorDriver_GetSpeedSetpointX100() / 100.0f;
}

/**
 * @brief 根据当前速度参数刷新全局 VMAX 缓存。
 *
 * 该函数只更新软件侧 velocity 和已应用速度缓存，不主动判断运动状态。
 * 调用方如果需要立即生效，应继续写入 TMC5130 VMAX。
 */
void MotorDriver_UpdateVelocityFromParams(void)
{
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);

    printf("速度初始化 | 线速度=%.2f m/min | 尺带长度=%.1f mm | 周长=%.1f mm | VMAX=%lu | 输出轴=%.3f r/s | 电机=%.3f r/s | 等效微步/s=%.1f\r\n",
           (double)MotorDriver_GetSpeedSetpointX100() / 100.0,
           Lcur_mm,
           MotorPosition_TapeInstantCircumferenceFromLength(Lcur_mm),
           (unsigned long)velocity,
           MotorDriver_VmaxToOutputRevPerSec(velocity),
           MotorDriver_VmaxToMotorRevPerSec(velocity),
           MotorDriver_VmaxToUstepsPerSec(velocity));
}

/**
 * @brief 判断业务方向参数是否合法。
 *
 * @param dir 业务方向，必须是 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN。
 * @return 方向合法返回 1，否则返回 0。
 */
int MotorDriver_IsDirValid(int dir)
{
    return (dir == MOTOR_DIRECTION_UP) || (dir == MOTOR_DIRECTION_DOWN);
}

/**
 * @brief 下发停止命令并刷新显示状态。
 *
 * 该函数用于命令切换、异常保护和主动停止场景。停止命令只表示开始减速，
 * 最终显示状态仍会结合驱动 vzero / rampstat 判断。
 */
void MotorDriver_StopAndMarkStopped(void)
{
    uint32_t ret;
    bool is_moving = true;
    uint32_t start_tick;

    ret = stpr_stop(&stepper);
    if (ret != NO_ERROR) {
        printf("电机停止命令写入失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        return;
    }

    /* 命令切换时不能只下发停止就返回，否则下一条命令会在电机减速过程中被读取执行。 */
    start_tick = HAL_GetTick();
    while (MotorDriver_TryReadMovingState(&stepper, &is_moving) && is_moving) {
        MotorPosition_SyncDebugDrumState(&stepper);
        if ((HAL_GetTick() - start_tick) > MOTOR_STOP_WAIT_TIMEOUT_MS) {
            printf("电机停止等待超时，继续退出当前操作\r\n");
            break;
        }
        HAL_Delay(10U);
    }

    MotorPosition_SyncDebugDrumState(&stepper);

    /* 停止命令只是开始减速，只有驱动确认 vzero 后才显示静止。 */
    if (!is_moving) {
        g_measurement.debug_data.motor_state = 0U;
    }
}

/**
 * @brief 检查是否有有效命令切换请求，并在需要时停止电机。
 *
 * @return 已检测到命令切换并停止返回 true；否则返回 false。
 */
bool MotorDriver_StopIfCommandSwitchRequested(void)
{
    if (HasEffectiveCommandSwitchRequest()) {
        printf("检测到命令切换请求，停止当前操作\r\n");
        MotorDriver_StopAndMarkStopped();
        return true;
    }
    return false;
}

/**
 * @brief 从 TMC5130 读取当前运动状态。
 *
 * 通过 RAMPSTAT/VACTUAL 等驱动状态推断电机是否仍在运动。
 * @param tmc5130 TMC5130 设备对象。
 * @param is_moving 输出运动状态。
 * @return 读取成功返回 true，通信失败返回 false。
 */
bool MotorDriver_TryReadMovingState(TMC5130TypeDef *tmc5130, bool *is_moving)
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

/**
 * @brief 根据驱动状态推断上层显示用运动状态。
 *
 * 显示状态只区分停止、上行、下行，读取失败时会尽量使用最近缓存状态兜底。
 * @param tmc5130 TMC5130 设备对象。
 * @return 0 表示停止，1 表示上行，2 表示下行。
 */
uint32_t MotorDriver_InferDisplayStateFromDriver(TMC5130TypeDef *tmc5130)
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

/**
 * @brief 按指定尺带长度计算恒线速度对应的 VMAX。
 *
 * 用于运行中速度补偿和下发运动前的速度计算。
 * @param L_mm 当前或目标参考尺带长度，单位 mm。
 * @return 对应 TMC5130 VMAX。
 */
uint32_t MotorDriver_ComputeUniformVelocityFromLength(double L_mm)
{
    const int32_t speed_x100 = MotorDriver_GetSpeedSetpointX100();
    double C_cur = MotorPosition_TapeInstantCircumferenceFromLength(L_mm);
    if (C_cur > 1e-6) {
        const double ticks_per_rev = (double)MotorPosition_TapeTicksPerRev();

        /* 目标输出轴微步速度：usteps/s
           线速度(mm/s) = speed_x100 / 6
           rev/s       = (speed_x100 / 6) / C_cur
           usteps/s    = rev/s * ticks_per_rev
                       = speed_x100 * ticks_per_rev / (6*C_cur) */
        const double usteps_per_s =
            ((double)speed_x100 * ticks_per_rev) / (6.0 * C_cur);

        return MotorDriver_UstepsPerSecToVmax(usteps_per_s);
    }

    return MotorDriver_ComputeBaseVelocityFromParams();
}

/**
 * @brief 运动过程中按卷径变化刷新 TMC5130 VMAX。
 *
 * 函数内部带刷新周期和变化阈值，避免高频重复写寄存器。
 * @param tmc5130 TMC5130 设备对象。
 * @param last_refresh_tick 上次刷新 tick，函数会在成功检查后更新。
 */
void MotorDriver_RefreshVelocityDuringRun(TMC5130TypeDef *tmc5130,
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
    MotorCtrl_UpdateDrumStateFromXActual(tmc5130, &drum);
    MotorPosition_UpdatePositionFromMotorSource(&drum);
    const double Lcur_mm = (g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) ?
                           ((double)g_measurement.debug_data.cable_length * 0.1) :
                           ((double)drum.motor_distance_01mm * 0.1);

    const uint32_t new_v = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);
    const uint32_t old_v = (s_motor_driver.applied_velocity != 0U) ? s_motor_driver.applied_velocity : velocity;

    uint32_t delta = (new_v >= old_v) ? (new_v - old_v) : (old_v - new_v);
    uint32_t threshold = (old_v * MOTOR_LINEAR_COMP_APPLY_DELTA_X100 + 99U) / 100U;

    if (threshold == 0U) {
        threshold = 1U;
    }

    if (delta >= threshold) {
        uint32_t ret = stpr_setVelocity(tmc5130, new_v);
        if (ret != NO_ERROR) {
            printf("速度刷新失败 | 错误码=0x%08lX | 目标VMAX=%lu\r\n",
                   (unsigned long)ret,
                   (unsigned long)new_v);
            return;
        }
        s_motor_driver.applied_velocity = new_v;
        velocity = new_v;

        printf("速度刷新 | 线速度=%.2f m/min | 尺带长度=%.1f mm | 周长=%.1f mm | 旧VMAX=%lu | 新VMAX=%lu | 输出轴=%.3f->%.3f r/s | 电机=%.3f->%.3f r/s | 微步/s=%.1f->%.1f\r\n",
               (double)MotorDriver_GetSpeedSetpointX100() / 100.0,
               Lcur_mm,
               MotorPosition_TapeInstantCircumferenceFromLength(Lcur_mm),
               (unsigned long)old_v,
               (unsigned long)new_v,
               MotorDriver_VmaxToOutputRevPerSec(old_v),
               MotorDriver_VmaxToOutputRevPerSec(new_v),
               MotorDriver_VmaxToMotorRevPerSec(old_v),
               MotorDriver_VmaxToMotorRevPerSec(new_v),
               MotorDriver_VmaxToUstepsPerSec(old_v),
               MotorDriver_VmaxToUstepsPerSec(new_v));
    }

    *last_refresh_tick = now_tick;
#else
    (void)tmc5130;
    (void)last_refresh_tick;
#endif
}

/**
 * @brief 处理运动接口传入的可选速度参数。
 *
 * speed_x100 为 0 时使用默认速度；非 0 时先限幅再用于本次运动。
 * @param speed_x100 请求速度，单位 0.01m/min。
 * @return 本次运动应使用的速度设定。
 */
uint32_t MotorDriver_ApplyOptionalSpeed(uint32_t speed_x100)
{
    /* 约定：
     * speed_x100 == 0 表示“调用方不想改速度，只沿用当前设定值”。
     * 这样老接口和新接口可以共用一套实现，而不会强制改写速度状态。 */
    if (speed_x100 == 0U) {
        return NO_ERROR;
    }
    return MotorCtrl_SetSpeed(speed_x100);
}

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
        *restore_speed_x100 = MotorDriver_GetDefaultSpeedSetpointX100();
    }

    if (speed_x100 == 0U) {
        return NO_ERROR;
    }

    if (restore_needed) {
        *restore_needed = true;
    }

    return MotorCtrl_SetSpeed(speed_x100);
}

/**
 * @brief 单次运动结束后恢复临时速度。
 *
 * @param restore_needed 是否需要恢复。
 * @param restore_speed_x100 需要恢复的速度，单位 0.01m/min。
 * @return 成功返回 NO_ERROR，否则返回通信错误码。
 */
uint32_t MotorDriver_EndTemporarySpeed(bool restore_needed,
                                               uint32_t restore_speed_x100)
{
    /* 只有 Begin 确认本次命令确实切换过速度，这里才执行恢复。 */
    if (!restore_needed) {
        return NO_ERROR;
    }
    return MotorCtrl_SetSpeed(restore_speed_x100);
}

/* ===================== 私有函数实现 ===================== */

/**
 * @brief 读取当前设备参数中的速度设定。
 *
 * 参数异常时返回默认兜底速度，保证后续换算不会使用 0 或越界值。
 * @return 当前速度设定，单位 0.01m/min。
 */
static int32_t MotorDriver_GetSpeedSetpointX100(void)
{
    return (int32_t)MotorDriver_ClampSpeedSetpointX100(g_measurement.debug_data.motor_speed);
}

/**
 * @brief 将速度设定限制在设备允许范围内。
 *
 * @param speed_x100 请求速度，单位 0.01m/min。
 * @return 限幅后的速度设定，单位 0.01m/min。
 */
static uint32_t MotorDriver_ClampSpeedSetpointX100(uint32_t speed_x100)
{
    if (speed_x100 < MOTOR_LINEAR_SPEED_MIN_X100) {
        return MOTOR_LINEAR_SPEED_MIN_X100;
    }
    if (speed_x100 > MOTOR_LINEAR_SPEED_MAX_X100) {
        return MOTOR_LINEAR_SPEED_MAX_X100;
    }
    return speed_x100;
}

/**
 * @brief 将电机电流设定限制在 TMC5130 可接受范围内。
 *
 * @param current 请求电流档位。
 * @return 限幅后的电流档位。
 */
static uint32_t MotorDriver_ClampCurrentSetting(uint32_t current)
{
    if ((current < MOTOR_CURRENT_MIN) || (current > MOTOR_CURRENT_MAX)) {
        return MOTOR_CURRENT_DEFAULT;
    }
    return current;
}

/**
 * @brief 将 TMC5130 VMAX 寄存器值换算为微步每秒。
 *
 * @param vmax TMC5130 VMAX 寄存器值。
 * @return 等效微步速度，单位 microsteps/s。
 */
static double MotorDriver_VmaxToUstepsPerSec(uint32_t vmax)
{
    return ((double)vmax * TMC5130_FCLK_HZ) / 16777216.0;
}

static double MotorDriver_VmaxToOutputRevPerSec(uint32_t vmax)
{
    const double ticks_per_rev = (double)MotorPosition_TapeTicksPerRev();
    if (ticks_per_rev <= 1e-6) {
        return 0.0;
    }
    return MotorDriver_VmaxToUstepsPerSec(vmax) / ticks_per_rev;
}

static double MotorDriver_VmaxToMotorRevPerSec(uint32_t vmax)
{
    return MotorDriver_VmaxToUstepsPerSec(vmax) / (1600.0 * 32.0);
}

/**
 * @brief 将 64 位速度换算结果压回 TMC5130 32 位寄存器范围。
 *
 * @param v 速度换算中间值。
 * @return 可写入 VMAX 的 32 位速度值。
 */
static uint32_t MotorDriver_ClampVelocityU64(uint64_t v)
{
    if (v < 1ULL) {
        return 1U;
    }
    if (v > (uint64_t)TMC5130_MAX_VELOCITY) {
        return (uint32_t)TMC5130_MAX_VELOCITY;
    }
    return (uint32_t)v;
}

/**
 * @brief 将微步每秒换算为 TMC5130 VMAX 寄存器值。
 *
 * @param usteps_per_s 目标微步速度。
 * @return VMAX 寄存器值。
 */
static uint32_t MotorDriver_UstepsPerSecToVmax(double usteps_per_s)
{
    /* TMC5130: usteps/s = VMAX * fCLK / 2^24
       => VMAX = usteps/s * 2^24 / fCLK */
    double vmax = usteps_per_s * 16777216.0 / TMC5130_FCLK_HZ;   /* 2^24 */

    if (vmax < 1.0) {
        vmax = 1.0;
    }

    return MotorDriver_ClampVelocityU64((uint64_t)llround(vmax));
}

/**
 * @brief 按设备速度参数计算当前卷径下的基础 VMAX。
 *
 * 该函数会读取当前尺带长度和卷筒模型，将线速度换算为步进电机速度。
 * @return 当前长度对应的 VMAX。
 */
static uint32_t MotorDriver_ComputeBaseVelocityFromParams(void)
{
    const int32_t speed_x100 = MotorDriver_GetSpeedSetpointX100();
    const double C0_mm = MotorPosition_TapeC0Mm();
    const double C_ref_mm = (C0_mm > 1e-6) ? C0_mm : (2.0 * M_PI * (double)TAPE_MIN_RADIUS_MM);
    const double ticks_per_rev = (double)MotorPosition_TapeTicksPerRev();

    /* 先算目标输出轴微步速度：usteps/s */
    const double usteps_per_s = ((double)speed_x100 * ticks_per_rev) / (6.0 * C_ref_mm);

    /* 再换成 TMC5130 VMAX */
    return MotorDriver_UstepsPerSecToVmax(usteps_per_s);
}

/**
 * @brief 获取默认运动速度设定。
 *
 * 当前默认值来自设备参数 max_motor_speed，并经过合法范围限制。
 * @return 默认速度，单位 0.01m/min。
 */
static uint32_t MotorDriver_GetDefaultSpeedSetpointX100(void)
{
    /* “默认恢复速度”统一取设备参数中的最大速度。
     * 后续如果要把“恢复默认速度”改成别的策略，只需要改这里。 */
    return MotorDriver_ClampSpeedSetpointX100(g_deviceParams.max_motor_speed);
}
