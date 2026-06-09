#include "motor_ctrl_internal.h"
#include "error_log.h"

#define MOTOR_DRIVER_INIT_POWER_READY_TIMEOUT_MS  3000U
#define MOTOR_DRIVER_INIT_POWER_READY_POLL_MS     100U
#define MOTOR_DRIVER_INIT_POWER_READY_LOG_MS      500U
#define MOTOR_DRIVER_DRVSTATUS_CS_ACTUAL_MASK     0x001F0000UL
#define MOTOR_DRIVER_DRVSTATUS_CS_ACTUAL_SHIFT    16U
#define MOTOR_DRIVER_RAMPSTAT_VZERO_MASK          0x400U
#define MOTOR_DRIVER_POSITION_TOLERANCE_TICKS     1024L

/**
 * @file motor_ctrl_driver_param.c
 * @brief 电机驱动初始化、速度/电流参数和底层状态判断。
 *
 * 本文件负责把设备参数转换成 TMC5130 可接受的速度、电流和运动状态口径。
 * 对外提供初始化、速度设置、电流设置和驱动异常检查；对内提供运行中速度刷新、
 * 停止命令和显示状态推断等基础能力。
 */

/* ===================== 私有函数声明 ===================== */

static int32_t MotorDriver_GetSpeedSetpointX100(void);
static uint32_t MotorDriver_ClampSpeedSetpointX100(uint32_t speed_x100);
static uint32_t MotorDriver_ClampCurrentSetting(uint32_t current);
static uint32_t MotorDriver_SetSpeedInternal(uint32_t speed_x100, bool print_result);
static double MotorDriver_VmaxToUstepsPerSec(uint32_t vmax);
static double MotorDriver_VmaxToOutputRevPerSec(uint32_t vmax);
static double MotorDriver_VmaxToMotorRevPerSec(uint32_t vmax);
static uint32_t MotorDriver_ClampVelocityU64(uint64_t v);
static uint32_t MotorDriver_UstepsPerSecToVmax(double usteps_per_s);
static uint32_t MotorDriver_ComputeBaseVelocityFromParams(void);
static uint32_t MotorDriver_GetDefaultSpeedSetpointX100(void);
static uint32_t MotorDriver_ClearInitResetFlag(void);
static uint32_t MotorDriver_CheckInitPowerReadyWithRetry(void);
static uint32_t MotorDriver_ReinitIfMotionNotReady(void);
static uint32_t MotorDriver_CheckMotionReadyInternal(bool ignore_encoder_ready);
static uint32_t MotorDriver_ReadTargetPositionOpen(TMC5130TypeDef *tmc5130, bool *target_open);
static uint32_t MotorDriver_AlignTargetToActual(TMC5130TypeDef *tmc5130);

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

void MotorCtrl_InvalidateDriverInit(void)
{
    /* 运行中掉电/复位只让“当前初始化有效性”失效，不能抹掉曾经完成初始化的事实。
     * 后续 MotorCtrl_Init() 应走非首次刷新配置分支，避免恢复 FRAM 位置和位置源。 */
    s_motor_driver.applied_velocity = 0U;
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    s_motor_driver.boot_safe_stop_done = false;
}

bool MotorCtrl_IsDriverInitValid(void)
{
    return s_motor_driver.initialized && s_motor_driver.boot_safe_stop_done;
}

/**
 * @brief 上电早期清除 TMC5130 残留运动状态。
 *
 * 该函数在外设初始化完成后、App_Init() 前调用：先保持驱动输出关闭，
 * 再把 VMAX 清零、XTARGET 对齐当前 XACTUAL，并切回位置模式。
 * 这样即使设备在运动中复位，TMC5130 也不会沿用旧速度或旧目标继续跑。
 * 本函数会访问 SPI 和 GPIO，只能在任务上下文调用，不能在中断中调用。
 */
uint32_t MotorCtrl_BootSafeStop(void)
{
    uint32_t ret = NO_ERROR;
    int32_t xactual = 0;

    s_motor_driver.initialized = false;
    s_motor_driver.applied_velocity = 0U;
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    s_motor_driver.boot_safe_stop_done = false;
    g_measurement.debug_data.motor_state = 0U;

    /* 先关 EN，确保后续寄存器清理期间功率级不会输出脉冲。 */
    stpr_disableDriver(&stepper);

    if (!stpr_writeInt(&stepper, TMC5130_VMAX, 0)) {
        ret = MOTOR_TMC_COMM_ERROR;
    }

    /* 用当前位置覆盖目标位置，防止旧 XTARGET 在驱动重新使能后继续生效。 */
    if (stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual)) {
        if (!stpr_writeInt(&stepper, TMC5130_XTARGET, xactual)) {
            ret = MOTOR_TMC_COMM_ERROR;
        }
    } else {
        ret = MOTOR_TMC_COMM_ERROR;
    }

    if (!stpr_writeInt(&stepper, TMC5130_RAMPMODE, TMC5130_MODE_POSITION)) {
        ret = MOTOR_TMC_COMM_ERROR;
    }

    stpr_disableDriver(&stepper);
    s_motor_driver.boot_safe_stop_done = (ret == NO_ERROR);

    if (ret != NO_ERROR) {
        printf("电机上电安全停机失败：0x%08lX\r\n", (unsigned long)ret);
    } else {
        printf("电机上电安全停机完成\r\n");
    }

    return ret;
}

/**
 * @brief 写运动寄存器前检查电机和位置源是否允许运动。
 *
 * 所有上层运动入口在写 VMAX/XTARGET/RAMPMODE 前调用这里。
 * 编码轮记步模式必须等编码器首帧有效；电机记步模式允许编码器后台异常，
 * 避免因为编码器悬空阻断电机记步模式下的受控运动。
 */
uint32_t MotorDriver_CheckMotionReady(void)
{
    return MotorDriver_CheckMotionReadyInternal(false);
}

/**
 * @brief 强制调试运动专用就绪检查。
 *
 * 只绕过编码器首帧就绪，仍要求上电安全停机和 TMC5130 完整初始化成功。
 * 该入口只给人工强制运动使用，正常测量和普通运动不能调用。
 */
uint32_t MotorDriver_CheckMotionReadyForceDebug(void)
{
    return MotorDriver_CheckMotionReadyInternal(true);
}

/**
 * @brief 运动入口发现驱动初始化状态失效时，先尝试重新初始化电机。
 *
 * 运行中 24V 断电或 TMC5130 复位会让底层调用 MotorCtrl_InvalidateDriverInit()；
 * 下一次业务重试不能直接返回 MOTOR_DISABLED，而应先重新下发 TMC5130 配置。
 */
static uint32_t MotorDriver_ReinitIfMotionNotReady(void)
{
    uint32_t ret;

    if (s_motor_driver.initialized && s_motor_driver.boot_safe_stop_done) {
        return NO_ERROR;
    }

    printf("电机运动准备 | 驱动未初始化或安全停机状态失效，尝试重新初始化\r\n");
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    g_measurement.debug_data.motor_state = 0U;

    ret = MotorCtrl_Init();
    if (ret != NO_ERROR) {
        printf("电机运动准备 | 自动重新初始化失败 | 返回=0x%08lX\r\n",
               (unsigned long)ret);
        return ret;
    }

    printf("电机运动准备 | 自动重新初始化完成\r\n");
    return NO_ERROR;
}

static uint32_t MotorDriver_CheckMotionReadyInternal(bool ignore_encoder_ready)
{
    uint32_t ret;

    ret = MotorDriver_ReinitIfMotionNotReady();
    if (ret != NO_ERROR) {
        return ret;
    }

    if (MotorCtrl_IsPositionSourceMotor()) {
        /* 电机记步模式下，编码器只作为后台采集对象，旧编码器错误不阻断运动。 */
        if ((g_measurement.device_status.error_code >= ENCODER_TIMEOUT) &&
            (g_measurement.device_status.error_code <= ENCODER_OCF_INCOMPLETE)) {
            g_measurement.device_status.error_code = NO_ERROR;
        }
        return NO_ERROR;
    }

    if (!Encoder_IsReady()) {
        if (ignore_encoder_ready) {
            printf("强制调试运动：忽略编码器首帧未就绪，仅保留驱动安全检查\r\n");
            return NO_ERROR;
        }
        /* 编码轮记步模式下没有首帧可信位置，必须禁止下发运动命令。 */
        g_measurement.device_status.error_code = ENCODER_TIMEOUT;
        printf("电机运动被拦截：编码器首帧尚未就绪\r\n");
        return ENCODER_TIMEOUT;
    }

    return NO_ERROR;
}

/**
 * @brief 初始化检查前清除 TMC5130 上电复位标志。
 *
 * 断开再恢复电机 24V 后，TMC5130 可能只留下 GSTAT[0] reset 标志。
 * 初始化阶段已经准备重新下发配置，这个纯 reset 标志不应阻断第一次初始化；
 * 若同时存在 drv_err 或 uv_cp，则仍交给运行期状态检查按真实故障处理。
 * @return NO_ERROR 表示无需处理或清除成功，否则返回通信错误码。
 */
static uint32_t MotorDriver_ClearInitResetFlag(void)
{
    int32_t gstat = 0;
    uint32_t gstat_raw;

    /* 先读取 GSTAT，判断是否只有 reset 标志；读取失败说明 SPI/芯片不可用。 */
    if (!stpr_tryReadInt(&stepper, TMC5130_GSTAT, &gstat)) {
        return MOTOR_TMC_COMM_ERROR;
    }

    gstat_raw = (uint32_t)gstat;
    /* 只处理纯 reset 标志；如果还带欠压或 drv_err，保留给后续健康检查归因。 */
    if (gstat_raw != 0x00000001UL) {
        return NO_ERROR;
    }

    printf("TMC5130初始化检测到芯片复位标志，清除后继续初始化 | GSTAT=0x%08lX\r\n",
           (unsigned long)gstat_raw);
    /* 写 1 清 reset 位，避免第一次上电恢复被历史复位标志阻断。 */
    if (!stpr_writeInt(&stepper, TMC5130_GSTAT, 0x01)) {
        return MOTOR_TMC_COMM_ERROR;
    }

    return NO_ERROR;
}

/**
 * @brief 初始化期等待 TMC5130 功率级实际电流建立。
 *
 * 电机 24V 刚恢复时，寄存器可能已经可写、GSTAT reset 也能清除，但 DRV_STATUS.CS_ACTUAL
 * 仍短时间为 0。初始化期允许等待几秒；超时后再按电机被禁止返回，交给自动恢复继续等待。
 */
static uint32_t MotorDriver_CheckInitPowerReadyWithRetry(void)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_log_tick = 0U;
    int32_t drvstatus = 0;

    while (1) {
        uint32_t now = HAL_GetTick();

        if (stpr_tryReadInt(&stepper, TMC5130_DRVSTATUS, &drvstatus)) {
            uint32_t cs_actual = (((uint32_t)drvstatus) & MOTOR_DRIVER_DRVSTATUS_CS_ACTUAL_MASK) >>
                                 MOTOR_DRIVER_DRVSTATUS_CS_ACTUAL_SHIFT;
            if (cs_actual != 0U) {
                if ((uint32_t)(now - start_tick) > 0U) {
                    printf("TMC5130初始化等待电流建立完成 | DRV_STATUS=0x%08lX | CS_ACTUAL=%lu\r\n",
                           (unsigned long)((uint32_t)drvstatus),
                           (unsigned long)cs_actual);
                }
                return NO_ERROR;
            }
        }

        if ((uint32_t)(now - start_tick) >= MOTOR_DRIVER_INIT_POWER_READY_TIMEOUT_MS) {
            printf("TMC5130初始化等待电流建立超时，按电机被禁止处理\r\n");
            return stpr_checkDriverPowerReady(&stepper);
        }

        if ((last_log_tick == 0U) ||
            ((uint32_t)(now - last_log_tick) >= MOTOR_DRIVER_INIT_POWER_READY_LOG_MS)) {
            printf("TMC5130初始化等待电流建立 | DRV_STATUS=0x%08lX\r\n",
                   (unsigned long)((uint32_t)drvstatus));
            last_log_tick = now;
        }
        HAL_Delay(MOTOR_DRIVER_INIT_POWER_READY_POLL_MS);
    }
}

/**
 * @brief 统一检查 TMC5130 通信、配置、故障标志和功率级状态。
 *
 * 不同模式只影响未初始化时的返回语义；GSTAT/DRV_STATUS 故障位先由
 * stpr_checkDriverStatus() 解析，随后统一追加 stpr_checkDriverPowerReady()，
 * 避免业务路径重复读取功率级状态。
 */
uint32_t MotorDriver_CheckHealth(MotorDriverHealthMode mode)
{
    uint32_t ret;

    /* 运行期/运动前发现驱动未初始化，直接按电机不可用处理，不继续读寄存器。 */
    if (!s_motor_driver.initialized && (mode != MOTOR_DRIVER_HEALTH_INIT_CHECK)) {
        return MOTOR_DISABLED;
    }

    if (mode == MOTOR_DRIVER_HEALTH_INIT_CHECK) {
        /* 初始化阶段允许清除纯 reset 标志；欠压和驱动错误仍由后续状态检查处理。 */
        ret = MotorDriver_ClearInitResetFlag();
        if (ret != NO_ERROR) {
            return ret;
        }
    }

    /* 先解析 GSTAT/DRV_STATUS 的真实故障位，再追加功率级电流建立检查。 */
    ret = stpr_checkDriverStatus(&stepper);
    if (ret != NO_ERROR) {
        return ret;
    }

    if (mode == MOTOR_DRIVER_HEALTH_INIT_CHECK) {
        ret = MotorDriver_CheckInitPowerReadyWithRetry();
    } else {
        ret = stpr_checkDriverPowerReady(&stepper);
    }
    if (ret != NO_ERROR) {
        return ret;
    }

    return NO_ERROR;
}


/**
 * @brief 应用电机速度参数，可选择是否打印用户操作结果。
 *
 * 公开接口保留简洁日志；内部临时速度、停机恢复等路径保持安静，避免正常运动流程刷屏。
 */
static uint32_t MotorDriver_SetSpeedInternal(uint32_t speed_x100, bool print_result)
{
    const uint32_t clamped_speed = MotorDriver_ClampSpeedSetpointX100(speed_x100);
    const char *apply_state = "空闲待下发";
    bool is_running = false;
    uint32_t ret = NO_ERROR;
    double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;

    /* 先更新“设定速度”本身。
     * 后续所有速度换算都以 debug_data.motor_speed 作为源头。 */
    g_measurement.debug_data.motor_speed = clamped_speed;

    if (s_motor_driver.initialized) {
        ret = MotorCtrl_IsDriverMoving(&stepper, &is_running);
        if (ret != NO_ERROR) {
            return ret;
        }
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
        /* 运行中改速：立即写入驱动，让本次运动立刻生效。 */
        ret = stpr_setVelocity(&stepper, velocity);
        if (ret != NO_ERROR) {
            return ret;
        }
        apply_state = "运行中已下发";
    }

    if (is_running && s_motor_driver.motion_command_active) {
        uint32_t inferred_state = MotorDriver_InferDisplayStateFromDriver(&stepper);
        if (((g_measurement.debug_data.motor_state != 1U) &&
             (g_measurement.debug_data.motor_state != 2U)) &&
            ((inferred_state == 1U) || (inferred_state == 2U))) {
            g_measurement.debug_data.motor_state = inferred_state;
        }
    } else if (!s_motor_driver.motion_wait_active) {
        if (!is_running) {
            s_motor_driver.motion_command_active = false;
            s_motor_driver.motion_wait_active = false;
        }
        g_measurement.debug_data.motor_state = 0U;
    }

    if (print_result) {
        printf("电机速度设置完成 | 请求=%.2f m/min | 生效=%.2f m/min | 尺带=%.1fmm | VMAX=%lu | 状态=%s\r\n",
               (double)speed_x100 / 100.0,
               (double)clamped_speed / 100.0,
               Lcur_mm,
               (unsigned long)velocity,
               apply_state);
    }

    return NO_ERROR;
}

static uint32_t MotorDriver_ReadTargetPositionOpen(TMC5130TypeDef *tmc5130, bool *target_open)
{
    int32_t xactual = 0;
    int32_t xtarget = 0;
    int64_t diff;

    if ((tmc5130 == NULL) || (target_open == NULL)) {
        return PARAM_ERROR;
    }

    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    if (!stpr_tryReadInt(tmc5130, TMC5130_XTARGET, &xtarget)) {
        return MOTOR_TMC_COMM_ERROR;
    }

    diff = (int64_t)xactual - (int64_t)xtarget;
    if (diff < 0) {
        diff = -diff;
    }

    *target_open = (diff > (int64_t)MOTOR_DRIVER_POSITION_TOLERANCE_TICKS);
    return NO_ERROR;
}

static uint32_t MotorDriver_AlignTargetToActual(TMC5130TypeDef *tmc5130)
{
    int32_t xactual = 0;

    if (tmc5130 == NULL) {
        return PARAM_ERROR;
    }

    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    if (!stpr_writeInt(tmc5130, TMC5130_XTARGET, xactual)) {
        return MOTOR_TMC_COMM_ERROR;
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
    const char *apply_state = "下次初始化生效";

    g_deviceParams.motor_current = clamped_current;

    if (s_motor_driver.initialized) {
        uint32_t ret;
        /* 电流参数写入后立即更新 TMC5130，避免必须重启才生效。 */
        ret = stpr_setCurrent(&stepper, (uint8_t)clamped_current);
        if (ret != NO_ERROR) {
            return ret;
        }
        apply_state = "已下发";
    }

    printf("电机电流设置完成 | 请求=%lu | 生效=%lu | 状态=%s\r\n",
           (unsigned long)current,
           (unsigned long)clamped_current,
           apply_state);

    return NO_ERROR;
}

/**
 * @brief 初始化 TMC5130 电机驱动和电机控制运行态。
 *
 * 首次初始化会恢复持久化位置和位置源；非首次初始化用于恢复/退出调试，
 * 只重写 TMC5130 寄存器配置，不恢复 FRAM 位置和位置源。
 * @return 成功返回 NO_ERROR，否则返回驱动通信或状态错误码。
 */
uint32_t MotorCtrl_Init(void)
{
    uint32_t motor_current = MotorDriver_ClampCurrentSetting(g_deviceParams.motor_current);
    const char *init_mode;

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
    init_mode = s_motor_driver.initialized ? "刷新配置" : "首次配置";
    if (s_motor_driver.initialized) {
        /* 初始化入口被重复调用时先做一次轻量探测，若运行期配置丢失可在后续分支重下发。 */
        (void)MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_RUNNING);
    }

    /* MotorCtrl_Init() 可能用于自动恢复，返回值只代表本次初始化结果，
     * 不能通过 CHECK_ERROR() 再读取历史全局错误码。 */
    if (!s_motor_driver.initialized) {
        uint32_t ret = stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, (uint8_t)motor_current);
        if (ret != NO_ERROR) {
            return ret;
        }
//      stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 16);
        stpr_enableDriver(&stepper);
        s_motor_driver.initialized = true;
        /* 使能后立即确认 24V 功率级和配置寄存器，避免未上电时仍显示初始化成功。 */
        ret = MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_INIT_CHECK);
        if (ret != NO_ERROR) {
            s_motor_driver.initialized = false;
            return ret;
        }
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

        /* 非首次初始化也必须重写整套 TMC5130 寄存器。
         * BE 等速度模式调试退出后，不能只刷新电流和 VMAX，否则 RAMPMODE、斜坡等
         * 配置可能沿用调试残留；非首次不恢复 FRAM 位置，也不恢复位置源。 */
        stpr_disableDriver(&stepper);
        ret = stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, (uint8_t)motor_current);
        if (ret != NO_ERROR) {
            s_motor_driver.initialized = false;
            return ret;
        }

        stpr_enableDriver(&stepper);
        s_motor_driver.initialized = true;
        ret = MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_INIT_CHECK);
        if (ret != NO_ERROR) {
            s_motor_driver.initialized = false;
            return ret;
        }

        /* stpr_initStepper() 会写默认速度寄存器；重写寄存器后再写当前业务速度。 */
        ret = stpr_setVelocity(&stepper, velocity);
        if (ret != NO_ERROR) {
            s_motor_driver.initialized = false;
            stpr_disableDriver(&stepper);
            return ret;
        }
        s_motor_driver.applied_velocity = velocity;
    }
    MotorPosition_SyncDebugDrumState(&stepper);
    /* 完整初始化成功后同样视为旧运动状态已经清理，可开放后续运动入口。 */
    s_motor_driver.boot_safe_stop_done = true;

    printf("电机初始化完成 | 方式=%s | 速度=%.2f m/min | 尺带=%.1fmm | 周长=%.1fmm | VMAX=%lu | 电流=%lu | 等效微步/s=%.1f\r\n",
           init_mode,
           (double)MotorDriver_GetSpeedSetpointX100() / 100.0,
           g_measurement.debug_data.cable_length / 10.0,
           MotorPosition_TapeInstantCircumferenceFromLength((double)g_measurement.debug_data.cable_length * 0.1),
           (unsigned long)velocity,
           (unsigned long)motor_current,
           MotorDriver_VmaxToUstepsPerSec(velocity));
    return NO_ERROR;
}

/**
 * @brief Read whether the TMC5130 driver is still moving.
 *
 * @param tmc5130 TMC5130 device object.
 * @param is_moving Output moving state when return is NO_ERROR.
 * @return NO_ERROR, PARAM_ERROR or MOTOR_TMC_COMM_ERROR.
 */
uint32_t MotorCtrl_IsDriverMoving(TMC5130TypeDef *tmc5130, bool *is_moving)
{
    return MotorDriver_ReadMovingState(tmc5130, is_moving);
}

/**
 * @brief 对外兼容的 TMC5130 驱动健康检查入口。
 *
 * 历史接口名保留为 Gstat，但当前系统只有一个全局 stepper，
 * 实际会统一检查 GSTAT/DRV_STATUS、配置寄存器和功率级状态。
 * @return NO_ERROR 或对应驱动故障错误码。
 */
uint32_t MotorCtrl_CheckDriverGstat(void)
{
    return MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_RUNNING);
}
/* ===================== 内部跨文件接口 ===================== */

/**
 * @brief 静默设置电机速度参数。
 *
 * 给停机恢复和内部临时速度使用，不打印用户操作日志。
 */
uint32_t MotorDriver_SetSpeedQuiet(uint32_t speed_x100)
{
    return MotorDriver_SetSpeedInternal(speed_x100, false);
}

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
 * 该函数只更新软件侧全局 velocity 缓存，不主动判断运动状态。
 * 调用方如果需要立即生效，应继续写入 TMC5130 VMAX。
 */
void MotorDriver_UpdateVelocityFromParams(void)
{
    const double Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);

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
uint32_t MotorDriver_StopAndMarkStopped(void)
{
    uint32_t ret;
    bool is_moving = true;
    uint32_t start_tick;

    ret = stpr_stop(&stepper);
    if (ret != NO_ERROR) {
        printf("电机停止命令写入失败，错误码：0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    /* 命令切换时不能只下发停止就返回，否则下一条命令会在电机减速过程中被读取执行。 */
    start_tick = HAL_GetTick();
    while (1) {
        ret = MotorDriver_ReadStoppingState(&stepper, &is_moving);
        if (ret != NO_ERROR) {
            printf("电机停止状态读取失败，错误码：0x%08lX\r\n", (unsigned long)ret);
            return ret;
        }
        if (!is_moving) {
            break;
        }
        ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
        if (ret != NO_ERROR) {
            return ret;
        }
        if ((HAL_GetTick() - start_tick) > MOTOR_STOP_WAIT_TIMEOUT_MS) {
            printf("电机停止等待超时，错误码：0x%08lX\r\n", (unsigned long)MOTOR_RUN_TIMEOUT);
            return MOTOR_RUN_TIMEOUT;
        }
        HAL_Delay(10U);
    }

    ret = MotorDriver_SyncPositionOrCheckHealth(&stepper);
    if (ret != NO_ERROR) {
        return ret;
    }

    ret = MotorDriver_AlignTargetToActual(&stepper);
    if (ret != NO_ERROR) {
        return ret;
    }

    /* 停止命令只是开始减速，只有驱动确认 vzero 后才显示静止。 */
    if (!is_moving) {
        s_motor_driver.motion_command_active = false;
        s_motor_driver.motion_wait_active = false;
        g_measurement.debug_data.motor_state = 0U;
    }
    return NO_ERROR;
}

/**
 * @brief 检查是否有有效命令切换请求，并在需要时停止电机。
 *
 * @return 无命令切换返回 NO_ERROR；命令切换且停止成功返回 STATE_SWITCH；停止失败返回实际错误码。
 */
uint32_t MotorDriver_StopIfCommandSwitchRequested(void)
{
    uint32_t ret;

    if (HasEffectiveCommandSwitchRequest()) {
        printf("检测到命令切换请求，停止当前操作\r\n");
        ret = MotorDriver_StopAndMarkStopped();
        if (ret != NO_ERROR) {
            return ret;
        }
        return COMMAND_SWITCH_ABORT;
    }
    return NO_ERROR;
}

/**
 * @brief Read current TMC5130 moving state.
 *
 * @param tmc5130 TMC5130 device object.
 * @param is_moving Output moving state when return is NO_ERROR.
 * @return NO_ERROR, PARAM_ERROR or MOTOR_TMC_COMM_ERROR.
 */
uint32_t MotorDriver_ReadMovingState(TMC5130TypeDef *tmc5130, bool *is_moving)
{
    int32_t rampstat = 0;
    int32_t rampstat_confirm = 0;
    int32_t vactual = 0;
    bool target_open = false;
    uint32_t ret;

    if ((tmc5130 == NULL) || (is_moving == NULL)) {
        return PARAM_ERROR;
    }

    *is_moving = true;

    if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat)) {
        return MOTOR_TMC_COMM_ERROR;
    }

    if (((uint32_t)rampstat & MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) != MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) {
        *is_moving = true;
        return NO_ERROR;
    }

    HAL_Delay(5U);
    if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat_confirm)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    if (((uint32_t)rampstat_confirm & MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) != MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) {
        *is_moving = true;
        return NO_ERROR;
    }

    if (!stpr_tryReadInt(tmc5130, TMC5130_VACTUAL, &vactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    if (vactual != 0) {
        *is_moving = true;
        return NO_ERROR;
    }

    ret = MotorDriver_ReadTargetPositionOpen(tmc5130, &target_open);
    if (ret != NO_ERROR) {
        return ret;
    }

    *is_moving = target_open;
    return NO_ERROR;
}

uint32_t MotorDriver_ReadStoppingState(TMC5130TypeDef *tmc5130, bool *is_moving)
{
    int32_t rampstat = 0;
    int32_t rampstat_confirm = 0;
    int32_t vactual = 0;

    if ((tmc5130 == NULL) || (is_moving == NULL)) {
        return PARAM_ERROR;
    }

    *is_moving = true;

    if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat)) {
        return MOTOR_TMC_COMM_ERROR;
    }

    if (((uint32_t)rampstat & MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) != MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) {
        return NO_ERROR;
    }

    HAL_Delay(5U);
    if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat_confirm)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    if (((uint32_t)rampstat_confirm & MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) != MOTOR_DRIVER_RAMPSTAT_VZERO_MASK) {
        return NO_ERROR;
    }

    if (!stpr_tryReadInt(tmc5130, TMC5130_VACTUAL, &vactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }

    *is_moving = (vactual != 0);
    return NO_ERROR;
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
            printf("速度刷新失败 | 错误码：0x%08lX | 目标VMAX=%lu\r\n",
                   (unsigned long)ret,
                   (unsigned long)new_v);
            return;
        }
        s_motor_driver.applied_velocity = new_v;
        velocity = new_v;

        printf("速度刷新 | 线速度=%.2f m/min | 尺带长度：%.1f mm | 周长=%.1f mm | 旧VMAX=%lu | 新VMAX=%lu | 输出轴=%.3f->%.3f r/s | 电机=%.3f->%.3f r/s | 微步/s=%.1f->%.1f\r\n",
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
    return MotorDriver_SetSpeedInternal(speed_x100, false);
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

    return MotorDriver_SetSpeedInternal(speed_x100, false);
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
    return MotorDriver_SetSpeedInternal(restore_speed_x100, false);
}

/**
 * @brief 恢复临时速度后按统一优先级返回运动结果。
 *
 * 阻塞型运动退出前调用该函数，先尽量把软件速度恢复到进入运动前的值。
 * 若原运动已经返回故障或 STATE_SWITCH，则原始返回值更能代表退出原因，不能被恢复速度失败覆盖；
 * 只有原运动成功时，恢复速度失败才作为最终错误返回。
 */
uint32_t MotorDriver_ReturnAfterTemporarySpeed(uint32_t ret,
                                                       bool restore_needed,
                                                       uint32_t restore_speed_x100)
{
    uint32_t restore_ret;

    /* 先执行恢复，避免临时速度泄漏到下一条命令；返回值优先级在下面统一判断。 */
    restore_ret = MotorDriver_EndTemporarySpeed(restore_needed, restore_speed_x100);

    /* 命令切换和真实运动故障必须原样向上传递，避免被恢复速度时的通信失败改写原因。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    /* 原运动成功时，恢复失败才是本次接口最终需要暴露的错误。 */
    return restore_ret;
}

/**
 * @brief 同步 XACTUAL 到调试位置，失败时用统一健康检查归因。
 *
 * 该函数用于运行期轮询和停机收尾；同步失败不立即按普通读数丢弃处理，
 * 而是继续检查 TMC5130 配置、GSTAT 和功率级，便于识别 24V 断电或复位。
 */
uint32_t MotorDriver_SyncPositionOrCheckHealth(TMC5130TypeDef *tmc5130)
{
    /* XACTUAL 正常时只刷新模型；同步失败时再读健康状态，区分 SPI 抖动和驱动掉电。 */
    if (MotorPosition_SyncDebugDrumState(tmc5130)) {
        return NO_ERROR;
    }

    return MotorDriver_CheckHealth(MOTOR_DRIVER_HEALTH_RUNNING);
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
