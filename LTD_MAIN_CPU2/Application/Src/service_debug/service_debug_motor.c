/**
 * @file service_debug_motor.c
 * @brief CPU2 串口维护电机运动、编码器往返和 TMC5130 原始寄存器诊断实现。
 *
 * 本文件是 service_debug 中唯一允许包含 motor_ctrl_internal.h 的实现，
 * 原始驱动状态只用于受控维护命令，不作为正常测量流程依赖。
 */

#include "service_debug_internal.h"
#include "AS5145.h"
#include "error_log.h"
#include "fault_manager.h"
#include "motor_ctrl.h"
#include "motor_ctrl_internal.h"
#include "sensor_service.h"
#include "serial_command_parser.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "system_parameter.h"
#include "weight.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define MOTOR_TEXT_ENCODER_MAX_ANGLE             4096.0f /* 串口 B/BE 编码器换算使用的一圈计数基准。 */
#define MOTOR_TEXT_ENCODER_POLL_MS               10U /* 串口 B/BE 编码器轮询周期，单位 ms。 */
#define MOTOR_TEXT_ENCODER_START_GRACE_MS        500U /* 串口 B/BE 启动后的编码器宽限时间，单位 ms。 */
#define MOTOR_TEXT_ENCODER_SENSOR_COMM_INTERVAL_MS 1000U /* 串口 B/BE 运行中传感器通信保活间隔，单位 ms。 */
#define MOTOR_TEXT_ENCODER_TIMEOUT_MARGIN_MS     5000U /* 串口 B/BE 编码器运行超时余量，单位 ms。 */
#define MOTOR_TEXT_ENCODER_TIMEOUT_MIN_MS        10000U /* 串口 B/BE 编码器运行最小超时时间，单位 ms。 */
#define MOTOR_TEXT_ENCODER_TIMEOUT_MAX_MS        120000U /* 串口 B/BE 编码器运行最大超时时间，单位 ms。 */
#define MOTOR_TEXT_ENCODER_TIMEOUT_SCALE         4U /* 串口 B/BE 按目标距离估算超时的倍率。 */
#define MOTOR_TEXT_ENCODER_MULTIPLIER_DEFAULT    1U /* 串口 B/BE 加减速倍率默认值。 */
#define MOTOR_TEXT_ENCODER_MULTIPLIER_MAX        20U /* 串口 B/BE 加减速倍率上限。 */
#define MOTOR_TEXT_ENCODER_DEFAULT_A1            (10 * 32) /* 串口 B/BE 第一段加速度默认寄存器值。 */
#define MOTOR_TEXT_ENCODER_DEFAULT_AMAX          (20 * 32) /* 串口 B/BE 最大加速度默认寄存器值。 */
#define MOTOR_TEXT_ENCODER_DEFAULT_D1            (10 * 32) /* 串口 B/BE 第一段减速度默认寄存器值。 */
#define MOTOR_TEXT_ENCODER_DEFAULT_DMAX          (20 * 32) /* 串口 B/BE 最大减速度默认寄存器值。 */
#define MOTOR_TEXT_RETRY_DELAY_MS                200U /* 串口电机测试失败重试前等待时间，单位 ms。 */
#define MOTOR_TEXT_STOP_POLL_MS                  20U /* 串口电机测试等待停机的轮询周期，单位 ms。 */
#define MOTOR_TEXT_RETRY_LOG_INTERVAL_MS         1000U /* 串口电机测试重试日志打印间隔，单位 ms。 */
#define MOTOR_TEXT_STOP_SETTLE_MS                50U /* 串口电机测试停机后的状态稳定等待时间，单位 ms。 */
#define MOTOR_TEXT_STOP_CONFIRM_MS               200U /* 串口电机测试停机确认等待时间，单位 ms。 */
typedef struct {
    /* 电机文本调试命令进入前保存的设备状态和故障码。 */
    DeviceState device_state; /* 故障或调试快照对应的设备主状态。 */
    uint32_t error_code; /* 电机文本调试接管前保存的完整统一故障码。 */
} MotorTextErrorSnapshot;

typedef struct {
    /* 电机文本调试命令临时改写前保存的四项加减速参数及有效标志。 */
    int32_t a1; /* 调试前保存的 TMC5130 第一段加速度参数 A1。 */
    int32_t amax; /* 调试前保存的 TMC5130 最大加速度参数 AMAX。 */
    int32_t d1; /* 调试前保存的 TMC5130 第一段减速度参数 D1。 */
    int32_t dmax; /* 调试前保存的 TMC5130 最大减速度参数 DMAX。 */
    uint8_t valid; /* 四项加减速参数均已成功读取的标志；退出调试时据此决定是否恢复。 */
} MotorTextRampSnapshot;

static int32_t s_motor_text_raw_target = 0; /* 电机文本调试命令当前保存的原始目标步数。 */

/* BE 测试退出时恢复进入前保存的 TMC5130 斜坡参数。 */
static void Test_MotorTextRestoreRampNoError(const MotorTextRampSnapshot *snapshot);

/**
 * @brief 保存进入 B/BE 诊断前的业务错误状态。
 * @note  只在串口 B/BE 测试上下文调用，避免诊断过程清错影响正常程序。
 *
 * @return 返回进入电机文本诊断前保存的错误码、故障来源和电机状态快照。
 */
static MotorTextErrorSnapshot Test_MotorTextCaptureErrorState(void)
{
    MotorTextErrorSnapshot snapshot;

    snapshot.device_state = g_measurement.device_status.device_state;
    snapshot.error_code = g_measurement.device_status.error_code;
    g_measurement.device_status.device_state = STATE_DEBUG_MODE;
    return snapshot;
}

/**
 * @brief 恢复进入 B/BE 诊断前的业务错误状态。
 * @note  只在 B/BE 退出路径调用，不改变命令切换和电机停机动作。
 *
 * @param snapshot 电机文本测试进入前保存的只读错误状态快照；用于测试退出时恢复全局错误码和相关故障上下文。
 */
static void Test_MotorTextRestoreErrorState(const MotorTextErrorSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    g_measurement.device_status.device_state = snapshot->device_state;
    g_measurement.device_status.error_code = snapshot->error_code;
}

/**
 * @brief 判断测试流程是否因外部命令切换而应立即退出。
 * @return 1 表示检测到有效命令切换请求，函数已执行电机慢停，调用方应立即退出串口测试；0 表示当前没有命令切换请求，可继续测试。
 */
uint8_t Test_ShouldAbortForCommandSwitch(void)
{
    if (!HasEffectiveCommandSwitchRequest()) {
        return 0;
    }

    printf("检测到命令切换请求，停止当前串口测试\r\n");
    MotorCtrl_SlowStop();
    return 1;
}

/**
 * @brief B/BE测试专用：清掉底层函数写入的全局错误态。
 * @note  只在串口调试任务上下文调用，不在中断中调用。
 */
static void Test_MotorTextClearIgnoredError(void)
{
    g_measurement.device_status.error_code = NO_ERROR;
    if (g_measurement.device_status.device_state == STATE_ERROR) {
        g_measurement.device_status.device_state = STATE_DEBUG_MODE;
    }
}

/**
 * @brief B/BE统一退出收尾，恢复进入前错误状态。
 * @note  所有B/BE提前返回路径都应通过该函数，避免诊断清错泄漏到业务状态机。
 *
 * @param error_snapshot 进入电机文本测试前保存的故障快照，用于退出时恢复原有错误上下文。
 */
static void Test_MotorTextExit(const MotorTextErrorSnapshot *error_snapshot)
{
    Test_MotorTextRestoreErrorState(error_snapshot);
}

/**
 * @brief A/B/BE测试专用命令切换检查，只停止并退出当前测试，不把错误码带给业务状态机。
 * @note  只在任务上下文调用；不会解析扭力、电机驱动故障或其它业务错误。
 *
 * @return 1 表示检测到命令切换且当前 A/B/BE 测试已安全停止；没有切换请求时返回 0。
 */
static uint8_t Test_ShouldAbortForCommandSwitchNoError(void)
{
    uint32_t stop_ret;

    if (!HasEffectiveCommandSwitchRequest()) {
        return 0U;
    }

    printf("A/B/BE检测到命令切换请求，停止当前串口测试\r\n");
    stop_ret = MotorDriver_StopAndMarkStopped();
    if (stop_ret != NO_ERROR) {
        printf("A/B/BE命令切换\t停止等待或切回位置模式失败\t返回=0x%08lX\r\n",
               (unsigned long)stop_ret);
    }
    g_measurement.debug_data.motor_state = 0U;
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    Test_MotorTextClearIgnoredError();
    return 1U;
}

/**
 * @brief 执行不检查电机电流门限的文字指令测试。
 * @return 返回文字指令测试采用的电机运行电流档；配置为 0 或非法时回退 MOTOR_CURRENT_DEFAULT。
 */
static uint32_t Test_MotorTextMotorCurrentNoCheck(void)
{
    if ((g_deviceParams.motor_current < MOTOR_CURRENT_MIN) ||
        (g_deviceParams.motor_current > MOTOR_CURRENT_MAX)) {
        return MOTOR_CURRENT_DEFAULT;
    }

    return g_deviceParams.motor_current;
}

/**
 * @brief B/BE测试专用：把本地累计目标重新对齐到驱动实际位置。
 * @note  只在串口调试上下文调用；读写失败仅清理临时错误，不让业务状态机报错。
 */
static void Test_MotorTextSyncRawTargetNoError(void)
{
    int32_t xactual = 0;

    if (stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual)) {
        s_motor_text_raw_target = xactual;
        (void)stpr_writeInt(&stepper, TMC5130_XTARGET, xactual);
    }
    (void)stpr_writeInt(&stepper, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief B/BE测试专用TMC5130原始初始化，只下发基础配置，不读取、不判断任何错误码。
 * @note  该入口只保证尽量把配置写下去；即使底层写失败，也由后续运动循环继续写寄存器。
 */
static void Test_MotorTextRawInit(void)
{
    uint32_t motor_current = Test_MotorTextMotorCurrentNoCheck();

    if (g_measurement.debug_data.motor_speed == 0U) {
        g_measurement.debug_data.motor_speed = g_deviceParams.max_motor_speed;
    }

    MotorDriver_UpdateVelocityFromParams();
    (void)stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1U, (uint8_t)motor_current);
    stpr_enableDriver(&stepper);
    s_motor_driver.initialized = true;
    s_motor_driver.applied_velocity = velocity;
    (void)stpr_writeInt(&stepper, TMC5130_VMAX, (int32_t)velocity);
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief A/B/BE调试命令切换停稳后，直接重新初始化电机配置。
 * @note  命令切换检查已先等待驱动停稳；BE 退出时先初始化回位置模式，再恢复进入 BE 前的斜坡参数。
 *
 * @param phase_name 用于日志标识当前阶段的只读文字。
 * @param ramp_snapshot 命令执行前保存的 TMC5130 斜坡参数快照，用于退出或重启时恢复。
 */
static void Test_MotorTextRecoverDriverAfterCommandSwitch(const char *phase_name,
                                                          const MotorTextRampSnapshot *ramp_snapshot)
{
    const char *name = (phase_name != NULL) ? phase_name : "B/BE退出";
    uint32_t ret;

    g_measurement.debug_data.motor_state = 0U;
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;

    ret = MotorCtrl_Init();
    if (ret != NO_ERROR) {
        printf("%s\t退出恢复\t电机重新初始化失败\t返回=0x%08lX\r\n",
               name,
               (unsigned long)ret);
    } else {
        printf("%s\t退出恢复\t电机已重新初始化\t模式=位置模式\r\n", name);
        if ((ramp_snapshot != NULL) && (ramp_snapshot->valid != 0U)) {
            Test_MotorTextRestoreRampNoError(ramp_snapshot);
            printf("%s\t退出恢复\tBE加速度已恢复\tA1=%ld\tAMAX=%ld\tD1=%ld\tDMAX=%ld\r\n",
                   name,
                   (long)ramp_snapshot->a1,
                   (long)ramp_snapshot->amax,
                   (long)ramp_snapshot->d1,
                   (long)ramp_snapshot->dmax);
        }
    }

    g_measurement.debug_data.motor_state = 0U;
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief B/BE测试专用准备流程，只响应命令切换，不因任何错误码退出。
 *
 * @param name 用于串口测试日志标识当前测试项、阶段或通信目标的 NUL 结尾只读名称。
 * @return 1 表示测试运动前置状态已经准备完成；命令切换或无法安全准备时返回 0，但不写业务错误态。
 */
static uint8_t Test_MotorTextPrepareNoExit(const char *name)
{
    (void)name;
    if (Test_ShouldAbortForCommandSwitchNoError()) {
        return 0U;
    }
    Test_MotorTextRawInit();
    return 1U;
}
/**
 * @brief 将B/BE测试距离换算为TMC5130 ticks，复用现有卷筒模型但不做故障检查。
 * @note  只做参数和数学换算，不访问硬件，不上报错误状态。
 *
 * @param move_mm 本次相对运动距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param ticks_out 用于返回根据当前卷筒模型换算的 TMC5130 目标微步数。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；PARAM_RANGE_ERROR 表示参数超出允许范围；NO_ERROR 表示操作成功。
 */
static uint32_t Test_MotorTextDistanceToTicksNoCheck(float move_mm, int dir, int32_t *ticks_out)
{
    double Lcur_mm;
    double dL_mm;
    double Ltar_mm;
    int64_t ticks64;
    double local_circumference_mm;
    bool use_local_circ;

    if (ticks_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (move_mm < 0.0f) {
        return PARAM_RANGE_ERROR;
    }
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_RANGE_ERROR;
    }
    if (move_mm == 0.0f) {
        *ticks_out = 0;
        return NO_ERROR;
    }

    Lcur_mm = (double)g_measurement.debug_data.cable_length * 0.1;
    velocity = MotorDriver_ComputeUniformVelocityFromLength(Lcur_mm);
    s_motor_driver.applied_velocity = velocity;

    dL_mm = (double)move_mm;
    if (dir == MOTOR_DIRECTION_UP) {
        dL_mm = -dL_mm;
    }
    Ltar_mm = Lcur_mm + dL_mm;

    local_circumference_mm = MotorPosition_GetLocalCircumferenceFromParams();
    use_local_circ = ((g_deviceParams.position_count_mode == POSITION_COUNT_MODE_MOTOR) &&
                      (local_circumference_mm > 1e-6));
    if (use_local_circ) {
        ticks64 = MotorPosition_RoundToInt64((dL_mm / local_circumference_mm) *
                                             (double)MotorPosition_TapeTicksPerRev());
    } else {
        const double C0 = MotorPosition_TapeC0Mm();
        const double t = MotorPosition_TapeThicknessMm();
        double ncur;
        double ntar;
        if (C0 <= 0.0) {
            return PARAM_CONFIG_MISSING;
        }
        ncur = MotorPosition_TapeTurnsFromSignedLength(Lcur_mm, C0, t);
        ntar = MotorPosition_TapeTurnsFromSignedLength(Ltar_mm, C0, t);
        ticks64 = MotorPosition_RoundToInt64((ntar - ncur) *
                                             (double)MotorPosition_TapeTicksPerRev());
    }

    if (ticks64 > (int64_t)INT32_MAX) {
        ticks64 = (int64_t)INT32_MAX;
    }
    if (ticks64 < (int64_t)INT32_MIN) {
        ticks64 = (int64_t)INT32_MIN;
    }

    *ticks_out = (int32_t)ticks64;
    return NO_ERROR;
}

/**
 * @brief B/BE测试专用相对运动下发，直接写TMC5130寄存器，不返回、不判断错误码。
 * @note  每段下发前会尽量用 XACTUAL 对齐本地目标；读失败时才沿用上次累计值。
 *
 * @param move_mm 本次相对运动距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 */
static void Test_MotorTextMoveByNoCheck(float move_mm, int dir)
{
    int32_t ticks = 0;

    (void)Test_MotorTextDistanceToTicksNoCheck(move_mm, dir, &ticks);
    Test_MotorTextSyncRawTargetNoError();
    s_motor_text_raw_target += ticks;
    velocity = MotorDriver_ComputeUniformVelocityFromLength((double)g_measurement.debug_data.cable_length * 0.1);
    s_motor_driver.applied_velocity = velocity;

    stpr_enableDriver(&stepper);
    g_measurement.debug_data.motor_state = (dir == MOTOR_DIRECTION_UP) ? 1U : 2U;
    (void)stpr_writeInt(&stepper, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    (void)stpr_writeInt(&stepper, TMC5130_VMAX, (int32_t)velocity);
    (void)stpr_writeInt(&stepper, TMC5130_XTARGET, s_motor_text_raw_target);
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief B指令回零专用绝对运动下发，直接写目标位置，不读取、不判断错误状态。
 *
 * @param target 本次比较、运动或写入的目标值。该值是准备写入 TMC5130 XTARGET 的原始微步位置，不执行失步检测。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 */
static void Test_MotorTextMoveToNoCheck(int32_t target, int dir)
{
    velocity = MotorDriver_ComputeUniformVelocityFromLength((double)g_measurement.debug_data.cable_length * 0.1);
    s_motor_driver.applied_velocity = velocity;
    s_motor_text_raw_target = target;

    stpr_enableDriver(&stepper);
    g_measurement.debug_data.motor_state = (dir == MOTOR_DIRECTION_UP) ? 1U : 2U;
    (void)stpr_writeInt(&stepper, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    (void)stpr_writeInt(&stepper, TMC5130_VMAX, (int32_t)velocity);
    (void)stpr_writeInt(&stepper, TMC5130_XTARGET, target);
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief 限制 BE 加速度倍率，0 或非法值按 1 档处理。
 *
 * @param multiplier 倍率。
 * @return 返回限制后的 BE 加速度倍率；0 或非法值回退 1，超过上限时返回允许的最大倍率。
 */
static uint32_t Test_MotorTextClampMultiplier(uint32_t multiplier)
{
    if (multiplier == 0U) {
        return MOTOR_TEXT_ENCODER_MULTIPLIER_DEFAULT;
    }
    if (multiplier > MOTOR_TEXT_ENCODER_MULTIPLIER_MAX) {
        return MOTOR_TEXT_ENCODER_MULTIPLIER_MAX;
    }
    return multiplier;
}

/**
 * @brief 限制 BE 线速度，单位 0.01m/min；0 表示沿用当前速度配置。
 *
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return 返回限制后的 BE 线速度，单位 0.01 m/min；输入 0 保持为 0，非零值钳位到允许范围。
 */
static uint32_t Test_MotorTextClampSpeedX100(uint32_t speed_x100)
{
    if (speed_x100 == 0U) {
        return 0U;
    }
    if (speed_x100 < MOTOR_LINEAR_SPEED_MIN_X100) {
        return MOTOR_LINEAR_SPEED_MIN_X100;
    }
    if (speed_x100 > MOTOR_LINEAR_SPEED_MAX_X100) {
        return MOTOR_LINEAR_SPEED_MAX_X100;
    }
    return speed_x100;
}

/**
 * @brief 获取 BE 本段实际采用的线速度，单位 0.01m/min。
 *
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return 返回 BE 当前分段实际采用的线速度，单位 0.01 m/min，已包含测试倍率和安全上限处理。
 */
static uint32_t Test_MotorTextEffectiveSpeedX100(uint32_t speed_x100)
{
    uint32_t effective_speed = Test_MotorTextClampSpeedX100(speed_x100);

    if (effective_speed != 0U) {
        return effective_speed;
    }

    effective_speed = g_measurement.debug_data.motor_speed;
    if (effective_speed == 0U) {
        effective_speed = g_deviceParams.max_motor_speed;
    }
    effective_speed = Test_MotorTextClampSpeedX100(effective_speed);
    if (effective_speed == 0U) {
        effective_speed = MOTOR_LINEAR_SPEED_MIN_X100;
    }
    return effective_speed;
}

/**
 * @brief 将 BE 线速度临时写入 speed_x100，再复用既有 VMAX 换算函数。
 *
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return 返回由临时 speed_x100 换算的 TMC5130 VMAX；计算结果为 0 时至少返回 1。
 */
static uint32_t Test_MotorTextComputeVelocityForSpeed(uint32_t speed_x100)
{
    uint32_t old_speed = g_measurement.debug_data.motor_speed;
    uint32_t applied_speed = Test_MotorTextClampSpeedX100(speed_x100);
    uint32_t computed_velocity;

    if (applied_speed != 0U) {
        g_measurement.debug_data.motor_speed = applied_speed;
    }
    computed_velocity = MotorDriver_ComputeUniformVelocityFromLength((double)g_measurement.debug_data.cable_length * 0.1);
    g_measurement.debug_data.motor_speed = old_speed;

    if (computed_velocity == 0U) {
        return 1U;
    }
    return computed_velocity;
}

/**
 * @brief 按倍率放大加速度寄存器值，并限制在 TMC5130 允许范围内。
 *
 * @param base_value 基址数值。
 * @param multiplier 倍率。
 * @param fallback_value 兜底值数值。
 * @return 返回按倍率换算并钳位后的 TMC5130 斜坡寄存器值。
 */
static int32_t Test_MotorTextScaleRampValue(int32_t base_value,
                                            uint32_t multiplier,
                                            int32_t fallback_value)
{
    uint64_t scaled;
    uint32_t base;

    if (base_value > 0) {
        base = (uint32_t)base_value;
    } else if (fallback_value > 0) {
        base = (uint32_t)fallback_value;
    } else {
        base = 1U;
    }

    scaled = (uint64_t)base *
             (uint64_t)Test_MotorTextClampMultiplier(multiplier);
    if (scaled > (uint64_t)TMC5130_MAX_ACCELERATION) {
        return (int32_t)TMC5130_MAX_ACCELERATION;
    }
    return (int32_t)scaled;
}

/**
 * @brief BE测试进入前读取当前斜坡参数，作为加速度 1 档基准。
 * @note  读取失败时使用驱动初始化默认值，退出时仍按快照恢复。
 *
 * @param snapshot TMC5130 斜坡寄存器快照输出对象；成功读取后保存本次测试开始前的速度、加速度和斜坡配置。
 */
static void Test_MotorTextCaptureRampNoError(MotorTextRampSnapshot *snapshot)
{
    uint8_t ok = 1U;

    if (snapshot == NULL) {
        return;
    }

    snapshot->a1 = MOTOR_TEXT_ENCODER_DEFAULT_A1;
    snapshot->amax = MOTOR_TEXT_ENCODER_DEFAULT_AMAX;
    snapshot->d1 = MOTOR_TEXT_ENCODER_DEFAULT_D1;
    snapshot->dmax = MOTOR_TEXT_ENCODER_DEFAULT_DMAX;
    snapshot->valid = 1U;

    if (!stpr_tryReadInt(&stepper, TMC5130_A1, &snapshot->a1)) {
        ok = 0U;
        snapshot->a1 = MOTOR_TEXT_ENCODER_DEFAULT_A1;
    }
    if (!stpr_tryReadInt(&stepper, TMC5130_AMAX, &snapshot->amax)) {
        ok = 0U;
        snapshot->amax = MOTOR_TEXT_ENCODER_DEFAULT_AMAX;
    }
    if (!stpr_tryReadInt(&stepper, TMC5130_D1, &snapshot->d1)) {
        ok = 0U;
        snapshot->d1 = MOTOR_TEXT_ENCODER_DEFAULT_D1;
    }
    if (!stpr_tryReadInt(&stepper, TMC5130_DMAX, &snapshot->dmax)) {
        ok = 0U;
        snapshot->dmax = MOTOR_TEXT_ENCODER_DEFAULT_DMAX;
    }

    if (ok == 0U) {
        printf("BE编码器测试\t加速度基准读取失败，使用默认1档\r\n");
    }
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief BE测试临时写入加速度倍率，不修改设备参数。
 *
 * @param snapshot 只读 TMC5130 斜坡寄存器快照；用于临时应用测试参数后恢复进入前的速度、加速度和斜坡配置。
 * @param accel_multiplier BE 调试使用的加速度倍率，非法值会限制到允许档位。
 */
static void Test_MotorTextApplyRampNoError(const MotorTextRampSnapshot *snapshot,
                                           uint32_t accel_multiplier)
{
    int32_t a1;
    int32_t amax;
    int32_t d1;
    int32_t dmax;
    uint32_t multiplier = Test_MotorTextClampMultiplier(accel_multiplier);

    a1 = Test_MotorTextScaleRampValue((snapshot != NULL) ? snapshot->a1 : 0,
                                      multiplier,
                                      MOTOR_TEXT_ENCODER_DEFAULT_A1);
    amax = Test_MotorTextScaleRampValue((snapshot != NULL) ? snapshot->amax : 0,
                                        multiplier,
                                        MOTOR_TEXT_ENCODER_DEFAULT_AMAX);
    d1 = Test_MotorTextScaleRampValue((snapshot != NULL) ? snapshot->d1 : 0,
                                      multiplier,
                                      MOTOR_TEXT_ENCODER_DEFAULT_D1);
    dmax = Test_MotorTextScaleRampValue((snapshot != NULL) ? snapshot->dmax : 0,
                                        multiplier,
                                        MOTOR_TEXT_ENCODER_DEFAULT_DMAX);

    (void)stpr_writeInt(&stepper, TMC5130_A1, a1);
    (void)stpr_writeInt(&stepper, TMC5130_AMAX, amax);
    (void)stpr_writeInt(&stepper, TMC5130_D1, d1);
    (void)stpr_writeInt(&stepper, TMC5130_DMAX, dmax);
    printf("BE编码器测试\t加速度设置\t倍率=%lu\tA1=%ld\tAMAX=%ld\tD1=%ld\tDMAX=%ld\r\n",
           (unsigned long)multiplier,
           (long)a1,
           (long)amax,
           (long)d1,
           (long)dmax);
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief BE测试退出时恢复进入前斜坡参数。
 *
 * @param snapshot 只读 TMC5130 斜坡寄存器快照；用于临时应用测试参数后恢复进入前的速度、加速度和斜坡配置。
 */
static void Test_MotorTextRestoreRampNoError(const MotorTextRampSnapshot *snapshot)
{
    if ((snapshot == NULL) || (snapshot->valid == 0U)) {
        return;
    }

    (void)stpr_writeInt(&stepper, TMC5130_A1, snapshot->a1);
    (void)stpr_writeInt(&stepper, TMC5130_AMAX, snapshot->amax);
    (void)stpr_writeInt(&stepper, TMC5130_D1, snapshot->d1);
    (void)stpr_writeInt(&stepper, TMC5130_DMAX, snapshot->dmax);
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief BE超时或异常重启前重新初始化电机，并重新应用本次加速度倍率。
 * @note  初始化成功前只重试初始化，不改变固定编码器原点和目标；命令切换时返回 STATE_SWITCH。
 *
 * @param phase_name 用于日志标识当前阶段的只读文字。
 * @param ramp_snapshot 命令执行前保存的 TMC5130 斜坡参数快照，用于退出或重启时恢复。
 * @param accel_multiplier BE 调试使用的加速度倍率，非法值会限制到允许档位。
 * @return NO_ERROR 表示电机重新初始化且本次加速度倍率已经恢复；STATE_SWITCH 表示测试被新命令打断，其他值为驱动初始化或速度设置错误。
 */
static uint32_t Test_MotorTextReinitForRestartNoExit(const char *phase_name,
                                                     const MotorTextRampSnapshot *ramp_snapshot,
                                                     uint32_t accel_multiplier)
{
    const char *name = (phase_name != NULL) ? phase_name : "BE重启";
    uint32_t retry_count = 0U;

    while (1) {
        uint32_t ret;

        if (Test_ShouldAbortForCommandSwitchNoError()) {
            return STATE_SWITCH;
        }

        retry_count++;
        g_measurement.debug_data.motor_state = 0U;
        s_motor_driver.motion_command_active = false;
        s_motor_driver.motion_wait_active = false;

        ret = MotorCtrl_Init();
        if (ret == NO_ERROR) {
            Test_MotorTextApplyRampNoError(ramp_snapshot, accel_multiplier);
            printf("%s\t重启恢复\t电机已重新初始化\t重试=%lu\t加速度倍率=%lu\r\n",
                   name,
                   (unsigned long)retry_count,
                   (unsigned long)Test_MotorTextClampMultiplier(accel_multiplier));
            Test_MotorTextClearIgnoredError();
            return NO_ERROR;
        }

        printf("%s\t重启恢复\t电机初始化失败，继续重试\t重试=%lu\t返回=0x%08lX\r\n",
               name,
               (unsigned long)retry_count,
               (unsigned long)ret);
        Test_MotorTextClearIgnoredError();
        HAL_Delay(MOTOR_TEXT_RETRY_DELAY_MS);
    }
}

/**
 * @brief BE测试专用点动式连续运动下发。
 * @note  不设置近距离XTARGET，直接进入速度模式；到编码器目标后由上层立即换向。
 *
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 */
static void Test_MotorTextStartJogNoCheck(int dir, uint32_t speed_x100)
{
    velocity = Test_MotorTextComputeVelocityForSpeed(speed_x100);
    s_motor_driver.applied_velocity = velocity;

    stpr_enableDriver(&stepper);
    g_measurement.debug_data.motor_state = (dir == MOTOR_DIRECTION_UP) ? 1U : 2U;
    (void)stpr_writeInt(&stepper, TMC5130_VMAX, (int32_t)velocity);
    (void)stpr_writeInt(&stepper,
                        TMC5130_RAMPMODE,
                        (dir == MOTOR_DIRECTION_UP) ? TMC5130_MODE_VELNEG : TMC5130_MODE_VELPOS);
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief 用RAMPSTAT.VZERO判断电机是否停稳，不解析驱动故障位。
 * @note  known=0表示本次通信读失败，调用方按继续等待或重试处理。
 *
 * @param known 用于返回当前是否已取得可信的电机停止状态。
 * @return 1 表示 RAMPSTAT 读取成功且 VZERO 置位；寄存器读取失败或电机尚未停止时返回 0。
 */
static uint8_t Test_MotorTextIsStoppedNoError(uint8_t *known)
{
    int32_t rampstat = 0;

    if (known != NULL) {
        *known = 0U;
    }
    if (!stpr_tryReadInt(&stepper, TMC5130_RAMPSTAT, &rampstat)) {
        Test_MotorTextClearIgnoredError();
        return 0U;
    }

    if (known != NULL) {
        *known = 1U;
    }
    return ((rampstat & TMC5130_RS_VZERO) != 0) ? 1U : 0U;
}

/**
 * @brief B/BE测试专用停稳二次确认，避免换向瞬间 VZERO 误判。
 * @note  第一次读到 VZERO 后延时，再二次读取 VZERO；期间只响应命令切换。
 *
 * @param phase_name 用于日志标识当前阶段的只读文字。
 * @param known 用于返回重试后是否已确认可信的电机停止状态。
 * @return 1 表示两次停稳检查均确认 VZERO；状态未知、命令切换或二次确认失败时返回 0。
 */
static uint8_t Test_MotorTextConfirmStoppedNoError(const char *phase_name, uint8_t *known)
{
    uint8_t first_known = 0U;
    uint8_t second_known = 0U;

    if (known != NULL) {
        *known = 0U;
    }

    if (Test_MotorTextIsStoppedNoError(&first_known) == 0U) {
        if (known != NULL) {
            *known = first_known;
        }
        return 0U;
    }

    HAL_Delay(MOTOR_TEXT_STOP_CONFIRM_MS);
    if (Test_ShouldAbortForCommandSwitchNoError()) {
        return 0U;
    }

    if (Test_MotorTextIsStoppedNoError(&second_known) == 0U) {
        if (known != NULL) {
            *known = second_known;
        }
        return 0U;
    }

    (void)phase_name;
    if (known != NULL) {
        *known = 1U;
    }
    return 1U;
}
/**
 * @brief 等待电机停稳，只看RAMPSTAT.VZERO，不检测扭力/过热等错误。
 * @note  只在任务上下文调用；通信读失败会继续等待并周期打印。
 *
 * @param phase_name 用于日志标识当前阶段的只读文字。
 * @return 1 表示在时限内观察到 TMC5130 RAMPSTAT.VZERO；命令切换或等待超时返回 0。
 */
static uint8_t Test_WaitMotorStoppedNoErrorCheck(const char *phase_name)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_log_tick = 0U;
    const char *name = (phase_name != NULL) ? phase_name : "B/BE";

    while (1) {
        uint8_t known = 0U;
        uint32_t now;

        if (Test_ShouldAbortForCommandSwitchNoError()) {
            return 0U;
        }

        now = HAL_GetTick();
        if ((Test_MotorTextConfirmStoppedNoError(name, &known) != 0U) &&
            ((now - start_tick) >= MOTOR_TEXT_STOP_SETTLE_MS)) {
            Test_MotorTextSyncRawTargetNoError();
            g_measurement.debug_data.motor_state = 0U;
            Test_MotorTextClearIgnoredError();
            return 1U;
        }

        if ((known == 0U) &&
            ((last_log_tick == 0U) ||
             ((now - last_log_tick) >= MOTOR_TEXT_RETRY_LOG_INTERVAL_MS))) {
            printf("%s\t等待停稳\t读取RAMPSTAT失败，继续等待\r\n", name);
            last_log_tick = now;
        }

        Test_MotorTextClearIgnoredError();
        HAL_Delay(MOTOR_TEXT_STOP_POLL_MS);
    }
}

/**
 * @brief B指令单段运动执行器，不检测任何错误码，只负责下发运动并等待停转。
 * @note  只响应新串口命令切换；其它错误状态会被清掉，不参与流程判断。
 *
 * @param phase_name 用于日志标识当前阶段的只读文字。
 * @param loop_index B 指令当前循环序号；仅用于阶段日志和测试进度显示，不参与运动距离或目标位置计算。
 * @param move_to_zero 零点。
 * @param move_mm 本次相对运动距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @return 1 表示单段运动已下发并等待到停转；命令切换、启动失败或等待超时返回 0。
 */
static uint8_t Test_MotorTextRunMoveStageNoExit(const char *phase_name,
                                                uint32_t loop_index,
                                                uint8_t move_to_zero,
                                                float move_mm,
                                                int dir)
{
    if (Test_ShouldAbortForCommandSwitchNoError()) {
        return 0U;
    }

    if (move_to_zero != 0U) {
        Test_MotorTextMoveToNoCheck(0, dir);
    } else {
        Test_MotorTextMoveByNoCheck(move_mm, dir);
    }

    printf("[第%lu轮]\t%s\t开始\r\n",
           (unsigned long)(loop_index + 1U),
           phase_name);
    if (Test_WaitMotorStoppedNoErrorCheck(phase_name) == 0U) {
        return 0U;
    }
    printf("[第%lu轮]\t%s\t完成\r\n",
           (unsigned long)(loop_index + 1U),
           phase_name);
    Test_MotorTextClearIgnoredError();
    return 1U;
}
/**
 * @brief 读取测试使用的当前编码器计数值。
 * @return 返回测试口径下取反后的当前编码器累计计数。
 */
static int32_t Test_GetEncoderValue(void)
{
    update_sensor_height_from_encoder();
    return -g_encoder_count;
}

/**
 * @brief 判断编码器是否已经到达测试目标位置。
 *
 * @param current 当前编码器计数或位置值。
 * @param target 本次比较、运动或写入的目标值。该值是编码器目标计数，函数结合运动方向判断当前位置是否已经到达或越过目标。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @return 1 表示编码器已经到达测试目标位置；0 表示编码器尚未到达测试目标位置。
 */
static uint8_t Test_EncoderTargetReached(int32_t current, int32_t target, int dir)
{
    if (dir == MOTOR_DIRECTION_DOWN) {
        return (current >= target) ? 1U : 0U;
    }

    return (current <= target) ? 1U : 0U;
}

/**
 * @brief 把测试运动距离从毫米转换为编码器计数。
 *
 * @param distance_mm 距离。
 * @return 返回按编码轮周长四舍五入得到的计数；输入距离非正时返回 0，非零距离最小返回 1。
 */
static int32_t Test_EncoderDistanceMmToCount(float distance_mm)
{
    float encoder_count;

    if ((distance_mm <= 0.0f) || (g_deviceParams.encoder_wheel_circumference_mm == 0U)) {
        return 0;
    }

    encoder_count = (distance_mm * MOTOR_TEXT_ENCODER_MAX_ANGLE * 1000.0f) /
                    (float)g_deviceParams.encoder_wheel_circumference_mm;
    if (encoder_count < 1.0f) {
        return 1;
    }

    return (int32_t)(encoder_count + 0.5f);
}

/**
 * @brief 把编码器计数转换为测试显示使用的毫米距离。
 *
 * @param encoder_count 待按编码轮周长换算为距离的编码器累计计数。
 * @return 计算后的业务数值。
 */
static float Test_EncoderCountToDistanceMm(int32_t encoder_count)
{
    if (g_deviceParams.encoder_wheel_circumference_mm == 0U) {
        return 0.0f;
    }

    return ((float)encoder_count * (float)g_deviceParams.encoder_wheel_circumference_mm) /
           (MOTOR_TEXT_ENCODER_MAX_ANGLE * 1000.0f);
}

/**
 * @brief 按 BE 距离和线速度估算单段超时，超时只重启本阶段，不改变固定区间。
 *
 * @param distance_mm 距离。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return 返回按距离和线速度估算并限制后的单段超时时间，单位 ms。
 */
static uint32_t Test_MotorTextComputeLegTimeoutMs(float distance_mm, uint32_t speed_x100)
{
    double effective_speed_x100;
    double expected_ms;
    double timeout_ms;

    effective_speed_x100 = (double)Test_MotorTextEffectiveSpeedX100(speed_x100);
    if (effective_speed_x100 < 1.0) {
        effective_speed_x100 = 1.0;
    }

    expected_ms = ((double)distance_mm * 6000.0) / effective_speed_x100;
    timeout_ms = expected_ms * (double)MOTOR_TEXT_ENCODER_TIMEOUT_SCALE +
                 (double)MOTOR_TEXT_ENCODER_TIMEOUT_MARGIN_MS;

    if (timeout_ms < (double)MOTOR_TEXT_ENCODER_TIMEOUT_MIN_MS) {
        return MOTOR_TEXT_ENCODER_TIMEOUT_MIN_MS;
    }
    if (timeout_ms > (double)MOTOR_TEXT_ENCODER_TIMEOUT_MAX_MS) {
        return MOTOR_TEXT_ENCODER_TIMEOUT_MAX_MS;
    }
    return (uint32_t)(timeout_ms + 0.5);
}

/**
 * @brief BE运动过程中按周期读取传感器，避免只在停机点读取。
 *
 * @param phase_name 用于日志标识当前阶段的只读文字。
 * @param enable_sensor_comm 非零表示每个运动行程后附加一次传感器通信检查，0 表示只测试电机。
 * @param last_comm_tick 输入输出最近一次传感器通信检查节拍，单位 ms。
 */
static void Test_MotorTextSensorCommDuringRun(const char *phase_name,
                                              uint8_t enable_sensor_comm,
                                              uint32_t *last_comm_tick)
{
    uint32_t now;

    if ((enable_sensor_comm == 0U) || (last_comm_tick == NULL)) {
        return;
    }

    now = HAL_GetTick();
    if ((*last_comm_tick != 0U) &&
        ((now - *last_comm_tick) < MOTOR_TEXT_ENCODER_SENSOR_COMM_INTERVAL_MS)) {
        return;
    }

    Test_SensorCommCheckAndPrintOnly(phase_name);
    *last_comm_tick = HAL_GetTick();
    Test_MotorTextClearIgnoredError();
}

/**
 * @brief 以无保护点动方式运行到固定编码器目标，轮询命令切换和可选传感器通信；超时或提前停稳时重建驱动后继续重试。
 *
 * @param target_encoder 目标编码器。
 * @param origin_encoder 编码器。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @param enable_sensor_comm 非零表示每个运动行程后附加一次传感器通信检查，0 表示只测试电机。
 * @param ramp_snapshot 命令执行前保存的 TMC5130 斜坡参数快照，用于退出或重启时恢复。
 * @param accel_multiplier BE 调试使用的加速度倍率，非法值会限制到允许档位。
 * @param phase_name 标识本次编码器目标运动阶段的 NUL 结尾只读名称，用于到位、超调、超时和失败日志。
 * @return PARAM_RANGE_ERROR 表示参数超出允许范围；NO_ERROR 表示操作成功。
 */
static uint32_t Test_MoveUntilEncoderTarget(int32_t target_encoder,
                                            int32_t origin_encoder,
                                            int dir,
                                            uint32_t speed_x100,
                                            uint32_t timeout_ms,
                                            uint8_t enable_sensor_comm,
                                            const MotorTextRampSnapshot *ramp_snapshot,
                                            uint32_t accel_multiplier,
                                            const char *phase_name)
{
    const char *name = (phase_name != NULL) ? phase_name : "BE";
    uint32_t start_tick;
    uint32_t last_comm_tick = 0U;
    uint32_t restart_count = 0U;

    if (Test_ShouldAbortForCommandSwitchNoError()) {
        return STATE_SWITCH;
    }
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_RANGE_ERROR;
    }

    if (Test_EncoderTargetReached(Test_GetEncoderValue(), target_encoder, dir)) {
        return NO_ERROR;
    }

    Test_MotorTextStartJogNoCheck(dir, speed_x100);
    printf("%s\t长距离运动已下发\t模式=点动连续\t速度=%.2fm/min\t速度x100=%lu\t超时=%lums\r\n",
           name,
           (double)Test_MotorTextEffectiveSpeedX100(speed_x100) / 100.0,
           (unsigned long)Test_MotorTextEffectiveSpeedX100(speed_x100),
           (unsigned long)timeout_ms);
    start_tick = HAL_GetTick();

    while (1) {
        int32_t current_encoder;
        int32_t trigger_error;
        int32_t restart_error;
        uint8_t stopped_known = 0U;
        uint32_t now_tick;

        if (Test_ShouldAbortForCommandSwitchNoError()) {
            return STATE_SWITCH;
        }

        now_tick = HAL_GetTick();
        current_encoder = Test_GetEncoderValue();

        if (Test_EncoderTargetReached(current_encoder, target_encoder, dir)) {
            trigger_error = current_encoder - target_encoder;
            printf("%s\t编码器到位换向\t当前位置=%ld(%.2fmm)\t目标=%ld(%.2fmm)\t偏差=%.2fmm\r\n",
                   name,
                   (long)current_encoder,
                   Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
                   (long)target_encoder,
                   Test_EncoderCountToDistanceMm(target_encoder - origin_encoder),
                   Test_EncoderCountToDistanceMm(trigger_error));
            Test_MotorTextClearIgnoredError();
            return NO_ERROR;
        }

        Test_MotorTextSensorCommDuringRun(name, enable_sensor_comm, &last_comm_tick);

        if ((timeout_ms > 0U) && ((now_tick - start_tick) >= timeout_ms)) {
            restart_error = current_encoder - target_encoder;
            restart_count++;
            printf("%s\t到位超时重启\t重启=%lu\t当前位置=%ld(%.2fmm)\t固定原点=%ld(0.00mm)\t固定目标=%ld(%.2fmm)\t偏差=%.2fmm\r\n",
                   name,
                   (unsigned long)restart_count,
                   (long)current_encoder,
                   Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
                   (long)origin_encoder,
                   (long)target_encoder,
                   Test_EncoderCountToDistanceMm(target_encoder - origin_encoder),
                   Test_EncoderCountToDistanceMm(restart_error));
            if (Test_MotorTextReinitForRestartNoExit(name, ramp_snapshot, accel_multiplier) == STATE_SWITCH) {
                return STATE_SWITCH;
            }
            Test_MotorTextStartJogNoCheck(dir, speed_x100);
            start_tick = HAL_GetTick();
            last_comm_tick = 0U;
            continue;
        }

        if (((now_tick - start_tick) > MOTOR_TEXT_ENCODER_START_GRACE_MS) &&
            (Test_MotorTextConfirmStoppedNoError(name, &stopped_known) != 0U)) {
            restart_error = current_encoder - target_encoder;
            restart_count++;
            printf("%s\t提前停稳重启\t重启=%lu\t当前位置=%ld(%.2fmm)\t固定原点=%ld(0.00mm)\t固定目标=%ld(%.2fmm)\t偏差=%.2fmm\r\n",
                   name,
                   (unsigned long)restart_count,
                   (long)current_encoder,
                   Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
                   (long)origin_encoder,
                   (long)target_encoder,
                   Test_EncoderCountToDistanceMm(target_encoder - origin_encoder),
                   Test_EncoderCountToDistanceMm(restart_error));
            if (Test_MotorTextReinitForRestartNoExit(name, ramp_snapshot, accel_multiplier) == STATE_SWITCH) {
                return STATE_SWITCH;
            }
            Test_MotorTextStartJogNoCheck(dir, speed_x100);
            start_tick = HAL_GetTick();
            last_comm_tick = 0U;
            continue;
        }

        (void)stopped_known;
        Test_MotorTextClearIgnoredError();
        HAL_Delay(MOTOR_TEXT_ENCODER_POLL_MS);
    }
}

/**
 * @brief 以 4×32 微步为单次增量执行长时间上行测试，并周期打印位置参考与扭力采样。
 *
 * 启用步进驱动后，每轮向上移动 -4×32 微步，等待两秒，再依次打印循环序号、传感器位置、电机位置参考和三次扭力读数。
 *
 * @note 该维护测试最多循环 24000 次并真实驱动电机；命令切换会立即返回，当前提前返回路径不会执行末尾的驱动关闭。
 */
void motor_step_up_text(void) {
    int i = 0;
    int32_t ticks = 4 * 32;
    printf("motor STEP text start\n");
    stpr_enableDriver(&stepper);
    printf("start up\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{传感器位置}%.1f", i, (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t{扭力值}%d\r\n", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }
    stpr_disableDriver(&stepper);
    printf("motor text over\n");
}

/**
 * @brief 以 4×32 微步为单次增量执行长时间下行测试，并周期打印位置参考与扭力采样。
 *
 * 启用步进驱动后，每轮向下移动 4×32 微步，等待两秒，再依次打印循环序号、传感器位置、电机位置参考和三次扭力读数。
 *
 * @note 该维护测试最多循环 24000 次并真实驱动电机；命令切换会立即返回，当前提前返回路径不会执行末尾的驱动关闭。
 */
void motor_step_down_text(void) {
    int i = 0;
    int32_t ticks = 4 * 32;
    printf("motor STEP text start\n");
    stpr_enableDriver(&stepper);
    printf("start down\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{传感器位置}%.1f", i, (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t{扭力值}%d\r\n", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }
    printf("down over!\n");
    stpr_disableDriver(&stepper);
    printf("motor text over\n");
}

/**
 * @brief 依次按 4、8 和 40 个整步的细分脉冲执行往返耐久测试，并持续打印编码器与扭力数据。
 *
 * 每种步长先按正脉冲方向运行固定总行程，再按负脉冲方向返回；步长增大时相应减少循环次数，使三个阶段的累计脉冲量一致。
 * 每次动作前后检查命令切换，动作后等待两秒并分三次输出循环序号、编码器计数和当前扭力，全部阶段完成后关闭步进驱动。
 *
 * @note 该维护测试会真实、长时间驱动电机；每次移动后等待并连续采集编码器与扭力。收到命令切换时会立即返回，当前提前返回路径不会执行函数末尾的驱动关闭。
 */
void motor_step_text(void) {
    int i = 0;
    int32_t ticks = 4 * 32;
    printf("motor STEP text start\n");
    stpr_enableDriver(&stepper);

    printf("4步进测试\n");
    printf("start down\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{torque}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("down over!\n");
    printf("start up\n");
    for (i = 0; i < 24000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -4 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{torque}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("8步进测试\n");
    printf("start down\n");
    for (i = 0; i < 12000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 8 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{torque}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("down over!\n");
    printf("start up\n");
    for (i = 0; i < 12000; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -8 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{torque}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("40步进测试\n");
    printf("start down\n");
    for (i = 0; i < 2400; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = 40 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{torque}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    printf("down over!\n");
    printf("start up\n");
    for (i = 0; i < 2400; i++) {
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        ticks = -40 * 32;
        stpr_moveBy(&stepper, &ticks, velocity);
        HAL_Delay(2000);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t{encoder}%d\t{torque}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\t", weight_parament.current_weight);
        HAL_Delay(100);
        if (Test_ShouldAbortForCommandSwitch()) {
            return;
        }
        printf("%d\r\n", weight_parament.current_weight);
    }

    stpr_disableDriver(&stepper);
    printf("motor text over\n");
}

/**
 * @brief A指令测试专用：只下发停止寄存器，不判断扭力、编码器或驱动错误。
 * @note  只在任务上下文调用；用于串口低检测调试，退出时恢复进入前错误状态。
 */
void motor_text_manual_stop(void)
{
    MotorTextErrorSnapshot error_snapshot = Test_MotorTextCaptureErrorState();

    (void)stpr_stop(&stepper);
    (void)stpr_writeInt(&stepper, TMC5130_VMAX, 0);
    g_measurement.debug_data.motor_state = 0U;
    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    Test_MotorTextClearIgnoredError();
    printf("A指令\t停止命令已下发\r\n");
    Test_MotorTextExit(&error_snapshot);
}

/**
 * @brief A指令测试专用：按指定方向执行一段低检测运动。
 * @note  运动期间只响应命令切换；不读取扭力、编码器错误或全局错误退出。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 */
void motor_text_manual_once(float run_distance_mm, int dir)
{
    const char *phase_name;
    MotorTextErrorSnapshot error_snapshot = Test_MotorTextCaptureErrorState();

    if ((run_distance_mm <= 0.0f) ||
        ((dir != MOTOR_DIRECTION_UP) && (dir != MOTOR_DIRECTION_DOWN))) {
        printf("A指令\t参数异常\t距离=%.2fmm\t方向=%d\r\n", run_distance_mm, dir);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    phase_name = (dir == MOTOR_DIRECTION_UP) ? "A上行" : "A下行";
    s_motor_text_raw_target = 0;
    Test_MotorTextClearIgnoredError();
    if (Test_MotorTextPrepareNoExit("A") == 0U) {
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    s_motor_text_raw_target = 0;
    (void)stpr_writeInt(&stepper, TMC5130_XACTUAL, 0);
    (void)stpr_writeInt(&stepper, TMC5130_XTARGET, 0);
    Test_MotorTextMoveByNoCheck(run_distance_mm, dir);
    printf("%s\t运动已下发\t距离=%.2fmm\r\n", phase_name, run_distance_mm);

    if (Test_WaitMotorStoppedNoErrorCheck(phase_name) == 0U) {
        Test_MotorTextRecoverDriverAfterCommandSwitch(phase_name, NULL);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    s_motor_driver.motion_command_active = false;
    s_motor_driver.motion_wait_active = false;
    Test_MotorTextClearIgnoredError();
    printf("%s\t运动完成\r\n", phase_name);
    Test_MotorTextExit(&error_snapshot);
}
/**
 * @brief 读取当前有效位置源并返回 mm 快照，供 BJ 点动测试打印前后位置。
 * @note  只在串口调试任务上下文调用；刷新失败由正式运动 API 自行返回错误码。
 *
 * @return 返回刷新当前有效位置源后取得的 sensor_position 快照，单位 mm。
 */
static float Test_MotorJogSnapshotPositionMm(void)
{
    float pos_mm = 0.0f;

    MotorCtrl_RefreshPositionFromActiveSource();
    MotorCtrl_SnapshotSensorPositionMm(&pos_mm);
    return pos_mm;
}

/**
 * @brief BJ 指令测试专用：初始化电机后直接调用点动相对运动正式接口。
 * @note  该函数不绕过 MotorCtrl_JogMoveAndWait 内部检测，用于现场验证新长距离点动控制方案。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param dir 运动方向。必须使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN；函数据此换算符号、目标位置、速度模式或到位条件。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 */
void motor_jog_text(float run_distance_mm, int dir, uint32_t speed_x100)
{
    MotorTextErrorSnapshot error_snapshot = Test_MotorTextCaptureErrorState();
    float start_mm;
    float end_mm;
    uint32_t ret;

    if ((run_distance_mm <= 0.0f) || (!MotorDriver_IsDirValid(dir))) {
        printf("BJ点动测试\t参数异常\t距离=%.2fmm\t方向=%d\r\n",
               run_distance_mm,
               dir);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    Test_MotorTextClearIgnoredError();
    ret = MotorCtrl_Init();
    if (ret != NO_ERROR) {
        printf("BJ点动测试\t初始化失败\t返回=0x%08lX\r\n", (unsigned long)ret);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    start_mm = Test_MotorJogSnapshotPositionMm();
    printf("BJ点动测试\t开始\t模式=相对\t方向=%s\t距离=%.2fmm\t速度=%.2fm/min\t速度x100=%lu\t起点=%.3fmm\r\n",
           MotorCtrl_DirectionText(dir),
           run_distance_mm,
           (double)Test_MotorTextEffectiveSpeedX100(speed_x100) / 100.0,
           (unsigned long)Test_MotorTextEffectiveSpeedX100(speed_x100),
           start_mm);

    ret = MotorCtrl_JogMoveAndWait(run_distance_mm, dir, speed_x100);
    end_mm = Test_MotorJogSnapshotPositionMm();
    printf("BJ点动测试\t结束\t返回=0x%08lX\t起点=%.3fmm\t终点=%.3fmm\t变化=%.3fmm\r\n",
           (unsigned long)ret,
           start_mm,
           end_mm,
           end_mm - start_mm);

    Test_MotorTextClearIgnoredError();
    Test_MotorTextExit(&error_snapshot);
}

/**
 * @brief BJP 指令测试专用：初始化电机后直接调用点动绝对位置正式接口。
 * @note  用于验证目标位置、提前降速、越界保护和命令切换等正式 API 行为。
 *
 * @param target_mm 目标位置，单位 mm。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 */
void motor_jog_to_position_text(float target_mm, uint32_t speed_x100)
{
    MotorTextErrorSnapshot error_snapshot = Test_MotorTextCaptureErrorState();
    float start_mm;
    float end_mm;
    float delta_mm;
    uint32_t ret;

    Test_MotorTextClearIgnoredError();
    ret = MotorCtrl_Init();
    if (ret != NO_ERROR) {
        printf("BJP点动到位测试\t初始化失败\t返回=0x%08lX\r\n", (unsigned long)ret);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    start_mm = Test_MotorJogSnapshotPositionMm();
    delta_mm = target_mm - start_mm;
    printf("BJP点动到位测试\t开始\t目标=%.3fmm\t当前位置=%.3fmm\t剩余=%.3fmm\t速度=%.2fm/min\t速度x100=%lu\r\n",
           target_mm,
           start_mm,
           delta_mm,
           (double)Test_MotorTextEffectiveSpeedX100(speed_x100) / 100.0,
           (unsigned long)Test_MotorTextEffectiveSpeedX100(speed_x100));

    ret = MotorCtrl_JogMoveToPosition(target_mm, speed_x100);
    end_mm = Test_MotorJogSnapshotPositionMm();
    printf("BJP点动到位测试\t结束\t返回=0x%08lX\t目标=%.3fmm\t终点=%.3fmm\t误差=%.3fmm\r\n",
           (unsigned long)ret,
           target_mm,
           end_mm,
           end_mm - target_mm);

    Test_MotorTextClearIgnoredError();
    Test_MotorTextExit(&error_snapshot);
}
/**
 * @brief 按指定距离连续执行下行与回零往返电机测试，并可在每个行程后检查传感器通信。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param enable_sensor_comm 非零表示每个运动行程后附加一次传感器通信检查，0 表示只测试电机。
 */
void motor_text(float run_distance_mm, uint8_t enable_sensor_comm)
{
    uint32_t loop_cnt = 0U;
    MotorTextErrorSnapshot error_snapshot = Test_MotorTextCaptureErrorState();

    if (run_distance_mm <= 0.0f) {
        printf("B电机测试\t参数异常\t运行距离=%.2fmm\r\n", run_distance_mm);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    s_motor_text_raw_target = 0;
    Test_MotorTextClearIgnoredError();
    printf("B电机测试\t开始\t距离=%.2fmm\t传感器通信=%u\r\n",
           run_distance_mm,
           (unsigned int)enable_sensor_comm);

    if (Test_MotorTextPrepareNoExit("B") == 0U) {
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    s_motor_text_raw_target = 0;
    (void)stpr_writeInt(&stepper, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    (void)stpr_writeInt(&stepper, TMC5130_XACTUAL, 0);
    (void)stpr_writeInt(&stepper, TMC5130_XTARGET, 0);
    Test_MotorTextClearIgnoredError();
    while (1) {
        if (Test_ShouldAbortForCommandSwitchNoError()) {
            Test_MotorTextRecoverDriverAfterCommandSwitch("B电机测试", NULL);
            Test_MotorTextExit(&error_snapshot);
            return;
        }

        if (Test_MotorTextRunMoveStageNoExit("B下行", loop_cnt, 0U,
                                            run_distance_mm, MOTOR_DIRECTION_DOWN) == 0U) {
            Test_MotorTextRecoverDriverAfterCommandSwitch("B下行", NULL);
            Test_MotorTextExit(&error_snapshot);
            return;
        }

        if (enable_sensor_comm != 0U) {
            Test_SensorCommCheckAndPrintOnly("B下行后");
            Test_MotorTextClearIgnoredError();
        }

        if (Test_MotorTextRunMoveStageNoExit("B上行回零", loop_cnt, 1U,
                                            0.0f, MOTOR_DIRECTION_UP) == 0U) {
            Test_MotorTextRecoverDriverAfterCommandSwitch("B上行回零", NULL);
            Test_MotorTextExit(&error_snapshot);
            return;
        }

        if (enable_sensor_comm != 0U) {
            Test_SensorCommCheckAndPrintOnly("B上行后");
            Test_MotorTextClearIgnoredError();
        }

        loop_cnt++;
        printf("[第%lu轮]\tB电机测试\t往返完成\r\n", (unsigned long)loop_cnt);
    }
}

/**
 * @brief 以固定编码器原点和下行目标连续往返测试，支持运动中传感器通信、超时或提前停稳重启及命令切换恢复。
 *
 * @param run_distance_mm 单个下行行程的目标距离，单位 mm。
 * @param enable_sensor_comm 非零表示每个运动行程后附加一次传感器通信检查，0 表示只测试电机。
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @param accel_multiplier BE 调试使用的加速度倍率，非法值会限制到允许档位。
 */
void motor_text_encoder(float run_distance_mm,
                        uint8_t enable_sensor_comm,
                        uint32_t speed_x100,
                        uint32_t accel_multiplier)
{
    uint32_t loop_cnt = 0U;
    uint32_t ret;
    uint32_t leg_timeout_ms;
    uint32_t accel_mul = Test_MotorTextClampMultiplier(accel_multiplier);
    int32_t run_encoder_count;
    int32_t origin_encoder;
    int32_t current_encoder;
    int32_t up_origin_encoder;
    int32_t down_target_encoder;
    MotorTextRampSnapshot ramp_snapshot = {0};
    MotorTextErrorSnapshot error_snapshot = Test_MotorTextCaptureErrorState();

    if (run_distance_mm <= 0.0f) {
        printf("BE编码器测试\t参数异常\t运行距离=%.2fmm\r\n", run_distance_mm);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    run_encoder_count = Test_EncoderDistanceMmToCount(run_distance_mm);
    if (run_encoder_count <= 0) {
        printf("BE编码器测试\t参数异常\t编码轮周长=%lu\t运行距离=%.2fmm\r\n",
               (unsigned long)g_deviceParams.encoder_wheel_circumference_mm,
               run_distance_mm);
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    s_motor_text_raw_target = 0;
    Test_MotorTextClearIgnoredError();
    if (Test_MotorTextPrepareNoExit("BE") == 0U) {
        Test_MotorTextExit(&error_snapshot);
        return;
    }

    Test_MotorTextCaptureRampNoError(&ramp_snapshot);
    Test_MotorTextApplyRampNoError(&ramp_snapshot, accel_mul);
    leg_timeout_ms = Test_MotorTextComputeLegTimeoutMs(run_distance_mm, speed_x100);

    origin_encoder = Test_GetEncoderValue();
    down_target_encoder = origin_encoder + run_encoder_count;

    printf("BE编码器测试\t开始\t距离=%.2fmm\t原点=%ld(0.00mm)\t下行目标=%ld(%.2fmm)\t编码增量=%ld(%.2fmm)\t方式=连续换向\t速度=%.2fm/min\t速度x100=%lu\t加速度倍率=%lu\t单段超时=%lums\t运动中传感器通信=%u\r\n",
           run_distance_mm,
           (long)origin_encoder,
           (long)down_target_encoder,
           Test_EncoderCountToDistanceMm(down_target_encoder - origin_encoder),
           (long)run_encoder_count,
           Test_EncoderCountToDistanceMm(run_encoder_count),
           (double)Test_MotorTextEffectiveSpeedX100(speed_x100) / 100.0,
           (unsigned long)Test_MotorTextEffectiveSpeedX100(speed_x100),
           (unsigned long)accel_mul,
           (unsigned long)leg_timeout_ms,
           (unsigned int)enable_sensor_comm);

    while (1) {
        if (Test_ShouldAbortForCommandSwitchNoError()) {
            Test_MotorTextRecoverDriverAfterCommandSwitch("BE编码器测试", &ramp_snapshot);
            Test_MotorTextExit(&error_snapshot);
            return;
        }

        current_encoder = Test_GetEncoderValue();
        printf("[第%lu轮]\tBE下行\t开始\t当前=%ld(%.2fmm)\t固定原点=%ld(0.00mm)\t固定目标=%ld(%.2fmm)\r\n",
               (unsigned long)(loop_cnt + 1U),
               (long)current_encoder,
               Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
               (long)origin_encoder,
               (long)down_target_encoder,
               Test_EncoderCountToDistanceMm(down_target_encoder - origin_encoder));

        ret = Test_MoveUntilEncoderTarget(down_target_encoder,
                                          origin_encoder,
                                          MOTOR_DIRECTION_DOWN,
                                          speed_x100,
                                          leg_timeout_ms,
                                          enable_sensor_comm,
                                          &ramp_snapshot,
                                          accel_mul,
                                          "BE下行");
        if (ret == STATE_SWITCH) {
            Test_MotorTextRecoverDriverAfterCommandSwitch("BE编码器测试", &ramp_snapshot);
            Test_MotorTextExit(&error_snapshot);
            return;
        }
        if (ret != NO_ERROR) {
            printf("BE下行\t阶段异常，先初始化电机再按固定区间重试\t返回=0x%08lX\r\n", (unsigned long)ret);
            if (Test_MotorTextReinitForRestartNoExit("BE下行", &ramp_snapshot, accel_mul) == STATE_SWITCH) {
                Test_MotorTextRecoverDriverAfterCommandSwitch("BE编码器测试", &ramp_snapshot);
                Test_MotorTextExit(&error_snapshot);
                return;
            }
            continue;
        }

        up_origin_encoder = Test_GetEncoderValue();
        printf("[第%lu轮]\tBE下行\t到位\t当前位置=%ld(%.2fmm)\t固定原点=%ld(0.00mm)\t固定下行目标=%ld(%.2fmm)\t动作=立即上行\r\n",
               (unsigned long)(loop_cnt + 1U),
               (long)up_origin_encoder,
               Test_EncoderCountToDistanceMm(up_origin_encoder - origin_encoder),
               (long)origin_encoder,
               (long)down_target_encoder,
               Test_EncoderCountToDistanceMm(down_target_encoder - origin_encoder));

        if (Test_ShouldAbortForCommandSwitchNoError()) {
            Test_MotorTextRecoverDriverAfterCommandSwitch("BE编码器测试", &ramp_snapshot);
            Test_MotorTextExit(&error_snapshot);
            return;
        }

        current_encoder = up_origin_encoder;
        printf("[第%lu轮]\tBE上行\t开始\t当前=%ld(%.2fmm)\t固定原点=%ld(0.00mm)\t固定下行目标=%ld(%.2fmm)\r\n",
               (unsigned long)(loop_cnt + 1U),
               (long)current_encoder,
               Test_EncoderCountToDistanceMm(current_encoder - origin_encoder),
               (long)origin_encoder,
               (long)down_target_encoder,
               Test_EncoderCountToDistanceMm(down_target_encoder - origin_encoder));

        ret = Test_MoveUntilEncoderTarget(origin_encoder,
                                          origin_encoder,
                                          MOTOR_DIRECTION_UP,
                                          speed_x100,
                                          leg_timeout_ms,
                                          enable_sensor_comm,
                                          &ramp_snapshot,
                                          accel_mul,
                                          "BE上行");
        if (ret == STATE_SWITCH) {
            Test_MotorTextRecoverDriverAfterCommandSwitch("BE编码器测试", &ramp_snapshot);
            Test_MotorTextExit(&error_snapshot);
            return;
        }
        if (ret != NO_ERROR) {
            printf("BE上行\t阶段异常，先初始化电机再按固定区间重试\t返回=0x%08lX\r\n", (unsigned long)ret);
            if (Test_MotorTextReinitForRestartNoExit("BE上行", &ramp_snapshot, accel_mul) == STATE_SWITCH) {
                Test_MotorTextRecoverDriverAfterCommandSwitch("BE编码器测试", &ramp_snapshot);
                Test_MotorTextExit(&error_snapshot);
                return;
            }
            continue;
        }

        loop_cnt++;
        current_encoder = Test_GetEncoderValue();
        printf("[第%lu轮]\tBE编码器测试\t往返到位\t当前=%ld(%.2fmm)\t动作=立即下行\r\n",
               (unsigned long)loop_cnt,
               (long)current_encoder,
               Test_EncoderCountToDistanceMm(current_encoder - origin_encoder));
        Test_MotorTextClearIgnoredError();
    }
}
/**
 * @brief 判断 TMC5130 GSTAT 是否属于测试允许的状态。
 *
 * @param gstat TMC5130 GSTAT 寄存器原始值。
 * @return 1 表示 TMC5130 GSTAT 属于测试允许的状态；0 表示 TMC5130 GSTAT 不属于测试允许的状态。
 */
static uint8_t Test_TMC5130_IsValidGstat(uint32_t gstat)
{
    return ((gstat & ~0x07UL) == 0UL);
}

/**
 * @brief TMC5130 静态 SPI 通信测试。
 *
 * 不启动电机，只重复读取 GSTAT/DRV_STATUS/IOIN/IFCNT，用于判断静止状态下
 * SPI 是否仍有 0xFFFFFFFF、0x00FFFFFF、0x00000100 等非法读数。
 */
void Test_TMC5130_SPI_Static(void)
{
    const uint32_t loops = 2000U;
    const uint32_t delay_ms = 10U;
    uint32_t gstat_ok = 0U;
    uint32_t gstat_invalid = 0U;
    uint32_t gstat_read_fail = 0U;
    uint32_t drv_read_fail = 0U;
    uint32_t ioin_read_fail = 0U;
    uint32_t ifcnt_read_fail = 0U;
    uint32_t chopconf_zero = 0U;      /* CHOPCONF 为 0 的次数，用于判断配置丢失/24V 掉电。 */
    uint32_t chopconf_read_fail = 0U; /* CHOPCONF 读取失败次数，用于区分总线失败和配置清零。 */
    uint32_t xactual_read_fail = 0U;  /* XACTUAL 读取失败次数，用于定位运行期位置读数异常。 */
    int32_t gstat = 0;
    int32_t drvstatus = 0;
    int32_t ioin = 0;
    int32_t ifcnt = 0;
    int32_t chopconf = 0;
    int32_t xactual = 0;

    printf("TMC5130 SPI静态测试开始 | loops=%lu | delay=%lums\r\n",
           (unsigned long)loops,
           (unsigned long)delay_ms);
    printf("测试期间不下发运动命令，只读取寄存器；如静止也大量非法，优先检查CS/MISO/供电/复位。\r\n");

    for (uint32_t i = 1U; i <= loops; ++i) {
        if (Test_ShouldAbortForCommandSwitch()) {
            break;
        }

        if (!stpr_tryReadInt(&stepper, TMC5130_GSTAT, &gstat)) {
            gstat_read_fail++;
            printf("[TMC SPI %lu] GSTAT读取失败\r\n", (unsigned long)i);
        } else if (!Test_TMC5130_IsValidGstat((uint32_t)gstat)) {
            gstat_invalid++;
            printf("[TMC SPI %lu] GSTAT非法=0x%08lX invalid=0x%08lX\r\n",
                   (unsigned long)i,
                   (unsigned long)((uint32_t)gstat),
                   (unsigned long)(((uint32_t)gstat) & ~0x07UL));
        } else {
            gstat_ok++;
        }

        if (!stpr_tryReadInt(&stepper, TMC5130_DRVSTATUS, &drvstatus)) {
            drv_read_fail++;
        }
        if (!stpr_tryReadInt(&stepper, TMC5130_IOIN, &ioin)) {
            ioin_read_fail++;
        }
        if (!stpr_tryReadInt(&stepper, TMC5130_IFCNT, &ifcnt)) {
            ifcnt_read_fail++;
        }
        /* 静态测试同时读取 CHOPCONF，验证 SPI 正常时配置是否被 24V 掉电清零。 */
        if (!stpr_tryReadInt(&stepper, TMC5130_CHOPCONF, &chopconf)) {
            chopconf_read_fail++;
        } else if (chopconf == 0) {
            chopconf_zero++;
        }
        /* XACTUAL 单独统计失败次数，便于和运行期“疑似 SPI 全 0”日志对照。 */
        if (!stpr_tryReadInt(&stepper, TMC5130_XACTUAL, &xactual)) {
            xactual_read_fail++;
        }

        if ((i == 1U) || ((i % 100U) == 0U)) {
            printf("[TMC SPI %lu] GSTAT=0x%08lX DRV=0x%08lX IOIN=0x%08lX IFCNT=%ld CHOP=0x%08lX XACTUAL=%ld | ok=%lu invalid=%lu fail=%lu cfg0=%lu\r\n",
                   (unsigned long)i,
                   (unsigned long)((uint32_t)gstat),
                   (unsigned long)((uint32_t)drvstatus),
                   (unsigned long)((uint32_t)ioin),
                   (long)ifcnt,
                   (unsigned long)((uint32_t)chopconf),
                   (long)xactual,
                   (unsigned long)gstat_ok,
                   (unsigned long)gstat_invalid,
                   (unsigned long)gstat_read_fail,
                   (unsigned long)chopconf_zero);
        }

        HAL_Delay(delay_ms);
    }

    printf("TMC5130 SPI静态测试结束 | GSTAT ok=%lu invalid=%lu read_fail=%lu | DRV_fail=%lu IOIN_fail=%lu IFCNT_fail=%lu CHOP_zero=%lu CHOP_fail=%lu XACTUAL_fail=%lu\r\n",
           (unsigned long)gstat_ok,
           (unsigned long)gstat_invalid,
           (unsigned long)gstat_read_fail,
           (unsigned long)drv_read_fail,
           (unsigned long)ioin_read_fail,
           (unsigned long)ifcnt_read_fail,
           (unsigned long)chopconf_zero,
           (unsigned long)chopconf_read_fail,
           (unsigned long)xactual_read_fail);
}
