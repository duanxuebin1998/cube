/*
 * test.c
 * 测试函数，用来临时测试某些功能
 *  Created on: Jul 22, 2025
 *      Author: Duan Xuebin
 */

#include <ltd_sensor_communication.h>
#include "test.h"
#include "measure.h"
#include "motor_ctrl.h"
#include "motor_ctrl_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include "system_parameter.h"
#include "measure_tank_height.h"
#include "measure_zero.h"
#include "measure_oilLevel.h"
#include "wireless_pairing.h"
#include "error_log.h"
#include "weight.h"
#include "system_parameter.h"
#include "sensor.h"
#include <mb85rs2m.h>
#include "my_crc.h"
#include "ad5421.h"
#include "encoder.h"
#include <stddef.h>
#define MOTOR_TEXT_ENCODER_MAX_ANGLE             4096.0f
#define MOTOR_TEXT_ENCODER_POLL_MS               10U
#define MOTOR_TEXT_ENCODER_START_GRACE_MS        500U
#define MOTOR_TEXT_ENCODER_SENSOR_COMM_INTERVAL_MS 1000U
#define MOTOR_TEXT_ENCODER_TIMEOUT_MARGIN_MS     5000U
#define MOTOR_TEXT_ENCODER_TIMEOUT_MIN_MS        10000U
#define MOTOR_TEXT_ENCODER_TIMEOUT_MAX_MS        120000U
#define MOTOR_TEXT_ENCODER_TIMEOUT_SCALE         4U
#define MOTOR_TEXT_ENCODER_MULTIPLIER_DEFAULT    1U
#define MOTOR_TEXT_ENCODER_MULTIPLIER_MAX        20U
#define MOTOR_TEXT_ENCODER_DEFAULT_A1            (10 * 32)
#define MOTOR_TEXT_ENCODER_DEFAULT_AMAX          (20 * 32)
#define MOTOR_TEXT_ENCODER_DEFAULT_D1            (10 * 32)
#define MOTOR_TEXT_ENCODER_DEFAULT_DMAX          (20 * 32)
#define MOTOR_TEXT_RETRY_DELAY_MS                200U
#define MOTOR_TEXT_STOP_POLL_MS                  20U
#define MOTOR_TEXT_RETRY_LOG_INTERVAL_MS         1000U
#define MOTOR_TEXT_STOP_SETTLE_MS                50U
#define MOTOR_TEXT_STOP_CONFIRM_MS               200U
#define COMM_TEST_WIRELESS_HOST_ADDR              1U
#define COMM_TEST_WIRELESS_SLAVE_ADDR             2U
typedef struct {
    DeviceState device_state;
    uint32_t error_code;
} MotorTextErrorSnapshot;

typedef struct {
    int32_t a1;
    int32_t amax;
    int32_t d1;
    int32_t dmax;
    uint8_t valid;
} MotorTextRampSnapshot;

static void Test_SensorCommCheckAndPrintOnly(const char *tag);
static void Test_MotorTextRestoreRampNoError(const MotorTextRampSnapshot *snapshot);
static uint32_t Test_EnsureMotorPositionSourceBeforeFollow(const char *follow_name, uint8_t *switched_to_motor);
static void Test_RunMeasureZeroOnce(void);
static void Test_RunMeasureAndFollowOilLevelOnce(void);


/**
 * @brief 保存进入 B/BE 诊断前的业务错误状态。
 * @note  只在串口 B/BE 测试上下文调用，避免诊断过程清错影响正常程序。
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
 */
static void Test_MotorTextRestoreErrorState(const MotorTextErrorSnapshot *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    g_measurement.device_status.device_state = snapshot->device_state;
    g_measurement.device_status.error_code = snapshot->error_code;
}

static int32_t s_motor_text_raw_target = 0;
static uint8_t Test_ShouldAbortForCommandSwitch(void)
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
 */
static void Test_MotorTextExit(const MotorTextErrorSnapshot *error_snapshot)
{
    Test_MotorTextRestoreErrorState(error_snapshot);
}

/**
 * @brief A/B/BE测试专用命令切换检查，只停止并退出当前测试，不把错误码带给业务状态机。
 * @note  只在任务上下文调用；不会解析称重、电机驱动故障或其它业务错误。
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
        return PARAM_ADDRESS_OVERFLOW;
    }
    if (move_mm < 0.0f) {
        return PARAM_ERROR;
    }
    if (!MotorDriver_IsDirValid(dir)) {
        return PARAM_ERROR;
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
            return PARAM_ERROR;
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
 * @brief 等待电机停稳，只看RAMPSTAT.VZERO，不检测称重/过热等错误。
 * @note  只在任务上下文调用；通信读失败会继续等待并周期打印。
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
 * @brief  记录单项通信测试结果
 * @note   仅用于手动测试日志汇总，不改变业务错误状态。
 */
static uint8_t Test_CommRecordResult(const char *name, uint32_t ret, uint32_t *ok_count, uint32_t *fail_count)
{
    if (ret == NO_ERROR) {
        (*ok_count)++;
        printf("[正常]\t%s\r\n", name);
        return 1U;
    }

    (*fail_count)++;
    printf("[异常]\t%s\t错误码=0x%08lX\r\n", name, (unsigned long)ret);
    return 0U;
}

/**
 * @brief  检查手动通信测试是否需要中止
 * @note   串口调试过程中收到新的有效命令时退出，避免测试函数长时间占用设备。
 */
static uint8_t Test_CommShouldStop(uint32_t *fail_count)
{
    if (Test_ShouldAbortForCommandSwitch()) {
        (*fail_count)++;
        printf("通信测试被命令切换打断\r\n");
        return 1U;
    }

    return 0U;
}

static int32_t Test_GetEncoderValue(void)
{
    update_sensor_height_from_encoder();
    return -g_encoder_count;
}

static uint8_t Test_EncoderTargetReached(int32_t current, int32_t target, int dir)
{
    if (dir == MOTOR_DIRECTION_DOWN) {
        return (current >= target) ? 1U : 0U;
    }

    return (current <= target) ? 1U : 0U;
}

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
        return PARAM_ERROR;
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

static uint8_t Test_ProcessCommandSwitchRequested(void)
{
    if (!HasEffectiveCommandSwitchRequest()) {
        return 0;
    }

    printf("检测到命令切换请求，停止当前串口命令\r\n");
    return 1;
}


/**
 * @brief 记录串口调试命令失败告警。
 * @note 串口调试命令只做现场提示，不在这里设置最终错误状态。
 */
static void Test_ProcessCommandWarnFailure(const char *operation, uint32_t error_code)
{
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    // 错误	阶段：错误报警	模块：通信	操作：operation	原因：ErrorLog_GetReasonByCode(error_code)	处理：仅记录
    ErrorLog_Warn(ERROR_LOG_MODULE_COMM,
                  operation,
                  ErrorLog_GetReasonByCode(error_code),
                  "仅记录");
}


typedef struct {
    DeviceState saved_state;
    uint8_t active;
} TestCommandDebugDisplaySnapshot;

/**
 * @brief 保存串口调试前的显示状态，并临时切到调试模式。
 * @note 只在任务上下文调用；不清错误码、不执行硬件动作。
 */
static void Test_EnterDebugDisplayState(TestCommandDebugDisplaySnapshot *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (snapshot->active == 0U) {
        snapshot->saved_state = g_measurement.device_status.device_state;
        snapshot->active = 1U;
    }
    g_measurement.device_status.device_state = STATE_DEBUG_MODE;
}


/**
 * @brief 恢复串口调试前的显示状态。
 * @note 仅当当前仍停留在调试模式时恢复，避免覆盖故障态或其他业务态。
 */
static void Test_RestoreDebugDisplayState(TestCommandDebugDisplaySnapshot *snapshot)
{
    if ((snapshot == NULL) || (snapshot->active == 0U)) {
        return;
    }

    if (g_measurement.device_status.device_state == STATE_DEBUG_MODE) {
        g_measurement.device_status.device_state = snapshot->saved_state;
    }
    snapshot->active = 0U;
}


#define TEST_COMMAND_BE_SPEED_DEFAULT      0U
#define TEST_COMMAND_BE_MULTIPLIER_DEFAULT 1U
#define TEST_COMMAND_BE_MULTIPLIER_MAX     20U

typedef struct {
    uint32_t speed_x100;
    uint32_t accel_multiplier;
    uint8_t enable_sensor_comm;
} TestCommandBeOptions;

/**
 * @brief 限制 BE 调试倍率；0 或负数按 1 档，过大按上限处理。
 */
static uint32_t TestCommand_ClampBeMultiplier(long value)
{
    if (value <= 0L) {
        return TEST_COMMAND_BE_MULTIPLIER_DEFAULT;
    }
    if (value > (long)TEST_COMMAND_BE_MULTIPLIER_MAX) {
        return TEST_COMMAND_BE_MULTIPLIER_MAX;
    }
    return (uint32_t)value;
}

/**
 * @brief 将 BE 命令速度从 m/min 转为 0.01m/min；0 表示沿用当前速度配置。
 */
static uint32_t TestCommand_ClampBeSpeedMMin(double value)
{
    double speed_x100;

    if (!(value > 0.0)) {
        return TEST_COMMAND_BE_SPEED_DEFAULT;
    }

    speed_x100 = value * 100.0;
    if (speed_x100 < (double)MOTOR_LINEAR_SPEED_MIN_X100) {
        return MOTOR_LINEAR_SPEED_MIN_X100;
    }
    if (speed_x100 > (double)MOTOR_LINEAR_SPEED_MAX_X100) {
        return MOTOR_LINEAR_SPEED_MAX_X100;
    }
    return (uint32_t)(speed_x100 + 0.5);
}

/**
 * @brief 解析 BJ/BJP 点动测试的可选速度，单位 m/min。
 * @note  找不到逗号或速度非法时返回 0，表示沿用当前默认速度配置。
 */
static uint32_t TestCommand_ParseJogSpeedX100(const char *arg_tail)
{
    const char *comma;
    char *endptr;
    double speed_m_min;

    if (arg_tail == NULL) {
        return TEST_COMMAND_BE_SPEED_DEFAULT;
    }

    comma = arg_tail;
    while ((*comma != '\0') && (*comma != ',')) {
        comma++;
    }
    if (*comma != ',') {
        return TEST_COMMAND_BE_SPEED_DEFAULT;
    }

    comma++;
    speed_m_min = strtod(comma, &endptr);
    if (endptr == comma) {
        return TEST_COMMAND_BE_SPEED_DEFAULT;
    }
    return TestCommand_ClampBeSpeedMMin(speed_m_min);
}

/**
 * @brief 处理 BJ/BJP 点动测试命令。
 * @return 已识别并处理返回 1，非 BJ 命令返回 0。
 */
static uint8_t TestCommand_HandleJogMotorTest(const uint8_t *command)
{
    const char *arg;
    char *endptr;
    double value;
    uint32_t speed_x100;
    int dir;

    if ((command == NULL) || (command[0] != 'B') || (command[1] != 'J')) {
        return 0U;
    }

    if (command[2] == 'P') {
        arg = (const char *)&command[3];
        value = strtod(arg, &endptr);
        if (endptr == arg) {
            printf("BJP点动到位测试\t目标参数无效\t命令=%s\r\n", (const char *)command);
            return 1U;
        }
        speed_x100 = TestCommand_ParseJogSpeedX100(endptr);
        printf("串口BJ指令\t命令参数\t模式=BJP绝对位置\t目标=%.3fmm\t速度=%.2fm/min\t速度x100=%lu\r\n",
               value,
               (double)speed_x100 / 100.0,
               (unsigned long)speed_x100);
        motor_jog_to_position_text((float)value, speed_x100);
        return 1U;
    }

    if ((command[2] != '+') && (command[2] != '-')) {
        printf("BJ点动测试\t用法：BJ+<mm>[,<速度m/min>]，BJ-<mm>[,<速度m/min>]，BJP<目标mm>[,<速度m/min>]\r\n");
        return 1U;
    }

    dir = (command[2] == '+') ? MOTOR_DIRECTION_UP : MOTOR_DIRECTION_DOWN;
    arg = (const char *)&command[3];
    value = strtod(arg, &endptr);
    if (endptr == arg) {
        printf("BJ点动测试\t距离参数无效\t命令=%s\r\n", (const char *)command);
        return 1U;
    }
    if (value < 0.0) {
        value = -value;
    }
    if (!(value > 0.0)) {
        printf("BJ点动测试\t距离参数无效\t距离=%.3fmm\r\n", value);
        return 1U;
    }

    speed_x100 = TestCommand_ParseJogSpeedX100(endptr);
    printf("串口BJ指令\t命令参数\t模式=BJ相对\t方向=%s\t距离=%.3fmm\t速度=%.2fm/min\t速度x100=%lu\r\n",
           MotorCtrl_DirectionText(dir),
           value,
           (double)speed_x100 / 100.0,
           (unsigned long)speed_x100);
    motor_jog_text((float)value, dir, speed_x100);
    return 1U;
}
/**
 * @brief 解析 BE 可选参数，兼容旧的 BE100S 和 BE100,1 传感器通信写法。
 * @note  新格式：BE距离,速度m/min,加速度倍率,S 或 BE距离,速度m/min,加速度倍率,1。
 */
static void TestCommand_ParseBeOptions(const char *arg, TestCommandBeOptions *options)
{
    const char *p;
    double values[4] = {0.0, 0.0, 0.0, 0.0};
    uint32_t value_count = 0U;

    if (options == NULL) {
        return;
    }

    options->speed_x100 = TEST_COMMAND_BE_SPEED_DEFAULT;
    options->accel_multiplier = TEST_COMMAND_BE_MULTIPLIER_DEFAULT;
    options->enable_sensor_comm = 0U;

    if (arg == NULL) {
        return;
    }

    p = arg;
    while (*p != '\0') {
        if (*p == 'S') {
            options->enable_sensor_comm = 1U;
            p++;
            continue;
        }
        if (*p == ',') {
            char *endptr;
            double value;

            p++;
            if (*p == 'S') {
                options->enable_sensor_comm = 1U;
                p++;
                continue;
            }
            value = strtod(p, &endptr);
            if (endptr == p) {
                p++;
                continue;
            }
            if (value_count < 4U) {
                values[value_count] = value;
                value_count++;
            }
            p = endptr;
            continue;
        }
        p++;
    }

    if ((value_count == 1U) && (values[0] == 1.0)) {
        options->enable_sensor_comm = 1U;
        return;
    }

    if (value_count >= 1U) {
        options->speed_x100 = TestCommand_ClampBeSpeedMMin(values[0]);
    }
    if (value_count >= 2U) {
        options->accel_multiplier = TestCommand_ClampBeMultiplier((long)values[1]);
    }
    if ((value_count >= 3U) && ((long)values[2] == 1L)) {
        options->enable_sensor_comm = 1U;
    }
}

static uint32_t Test_EnsureMotorPositionSourceBeforeFollow(const char *follow_name, uint8_t *switched_to_motor)
{
    uint32_t ret;

    if (switched_to_motor != NULL) {
        *switched_to_motor = 0U;
    }

    if (MotorCtrl_IsPositionSourceMotor()) {
        return NO_ERROR;
    }

    if (g_deviceParams.position_source_auto_switch != POSITION_SOURCE_AUTO_SWITCH_ENABLE) {
        printf("%s\tposition source auto switch disabled\r\n", follow_name);
        return NO_ERROR;
    }

    printf("%s\tswitch position source to motor count\r\n", follow_name);
    ret = MotorCtrl_SwitchPositionSourceToMotor();
    if (ret != NO_ERROR) {
        printf("%s\tswitch motor count failed:0x%08lX\r\n", follow_name, (unsigned long)ret);
        return ret;
    }

    if (switched_to_motor != NULL) {
        *switched_to_motor = 1U;
    }

    return NO_ERROR;
}

static void Test_RunMeasureZeroOnce(void)
{
    uint32_t ret;

    (void)MeasureStart();
    g_measurement.device_status.device_state = STATE_BACKZEROING;
    ret = (uint32_t)SearchZero();
    SET_ERROR(ret);
    g_measurement.device_status.device_state = STATE_STANDBY;
}

static void Test_RunMeasureAndFollowOilLevelOnce(void)
{
    uint32_t ret;

    (void)MeasureStart();
    g_measurement.device_status.device_state = STATE_FINDOIL;
    if ((g_measurement.device_status.zero_point_status == 1) &&
        (g_deviceParams.error_auto_back_zero == 1)) {
        printf("Oil measure\tback zero required\r\n");
        ret = (uint32_t)SearchZero();
        SET_ERROR(ret);
        printf("Oil measure\tback zero done\r\n");
    }

    ret = SearchOilLevel();
    SET_ERROR(ret);

    if (!MotorCtrl_IsPositionSourceMotor()) {
        uint8_t switched_to_motor = 0U;
        ret = Test_EnsureMotorPositionSourceBeforeFollow("Oil follow", &switched_to_motor);
        SET_ERROR(ret);
        if (switched_to_motor != 0U) {
            g_measurement.device_status.device_state = STATE_FINDOIL;
            ret = SearchOilLevel();
            SET_ERROR(ret);
        }
    }

    g_measurement.device_status.device_state = STATE_FLOWOIL;
    ret = FollowOilLevel();
    SET_ERROR(ret);
}

/**
 * @brief 处理从 measure.c 拆出的串口测试命令。
 * @return 已识别并处理返回 1，非测试命令返回 0。
 */
uint8_t Test_ProcessSerialCommand(uint8_t *command)
{
    uint32_t ret = NO_ERROR;
    TestCommandDebugDisplaySnapshot debug_display = { STATE_STANDBY, 0U };

    if ((command == NULL) || (command[0] == '\0')) {
        return 0U;
    }

    if ((command[0] == 'S') && (command[1] == 'C') && (command[2] == '\0')) {
        /* 通信测试不依赖电机初始化，放在 MeasureStart 前便于排查传感器和无线链路。 */
        printf("串口调试\t执行传感器与无线通信综合测试指令\r\n");
        Test_EnterDebugDisplayState(&debug_display);
        SensorWireless_CommTest();
        Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
    }

    if ((command[0] == 'S') && (command[1] == 'P')) {
        if (command[2] == 'S' && command[3] == '\0') {
            (void)WirelessPairing_DebugScan();
            return 1U;
        }
        if (command[2] == 'R' && command[3] == '\0') {
            (void)WirelessPairing_RunByRssi();
            return 1U;
        }
        if (command[2] == 'N' && command[3] == '=') {
            (void)WirelessPairing_RunByName((const char *)&command[4]);
            return 1U;
        }
        if (command[2] == 'C' && command[3] == '\0') {
            /* SPC 只查询当前连接，不触发扫描或默认连接保存，便于现场确认已连接从机。 */
            (void)WirelessPairing_PrintConnectionStatus();
            return 1U;
        }
        printf("SP调试命令\t用法：SPS=扫描，SPR=按RSSI匹配，SPN=<名称>=按蓝牙名称匹配，SPC=读取连接状态\r\n");
        return 1U;
    }
    if (command[0] == 'B') {
        if (TestCommand_HandleJogMotorTest(command)) {
            return 1U;
        }

        const uint8_t use_encoder_count = (command[1] == 'E') ? 1U : 0U;
        const char *arg = (const char *)&command[use_encoder_count ? 2U : 1U];
        int value = atoi(arg);
        uint8_t enable_sensor_comm = 0U;
        TestCommandBeOptions be_options;

        TestCommand_ParseBeOptions(arg, &be_options);

        /* B100: motor model distance in mm; BE100: encoder-based distance in mm. */
        for (uint32_t i = use_encoder_count ? 2U : 1U; command[i] != '\0'; ++i) {
            if ((command[i] == 'S') ||
                ((command[i] == ',') && (command[i + 1U] == '1'))) {
                enable_sensor_comm = 1U;
                break;
            }
        }

        if (value <= 0) {
            if (use_encoder_count) {
                printf("BE编码器测试\t距离参数无效\t距离=%dmm\r\n", value);
            } else {
                printf("B电机测试\t距离参数无效\t距离=%dmm\r\n", value);
            }
            return 1U;
        }

        if (use_encoder_count) {
            printf("串口B指令\t命令参数\t模式=BE编码器\t距离=%dmm\t速度=%.2fm/min\t速度x100=%lu\t加速度倍率=%lu\t运动中传感器通信=%u\r\n",
                   value,
                   (double)be_options.speed_x100 / 100.0,
                   (unsigned long)be_options.speed_x100,
                   (unsigned long)be_options.accel_multiplier,
                   (unsigned int)be_options.enable_sensor_comm);
            motor_text_encoder((float)value,
                               be_options.enable_sensor_comm,
                               be_options.speed_x100,
                               be_options.accel_multiplier);
            /* BE 正常设计为循环测试；如果返回，通常是命令切换或参数异常。 */
            printf("串口B指令\tBE已返回\t距离=%dmm\t待执行命令=%lu\t当前命令=%lu\t新串口命令=%u\r\n",
                   value,
                   (unsigned long)g_deviceParams.command,
                   (unsigned long)g_measurement.device_status.current_command,
                   (unsigned int)new_command_ready);
        } else {
            printf("串口B指令\t命令参数\t模式=B电机\t距离=%dmm\t传感器通信=%u\r\n",
                   value,
                   (unsigned int)enable_sensor_comm);
            motor_text((float)value, enable_sensor_comm);
        }
        return 1U;
    }
    /* A 指令走与 B/BE 一致的低检测电机路径；其它调试/恢复动作仍先执行 MeasureStart。 */
    if (command[0] != 'A') {
        ret = (uint32_t)MeasureStart();
        if (ret != NO_ERROR) {
            printf("串口命令\t启动失败\t电机初始化错误码=0x%08lX\\r\\n", (unsigned long)ret);
            return 1U;
        }
    }
    if (command[0] == 'A') {
        Test_EnterDebugDisplayState(&debug_display);
        if (command[1] == '0') {
            motor_text_manual_stop();
        } else if (command[1] == '+') {
            int mm = atoi((char*) &command[2]);
            printf("串口上行\t距离=%dmm\\r\\n", mm);
            motor_text_manual_once((float)mm, MOTOR_DIRECTION_UP);
        } else if (command[1] == '-') {
            int mm = atoi((char*) &command[2]);
            printf("串口下行\t距离=%dmm\\r\\n", mm);
            motor_text_manual_once((float)mm, MOTOR_DIRECTION_DOWN);
        }
        Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
    }

    if (command[0] == 'C') {
        printf("串口调试\t电机4步进分辨率测试开始\r\n");
        Test_EnterDebugDisplayState(&debug_display);
        motor_step_text();
        Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
    }
    if (command[0] == 'D') {
        printf("串口调试\t电机4步进下行触底测试开始\r\n");
        Test_EnterDebugDisplayState(&debug_display);
        motor_step_down_text();
        Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
    }
    if (command[0] == 'E') {
        printf("串口调试\t电机4步进上行碰零点测试开始\r\n");
        Test_EnterDebugDisplayState(&debug_display);
        motor_step_up_text();
        Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
    }
    if (command[0] == 'F') {
        printf("串口调试\t罐底测量重复性测试开始\r\n");
        while (1) {
            if (Test_ProcessCommandSwitchRequested()) {
                return 1U;
            }
            ret = SearchBottom();
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                return 1U;
            }
        }
    }
    if (command[0] == 'H') {
        printf("串口调试\t零点/罐底测量重复性测试开始\r\n");
        while (1) {
            if (Test_ProcessCommandSwitchRequested()) {
                return 1U;
            }
            Test_RunMeasureZeroOnce();
            if (Test_ProcessCommandSwitchRequested()) {
                return 1U;
            }
            ret = SearchBottom();
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                return 1U;
            }
        }
    }
    if (command[0] == 'J') {
        printf("串口调试\t液位测量重复性测试开始\r\n");
        while (1) {
            if (Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                return 1U;
            }
            Test_RunMeasureAndFollowOilLevelOnce();
            if (Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                return 1U;
            }
        }
    }
    if (command[0] == 'L') {
        printf("串口调试\t编码值清零\r\n");
        Test_EnterDebugDisplayState(&debug_display);
        set_encoder_zero();
        Test_RestoreDebugDisplayState(&debug_display);
        /* 手动清编码器零点不清电机记步基准，避免电机记步位置口径被重置。 */
        return 1U;
    }
    if (command[0] == 'M') {
        Test_EnterDebugDisplayState(&debug_display);
        printf("串口调试\t电机高温循环测试开始\r\n");
        while (1) {
            if (Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            ret = MotorCtrl_MoveNoWait(100000, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            HAL_Delay(1000);
            if (Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            ret = stpr_waitMove(&stepper);
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
        }
    }
    if (command[0] == 'N') {
        Test_EnterDebugDisplayState(&debug_display);
        printf("串口调试\t电机300mm往返测试开始\r\n");
        while (1) {
            if (Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            stpr_enableDriver(&stepper);
            ret = MotorCtrl_MoveNoWait(300, MOTOR_DIRECTION_DOWN, MotorCtrl_GetDefaultSpeedX100());
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            HAL_Delay(1000);
            printf("电机往返测试\t下行开始\r\n");
            HAL_Delay(1000);
            printf("电机往返测试\t下行指令已等待\r\n");
            ret = stpr_waitMove(&stepper);
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            ret = MotorCtrl_MoveNoWait(300, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            printf("电机往返测试\t上行回零开始\r\n");
            HAL_Delay(1000);
            ret = stpr_waitMove(&stepper);
            if ((ret == STATE_SWITCH) || Test_ProcessCommandSwitchRequested()) {
                MotorCtrl_SlowStop();
                Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
            }
            printf("电机往返测试\t上行回零完成\r\n");
            HAL_Delay(1000);
            stpr_disableDriver(&stepper);
        }
    }
    /* TFIT：卷筒参数拟合调试命令。
     * 推荐流程：T1 开始采样 -> 运行电机 -> TS/TR 查看或求解 -> TP/TU 应用参数。 */
    if (command[0] == 'T') {
        Test_EnterDebugDisplayState(&debug_display);
        switch (command[1]) {
        case '1':
            /* 开始自动采样 */
            MotorCtrl_TapeFitStart();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case '2':
            /* 局部 TFIT：把当前位置作为新的 0 圈起点 */
            MotorCtrl_TapeFitStartLocalOrigin();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case '0':
            /* 停止自动采样 */
            MotorCtrl_TapeFitStop();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'A':
            /* 手动添加样本 */
            MotorCtrl_TapeFitAddCurrentSample();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'S':
            /* 打印拟合状态 */
            MotorCtrl_TapeFitPrintStatus();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'R':
            /* 求解拟合参数 */
            ret = MotorCtrl_TapeFitSolve();
            if (ret != NO_ERROR) {
                printf("卷筒拟合\t全局求解失败\t错误码=0x%08lX\r\n", (unsigned long)ret);
                Test_ProcessCommandWarnFailure("卷筒拟合全局求解", ret);
            }
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'V':
            /* 局部 TFIT：用当前位置起点采样求解局部厚度/周长 */
            ret = MotorCtrl_TapeFitSolveLocalOrigin();
            if (ret != NO_ERROR) {
                printf("卷筒拟合\t局部求解失败\t错误码=0x%08lX\r\n", (unsigned long)ret);
                Test_ProcessCommandWarnFailure("卷筒拟合局部求解", ret);
            }
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'P':
            /* 仅应用拟合出的厚度 t */
            ret = MotorCtrl_TapeFitApply(false, true);
            if (ret != NO_ERROR) {
                printf("卷筒拟合\t应用尺带厚度失败\t错误码=0x%08lX\r\n", (unsigned long)ret);
                Test_ProcessCommandWarnFailure("卷筒拟合应用尺带厚度", ret);
            }
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'U':
            /* 同时应用拟合出的 C0 和 t */
            ret = MotorCtrl_TapeFitApply(true, true);
            if (ret != NO_ERROR) {
                printf("卷筒拟合\t应用首圈周长和厚度失败\t错误码=0x%08lX\r\n", (unsigned long)ret);
                Test_ProcessCommandWarnFailure("卷筒拟合应用首圈周长和厚度", ret);
            }
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        default:
            printf("卷筒拟合命令\t用法：T1=全局开始，T2=局部开始，T0=停止，TA=采样，TS=状态，TR=全局求解，TV=局部求解，TP=应用厚度，TU=应用首圈周长和厚度\r\n");
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        }
    }
    /* Y：位置源手动切换调试命令。
     * YM：以当前编码轮尺带长度为基准，切换到电机记步；切换过程会下行一周标定局部周长再返回。
     * YE：切回编码轮记步，后续 cable_length / sensor_position 重新由编码轮刷新。
     * YS：打印当前记步模式，以及电机尺带和编码轮尺带参考值。
     * YC：电机记步诊断，统一打印基准、局部周长、寄存器和状态校验。 */
    if (command[0] == 'Y') {
        Test_EnterDebugDisplayState(&debug_display);
        switch (command[1]) {
        case 'M':
            printf("位置源切换\t编码轮切换到电机记步\r\n");
            ret = MotorCtrl_SwitchPositionSourceToMotor();
            if (ret != NO_ERROR) {
                printf("位置源切换\t切换电机记步失败\t错误码=0x%08lX\r\n", (unsigned long)ret);
                Test_ProcessCommandWarnFailure("切换电机记步", ret);
            } else {
                printf("位置源切换\t电机记步切换完成\r\n");
            }
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'E':
            printf("位置源切换\t电机记步切换到编码轮\r\n");
            ret = MotorCtrl_SwitchPositionSourceToEncoder();
            if (ret != NO_ERROR) {
                printf("位置源切换\t切换编码轮记步失败\t错误码=0x%08lX\r\n", (unsigned long)ret);
                Test_ProcessCommandWarnFailure("切换编码轮记步", ret);
            } else {
                printf("位置源切换\t编码轮记步切换完成\r\n");
            }
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'S':
            MotorCtrl_PrintPositionCompare();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'C':
            MotorCtrl_PrintMotorCountStatus();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        case 'T':
            Test_TMC5130_SPI_Static();
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        default:
            printf("位置源命令\t用法：YM=切换电机记步，YE=切换编码轮记步，YS=显示位置源，YC=电机记步诊断，YT=静态SPI测试\r\n");
            Test_RestoreDebugDisplayState(&debug_display);
                return 1U;
        }
    }
    if (command[0] == 'X') {
        printf("串口调试\t执行单点测量展示指令\r\n");
        Demo_SinglePointDisplayMock();
        return 1U;
    }


    return 0U;
}

//电机小步进上行测试
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
        printf("%d\t{传感器位置}%.1f", i, (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t{称重值}%d\r\n", weight_parament.current_weight);
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

//电机小步进下行测试
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
        printf("%d\t{传感器位置}%.1f", i, (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t{称重值}%d\r\n", weight_parament.current_weight);
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

//电机步进测试
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
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
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
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
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
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
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
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
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
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
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
        printf("%d\t{encoder}%d\t{weight}%d\t", i, (int)g_encoder_count, weight_parament.current_weight);
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
///*********************** 测试函数 ***********************/
void Test_Params_Storage(void) {
	// 备份原始参数
	DeviceParameters original = g_deviceParams;

	// 测试写读校验
	g_deviceParams.tankHeight = 1234; // 测试数据
	save_device_params(); //存储

	if (load_device_params()) {
		if (g_deviceParams.tankHeight != 1234) {
			printf("数据加载失败");
			// 数据验证失败处理
		} else {
			printf("数据加载成功: tankHeight = %lu", g_deviceParams.tankHeight);
		}
	} else {
		printf("CRC校验失败");
	}

	// 恢复原始参数
	g_deviceParams = original;
	save_device_params(); //存储
}

#define TEST_ENCODER_SLOT_SIZE  (0x40u)
#define TEST_ENCODER_A_ADDRESS   FRAM_ANGLE_ADDRESS
#define TEST_ENCODER_B_ADDRESS   (TEST_ENCODER_A_ADDRESS + TEST_ENCODER_SLOT_SIZE)

void Test_ParamEncoder_AB_Backup(void)
{
    DeviceParameters param_backup = g_deviceParams;
    uint8_t param_a_raw[sizeof(DeviceParameters)] = {0};
    uint8_t param_b_raw[sizeof(DeviceParameters)] = {0};

    uint8_t encoder_a_raw[TEST_ENCODER_SLOT_SIZE] = {0};
    uint8_t encoder_b_raw[TEST_ENCODER_SLOT_SIZE] = {0};

    const uint32_t param_magic_offset = (uint32_t)offsetof(DeviceParameters, magic);

    ReadMultiData(param_a_raw, FRAM_PARAM_A_ADDRESS, sizeof(param_a_raw));
    ReadMultiData(param_b_raw, FRAM_PARAM_B_ADDRESS, sizeof(param_b_raw));
    ReadMultiData(encoder_a_raw, TEST_ENCODER_A_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    ReadMultiData(encoder_b_raw, TEST_ENCODER_B_ADDRESS, TEST_ENCODER_SLOT_SIZE);

    printf("\r\n===== AB双备份回退测试开始 =====\r\n");

    /* Case-1: 参数A损坏，读取应回退到B */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, FRAM_PARAM_A_ADDRESS + param_magic_offset);

    if (load_device_params()) {
        printf("[通过] 参数区: A损坏后已回退到B\r\n");
    } else {
        printf("[失败] 参数区: A损坏后未能回退到B\r\n");
    }

    /* Case-2: 参数A/B都损坏，读取应报错 */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, FRAM_PARAM_A_ADDRESS + param_magic_offset);
    WriteSingleData(0u, FRAM_PARAM_B_ADDRESS + param_magic_offset);

    if ((!load_device_params()) && (g_measurement.device_status.error_code == PARAM_EEPROM_FAIL)) {
        printf("[通过] 参数区: A/B都损坏时已报错 PARAM_EEPROM_FAIL\r\n");
    } else {
        printf("[失败] 参数区: A/B都损坏时报错不符合预期, 错误码：0x%08lX\r\n",
               (unsigned long)g_measurement.device_status.error_code);
    }

    /* 恢复参数区并回写双分区 */
    WriteMultiData(param_a_raw, FRAM_PARAM_A_ADDRESS, sizeof(param_a_raw));
    WriteMultiData(param_b_raw, FRAM_PARAM_B_ADDRESS, sizeof(param_b_raw));
    g_deviceParams = param_backup;
    save_device_params();

    /* Case-3: 编码A损坏，初始化应回退到B */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, TEST_ENCODER_A_ADDRESS);
    Initialize_Encoder();

    if (g_measurement.device_status.error_code != ENCODER_POWERON_FAIL) {
        printf("[通过] 编码区: A损坏后已回退到B\r\n");
    } else {
        printf("[失败] 编码区: A损坏后未能回退到B\r\n");
    }

    /* Case-4: 编码A/B都损坏，应报错 */
    g_measurement.device_status.error_code = NO_ERROR;
    WriteSingleData(0u, TEST_ENCODER_A_ADDRESS);
    WriteSingleData(0u, TEST_ENCODER_B_ADDRESS);
    Initialize_Encoder();

    if (g_measurement.device_status.error_code == ENCODER_POWERON_FAIL) {
        printf("[通过] 编码区: A/B都损坏时已报错 ENCODER_POWERON_FAIL\r\n");
    } else {
        printf("[失败] 编码区: A/B都损坏时报错不符合预期, 错误码：0x%08lX\r\n",
               (unsigned long)g_measurement.device_status.error_code);
    }

    /* 恢复编码区 */
    WriteMultiData(encoder_a_raw, TEST_ENCODER_A_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    WriteMultiData(encoder_b_raw, TEST_ENCODER_B_ADDRESS, TEST_ENCODER_SLOT_SIZE);
    Initialize_Encoder();

    printf("===== AB双备份回退测试结束 =====\r\n\r\n");
}
static void Test_SensorCommPrintResult(const char *tag,
                                       const char *name,
                                       uint32_t ret,
                                       uint32_t *fail_count)
{
    if (ret == NO_ERROR) {
        printf("[传感器][正常] %s %s\r\n", tag, name);
        return;
    }

    if (fail_count != NULL) {
        (*fail_count)++;
        printf("[传感器][异常] %s %s失败，错误码：%lu，失败次数：%lu\r\n",
               tag,
               name,
               (unsigned long)ret,
               (unsigned long)(*fail_count));
    } else {
        printf("[传感器][异常] %s %s失败，错误码：%lu\r\n",
               tag,
               name,
               (unsigned long)ret);
    }
}

static void __attribute__((unused)) Sensor_CommCheckAndLog(const char *tag)
{
    float temp = 0.0f;
    float frequency = 0.0f;
    float density = 0.0f;
    static uint32_t comm_fail_cnt = 0U;
    uint32_t ret;

    if (g_deviceParams.sensorType == DSM_SENSOR) {
        printf("[传感器] %s 类型=DSM一代(%lu)，单次通信=读取频率/密度/温度\r\n",
               tag,
               (unsigned long)g_deviceParams.sensorType);

        ret = (uint32_t)DSM_Read_Frequency_Density_Temp(&frequency, &density, &temp);
        Test_SensorCommPrintResult(tag, "DSM一代读取频率/密度/温度", ret, &comm_fail_cnt);
        if (ret == NO_ERROR) {
            printf("[传感器][正常] %s DSM一代 频率=%.3f Hz 密度=%.3f 温度=%.3f C\r\n",
                   tag,
                   frequency,
                   density,
                   temp);
        }
        return;
    }

    if (g_deviceParams.sensorType == LTD_SENSOR) {
        printf("[传感器] %s 类型=LTD/V2(%lu)，单次通信=读取密度\r\n",
               tag,
               (unsigned long)g_deviceParams.sensorType);

        ret = (uint32_t)DSM_V2_Read_Density(&density);
        Test_SensorCommPrintResult(tag, "LTD/V2读取密度", ret, &comm_fail_cnt);
        if (ret == NO_ERROR) {
            printf("[传感器][正常] %s LTD/V2 密度=%.3f\r\n", tag, density);
        }
        return;
    }

    comm_fail_cnt++;
    printf("[传感器][异常] %s 未知传感器类型：%lu，失败次数：%lu\r\n",
           tag,
           (unsigned long)g_deviceParams.sensorType,
           (unsigned long)comm_fail_cnt);
}
/**
 * @brief S后缀通信检查只打印通信结果，不改变B/BE退出条件和全局错误态。
 * @note  只在任务上下文调用；保留进入前的device_state和error_code。
 */
static void Test_SensorCommCheckAndPrintOnly(const char *tag)
{
    DeviceState saved_state = g_measurement.device_status.device_state;
    uint32_t saved_error = g_measurement.device_status.error_code;

    Sensor_CommCheckAndLog(tag);

    g_measurement.device_status.device_state = saved_state;
    g_measurement.device_status.error_code = saved_error;
}


/**
 * @brief A指令测试专用：只下发停止寄存器，不判断称重、编码器或驱动错误。
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
 * @note  运动期间只响应命令切换；不读取称重、编码器错误或全局错误退出。
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
/* ========================= 主测试函数 ========================= */
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
#include <ltd_sensor_communication.h>
#include <stdio.h>

/**
 * @brief  测试V2协议通讯与关键参数读取
 * @note   可在初始化完成后调用，例如 main() 或 sensor init 后
 */
void DSM_V2_Test_AllParams(void) {
	printf("\r\n===== DSM V2 通讯测试开始 =====\r\n");

//    // 1. 切换到液位模式
//    int ret = DSM_V2_SwitchToLevelMode();
//    if (ret == NO_ERROR)
//        printf("切换液位模式成功\r\n");
//    else {
//        printf("切换液位模式失败，错误码 %d\r\n", ret);
//        return; // 通讯异常，后面读也没意义
//    }

	// 2. 定义变量
	float temp = 0, rho = 0, mu = 0, nu = 0;
	uint32_t freq = 0, sensor_id = 0;

	// 3. 依次读取各参数
//    if (DSM_V2_Read_SoftwareVersion(&ver) == NO_ERROR)
//        printf("软件版本: %.3f\r\n", ver);
//    else printf("读取软件版本失败\r\n");

	if (DSM_V2_Read_Temperature(&temp) == NO_ERROR) {
		printf("温度值: %.3f ℃\r\n", temp);
		g_measurement.single_point_monitoring.temperature = (int) (temp * 100) + 20000;
	} else
		printf("读取温度失败\r\n");

	if (DSM_V2_Read_Density(&rho) == NO_ERROR) {
		g_measurement.single_point_monitoring.density = (int) (rho * 10);
		printf("密度值: %.3f\r\n", rho);
	} else
		printf("读取密度失败\r\n");

	if (DSM_V2_Read_DynamicViscosity(&mu) == NO_ERROR)
		printf("动力粘度: %.3f\r\n", mu);
	else
		printf("读取动力粘度失败\r\n");

	if (DSM_V2_Read_KinematicViscosity(&nu) == NO_ERROR)
		printf("运动粘度: %.3f\r\n", nu);
	else
		printf("读取运动粘度失败\r\n");

    if (DSM_V2_Read_LevelFrequency(&freq) == NO_ERROR)
        printf("液位频率: %lu Hz\r\n", (unsigned long)freq);
    else printf("读取液位频率失败\r\n");

    if (DSM_V2_Read_SensorID(&sensor_id) == NO_ERROR)
        printf("传感器号: %lu\r\n", (unsigned long)sensor_id);
    else printf("读取传感器号失败\r\n");

	printf("===== DSM V2 通讯测试结束 =====\r\n\r\n");
}
/**
 * @brief  传感器与无线链路综合通信测试
 * @note   手动调试入口，建议在系统初始化完成后临时调用；函数会执行传感器识别，
 *         并刷新 g_deviceParams.sensorType/sensorID，正式流程中不要周期性调用。
 */
void SensorWireless_CommTest(void)
{
    uint32_t ok_count = 0U;
    uint32_t fail_count = 0U;
    uint32_t ret;
    float temp = 0.0f;
    float density = 0.0f;
    float frequency = 0.0f;

    printf("\r\n===== 传感器与无线通信测试开始 =====\r\n");

    ret = WIRELESS_ProbeNode(COMM_TEST_WIRELESS_HOST_ADDR);
    if (Test_CommRecordResult("无线主机链路探测", ret, &ok_count, &fail_count)) {
        if (Test_CommShouldStop(&fail_count)) {
            return;
        }
        ret = WIRELESS_PrintInfo(COMM_TEST_WIRELESS_HOST_ADDR);
        (void)Test_CommRecordResult("读取无线主机信息", ret, &ok_count, &fail_count);
    }

    if (Test_CommShouldStop(&fail_count)) {
        return;
    }

    ret = WIRELESS_ProbeNode(COMM_TEST_WIRELESS_SLAVE_ADDR);
    if (Test_CommRecordResult("无线从机链路探测", ret, &ok_count, &fail_count)) {
        if (Test_CommShouldStop(&fail_count)) {
            return;
        }
        ret = WIRELESS_PrintInfo(COMM_TEST_WIRELESS_SLAVE_ADDR);
        (void)Test_CommRecordResult("读取无线从机信息", ret, &ok_count, &fail_count);
    }

    if (Test_CommShouldStop(&fail_count)) {
        return;
    }

    ret = DetectSensorType();
    if (!Test_CommRecordResult("传感器协议识别", ret, &ok_count, &fail_count)) {
        printf("协议识别失败，跳过传感器参数读取\r\n");
        printf("===== 通信测试结束，成功=%lu 失败=%lu =====\r\n\r\n",
               (unsigned long)ok_count,
               (unsigned long)fail_count);
        return;
    }

    printf("识别结果: sensorType=%lu sensorID=%lu\r\n",
           (unsigned long)g_deviceParams.sensorType,
           (unsigned long)g_deviceParams.sensorID);

    if (g_deviceParams.sensorType == DSM_SENSOR) {
        ret = (uint32_t)DSM_Read_Frequency_Density_Temp(&frequency, &density, &temp);
        if (Test_CommRecordResult("DSM一代单次读取频率/密度/温度", ret, &ok_count, &fail_count)) {
            printf("DSM一代密度数据: 频率=%.3f Hz 密度=%.3f 温度=%.3f\r\n", frequency, density, temp);
        }
    } else if (g_deviceParams.sensorType == LTD_SENSOR) {
        ret = (uint32_t)DSM_V2_Read_Density(&density);
        if (Test_CommRecordResult("LTD/V2单次读取密度", ret, &ok_count, &fail_count)) {
            printf("LTD/V2密度: %.3f\r\n", density);
        }
    } else {
        fail_count++;
        printf("未知传感器类型: %lu\r\n", (unsigned long)g_deviceParams.sensorType);
    }
    printf("===== 通信测试结束，成功=%lu 失败=%lu =====\r\n\r\n",
           (unsigned long)ok_count,
           (unsigned long)fail_count);
}
static uint8_t Demo_SinglePointDisplay_ShouldAbort(void)
{
    if (!HasEffectiveCommandSwitchRequest()) {
        return 0;
    }

    printf("单点展示检测到新命令，退出当前演示\r\n");
    g_measurement.device_status.device_state = STATE_STANDBY;
    g_measurement.debug_data.motor_state = 0U;
    return 1;
}

static void Demo_SinglePointDisplay_UpdateResult(volatile DensityMeasurement *result,
                                                 uint32_t temperature_raw,
                                                 uint32_t density_raw,
                                                 uint32_t pos_01mm,
                                                 uint32_t standard_density_raw,
                                                 uint32_t vcf20_raw,
                                                 uint32_t weight_density_raw)
{
    if (result == NULL) {
        return;
    }

    result->temperature = temperature_raw;
    result->density = density_raw;
    result->temperature_position = pos_01mm;
    result->standard_density = standard_density_raw;
    result->vcf20 = vcf20_raw;
    result->weight_density = weight_density_raw;
}

void Demo_SinglePointDisplayMock(void)
{
    static const int16_t temp_wave_x100[]    = { 0, 6, 12, 18, 24, 18, 12, 6, 0, -4, -8, -4 };
    static const int16_t density_wave_x10[]  = { 0, 1, 2, 3, 2, 1, 0, -1, -2, -1, 0, 1 };
    static const int16_t pos_wave_01mm[]     = { 0, 2, 4, 6, 8, 6, 4, 2, 0, -2, -4, -2 };
    const uint32_t wave_count = (uint32_t)(sizeof(temp_wave_x100) / sizeof(temp_wave_x100[0]));
    uint32_t target_pos_01mm;
    uint32_t tank_height_01mm;
    uint32_t start_pos_01mm;
    uint32_t current_pos_01mm;
    uint32_t cable_01mm;
    uint32_t i;

    target_pos_01mm = g_deviceParams.singlePointMeasurementPosition;
    if (target_pos_01mm == 0U) {
        if (g_measurement.debug_data.sensor_position > 0) {
            target_pos_01mm = (uint32_t)g_measurement.debug_data.sensor_position;
        } else if (g_deviceParams.tankHeight > 0U) {
            target_pos_01mm = g_deviceParams.tankHeight / 2U;
        } else {
            target_pos_01mm = 15000U;
        }
    }

    tank_height_01mm = g_deviceParams.tankHeight;
    if ((tank_height_01mm == 0U) || (tank_height_01mm <= target_pos_01mm)) {
        tank_height_01mm = target_pos_01mm + 12000U;
    }

    start_pos_01mm = (target_pos_01mm > 3000U) ? (target_pos_01mm - 3000U) : 0U;
    current_pos_01mm = start_pos_01mm;

    printf("\r\n===== 单点测量展示模式开始 =====\r\n");
    printf("展示说明: 不读真实传感器、不驱动电机，只刷新单点测量显示字段\r\n");
    printf("停止方式: 下发任意新命令即可退出展示\r\n");

    g_measurement.device_status.zero_point_status = 0U;
    g_measurement.device_status.error_code = NO_ERROR;
    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
    g_measurement.debug_data.motor_state = 2U;
    g_measurement.debug_data.motor_speed = 120U;

    for (i = 0U; i < 6U; i++) {
        if (Demo_SinglePointDisplay_ShouldAbort()) {
            return;
        }

        current_pos_01mm = start_pos_01mm +
                (uint32_t)(((uint64_t)(target_pos_01mm - start_pos_01mm) * (uint64_t)(i + 1U)) / 6U);
        cable_01mm = (tank_height_01mm > current_pos_01mm) ? (tank_height_01mm - current_pos_01mm) : 0U;

        g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
        g_measurement.debug_data.sensor_position = (int32_t)current_pos_01mm;
        g_measurement.debug_data.cable_length = (int32_t)cable_01mm;
        g_measurement.debug_data.motor_distance = (int32_t)cable_01mm;
        g_measurement.debug_data.temperature = 22580U;
        g_measurement.debug_data.frequency = 121300U + i * 20U;
        g_measurement.debug_data.air_frequency = 121900U;
        g_measurement.debug_data.current_amplitude = 88U + i;
        g_measurement.debug_data.current_weight = 3200U + i * 10U;

        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_measurement,
                                             22580U,
                                             8348U,
                                             current_pos_01mm,
                                             8338U,
                                             9997U,
                                             8342U);
        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_monitoring,
                                             22580U,
                                             8348U,
                                             current_pos_01mm,
                                             8338U,
                                             9997U,
                                             8342U);

        printf("单点展示\t运行到测量点 [%lu/6] 位置=%.1fmm\r\n",
               (unsigned long)(i + 1U),
               current_pos_01mm / 10.0f);
        HAL_Delay(500);
    }

    printf("单点展示\t已到达展示点，开始刷新虚拟温度/密度\r\n");

    for (i = 0U;; i++) {
        uint32_t idx = i % wave_count;
        uint32_t temperature_raw;
        uint32_t density_raw;
        uint32_t standard_density_raw;
        uint32_t weight_density_raw;
        uint32_t vcf20_raw;

        if (Demo_SinglePointDisplay_ShouldAbort()) {
            return;
        }

        current_pos_01mm = (uint32_t)((int32_t)target_pos_01mm + pos_wave_01mm[idx]);
        cable_01mm = (tank_height_01mm > current_pos_01mm) ? (tank_height_01mm - current_pos_01mm) : 0U;

        temperature_raw = (uint32_t)(20000 + 2650 + temp_wave_x100[idx]);
        density_raw = (uint32_t)(8350 + density_wave_x10[idx]);
        standard_density_raw = density_raw - 8U;
        weight_density_raw = density_raw - 4U;
        vcf20_raw = 9995U + (idx % 6U);

        g_measurement.device_status.device_state = STATE_SINGLEPOINTING;
        g_measurement.debug_data.motor_state = 0U;
        g_measurement.debug_data.motor_speed = 0U;
        g_measurement.debug_data.sensor_position = (int32_t)current_pos_01mm;
        g_measurement.debug_data.cable_length = (int32_t)cable_01mm;
        g_measurement.debug_data.motor_distance = (int32_t)cable_01mm;
        g_measurement.debug_data.temperature = temperature_raw;
        g_measurement.debug_data.frequency = 121500U + idx * 15U;
        g_measurement.debug_data.air_frequency = 121980U;
        g_measurement.debug_data.current_amplitude = 96U + (idx % 5U);
        g_measurement.debug_data.current_weight = 3280U + (idx % 4U) * 8U;

        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_measurement,
                                             temperature_raw,
                                             density_raw,
                                             current_pos_01mm,
                                             standard_density_raw,
                                             vcf20_raw,
                                             weight_density_raw);
        Demo_SinglePointDisplay_UpdateResult(&g_measurement.single_point_monitoring,
                                             temperature_raw,
                                             density_raw,
                                             current_pos_01mm,
                                             standard_density_raw,
                                             vcf20_raw,
                                             weight_density_raw);

        printf("单点展示\t状态=固定点测量中 位置=%.1fmm 温度=%.2fC 密度=%.1f 标密=%.1f VCF20=%lu 重量密度=%.1f\r\n",
               current_pos_01mm / 10.0f,
               RAW_TO_TEMP(temperature_raw),
               RAW_TO_DENSITY(density_raw),
               RAW_TO_DENSITY(standard_density_raw),
               (unsigned long)vcf20_raw,
               RAW_TO_DENSITY(weight_density_raw));

        HAL_Delay(500);
    }
}
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
//测试主函数
void Test_main(void) {
	Test_FRAM_ReadWrite(); //测试FRAM读写
//	motor_text(300.0f, 0U);
	Test_Params_Storage(); //测试参数存储
	CRC32_HAL_Test(); //CRC校验测试
}

