/**
 * @file service_debug_command.c
 * @brief CPU2 串口维护命令解析、状态接管和各诊断模块分发入口。
 *
 * service_debug_command.c
 * 测试函数，用来临时测试某些功能
 *  Created on: Jul 22, 2025
 *      Author: Duan Xuebin
 */

#include "Protocols/MultiparamV3/multiparam_v3_communication.h"
#include "Protocols/Dsm/dsm_sensor_communication.h"
#include "Protocols/Dm4/V4/multiparam_v4_communication.h"
#include "service_debug.h"
#include "service_debug_internal.h"
#include "serial_command_parser.h"
#include "service_debug_fixed_frequency.h"
#include "measure.h"
#include "measure_density.h"
#include "motor_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "system_parameter.h"
#include "measure_tank_height.h"
#include "measure_zero.h"
#include "measure_oil_level.h"
#include "Wireless/wireless_pairing.h"
#include "error_log.h"
#include "weight.h"
#include "system_parameter.h"
#include "sensor_service.h"
#include "sensor_runtime.h"
#include <mb85rs2m.h>
#include "my_crc.h"
#include "ad5421.h"
#include "AoOutput/ao_output.h"
#include "encoder.h"
#include <stddef.h>
#include <string.h>
#define TEST_AO_CURRENT_MIN_X100                 320U /* 模拟量输出测试允许的最小电流，单位 0.01mA。 */
#define TEST_AO_CURRENT_MAX_X100                 2400U /* 模拟量输出测试允许的最大电流，单位 0.01mA。 */
#define TEST_AO_HOLD_MS                          5000U /* 模拟量输出测试单点保持时间，单位 ms。 */
#define TEST_AO_REFRESH_MS                       500U /* 模拟量输出测试刷新周期，单位 ms。 */
#define TEST_AO_DIAG_DELAY_MS                   10U /* 模拟量输出测试写电流后等待 AD5421 状态稳定的时间。 */

/**
 * @brief S后缀通信检查只打印通信结果，不改变B/BE退出条件和全局错误态。
 * @note  只在任务上下文调用；保留进入前的device_state和error_code。
 *
 * @param tag 用于区分诊断来源的只读标签文字。
 */
/**
 * @brief BE测试退出时恢复进入前斜坡参数。
 *
 * @param snapshot 只读 TMC5130 斜坡寄存器快照；用于临时应用测试参数后恢复进入前的速度、加速度和斜坡配置。
 */
/**
 * @brief 跟随测试前按自动切换配置尝试改用电机记步，并通过输出标志报告本次是否切换。
 *
 * @param follow_name 本次测试跟随流程的现场名称，用于记录位置源切换结果。
 * @param switched_to_motor 用于返回测试流程本次是否已切换到电机位置源。
 * @return NO_ERROR 表示跟随测试前按自动切换配置尝试改用电机记步，并通过输出标志报告本次是否切换已完成；其他值为调用链原样传播的参数、状态、通信、传感器或电机错误码。
 */
static uint32_t Test_EnsureMotorPositionSourceBeforeFollow(const char *follow_name, uint8_t *switched_to_motor);
/**
 * @brief 执行一次零点测量测试并记录结果。
 */
static void Test_RunMeasureZeroOnce(void);
/**
 * @brief 执行一次液位测试链：必要时回零、搜索液位、按配置切换位置源并复搜，最后进入液位跟随。
 */
static void Test_RunMeasureAndFollowOilLevelOnce(void);
/**
 * @brief 处理 AO 正式回读电流命令。
 *
 * @details 调用场景：串口发送 AO400、AO1200、AO2000 或 AOS 时验证正式回读和物理电流输出。
 * @note 关键约束：命令在 MeasureStart 前处理，避免被 A 电机测试路径截获。
 *
 * @param command 已经通过语法分类的 AO 输出测试或回读命令文本。
 * @return 1 表示当前命令已由 AO 回读电流测试处理；不是该命令或参数非法时返回 0。
 */
static uint8_t TestCommand_HandleAoOutputTest(const uint8_t *command);


/**
 * @brief  记录单项通信测试结果
 * @note   仅用于手动测试日志汇总，不改变业务错误状态。
 *
 * @param name 用于串口测试日志标识当前测试项、阶段或通信目标的 NUL 结尾只读名称。
 * @param ret 上一层调用返回的结果码。串口测试流程只据此累计和打印测试结果，不把测试失败提升为正式测量故障。
 * @param ok_count 输入输出通信成功次数；本次成功时递增。
 * @param fail_count 输入输出通信失败次数；本次失败时递增。
 * @return 1 表示本次通信测试成功且成功计数已更新；失败时更新失败计数并返回 0。
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
 * @brief 处理测试过程中收到的外部命令切换请求。
 * @return 1 表示检测到并已处理外部命令切换请求；没有有效切换请求时返回 0。
 */
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
 *
 * @param operation 本次测试命令的现场可读操作名称，用于失败告警。
 * @param error_code 待记录、转换或判断的错误码。该值是串口测试命令的失败结果，用于决定警告输出而不覆盖正式测量故障。
 */
static void Test_ProcessCommandWarnFailure(const char *operation, uint32_t error_code)
{
    if ((error_code == NO_ERROR) || (error_code == STATE_SWITCH)) {
        return;
    }

    ErrorLog_Warn(ERROR_LOG_MODULE_COMM,
                  operation,
                  ErrorLog_GetReasonByCode(error_code),
                  "仅记录");
}


/* 测试命令临时接管调试显示时的恢复快照；记录原设备状态及接管标志，退出测试后用于恢复。 */
typedef struct {
    /* 测试命令接管 CPU3 调试显示前保存的设备状态和接管标志。 */
    DeviceState saved_state; /* 测试命令接管调试显示前的设备主状态。 */
    uint8_t active; /* 测试命令当前已经接管 CPU3 调试显示的标志。 */
} TestCommandDebugDisplaySnapshot;

/**
 * @brief 保存串口调试前的显示状态，并临时切到调试模式。
 * @note 只在任务上下文调用；不清错误码、不执行硬件动作。
 *
 * @param snapshot 串口测试命令进入前的显示状态快照；保存设备状态、当前命令和显示相关字段，供测试结束后恢复。
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
 *
 * @param snapshot 串口测试命令进入前的显示状态快照；保存设备状态、当前命令和显示相关字段，供测试结束后恢复。
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


#define TEST_COMMAND_BE_SPEED_DEFAULT      0U /* BE 测试命令默认速度参数。 */
#define TEST_COMMAND_BE_MULTIPLIER_DEFAULT 1U /* BE 测试命令默认加减速倍率。 */
#define TEST_COMMAND_BE_MULTIPLIER_MAX     20U /* BE 测试命令允许的最大加减速倍率。 */

typedef struct {
    /* BE 测试命令解析后的速度、加速度倍率和传感器通信开关。 */
    uint32_t speed_x100; /* BE 测试命令指定的电机速度，单位为 0.01 m/min。 */
    uint32_t accel_multiplier; /* BE 测试命令指定的加速度倍率。 */
    uint8_t enable_sensor_comm; /* BE 测试期间是否保持传感器通信的开关。 */
} TestCommandBeOptions;

/**
 * @brief 限制 BE 调试倍率；0 或负数按 1 档，过大按上限处理。
 *
 * @param value 待限制到合法范围的原始输入值。
 * @return 返回限制后的 BE 调试倍率；0 或负数回退 1，超过上限时返回允许的最大倍率。
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
 *
 * @param value 待限制到合法范围的原始输入值。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint32_t TestCommand_ClampBeSpeedMMin(double value)
{
    double speed_x100;

    if (!(value > 0.0)) {
        return TEST_COMMAND_BE_SPEED_DEFAULT;
    }

    speed_x100 = value * 100.0;
    return MotorCtrl_ClampSpeedX100((uint32_t)(speed_x100 + 0.5));
}

/**
 * @brief 解析 BJ/BJP 点动测试的可选速度，单位 m/min。
 * @note  找不到逗号或速度非法时返回 0，表示沿用当前默认速度配置。
 *
 * @param arg_tail BJ 或 BJP 命令的剩余文本；函数搜索首个逗号，并从逗号后尝试解析 m/min 浮点速度。
 * @return 未找到逗号、数值无法解析或数值不大于 0 时返回 0，表示沿用默认速度；否则返回换算并钳位后的 0.01 m/min 速度值。
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
 *
 * @param command 已经通过语法分类的 BJ 或 BJP 测试命令文本。
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
 * @brief 执行一次 AO 正式电流下发，并在短时间内重复写入同一电流值。
 *
 * @details 调用场景：串口 AO 命令现场验证 AD5421 控制寄存器回读、故障寄存器回读和电流环输出。
 * @note 关键约束：使用正式 Ad5421Init() 初始化路径；诊断失败只提示，不提前结束电流保持。
 *
 * @param current_mA_x100 本次设置、限制或测试使用的模拟输出电流，单位 0.01 mA。
 */
static void TestCommand_RunAoOutputPoint(uint32_t current_mA_x100)
{
    uint32_t init_ret;
    uint32_t write_ret;
    uint32_t diag_ret;
    uint32_t elapsed_ms = 0U;

    init_ret = Ad5421Init();
    printf("AO FORMAL\tformal init=0x%08lX\tfault=0x%04lX\tflags=0x%08lX\tcurrent_x100=%lu\r\n",
           (unsigned long)init_ret,
           (unsigned long)AD5421_GetFaultRegister(),
           (unsigned long)AD5421_GetFaultFlags(),
           (unsigned long)current_mA_x100);
    if (init_ret != NO_ERROR) {
        printf("AO FORMAL\tinit failed, skip write\r\n");
        return;
    }

    while (elapsed_ms < TEST_AO_HOLD_MS) {
        write_ret = AD5421_SetCurrentX100(current_mA_x100);
        if (write_ret == NO_ERROR) {
            HAL_Delay(TEST_AO_DIAG_DELAY_MS);
            diag_ret = AD5421_PollDiagnostics();
        } else {
            diag_ret = write_ret;
        }
        printf("AO FORMAL\twrite current=%.2fmA\twrite=0x%08lX\tdiag=0x%08lX\tfault=0x%04lX\tflags=0x%08lX\telapsed=%lums\r\n",
               (double)current_mA_x100 / 100.0,
               (unsigned long)write_ret,
               (unsigned long)diag_ret,
               (unsigned long)AD5421_GetFaultRegister(),
               (unsigned long)AD5421_GetFaultFlags(),
               (unsigned long)elapsed_ms);
        if (write_ret != NO_ERROR) {
            return;
        }
        if (diag_ret != NO_ERROR) {
            printf("AO FORMAL\tdiag warning, keep current for meter check\r\n");
        }
        if (Test_ProcessCommandSwitchRequested() != 0U) {
            return;
        }
        HAL_Delay(TEST_AO_REFRESH_MS);
        elapsed_ms += TEST_AO_REFRESH_MS;
    }
}
/**
 * @brief 处理 AO 正式回读电流命令。
 *
 * @details 调用场景：串口发送 AO400、AO1200、AO2000 或 AOS 时验证正式回读和物理电流输出。
 * @note 关键约束：命令在 MeasureStart 前处理，避免被 A 电机测试路径截获。
 *
 * @param command 已经通过语法分类的 AO 输出测试或回读命令文本。
 * @return 1 表示当前命令已由 AO 回读电流测试处理；不是该命令或参数非法时返回 0。
 */
static uint8_t TestCommand_HandleAoOutputTest(const uint8_t *command)
{
    const uint32_t scan_points[] = {400U, 1200U, 2000U, 2200U};
    const char *arg;
    char *endptr;
    unsigned long current_mA_x100;

    if ((command == NULL) || (command[0] != 'A') || (command[1] != 'O')) {
        return 0U;
    }

    AoOutput_SuspendTimerRefresh();

    if ((command[2] == 'S') && (command[3] == '\0')) {
        for (uint32_t i = 0U; i < (sizeof(scan_points) / sizeof(scan_points[0])); i++) {
            if (Test_ProcessCommandSwitchRequested() != 0U) {
                AoOutput_ResumeTimerRefresh();
                return 1U;
            }
            TestCommand_RunAoOutputPoint(scan_points[i]);
        }
        printf("AO FORMAL\tscan done\r\n");
        AoOutput_ResumeTimerRefresh();
        return 1U;
    }

    arg = (const char *)&command[2];
    current_mA_x100 = strtoul(arg, &endptr, 10);
    if ((endptr == arg) || (*endptr != '\0') ||
        (current_mA_x100 < TEST_AO_CURRENT_MIN_X100) ||
        (current_mA_x100 > TEST_AO_CURRENT_MAX_X100)) {
        printf("AO FORMAL\tusage: AO400/AO1200/AO2000/AO2200 or AOS, unit=0.01mA\r\n");
        AoOutput_ResumeTimerRefresh();
        return 1U;
    }

    TestCommand_RunAoOutputPoint((uint32_t)current_mA_x100);
    AoOutput_ResumeTimerRefresh();
    return 1U;
}
/**
 * @brief 解析 BE 可选参数，兼容旧的 BE100S 和 BE100,1 传感器通信写法。
 * @note  新格式：BE距离,速度m/min,加速度倍率,S 或 BE距离,速度m/min,加速度倍率,1。
 *
 * @param arg 待解析的完整命令参数文本。
 * @param options 用于接收解析后的 BE 距离、速度、倍率和通信选项。
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

/**
 * @brief 跟随测试前按自动切换配置尝试改用电机记步，并通过输出标志报告本次是否切换。
 *
 * @param follow_name 本次测试跟随流程的现场名称，用于记录位置源切换结果。
 * @param switched_to_motor 用于返回测试流程本次是否已切换到电机位置源。
 * @return NO_ERROR 表示无需切换或已经切到电机记步，并通过输出标志报告结果；其他值为位置源切换、周长标定、位置同步或参数保存错误。
 */
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

/**
 * @brief 执行一次零点测量测试并记录结果。
 */
static void Test_RunMeasureZeroOnce(void)
{
    uint32_t ret;

    (void)MeasureStart();
    g_measurement.device_status.device_state = STATE_BACKZEROING;
    ret = (uint32_t)SearchZero();
    SET_ERROR(ret);
    g_measurement.device_status.device_state = STATE_STANDBY;
}

/**
 * @brief 执行一次液位测试链：必要时回零、搜索液位、按配置切换位置源并复搜，最后进入液位跟随。
 */
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

/*
 * 处理多参数V4传感器串口调试命令，不启动电机且不修改整机全局故障状态。
 * V4P会显式重建V4通信上下文；其余交互操作保持当前模式并通过原包跟踪输出每次事务。
 */
/**
 * @brief 校验并分派 CPU2 串口维护测试命令。
 *
 * 空缓冲区或未被 SerialCommandParser 严格识别为测试类的命令立即返回，避免仅凭首字符前缀误启动电机或维护动作。
 * 传感器与无线通信、蓝牙配对、模拟量输出、B/BE 电机往返等无需通用测量初始化的命令优先处理；除 A 手动运动和 X 展示模拟外，其余后续测试在分派前调用 MeasureStart。
 * 支持固定频率找液位、电机步进与重复性试验、卷筒参数拟合、位置源切换和静态 SPI 等维护入口；需要 OLED 调试状态的分支会保存并恢复原显示状态。
 * 长时间循环测试会反复检查新命令切换请求，并在需要时慢停电机后退出；这些命令会真实操作传感器、电机、模拟量输出或参数，不得在中断上下文调用。
 *
 * @param command 已完成基础收帧的可修改串口测试命令缓冲区，以 NUL 结尾。
 * @return 已识别并处理返回 1，非测试命令返回 0；空命令或没有匹配到具体测试分支时也返回 0，由上层继续处理。
 * @note 返回 1 只表示测试命令已被本函数消费，不等同于对应硬件测试成功；实际结果由打印、错误码和设备状态判断。
 */
uint8_t ServiceDebug_ProcessSerialCommand(uint8_t *command)
{
    uint32_t ret = NO_ERROR;
    TestCommandDebugDisplaySnapshot debug_display = { STATE_STANDBY, 0U };


    if ((command == NULL) || (command[0] == '\0')) {
        return 0U;
    }

    /* 只有通过严格校验的测试命令才能进入旧动作分发器，避免前缀误执行。 */
    if (SerialCommandParser_Parse(command).kind != SERIAL_COMMAND_KIND_TEST) {
        return 0U;
    }

    if ((command[0] == 'L') && (command[1] == 'F') &&
        (command[2] == '?')) {
        ServiceDebug_FixedFrequencyPrintConfig();
        return 1U;
    }

    if (TestCommand_HandleAoOutputTest(command) != 0U) {
        return 1U;
    }

    if (TestCommand_HandleV3(command) != 0U) {
        return 1U;
    }
    if (TestCommand_HandleV4(command) != 0U) {
        return 1U;
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
    if ((command[0] != 'A') && (command[0] != 'X')) {
        ret = (uint32_t)MeasureStart();
        if (ret != NO_ERROR) {
            printf("串口命令\t启动失败\t电机初始化错误码=0x%08lX\\r\\n", (unsigned long)ret);
            return 1U;
        }
    }
    if ((command[0] == 'L') && (command[1] == 'F')) {
        Test_EnterDebugDisplayState(&debug_display);
        ret = ServiceDebug_FixedFrequencyRun(command);
        Test_RestoreDebugDisplayState(&debug_display);
        Test_ProcessCommandWarnFailure("固定频率找液位", ret);
        printf("LF COMMAND done code=0x%08lX\r\n", (unsigned long)ret);
        return 1U;
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

/**
 * @brief 统一打印一次传感器调用结果；失败时按可选计数器累计并输出错误码。
 *
 * @param tag 用于区分诊断来源的只读标签文字。
 * @param name 用于串口测试日志标识当前测试项、阶段或通信目标的 NUL 结尾只读名称。
 * @param ret 上一层调用返回的结果码。串口测试流程只据此累计和打印测试结果，不把测试失败提升为正式测量故障。
 * @param fail_count 传感器通信测试累计失败次数。
 */
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

/**
 * @brief 检查当前传感器通信状态并输出带标签的诊断日志。
 *
 * @param tag 用于区分诊断来源的只读标签文字。
 */
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

    if (g_deviceParams.sensorType == DM4_SENSOR) {
        printf("[传感器] %s 类型=DM4(%lu)，单次通信=读取频率/密度/温度\r\n",
               tag,
               (unsigned long)g_deviceParams.sensorType);

        /* 通信自检先建立密度模式，避免把上一次液位流程的残留模式误报为通信失败。 */
        ret = SensorService_EnableDensityMode();
        Test_SensorCommPrintResult(tag, "DM4切换密度模式", ret, &comm_fail_cnt);
        if (ret != NO_ERROR) {
            return;
        }
        ret = SensorService_ReadDensity(&frequency, &density, &temp);
        Test_SensorCommPrintResult(tag, "DM4读取频率/密度/温度", ret, &comm_fail_cnt);
        if (ret == NO_ERROR) {
            printf("[传感器][正常] %s DM4 频率=%.3f Hz 密度=%.3f 温度=%.3f C\r\n",
                   tag,
                   frequency,
                   density,
                   temp);
        }
        return;
    }

    if (g_deviceParams.sensorType == LTD_SENSOR) {
        printf("[传感器] %s 类型=多参数协议V3.0/LTD(%lu)，单次通信=读取密度\r\n",
               tag,
               (unsigned long)g_deviceParams.sensorType);

        ret = (uint32_t)MULTIPARAM_V3_Read_Density(&density);
        Test_SensorCommPrintResult(tag, "多参数协议V3.0/LTD读取密度", ret, &comm_fail_cnt);
        if (ret == NO_ERROR) {
            printf("[传感器][正常] %s 多参数协议V3.0/LTD 密度=%.3f\r\n", tag, density);
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
 *
 * @param tag 用于区分诊断来源的只读标签文字。
 */
void Test_SensorCommCheckAndPrintOnly(const char *tag)
{
    DeviceState saved_state = g_measurement.device_status.device_state;
    uint32_t saved_error = g_measurement.device_status.error_code;

    Sensor_CommCheckAndLog(tag);

    g_measurement.device_status.device_state = saved_state;
    g_measurement.device_status.error_code = saved_error;
}


#include "Protocols/MultiparamV3/multiparam_v3_communication.h"
#include "Protocols/Dsm/dsm_sensor_communication.h"
#include <stdio.h>

/**
 * @brief  测试多参数传感器通信协议 V3.0通讯与关键参数读取
 * @note   可在初始化完成后调用，例如 main() 或 sensor init 后
 */
void MULTIPARAM_V3_Test_AllParams(void) {
	printf("\r\n===== 多参数传感器通信协议 V3.0 通讯测试开始 =====\r\n");

/* int ret = MULTIPARAM_V3_SwitchToLevelMode(); */
/* if (ret == NO_ERROR) */
/* printf("切换液位模式成功\r\n"); */
/* else { */
/* printf("切换液位模式失败，错误码 %d\r\n", ret); */
/* } */

	/* 2. 定义变量 */
	float temp = 0, rho = 0, mu = 0, nu = 0;
	uint32_t freq = 0, sensor_id = 0;

	/* 3. 依次读取各参数 */
/* if (MULTIPARAM_V3_Read_SoftwareVersion(&ver) == NO_ERROR) */
/* printf("软件版本: %.3f\r\n", ver); */
/* else printf("读取软件版本失败\r\n"); */

	if (MULTIPARAM_V3_Read_Temperature(&temp) == NO_ERROR) {
		printf("温度值: %.3f ℃\r\n", temp);
	} else
		printf("读取温度失败\r\n");

	if (MULTIPARAM_V3_Read_Density(&rho) == NO_ERROR) {
		printf("密度值: %.3f\r\n", rho);
	} else
		printf("读取密度失败\r\n");

	if (MULTIPARAM_V3_Read_DynamicViscosity(&mu) == NO_ERROR)
		printf("动力粘度: %.3f\r\n", mu);
	else
		printf("读取动力粘度失败\r\n");

	if (MULTIPARAM_V3_Read_KinematicViscosity(&nu) == NO_ERROR)
		printf("运动粘度: %.3f\r\n", nu);
	else
		printf("读取运动粘度失败\r\n");

    if (MULTIPARAM_V3_Read_LevelFrequency(&freq) == NO_ERROR)
        printf("液位频率: %lu Hz\r\n", (unsigned long)freq);
    else printf("读取液位频率失败\r\n");

    if (MULTIPARAM_V3_Read_SensorID(&sensor_id) == NO_ERROR)
        printf("传感器号: %lu\r\n", (unsigned long)sensor_id);
    else printf("读取传感器号失败\r\n");

	printf("===== 多参数传感器通信协议 V3.0 通讯测试结束 =====\r\n\r\n");
}
/**
 * @brief  传感器与蓝牙链路综合通信测试
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

    printf("\r\n===== 传感器与蓝牙通信测试开始 =====\r\n");

    if (Test_ProcessCommandSwitchRequested() != 0U) {
        return;
    }

    /* 识别流程先监听V4主动帧，未命中才查询蓝牙，避免主动DMA占用UART6时进入AT模式。 */
    ret = SensorService_Detect();
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
    } else if (g_deviceParams.sensorType == DM4_SENSOR) {
        /* 综合通信测试也必须显式建立业务模式，不能依赖传感器上一次运行状态。 */
        ret = SensorService_EnableDensityMode();
        if (Test_CommRecordResult("DM4切换密度模式", ret, &ok_count, &fail_count)) {
            ret = SensorService_ReadDensity(&frequency, &density, &temp);
            if ((ret == SENSOR_DATA_STALE) &&
                (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE)) {
                /* 空气中允许密度无效；交互查询已完成，综合通信测试不计故障。 */
                ok_count++;
                printf("[正常]\tDM4单次读取频率/密度/温度\t交互通信正常，当前密度无有效值（空气中允许）\r\n");
            } else if (Test_CommRecordResult("DM4单次读取频率/密度/温度", ret, &ok_count, &fail_count)) {
                printf("DM4密度数据: 频率=%.3f Hz 密度=%.3f 温度=%.3f\r\n", frequency, density, temp);
            }
        }
    } else if (g_deviceParams.sensorType == LTD_SENSOR) {
        ret = (uint32_t)MULTIPARAM_V3_Read_Density(&density);
        if (Test_CommRecordResult("多参数协议V3.0/LTD单次读取密度", ret, &ok_count, &fail_count)) {
            printf("多参数协议V3.0/LTD密度: %.3f\r\n", density);
        }
    } else {
        fail_count++;
        printf("未知传感器类型: %lu\r\n", (unsigned long)g_deviceParams.sensorType);
    }
    printf("===== 通信测试结束，成功=%lu 失败=%lu =====\r\n\r\n",
           (unsigned long)ok_count,
           (unsigned long)fail_count);
}
/**
 * @brief 依次执行 FRAM 读写、设备参数存储和硬件 CRC32 调试测试。
 */
void Test_main(void) {
	Test_FRAM_ReadWrite(); /* 测试FRAM读写 */
/* motor_text(300.0f, 0U); */
	Test_Params_Storage(); /* 测试参数存储 */
	CRC32_HAL_Test(); /* CRC校验测试 */
}
