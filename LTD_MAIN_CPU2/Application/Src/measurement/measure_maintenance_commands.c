/*
 * measure_maintenance_commands.c
 *
 * 文件职责：封装维护模式、手动运动、扭力标定和绝对位置运动命令。
 */

#include "measure_commands_internal.h"

#include <stdio.h>

#include "encoder.h"
#include "fault_manager.h"
#include "fault_recovery.h"
#include "measure_density.h"
#include "motor_ctrl.h"
#include "Relay/relay_output.h"
#include "system_parameter.h"
#include "weight.h"

/**
 * @brief 取消当前测量，停止电机并恢复可接收命令状态。
 */
void CMD_CancelMeasurement(void)
{
    uint32_t stop_ret;

    printf("执行取消当前测量指令\r\n");
    FaultRecovery_Cancel("cancel measurement");
    SiProfile_HandleCancel();

    /*
     * 用户主动取消不是故障。待执行取消命令已由主循环原子取走；
     * 此处只清当前执行态，不能覆盖取消期间并发到达的下一条命令。
     */
    g_measurement.device_status.current_command = CMD_NONE;
    if ((!Encoder_HasLatchedFault()) &&
        (g_measurement.device_status.error_code == STATE_SWITCH)) {
        g_measurement.device_status.error_code = NO_ERROR;
    }

    stop_ret = MotorCtrl_SlowStop();
    /* 取消测量时慢停失败只记录诊断；除命令切换外，取消流程仍清理命令并进入待机，避免界面长期卡在运行态。 */
    if ((stop_ret != NO_ERROR) && (stop_ret != STATE_SWITCH)) {
        printf("取消测量\t停止电机返回：0x%08lX\r\n", (unsigned long)stop_ret);
    }

    g_measurement.device_status.device_state = STATE_STANDBY;
    printf("取消测量\t设备已进入待机\r\n");
}

/**
 * @brief 进入维护模式并停止当前自动测量动作。
 */
void CMD_EnterMaintenanceMode(void)
{
    /* 维护模式只叠加报警输出屏蔽，不占用设备状态，也不阻塞后续指令和测量。 */
    g_measurement.device_status.maintenance_mode_active = 1U;
    printf("维护模式已开启\r\n");
}

/**
 * @brief 退出非阻塞维护模式。
 * @note 只撤销维护屏蔽，不改写当前设备状态、故障码或正在执行的测量。
 */
void CMD_ExitMaintenanceMode(void)
{
    g_measurement.device_status.maintenance_mode_active = 0U;
    printf("维护模式已退出\r\n");
}

/**
 * @brief 请求清除四路继电器锁存报警。
 * @note 请求只作用于运行态，由继电器更新周期消费，不触发参数持久化。
 */
void CMD_ClearAllRelayLatchedAlarms(void)
{
    RelayOutput_RequestClearAllLatchedAlarms();
    printf("已请求清除全部继电器锁存报警\r\n");
}

/**
 * @brief 按命令参数指定的距离和默认速度执行手动上行，并在动作期间抑制自动报警与液位更新。
 *
 * @note 无论动作正常完成、发生命令切换还是返回错误，退出前都要清除手动报警和液位更新抑制标志。
 */
void CMD_MoveUp(void)
{
    uint32_t ret = 0;

    printf("电机上行操作\n");
    /* 手动上行由外部协议触发时，不应被自动报警/液位跟随逻辑误判为自动测量动作。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_RUNUPING;

    ret = MotorCtrl_MoveAndWait(
            (float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE) / 10.0f,
            MOTOR_DIRECTION_UP,
            MotorCtrl_GetDefaultSpeedX100());

    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }

    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
    g_measurement.device_status.device_state = STATE_RUNUPOVER;
    return;
}

/**
 * @brief 按命令参数指定的距离和默认速度执行手动下行，并在动作期间抑制自动报警与液位更新。
 *
 * @note 无论动作正常完成、发生命令切换还是返回错误，退出前都要清除手动报警和液位更新抑制标志。
 */
void CMD_MoveDown(void)
{
    uint32_t ret = 0;

    printf("电机下行操作\n");
    /* 手动下行同样设置抑制位，CPU3 据此把 SI 报警/液位更新状态与自动测量隔离。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_RUNDOWNING;

    ret = MotorCtrl_MoveAndWait(
            (float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE) / 10.0f,
            MOTOR_DIRECTION_DOWN,
            MotorCtrl_GetDefaultSpeedX100());

    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }

    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
    g_measurement.device_status.device_state = STATE_RUNDOWNOVER;
    return;
}

/**
 * @brief 电机强制上行指令（无检测）。
 */
void CMD_ForceMoveUp(void)
{
    uint32_t ret;

    printf("电机强制上行操作\r\n");
    /* 强制运行无检测，必须显式告诉 CPU3 当前液位/报警状态不应作为自动流程结果。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_FORCE_RUNUPING;
    printf("强制上行距离: %.1f mm\r\n", (float) DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE) / 10.0f);
    /* 强制调试运动允许忽略编码器首帧未就绪，但不绕过驱动安全检查。 */
    ret = MotorCtrl_MoveBlockingNoDetectForceDebug(
        (float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE) / 10.0f,
        MOTOR_DIRECTION_UP,
        MotorCtrl_GetDefaultSpeedX100());
    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }
    printf("电机强制上行操作完成\r\n");
    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
/* MotorCtrl_MoveAndWait( */
/* (float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE) / 10.0f, */
/* MOTOR_DIRECTION_UP); */
    g_measurement.device_status.device_state = STATE_FORCE_RUNUP_OVER;
    return;
}

/**
 * @brief 电机强制下行指令（无检测）。
 */
void CMD_ForceMoveDown(void)
{
    uint32_t ret;

    printf("电机强制下行操作\r\n");
    /* 强制下行属于手动动作，协议辅助状态只反映抑制语义，不改变原电机控制流程。 */
    g_measurement.device_status.manual_alarm_inhibit = 1U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 1U;
    g_measurement.device_status.device_state = STATE_FORCE_RUNDOWNING;

    /* 强制调试运动允许忽略编码器首帧未就绪，但不绕过驱动安全检查。 */
    ret = MotorCtrl_MoveBlockingNoDetectForceDebug(
        (float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_MOTOR_COMMAND_DISTANCE) / 10.0f,
        MOTOR_DIRECTION_DOWN,
        MotorCtrl_GetDefaultSpeedX100());
    if (ret == STATE_SWITCH) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        return;
    }
    if (ret != NO_ERROR) {
        g_measurement.device_status.manual_alarm_inhibit = 0U;
        g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
        SET_ERROR(ret);
    }

    g_measurement.device_status.manual_alarm_inhibit = 0U;
    g_measurement.oil_measurement.manual_level_update_inhibit = 0U;
    g_measurement.device_status.device_state = STATE_FORCE_RUNDOWN_OVER;
    return;
}

/**
 * @brief 切换至空载扭力采集状态，完成稳定等待、范围校验和参数保存后发布完成状态。
 *
 * 底层采集等待五秒使扭力稳定，将当前原始扭力作为空载值，并在绝对值不超过允许上限时写入设备参数和 FRAM。
 * 采集、命令切换或范围校验结果通过 SET_ERROR 进入统一错误处理。
 */
void CMD_SetEmptyWeight(void)
{
    uint32_t ret = 0;

    printf("执行设置空载扭力指令\n");
    g_measurement.device_status.device_state = STATE_GET_EMPTYWEIGHT;

    ret = get_empty_weight();

    if (ret == STATE_SWITCH) {
        /* 命令切换属于正常退出，不发布本次无效的空载采集完成状态。 */
        return;
    }
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_GET_EMPTYWEIGHT_OVER;
    return;
}

/**
 * @brief 切换至满载扭力采集状态，完成稳定等待、范围校验和参数保存后发布完成状态。
 *
 * 底层采集等待五秒使扭力稳定，将当前扭力作为满载值，并在最小值和最大值范围内写入设备参数和 FRAM。
 * 采集、命令切换或范围校验结果通过 SET_ERROR 进入统一错误处理。
 */
void CMD_SetFullWeight(void)
{
    uint32_t ret = 0;

    printf("执行设置满载扭力指令\n");
    g_measurement.device_status.device_state = STATE_GET_FULLWEIGHT;

    ret = get_full_weight();

    if (ret == STATE_SWITCH) {
        /* 命令切换属于正常退出，不发布本次无效的满载采集完成状态。 */
        return;
    }
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_GET_FULLWEIGHT_OVER;
    return;
}

/**
 * @brief 运行到指定绝对位置（mm）
 * 依赖：
 *  - MotorCtrl_JogMoveToPosition(float target_mm, uint32_t speed_x100)
 *  - CHECK_COMMAND_SWITCH(x) / SET_ERROR(x)
 *  - g_measurement.device_status.device_state
 *  - 目标位置参数来源（见下方 get_target_mm()）
 */
void CMD_RunToPosition(void)
{
    uint32_t ret = NO_ERROR;
    float target_mm = 0.0f;

    g_measurement.device_status.device_state = STATE_RUN_TO_POSITIONING;


    target_mm = (float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_DENSITY_DISTRIBUTION_OIL_LEVEL)/10.0;
    ret = MotorCtrl_JogMoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());

    /* MotorCtrl_JogMoveToPosition 里如果你也加了 CHECK_COMMAND_SWITCH，就能更快退出；
       若没加，这里至少在调用前/后能响应一次切换。 */

    if (ret == STATE_SWITCH) {
        printf("运行到指定位置\t命令切换，中止\r\n");
        /* 中止：通常不记为错误，回到待机或保持上层状态机处理 */
        g_measurement.device_status.device_state = STATE_STANDBY;
        return;
    }

    if (ret != NO_ERROR) {
        printf("运行到指定位置\t失败 错误码：0x%lX\r\n", ret);
        SET_ERROR(ret);
        g_measurement.device_status.device_state = STATE_ERROR;
        return;
    }

    /* 3) 成功完成 */
    printf("运行到指定位置\t完成\r\n");
    g_measurement.device_status.device_state = STATE_RUN_TO_POSITION_OVER;
}
