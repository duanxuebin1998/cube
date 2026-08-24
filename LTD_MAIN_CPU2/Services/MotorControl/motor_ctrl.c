/**
 * @file motor_ctrl.c
 * @brief 电机控制模块的共享运行态定义。
 *
 * 本文件只保存跨功能域确实需要共享的运行态，例如当前下发到 TMC5130 的 VMAX、
 * 驱动初始化标记，以及电机记步模式的切换基准。具体业务逻辑按职责放在同目录其它
 * motor_ctrl_*.c 文件中，避免主文件继续堆叠。
 */

#include "motor_ctrl_internal.h"

/* ===================== 私有类型/状态 ===================== */

/* 全局速度：由 MotorDriver_UpdateVelocityFromParams() 根据参数动态更新。 */
uint32_t velocity = MOTOR_VELOCITY_BASE;

/* 驱动运行态：集中保存驱动侧跨文件共享状态。 */
MotorDriverRuntime s_motor_driver = {
    .applied_velocity = 0U,
    .initialized = false,
    .motion_command_active = false,
    .motion_wait_active = false,
    .continuous_velocity_active = false,
    .continuous_velocity_start_tick = 0U,
    .boot_safe_stop_done = false,
};

/* 位置运行态：集中保存电机记步基准。 */
MotorPositionRuntime s_motor_position = {
    .count_base_step = 0,
    .count_base_length_01mm = 0,
    .count_base_turns = 0.0,
};
