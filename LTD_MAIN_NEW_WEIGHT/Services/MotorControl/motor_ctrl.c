/*
 * motor_ctrl.c
 *
 *  Created on: Jan 18, 2025
 *      Author: 1
 */

#include "motor_ctrl.h"
#include "fault_manager.h"
#include "spi.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure_tank_height.h"
#include <math.h>
#include <inttypes.h>

// ===== 可调参数 =====
#define EPS_MM                   0.20f    // 到位公差 (±)
#define VELOCITY_MAX 2
// 丢步检测参数
#define LOST_STEP_WINDOW        3       // 检测窗口（3次）
#define LOST_STEP_THRESHOLD     2       // 3秒内总位移小于0.2mm判定丢步
#define LOST_STEP_INTERVAL_MS   1000    // 每1秒检测一次
#define LOST_STEP_SUPPRESS_MS   1000    // 报警后1秒内不重复报警
#define MOTOR_MOVE_RETRY_MAX    3       // 最大重试次数，包含初次+补偿重跑

// --- 内部状态变量 ---
static int pos_buf[LOST_STEP_WINDOW];
static int write_idx = 0;
static int samples = 0;
static uint32_t last_check_tick = 0;
static uint32_t last_alarm_tick = 0;
uint32_t velocity = 1600 * VELOCITY_MAX * 32;

static uint32_t stpr_checkGstat(TMC5130TypeDef *tmc5130);
static uint32_t stpr_wait_until_stop(TMC5130TypeDef *tmc5130);

uint32_t motor_Init(void) {
    stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 14);
    stpr_enableDriver(&stepper);
    return NO_ERROR; // 初始化电机，返回无错误状态
}

// 控制电机移动函数
uint32_t motorMoveNoWait(float mm, int dir) {
    if (mm < 0) {
        return MOTOR_STEP_ERROR; // 距离必须为正值，直接返回
    }
    g_measurement.debug_data.motor_state = dir; // 记录最后一次移动距离
    printf("start move %.2fmm, dir %d\n", mm, dir);
    // 计算对应的步数
    int32_t ticks = (int32_t)((mm * 30 * 1600 * 32) / 600); // 转换为步数
    if (dir == MOTOR_DIRECTION_UP) {
        ticks = -ticks; // 如果方向向上，步数取负值
    }

    // 调用实际的步进电机控制函数
    stpr_moveBy(&stepper, &ticks, velocity); // 最大速度为 1600 * 2 * 32
    return NO_ERROR; // 返回无错误状态
}

uint32_t motorMoveAndWaitUntilStop(float mm, int dir) {
    uint32_t ret;
    float startPos_mm;
    float currentPos_mm;
    float targetPos_mm;
    float total_cmd_mm = mm;    // 总共期望走的距离 (正数)
    float moved_mm;             // 已走的距离 (正数)
    float remain_mm;            // 剩余还要走的距离 (正数)
    uint8_t attempt = 0;

    // 记录运动开始时的绝对起点位置 (mm)
    startPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
    targetPos_mm = startPos_mm + (dir > 0 ? total_cmd_mm : -total_cmd_mm);

    // 剩余路程初始化为整段
    remain_mm = total_cmd_mm;

    while (attempt < MOTOR_MOVE_RETRY_MAX && remain_mm > 0.0f) {

        attempt++;

        // 下发当前这段需要走的距离（remain_mm，方向同 dir）
        motorMoveNoWait(remain_mm, dir);

        // 给驱动一点时间起步，避免立即 wait 导致误判
        HAL_Delay(1000);

        // 阻塞等待步进电机完成当前 attempt 的运动
        ret = stpr_wait_until_stop(&stepper);

        // 读取当前位置，计算已走距离和剩余距离
        currentPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
        moved_mm = fabsf(currentPos_mm - startPos_mm); // 从最初起点算累计位移
        remain_mm = total_cmd_mm - moved_mm;
        if (remain_mm < 0.0f) {
            // 过冲(overshoot)理论上不常见，但保护一下，避免负数导致再次下发
            remain_mm = 0.0f;
        }

        if (ret == NO_ERROR) {
            // 这一段走完时并未报错，可能已经到位，也可能还有剩余没走
            printf("电机段运行成功: attempt=%d, 累计位移=%.2f mm, 剩余=%.2f mm\r\n",
                   attempt, moved_mm, remain_mm);
            break; // 成功走完，直接返回
        } else {
            // 这一段出现了故障，stpr_wait_until_stop() 已经让电机停下
            printf("警告：电机在第%d次运行过程中异常停止，错误0x%X\r\n",
                   attempt, (unsigned int)ret);

            // 如果还有尝试机会，我们继续循环，会用剩余距离 remain_mm 再跑
            if (attempt >= MOTOR_MOVE_RETRY_MAX) {
                // 没有重试机会了，直接上报这个错误
                CHECK_ERROR(ret); // 统一故障处理机制
                return ret;
            } else {
                // 还有重试机会：打印一下准备补偿
                printf("尝试继续补偿剩余距离 %.2f mm (方向=%d)\r\n", remain_mm, dir);
                HAL_Delay(200); // 稍作延时
            }
        }

        // while 会继续，直到 remain_mm == 0 或 attempt 用完
    }

    // 最终位置再读一次，进行丢步评估
    currentPos_mm = (float)g_measurement.debug_data.sensor_position / 10.0f;
    moved_mm = fabsf(currentPos_mm - startPos_mm);

    // diff 计算为百分比误差
    // diff_pct = ((实际位移 - 期望位移) / 期望位移) * 100%
    float diff_pct = 100.0f * (moved_mm - total_cmd_mm) / total_cmd_mm;

    // 丢步判断：实际少走了 70% 以上（只走了不到 30%）
    if ((diff_pct < -70.0f) && (total_cmd_mm > 2.0f)) {
        printf("警告：检测到电机可能丢步！\r\n");
        printf("目标=%.2f mm, 实际=%.2f mm, 误差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        return ENCODER_LOST_STEP;
    } else {
        printf("电机移动完成。\r\n");
        printf("期望总位移=%.2f mm, 实际=%.2f mm, 偏差=%.2f %%\r\n",
               total_cmd_mm, moved_mm, diff_pct);
        printf("起点=%.2f mm, 目标位置=%.2f mm, 最终位置=%.2f mm\r\n",
               startPos_mm, targetPos_mm, currentPos_mm);
    }

    return NO_ERROR;
}



uint32_t motorMove_up(void) {
    motorMoveNoWait(100000, MOTOR_DIRECTION_UP);
    MotorLostStep_Init(); // 初始化丢步检测
    return NO_ERROR;
}

uint32_t motorMove_down(void) {
    motorMoveNoWait(100000, MOTOR_DIRECTION_DOWN);
    MotorLostStep_Init(); // 初始化丢步检测
    return NO_ERROR;
}

void motorRun(float distance_mm,            // 移动距离（毫米）
              int direction,                // 运动方向
              bool enable_step_loss_detection,  // 是否启用丢步检测
              bool enable_weight_detection      // 是否启用称重检测
             ) {
    // TODO: 实现综合运动逻辑
}

/**
 * @brief 判断电机是否正在运动
 * @param tmc5130 指向驱动器结构体
 * @return true 表示仍在运动；false 表示已停止
 */
bool stpr_isMoving(TMC5130TypeDef *tmc5130) {
    uint32_t rampstat = stpr_readInt(tmc5130, TMC5130_RAMPSTAT);
    return ((rampstat & 0x400) != 0x400); // bit10=1 表示停止
}

/**
 * @brief 检查 TMC5130 的 GSTAT 状态寄存器并处理错误
 * @param tmc5130 指向驱动器结构体
 * @return 错误码
 */
static uint32_t stpr_checkGstat(TMC5130TypeDef *tmc5130) {
    uint32_t gstat = stpr_readInt(tmc5130, TMC5130_GSTAT);
    uint32_t ret = NO_ERROR;

    if (gstat == 0)
        return NO_ERROR;

    if (gstat & (1 << 0)) {
        printf("TMC5130: 芯片复位检测到（bit0=1）\n");
    }

    if (gstat & (1 << 1)) {
        printf("TMC5130: 驱动器因过热或短路被关闭\n");
        ret = MOTOR_OVERTEMPERATURE;
    }

    if (gstat & (1 << 2)) {
        printf("TMC5130: Charge pump 欠压\n");
        ret = MOTOR_CHARGE_PUMP_UNDER_VOLTAGE;
    }

    // 清除状态位
    stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07);

    if (ret != NO_ERROR)
        CHECK_ERROR(ret);

    return ret;
}

/**
 * @brief 等待电机停止（阻塞），期间检测碰撞与芯片异常
 * @param tmc5130 指向 TMC5130 驱动结构体
 * @return 错误码（NO_ERROR 表示正常）
 */
static uint32_t stpr_wait_until_stop(TMC5130TypeDef *tmc5130) {
    uint32_t ret = NO_ERROR;
    uint32_t startTick = HAL_GetTick();
    const uint32_t MAX_WAIT_MS = 60000 * 60; // 超时时间：60s*60，可按需调整

    while (stpr_isMoving(tmc5130)) {
        // 碰撞检测（含日志与错误上报）
        ret = CheckWeightCollision();
        CHECK_ERROR(ret);

        // 检查并处理 TMC5130 GSTAT 状态
        ret = stpr_checkGstat(tmc5130);
        CHECK_ERROR(ret);

        // 超时保护
        if (HAL_GetTick() - startTick > MAX_WAIT_MS) {
            printf("TMC5130: 等待停止超时！\r\n");
            RETURN_ERROR(MOTOR_RUN_TIMEOUT);
        }

        HAL_Delay(50);
    }

    printf("TMC5130: 已停止。\r\n");
    return NO_ERROR;
}

// 电机紧急刹车
uint32_t motorQuickStop(void) {
    if (stpr_isMoving(&stepper)) {
        stpr_stop(&stepper);
        stpr_disableDriver(&stepper); // 关闭驱动
        HAL_Delay(1000);
        HAL_Delay(1000);
        HAL_Delay(1000);
        HAL_Delay(1000);
        stpr_enableDriver(&stepper); // 重新使能
    }
    return NO_ERROR;
}

// 电机停止
uint32_t motorSlowStop(void) {
    stpr_stop(&stepper);
    return NO_ERROR;
}

/**
 * @brief 丢步检测初始化
 *        在每次新的运动阶段开始前调用。
 */
void MotorLostStep_Init(void) {
    write_idx = 0;
    samples = 0;
    last_check_tick = 0;
    last_alarm_tick = 0;
    for (int i = 0; i < LOST_STEP_WINDOW; i++) {
        pos_buf[i] = 0;
    }
}

/**
 * @brief 连续调用丢步检测函数（高频调用也可）
 * @param currentPos 当前电机位置(0.1mm)
 * @retval 0 - 正常；非0 - 检测到丢步（并触发报警）
 */
uint32_t Motor_CheckLostStep_AutoTiming(int32_t currentPos) {
    uint32_t now = HAL_GetTick();

    // 若未到检测周期（<1秒），直接返回
    if (now - last_check_tick < LOST_STEP_INTERVAL_MS)
        return NO_ERROR;

    // 更新检测时间戳
    last_check_tick = now;

    // 记录当前位置（1秒一次）
    pos_buf[write_idx] = currentPos;
    write_idx = (write_idx + 1) % LOST_STEP_WINDOW;
    if (samples < LOST_STEP_WINDOW)
        samples++;

    // 样本不足3次不检测
    if (samples < LOST_STEP_WINDOW)
        return NO_ERROR;

    // 计算最近3次检测的总位移
    int total_movement = 0;
    int oldest_idx = write_idx;
    for (int k = 0; k < LOST_STEP_WINDOW - 1; k++) {
        int idx_a = (oldest_idx + k) % LOST_STEP_WINDOW;
        int idx_b = (oldest_idx + k + 1) % LOST_STEP_WINDOW;
        total_movement += abs(pos_buf[idx_b] - pos_buf[idx_a]);
    }

    // 丢步判断 + 报警抑制
    if ((total_movement < LOST_STEP_THRESHOLD) &&
        (g_measurement.debug_data.cable_length > 1000) &&
        (g_measurement.debug_data.cable_length < bottom_value - 1000)) {

        if (now - last_alarm_tick >= LOST_STEP_SUPPRESS_MS) {
            last_alarm_tick = now;
            printf("电机丢步报警：3秒内总移动 %.3f mm < %.3f mm\n",
                   (float)total_movement / 10.0f,
                   (float)LOST_STEP_THRESHOLD / 10.0f);
            return ENCODER_LOST_STEP;
        }
    } else {
        printf("丢步检测 电机运行正常：当前电机位置：%.1f mm 3秒内总移动 %.3f mm\n",
               (float)currentPos / 10.0f,
               (float)total_movement / 10.0f);
    }

    return NO_ERROR;
}

// 读取位置快照（单位：mm）。如有需要可加临界区保护。
static inline void snapshot_sensor_pos_mm(float *pos_mm) {
    uint32_t pos_01mm = (uint32_t)g_measurement.debug_data.sensor_position; // 0.1mm
    *pos_mm = (float)pos_01mm / 10.0f;
}

/**
 * @brief 一次下发移动到绝对位置（阻塞），靠反馈提前停
 * @param target_mm  目标位置（绝对 mm）
 * @return           NO_ERROR / 错误码
 */
uint32_t motorMoveToPositionOneShot(float target_mm) {
    uint32_t ret;
    float start_mm;
    for (int i = 0; i < 5; i++) {

        snapshot_sensor_pos_mm(&start_mm);
        printf("Motor move to position: start=%.3fmm, target=%.3fmm\r\n",
               start_mm, target_mm);

        // 已在目标附近
        float delta = target_mm - start_mm;
        if (fabsf(delta) <= EPS_MM) {
            printf("Already in position: cur=%.3fmm ~ target=%.3fmm (±%.2fmm)\r\n",
                   start_mm, target_mm, EPS_MM);
            return NO_ERROR;
        }

        int dir = (delta > 0.0f) ? +1 : -1;
        float plan_mm = fabsf(delta);
        if (plan_mm < (EPS_MM * 2.0f))
            plan_mm = (EPS_MM * 2.0f); // 最小下发距离

        ret = motorMoveAndWaitUntilStop(plan_mm, dir);
        CHECK_ERROR(ret);
    }
    return NO_ERROR;
}

/**
 * @brief 相对位移一次下发（阻塞），内部换算为绝对目标后调用
 * @param mm   相对位移（正数=上行/正向，负数=下行/反向）
 */
uint32_t motorMoveRelativeOneShot(float mm) {
    float cur;
    snapshot_sensor_pos_mm(&cur);
    return motorMoveToPositionOneShot(cur + mm);
}
