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
#define VELOCITY_MAX 2
//丢步检测参数
#define LOST_STEP_WINDOW        3       // 检测窗口（3次）
#define LOST_STEP_THRESHOLD     2    // 3秒内总位移小于0.2mm判定丢步
#define LOST_STEP_INTERVAL_MS   1000    // 每1秒检测一次
#define LOST_STEP_SUPPRESS_MS   1000    // 报警后1秒内不重复报警
// --- 内部状态变量 ---
static int pos_buf[LOST_STEP_WINDOW];
static int write_idx = 0;
static int samples = 0;
static uint32_t last_check_tick = 0;
static uint32_t last_alarm_tick = 0;
uint32_t velocity = 1600 * VELOCITY_MAX * 32;

static bool stpr_isMoving(TMC5130TypeDef *tmc5130);
static uint32_t stpr_checkGstat(TMC5130TypeDef *tmc5130);
static uint32_t stpr_wait_until_stop(TMC5130TypeDef *tmc5130);

uint32_t motor_Init(void) {
	stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 19);
	stpr_enableDriver(&stepper);
	return NO_ERROR; // 初始化电机，返回无错误状态
}

// 控制电机移动函数
uint32_t motorMoveNoWait(float mm, int dir) {
	if (mm < 0) {
		return NO_ERROR; // 距离必须为正值，直接返回 duan
	}
	printf("start move %.2fmm, dir %d\n", mm, dir);
	// 计算对应的步数
	int32_t ticks = (int32_t) ((mm * 30 * 1600 * 32) / 600); // 转换为步数
	if (dir == MOTOR_DIRECTION_UP) {
		ticks = -ticks; // 如果方向向上，步数取负值
	}

	// 调用实际的步进电机控制函数
	stpr_moveBy(&stepper, &ticks, velocity); // 最大速度为 1600 * 2 * 32
	return NO_ERROR; // 返回无错误状态
}
uint32_t motorMoveAndWaitUntilStop(float mm, int dir) {
	uint32_t ret;
	float targetPos, actualPos;
	float diff;
	float startPos = (float) g_measurement.debug_data.sensor_position / 10.0f;
	targetPos = startPos + (dir > 0 ? mm : -mm);
	// 启动运动
	motorMoveNoWait(mm, dir);
	HAL_Delay(1000);
	// 等待电机运动完成
	ret = stpr_wait_until_stop(&stepper);
	CHECK_ERROR(ret); // 等待电机移动完成，并检查是否有错误
	// 计算实际移动距离
	actualPos = (float) g_measurement.debug_data.sensor_position / 10.0f;
	// 计算位置偏差
	diff = 100 * (fabsf(actualPos - startPos) - mm) / mm;
	// 丢步判断（可根据机械分辨率调整阈值）
	if ((diff > 70) && (mm > 2.0)) { // 超过0.5mm认为丢步
		printf("⚠️ 警告：检测到电机可能丢步！目标=%.2f mm, 实际=%.2f mm, 差值=%.2f %%\r\n", mm, fabsf(actualPos - startPos), diff);
		return ENCODER_LOST_STEP; // 自定义错误码
	} else {
		printf("✅ 电机移动完成。目标=%.2f mm, 实际=%.2f mm, 差值=%.2f %%\r\n", mm, fabsf(actualPos - startPos), diff);
	}
	return NO_ERROR; // 等待电机移动完成
}
/**
 * @brief 电机带回差补偿的移动函数
 *        上行：正常走
 *        下行：多走20mm，再往上走20mm克服回差
 * @param mm   目标移动距离 (mm)
 * @param dir  方向：0=下行，1=上行
 * @return uint32_t 错误码
 */
uint32_t motorMoveWithBacklash(float mm, int dir) {
	uint32_t ret;

	if (dir == MOTOR_DIRECTION_UP) {
		// 上行：正常走
		ret = motorMoveAndWaitUntilStop(mm, dir);
		CHECK_ERROR(ret);
	} else {
		// 下行：先比目标多走20mm
		ret = motorMoveAndWaitUntilStop(mm + 20.0f, dir);
		CHECK_ERROR(ret);

		// 再往上走20mm，消除回差
		ret = motorMoveAndWaitUntilStop(20.0f, 1);
		CHECK_ERROR(ret);
	}

	return NO_ERROR;
}

uint32_t motorMove_up(void) {
	motorMoveNoWait(100000, MOTOR_DIRECTION_UP);
	MotorLostStep_Init(); // 初始化丢步检测
	return NO_ERROR; // 向上移动，返回无错误状态
}
uint32_t motorMove_down(void) {
	motorMoveNoWait(100000, MOTOR_DIRECTION_DOWN);
	MotorLostStep_Init(); // 初始化丢步检测
	return NO_ERROR; // 向下移动，返回无错误状态
}
void motorRun(float distance_mm,      		// 移动距离（毫米）
		int direction,          			// 运动方向
		bool enable_step_loss_detection,  	// 是否启用丢步检测
		bool enable_weight_detection      	// 是否启用称重检测
		) {

}

/**
 * @brief 判断电机是否正在运动
 * @param tmc5130 指向驱动器结构体
 * @return true 表示仍在运动；false 表示已停止
 */
static bool stpr_isMoving(TMC5130TypeDef *tmc5130) {
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
 * @note 旧名称 stpr_waitMove 已被替换为 stpr_wait_until_stop
 */
static uint32_t stpr_wait_until_stop(TMC5130TypeDef *tmc5130) {
	uint32_t ret = NO_ERROR;
	uint32_t startTick = HAL_GetTick();
	const uint32_t MAX_WAIT_MS = 60000 * 60; // 超时时间：60s*60，可按需调整

	while (stpr_isMoving(tmc5130)) {
		// 碰撞检测（含日志与错误上报）
		CheckWeightCollision();
		CHECK_ERROR(ret);

		// 检查并处理 TMC5130 GSTAT 状态
		ret = stpr_checkGstat(tmc5130);
		CHECK_ERROR(ret);

		// 超时保护
		if (HAL_GetTick() - startTick > MAX_WAIT_MS) {
			printf("TMC5130: 等待停止超时！\r\n");
			CHECK_ERROR(MOTOR_RUN_TIMEOUT);
		}

		HAL_Delay(50);
	}

	printf("TMC5130: 已停止。\r\n");
	return NO_ERROR;
}

void motor_text(void) {
	printf("motor text start\n");
	while (1) {
		stpr_enableDriver(&stepper); //使能电机
//		stpr_initStepper(&stepper, &hspi2, GPIOB, GPIO_PIN_12, 1, 18);
		motorMoveNoWait(1000, MOTOR_DIRECTION_DOWN);
		HAL_Delay(1000);
		printf("start down\n");
		stpr_waitMove(&stepper);
		printf("down over!\n");
		HAL_Delay(1000);
		stpr_moveTo(&stepper, 0, velocity);
		printf("start up to zero\n");
		HAL_Delay(1000);
		stpr_waitMove(&stepper);
		printf("上行结束\n");
		HAL_Delay(1000);
		stpr_disableDriver(&stepper); //使能电机
	}
}

//电机紧急刹车
uint32_t motorQuickStop(void) {
//	printf("急刹前\t{传感器位置}%.1f\t{称重值}%d\r\n", (float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
	if (stpr_isMoving(&stepper)) {
		stpr_stop(&stepper);
		stpr_disableDriver(&stepper); //使能电机
		HAL_Delay(1000);
		HAL_Delay(1000);
		HAL_Delay(1000);
		HAL_Delay(1000);
		stpr_enableDriver(&stepper); //使能电机
//	printf("急刹后\t{传感器位置}%.1f\t{称重值}%d\r\n", (float)(g_measurement.debug_data.sensor_position)/10.0, weight_parament.current_weight);
	}
	return NO_ERROR; // 返回无错误状态
}

//电机停止
uint32_t motorSlowStop(void) {
	stpr_stop(&stepper);

	return NO_ERROR; // 返回无错误状态
}

/**
 * @brief  丢步检测初始化
 *         在每次新的运动阶段开始前调用。
 */
void MotorLostStep_Init(void) {
	write_idx = 0;
	samples = 0;
	last_check_tick = 0;
	last_alarm_tick = 0;
	for (int i = 0; i < LOST_STEP_WINDOW; i++) {
		pos_buf[i] = 0.0f;
	}
}
/**
 * @brief  连续调用丢步检测函数（高频调用也可）
 * @param  currentPos 当前电机位置(mm)
 * @retval 0 - 正常；1 - 检测到丢步（并触发报警）
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
	if ((total_movement < LOST_STEP_THRESHOLD) // 超过阈值判定丢步
	&& (g_measurement.debug_data.cable_length > 1000) //零点附近不检测
			&& (g_measurement.debug_data.cable_length < bottom_value - 1000) //盲区以上检测
			)

			{
		if (now - last_alarm_tick >= LOST_STEP_SUPPRESS_MS) {
			last_alarm_tick = now;
			printf("⚠️ 电机丢步报警：3秒内总移动 %.3f mm < %.3f mm\n", (float) total_movement / 10.0, (float) LOST_STEP_THRESHOLD / 10.0);
			return ENCODER_LOST_STEP;
		}
	} else {
		printf("丢步检测\t电机运行正常：当前电机位置：%.1f mm\t3秒内总移动 %.3f mm\n", (float) currentPos / 10.0, (float) total_movement / 10.0);
	}

	return NO_ERROR;
}
