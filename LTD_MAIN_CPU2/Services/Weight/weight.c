/*
 * weight.c
 *
 *  Created on: Jan 18, 2025
 *      Author: Duan Xuebin
 *  该文件实现了扭力传感器的相关操作，包括获取空载扭力、获取稳定扭力、判断扭力状态等功能。
 */

#include "main.h"
#include "weight.h"

#include "stdio.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "AS5145.h"

#include "motor_ctrl.h"
#include "error_log.h"
#include "system_parameter.h"
#include "abortable_delay.h"
#define WEIGHT_DEBUG /* 扭力处理参数：扭力 调试。 */
#define MAX_WEIGHT 20000 /* 最大扭力限制 */
#define MIN_WEIGHT -2000 /* 最小扭力限制 */
#define MAX_EMPTY_WEIGHT 20000 /* 最大扭力限制 */
/* 采样和更新周期定义 */
#define WEIGHT_SAMPLE_INTERVAL 200 /* 扭力处理参数：扭力 SAMPLE 间隔。 */
#define WEIGHT_FILTER_OLD_FACTOR 8 /* 扭力一阶滤波旧值权重。 */
#define WEIGHT_FILTER_NEW_FACTOR 2 /* 扭力一阶滤波新值权重。 */
#define WEIGHT_FILTER_DIVISOR    (WEIGHT_FILTER_OLD_FACTOR + WEIGHT_FILTER_NEW_FACTOR) /* 扭力一阶滤波权重除数。 */
#define WEIGHT_COMM_TIMEOUT_MS 1000U /* 扭力处理参数：扭力 通信 超时 毫秒。 */
#define TORQUE_TEMPERATURE_INVALID_BITS 0x7FC00000UL /* 尚未收到有效温度时发布安静NaN位模式。 */

/* 全局变量，存储当前扭力传感器的原始扭力值 */
int16_t g_weight; /* 最近一次通过称重通信校验的原始扭力/重量计数。 */

/* 全局结构体，存储扭力相关参数（空载、稳定、当前扭力等） */
Weight_ParamentTypeDef weight_parament = { 0 };
static uint32_t s_weight_last_rx_tick = 0U; /* 最近一次收到有效称重数据的 HAL 毫秒节拍。 */
static volatile uint32_t s_weight_frame_sequence = 0U; /* 有效帧由接收中断递增，供过程区分新旧称重；自然回绕。 */
static uint8_t s_weight_timeout_reported = 0U; /* 当前称重通信超时已经上报、等待有效数据恢复的标志。 */

/**
 * @brief 判断当前是否处于允许零点搜索的设备状态。
 * @return true 表示零点搜索态，false 表示其他运行状态。
 */
static uint8_t Weight_IsZeroSearchState(void)
{
    DeviceState state = g_measurement.device_status.device_state;

    return (state == STATE_BACKZEROING) ||
           (state == STATE_FINDZEROING);
}

/**
 * @brief 判断错误码是否属于扭力模块通信故障。
 *
 * @param error_code 待分类的整机错误码；函数只识别扭力通信超时、扭力帧校验失败和扭力数据无效三类代码。
 * @return true 表示错误码属于扭力模块通信超时、帧校验或数据失效故障；否则返回 false。
 */
static uint8_t Weight_IsCommErrorCode(uint32_t error_code) {
	return (error_code == WEIGHT_COMM_TIMEOUT);
}

/**
 * @brief 打印当前尺带扭力参考值和标定状态。
 *
 * @param cable_mm 当前尺带长度，单位 mm，用于换算并打印扭力参考值。
 */
static void Weight_PrintCableRefs(float cable_mm)
{
	printf("尺带长度 : %.1f mm", cable_mm);
	MotorCtrl_PrintPositionRefs();
	printf("\r\n");
}

/**
 * @brief 从设备参数加载空载和满载扭力，并初始化扭力通信有效性状态。
 *
 * @return 固定返回 NO_ERROR；当前初始化过程只复制参数并复位运行态，不包含可失败的硬件访问。
 */
uint32_t weight_init() {
	weight_parament.empty_weight = g_deviceParams.empty_weight;           /* 从设备参数中获取空载扭力 */
	weight_parament.full_weight = g_deviceParams.full_weight;           /* 从设备参数中获取满载扭力 */
	g_measurement.debug_data.torque_temperature_bits = TORQUE_TEMPERATURE_INVALID_BITS;
	s_weight_last_rx_tick = HAL_GetTick();
	s_weight_timeout_reported = 0U;
	return NO_ERROR;
}

/**
 * @brief 回零退让成功后，把当前扭力设为新的稳定基线。
 */
void Weight_RebaseStableWeight(void)
{
	weight_parament.stable_weight = weight_parament.current_weight;
}

/**
 * @brief 记录扭力模块已收到一帧有效数据并刷新通信时间戳。
 */
void Weight_MarkFrameReceived(void) {
	uint32_t now = HAL_GetTick();
	s_weight_last_rx_tick = now;
	/* 调用前当前重量已更新；只记录序号，不在此增加等待或打印。 */
	s_weight_frame_sequence++;
	s_weight_timeout_reported = 0U;
	/* 收到新扭力帧后只清除扭力通信类故障并记录恢复，其他模块已经锁存的错误不得被覆盖。 */
	if (Weight_IsCommErrorCode(g_measurement.device_status.error_code)) {
		ErrorLog_Recover(ERROR_LOG_MODULE_WEIGHT,
		                 ERROR_LOG_OP_READ_INT_PARAM,
		                 ERROR_LOG_REASON_WEIGHT_RECOVER,
		                 1U,
		                 1U);
		g_measurement.device_status.error_code = NO_ERROR;
	}
}

/**
 * @brief 根据最近有效重量帧时间更新通信超时状态。
 *
 * @param keep_global_error 非零时保留当前已经锁存的非扭力通信故障，不允许本次超时检查用 WEIGHT_COMM_TIMEOUT 覆盖其他模块根因。
 * @return NO_ERROR 表示最近有效重量帧仍在超时窗口内；超时且允许更新时返回 WEIGHT_COMM_TIMEOUT，keep_global_error 要求保留其他根因时返回当前已锁存错误码。
 */
static uint32_t Weight_CheckCommunicationTimeoutInternal(uint8_t keep_global_error)
{
	uint32_t now = HAL_GetTick();
	uint32_t error_code = g_measurement.device_status.error_code;

	/* 调用方要求保留全局错误时，非扭力通信故障具有更高优先级，超时检查不得把它替换为扭力超时。 */
	if (keep_global_error &&
		(error_code != NO_ERROR) &&
		(error_code != STATE_SWITCH) &&
		(!Weight_IsCommErrorCode(error_code))) {
		return error_code;
	}

	/* 距离最近一帧尚未达到通信超时门限时返回正常，避免低帧率链路被提前判故障。 */
	if ((now - s_weight_last_rx_tick) < WEIGHT_COMM_TIMEOUT_MS) {
		return NO_ERROR;
	}

	/* 同一次连续失联只在首次越过门限时输出告警；收到下一帧后 s_weight_timeout_reported 才会复位。 */
	if (!s_weight_timeout_reported) {
		s_weight_timeout_reported = 1U;
		ErrorLog_Warn(ERROR_LOG_MODULE_WEIGHT,
		              ERROR_LOG_OP_READ_INT_PARAM,
		              ERROR_LOG_REASON_WEIGHT_TIMEOUT,
		              ERROR_LOG_ACTION_CONTINUE);
		printf("扭力通信超时 | 超时阈值=%lums | 距上次接收=%lums\r\n",
		       (unsigned long)WEIGHT_COMM_TIMEOUT_MS,
		       (unsigned long)(now - s_weight_last_rx_tick));
	}

	g_measurement.device_status.error_code = WEIGHT_COMM_TIMEOUT;
	return WEIGHT_COMM_TIMEOUT;
}

/**
 * @brief 检查共享扭力通信是否超过允许静默时间。
 * @return 返回共享扭力通信超时检查结果；NO_ERROR 表示链路未超时，其他值为对应通信故障码。
 */
uint32_t Weight_CheckCommunicationTimeout(void)
{
	return Weight_CheckCommunicationTimeoutInternal(1U);
}

/* 返回最近有效称重帧序号；非阻塞，可用于确认动作结束后收到新帧。 */
uint32_t Weight_GetFrameSequence(void)
{
	return s_weight_frame_sequence;
}

/**
 * @brief 检查本机扭力通信链路是否超过允许静默时间。
 * @return 返回本机扭力通信链路超时检查结果；NO_ERROR 表示链路未超时，其他值为对应通信故障码。
 */
uint32_t Weight_CheckOwnCommunicationTimeout(void)
{
	return Weight_CheckCommunicationTimeoutInternal(0U);
}


/**
 * @brief 获取空载扭力
 *        延时5秒后，将当前扭力值记录为空载扭力
 *        并通过串口打印空载扭力
 *
 * @return NO_ERROR 表示空载等待完成并记录空载扭力；等待被命令切换、通信故障或超时终止时返回相应 wait_ret。
 */
uint32_t get_empty_weight(void) {
	printf("正在获取空载扭力值...\r\n"); /* 打印获取空载扭力信息 */
	uint32_t wait_ret = AbortableDelay_CommandSwitch(5000U, 100U); /* 等待5秒，确保扭力稳定 */
	if (wait_ret != NO_ERROR) {
		return wait_ret;
	}
	weight_parament.empty_weight = g_weight; /* 记录空载扭力 */
	printf("空载扭力值获取成功\t空载扭力：\t%d\r\n", weight_parament.empty_weight); /* 打印空载扭力 */
	if (abs(weight_parament.empty_weight) > MAX_EMPTY_WEIGHT) { /* 检查空载扭力是否在合理范围内 */
		printf("空载扭力值异常，请检查传感器或重新校准\r\n");
		RETURN_ERROR(WEIGHT_DRIFT_ERROR); /* 如果空载扭力不在合理范围内，打印错误信息并返回 */
	} else {
		g_deviceParams.empty_weight = weight_parament.empty_weight; /* 更新设备参数中的空载扭力 */
		printf("空载扭力值获取成功\t空载扭力：\t%d\r\n", weight_parament.empty_weight); /* 打印空载扭力 */
		save_device_params(); /* 保存设备参数 */
	}
	return NO_ERROR; /* 获取空载扭力成功，返回无错误状态 */
}
/**
 * @brief 获取满载扭力
 *        延时5秒后，计算当前扭力与空载扭力的差值，作为满载扭力
 *        并通过串口打印满载扭力
 *
 * @return NO_ERROR 表示满载等待完成并记录满载扭力；等待被命令切换、通信故障或超时终止时返回相应 wait_ret。
 */
uint32_t get_full_weight(void) {
	printf("正在等待扭力值稳定...\r\n"); /* 打印等待信息 */
	uint32_t wait_ret = AbortableDelay_CommandSwitch(5000U, 100U); /* 等待5秒，确保扭力稳定 */
	if (wait_ret != NO_ERROR) {
		return wait_ret;
	}

	weight_parament.full_weight = weight_parament.current_weight; /* 计算满载扭力 */

	if (weight_parament.full_weight < MIN_WEIGHT) {
		printf("满载扭力小于最小限制，可能需要重新校准\r\n");
		RETURN_ERROR(WEIGHT_UNDER_RANGE); /* 小于最小限制，报错 */
	} else if (weight_parament.full_weight > MAX_WEIGHT) {
		printf("满载扭力超出范围，可能需要重新校准\r\n");
		RETURN_ERROR(WEIGHT_OUT_OF_RANGE); /* 超出范围，报错 */
	} else {
		g_deviceParams.full_weight = weight_parament.full_weight; /* 更新设备参数中的满载扭力 */
		printf("满载扭力值获取成功\t满载的扭力：\t%d\r\n", weight_parament.full_weight); /* 打印满载扭力 */
		save_device_params(); /* 保存设备参数 */
	}

	printf("满载扭力值获取成功\t满载的扭力：\t%d\r\n", weight_parament.full_weight); /* 打印满载扭力 */
	return NO_ERROR; /* 获取成功 */
}


/**
 * @brief 零点状态检测（基于扭力）
 * @return ZERO（到达零点）或 NORMAL（未到零点）
 */
Weight_StateTypeDef check_zero_point_status(void)
{
    int threshold_ratio = g_deviceParams.zero_weight_threshold_ratio;  /* % */
    int full_weight    = weight_parament.full_weight;
    int current        = weight_parament.current_weight;

    int upper_limit;
    Weight_StateTypeDef state;

    /* ---------- 防御性处理 ---------- */
    if (threshold_ratio < 0) {
        threshold_ratio = 0;
    }
    if (threshold_ratio > 100) {
        threshold_ratio = 100;
    }

    /* ---------- 阈值计算 ---------- */
    upper_limit = full_weight * (100 + threshold_ratio) / 100;

    /* ---------- 状态判断 ---------- */
    if (current > upper_limit) {
        state = ZERO;
    } else {
        state = NORMAL;
    }

    /* ---------- 日志输出 ---------- */
    printf("零点状态检测 | 当前:%d | 满载:%d | 阈值:+%d%% | 判定:%s\r\n",
           current,
           full_weight,
           threshold_ratio,
           (state == ZERO) ? "到达零点" : "正常");

    return state;
}

/**
 * @brief 扭力统一碰撞/极限检测
 *        上行: 先判零点阈值，再判变重阈值
 *        下行: 当前重量小于等于固定500，或相对变轻超限，均返回碰撞错误
 *
 * 规则：
 *  1) 碰撞阈值（相对变化）：
 *     - 上行变重阈值 = full_weight * g_deviceParams.weight_upper_limit_ratio / 100
 *     - 下行变轻阈值 = full_weight * g_deviceParams.weight_lower_limit_ratio / 100
 *
 *  2) 零点阈值：g_deviceParams.zero_weight_threshold_ratio（百分比，加在 full_weight 上）
 *
 *  3) 探底扭力阈值是专用流程的减重量，不参与本函数；专用探底须先判候选，再调用通用保护。
 *
 *  4) 尺带长度 < weight_ignore_zone 或非上下行时跳过重量判定；入口仍检查通信。
 *
 * @return WEIGHT_COLLISION_DETECTED 表示检测到碰撞/到达极限；NO_ERROR 表示正常
 */
uint32_t CheckWeightCollision(void)
{
	uint32_t comm_ret = Weight_CheckCommunicationTimeout();
	if (comm_ret != NO_ERROR) {
		return comm_ret;
	}

	int32_t cur_weight	= (int32_t)weight_parament.current_weight;
	int32_t stable_weight	= (int32_t)weight_parament.stable_weight;
	int32_t full_weight	= (int32_t)weight_parament.full_weight;

	int32_t diff = cur_weight - stable_weight;

	uint32_t motor_dir = (uint32_t)g_measurement.debug_data.motor_state;

	/* 尺带长度/位置（debug_data: 0.1mm） */
	float cable_mm	= g_measurement.debug_data.cable_length / 10.0f;
	float sensor_mm	= g_measurement.debug_data.sensor_position / 10.0f;
	char detail[192];

	/* ==========================
	 *  保护：零点附近不做碰撞检测
	 * ========================== */
	if (cable_mm < (float)g_deviceParams.weight_ignore_zone/10.0) {
#ifdef WEIGHT_DEBUG
		printf("扭力跳过 | 原因:零点保护 | 方向：%lu 当前扭力=%ld 稳定扭力=%ld 差值：%+ld 满载扭力=%ld 尺带长度：%.1f",
				(unsigned long)motor_dir,
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)full_weight,
				cable_mm);
		MotorCtrl_PrintPositionRefs();
		printf(" 传感器位置=%.1f | 零点保护区=%lu\r\n",
				sensor_mm,
				(unsigned long)g_deviceParams.weight_ignore_zone);
#endif
		return NO_ERROR;
	}

	/* ==========================
	 *    电机状态检查
	 * ========================== */
	if ((motor_dir != 1U) && (motor_dir != 2U)) {
#ifdef WEIGHT_DEBUG
		printf("扭力检测 | 方向：%lu(无效) 当前扭力=%ld 稳定扭力=%ld 差值：%+ld 满载扭力=%ld 尺带长度：%.1f",
				(unsigned long)motor_dir,
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)full_weight,
				cable_mm);
		MotorCtrl_PrintPositionRefs();
		printf(" 传感器位置=%.1f\r\n",
				sensor_mm);
#endif
		return NO_ERROR;
	}

	/* ==========================
	 *   相对碰撞阈值（整数）
	 * ========================== */
	int32_t upper_threshold =
		(int32_t)(((int64_t)full_weight * (int64_t)g_deviceParams.weight_upper_limit_ratio) / 100);

	int32_t lower_threshold =
		(int32_t)(((int64_t)full_weight * (int64_t)g_deviceParams.weight_lower_limit_ratio) / 100);

	/* 零点阈值：full_weight * (100 + ratio)% / 100 */
	int32_t zero_limit =
		(int32_t)(((int64_t)full_weight * (int64_t)(100 + g_deviceParams.zero_weight_threshold_ratio)) / 100);

	/* 绝对低重量保护不随动态基准漂移，也不读取专用探底减重量参数。 */
	int32_t bottom_limit = WEIGHT_ABSOLUTE_MIN;

	/* ==========================
	 *       上行检测
	 * ========================== */
	if (motor_dir == 1U) {

		/* 1) 零点阈值 */
		if (cur_weight > zero_limit) {
			if (Weight_IsZeroSearchState()) {
				/* 回零/标零上行时，超过零点阈值是正常到零点信号，不能被通用防撞抢先报 18-3。 */
				return NO_ERROR;
			}
			printf("\r\n====== 扭力碰撞报警(上行) ======\r\n");
			printf("原因: 超过零点阈值(认为已到零点)\r\n");
			printf("当前扭力 : %ld\r\n", (long)cur_weight);
			printf("零点阈值 : %ld (full:%ld +%d%%)\r\n",
					(long)zero_limit,
					(long)full_weight,
					(int)g_deviceParams.zero_weight_threshold_ratio);
			printf("稳定扭力 : %ld\r\n", (long)stable_weight);
			printf("差值     : %ld\r\n", (long)diff);
			printf("上限阈值 : %ld (ratio:%ld%%)\r\n",
					(long)upper_threshold,
					(long)g_deviceParams.weight_upper_limit_ratio);
			Weight_PrintCableRefs(cable_mm);
			printf("传感器位置 : %.1f mm\r\n", sensor_mm);
			printf("================================\r\n");
            snprintf(detail, sizeof(detail),
                     "方向：%lu,当前：%ld,稳定：%ld,差值：%ld,零点：%ld,上限：%ld,绝对下限：%ld,下限：%ld,尺带：%.1f,传感器：%.1f",
                     (unsigned long)motor_dir,
                     (long)cur_weight,
                     (long)stable_weight,
                     (long)diff,
                     (long)zero_limit,
                     (long)upper_threshold,
                     (long)bottom_limit,
                     (long)lower_threshold,
                     (double)cable_mm,
                     (double)sensor_mm);
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_WEIGHT,
                                ERROR_LOG_OP_WEIGHT_COLLISION,
                                ERROR_LOG_REASON_COLLISION,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
			return WEIGHT_COLLISION_DETECTED;
		}

		/* 2) 相对变重阈值 */
		if (diff > upper_threshold) {
			printf("\r\n====== 扭力碰撞报警(上行) ======\r\n");
			printf("原因: 扭力增加超过上限阈值\r\n");
			printf("当前扭力 : %ld\r\n", (long)cur_weight);
			printf("稳定扭力 : %ld\r\n", (long)stable_weight);
			printf("扭力增加 : %ld (阈值:%ld)\r\n", (long)diff, (long)upper_threshold);
			printf("满载扭力 : %ld (ratio:%ld%%)\r\n",
					(long)full_weight,
					(long)g_deviceParams.weight_upper_limit_ratio);
			printf("零点阈值 : %ld (+%d%%)\r\n",
					(long)zero_limit,
					(int)g_deviceParams.zero_weight_threshold_ratio);
			Weight_PrintCableRefs(cable_mm);
			printf("传感器位置 : %.1f mm\r\n", sensor_mm);
			printf("================================\r\n");
            snprintf(detail, sizeof(detail),
                     "方向：%lu,当前：%ld,稳定：%ld,差值：%ld,零点：%ld,上限：%ld,绝对下限：%ld,下限：%ld,尺带：%.1f,传感器：%.1f",
                     (unsigned long)motor_dir,
                     (long)cur_weight,
                     (long)stable_weight,
                     (long)diff,
                     (long)zero_limit,
                     (long)upper_threshold,
                     (long)bottom_limit,
                     (long)lower_threshold,
                     (double)cable_mm,
                     (double)sensor_mm);
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_WEIGHT,
                                ERROR_LOG_OP_WEIGHT_COLLISION,
                                ERROR_LOG_REASON_COLLISION,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
			return WEIGHT_COLLISION_DETECTED;
		}

#ifdef WEIGHT_DEBUG
		printf("扭力正常 | 方向:上行 | 当前:%ld | 稳定:%ld | 变化:%+ld | 零点阈值:%ld | 上限阈值:%ld | 满载:%ld | 尺带:%.1fmm",
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)zero_limit,
				(long)upper_threshold,
				(long)full_weight,
				cable_mm);
		MotorCtrl_PrintPositionRefs();
		printf(" | 位置:%.1fmm\r\n",
				sensor_mm);
#endif
		return NO_ERROR;
	}

	/* ==========================
	 *       下行检测
	 * ========================== */
	{
		/* 缓慢或持续卸载由绝对下限兜底；上行释放和专用触底候选优先级保持不变。 */
		if ((cur_weight <= bottom_limit) || (-diff > lower_threshold)) {
			printf("\r\n====== 扭力碰撞报警(下行) ======\r\n");
			printf("原因: %s\r\n", (cur_weight <= bottom_limit) ?
                   "当前扭力低于或等于绝对下限" : "扭力减少超过相对下限阈值");
			printf("绝对重量下限 : %ld\r\n", (long)bottom_limit);
			printf("当前扭力 : %ld\r\n", (long)cur_weight);
			printf("稳定扭力 : %ld\r\n", (long)stable_weight);
			printf("扭力减少 : %ld (阈值:%ld)\r\n", (long)(-diff), (long)lower_threshold);
			printf("满载扭力 : %ld (ratio:%ld%%)\r\n",
					(long)full_weight,
					(long)g_deviceParams.weight_lower_limit_ratio);
			Weight_PrintCableRefs(cable_mm);
			printf("传感器位置 : %.1f mm\r\n", sensor_mm);
			printf("================================\r\n");
            snprintf(detail, sizeof(detail),
                     "方向：%lu,当前：%ld,稳定：%ld,差值：%ld,零点：%ld,上限：%ld,绝对下限：%ld,下限：%ld,尺带：%.1f,传感器：%.1f",
                     (unsigned long)motor_dir,
                     (long)cur_weight,
                     (long)stable_weight,
                     (long)diff,
                     (long)zero_limit,
                     (long)upper_threshold,
                     (long)bottom_limit,
                     (long)lower_threshold,
                     (double)cable_mm,
                     (double)sensor_mm);
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_WEIGHT,
                                ERROR_LOG_OP_WEIGHT_COLLISION,
                                ERROR_LOG_REASON_COLLISION,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
			return WEIGHT_COLLISION_DETECTED;
		}

#ifdef WEIGHT_DEBUG
		printf("扭力正常 | 方向:下行 | 当前:%ld | 稳定:%ld | 变化:%+ld | 绝对下限:%ld | 下限阈值:%ld | 满载:%ld | 尺带:%.1fmm",
				(long)cur_weight,
				(long)stable_weight,
				(long)diff,
				(long)bottom_limit,
				(long)lower_threshold,
				(long)full_weight,
				cable_mm);
		MotorCtrl_PrintPositionRefs();
		printf(" | 位置:%.1fmm | 罐底模式=%lu\r\n",
				sensor_mm,
				(unsigned long)g_deviceParams.bottom_detect_mode);
#endif
		return NO_ERROR;
	}
}




/**
 * @brief 扭力稳态更新（整数一阶低通）
 * @param currWeight 当前扭力实时值
 *
 * 公式：stable = stable * 0.8 + curr * 0.2
 * 为避免浮点运算，按 8:2 的整数权重实现。
 */
void Weight_Update(int32_t currWeight) {
	static uint32_t lastTime = 0;
	uint32_t now = HAL_GetTick();
	int64_t filtered_sum;

	if (now - lastTime < WEIGHT_SAMPLE_INTERVAL)
		return;  /* 每200ms更新一次 */
	lastTime = now;

	filtered_sum = (int64_t)weight_parament.stable_weight * WEIGHT_FILTER_OLD_FACTOR +
			(int64_t)currWeight * WEIGHT_FILTER_NEW_FACTOR;
	if (filtered_sum >= 0) {
		weight_parament.stable_weight = (int32_t)((filtered_sum + (WEIGHT_FILTER_DIVISOR / 2)) / WEIGHT_FILTER_DIVISOR);
	} else {
		weight_parament.stable_weight = (int32_t)((filtered_sum - (WEIGHT_FILTER_DIVISOR / 2)) / WEIGHT_FILTER_DIVISOR);
	}
	g_measurement.debug_data.current_weight = currWeight; /* 更新当前扭力到调试数据 */
	/* 可选调试输出 */
	/* printf("稳态扭力: %ld\n", weightFilter.stableWeight); */

	return;
}
