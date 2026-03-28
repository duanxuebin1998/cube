/*
 * measure_findOil.c
 * 液位测量核心模块 - 通过密度传感器频率寻找/跟踪液位
 *
 * 核心功能：
 * 1. 通过频率特征识别液位位置
 * 2. 动态调整电机位置使传感器稳定在液位界面
 * 3. 处理液位过低/过高/振动管角度异常等边界情况
 * 4. 实现液位跟随算法
 *
 * 设计要点：
 * - 基于密度传感器在空气/油中频率差异原理（空气约6500Hz，油约4500Hz）
 * - 采用自适应步长控制策略：小偏差小步进，大偏差加速移动
 * - 多重保护机制：电机丢步检测、传感器倾斜保护、盲区处理、死循环预防
 */

#include "main.h"
#include "measure_oilLevel.h"
#include "motor_ctrl.h"
#include "sensor.h"
#include "weight.h"
#include "measure_zero.h"
#include "system_parameter.h"
#include "encoder.h"

// 函数原型声明
static int SearchOil();   // 粗略搜索液位
static int SearchAir(); // 精确搜索液位
static int SearchOilPrecise(float per_mm_Frequency);
static int determineTheSensorPositionAndUpdateTheLevelValue(void);
static int waitForTheLiquidLevelToExceedTheBlindZone(void);
uint32_t FollowOilLevel(void);

/**
 * @brief 液位测量与跟随主流程
 *        包含3个阶段：启用液位模式、搜索液位、跟随液位
 *        每个阶段失败时自动重试3次（状态切换时不重试）
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
/**
 * @brief 搜索并跟随液位
 *
 * 该函数执行液位测量的完整流程，包括回零点检查、液位搜索和液位跟随。
 * 首先检查设备是否需要回零点，如需要则先执行回零点操作。
 * 然后进行液位搜索，最多尝试3次。搜索成功后，根据设备状态执行液位位置修正。
 * 最后进入液位跟随状态，同样最多尝试3次，完成液位的持续跟踪。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 操作成功完成
 *                  - 其他错误码: 操作失败，具体错误码由 SearchZero、SearchOilLevel 或 FollowOilLevel 返回
 *
 * @note 函数执行流程：
 *       1. 检查并执行回零点操作（如果设备需要）
 *       2. 搜索液位（最多3次尝试）
 *       3. 修正液位位置（如果处于校准状态）
 *       4. 跟随液位（最多3次尝试）
 *
 * @note 每次失败后会延迟1秒后重试，如果检测到状态切换（STATE_SWITCH）则中止当前步骤
 */
uint32_t SearchAndFollowOilLevel(void) {
	uint32_t ret;
	uint8_t try_times;
	printf("液位测量\t开始\r\n");
	if ((g_measurement.device_status.zero_point_status == 1)&&(g_deviceParams.error_auto_back_zero==1)){
		printf("液位测量\t设备需要回零点\r\n");
		ret = SearchZero();  // 如果设备需要回零点，先执行回零点测量
		CHECK_ERROR(ret);  // 检查回零点是否成功
		printf("液位测量\t回零点完成\r\n");
	}
	printf("液位测量\t初始重量：%d\r\n", weight_parament.stable_weight);
	/*************** Step 1: 搜索液位 ***************/
	try_times = 0;
	while (try_times < 3) {
		try_times++;
		printf("液位流程\t液位搜索第%d次尝试\r\n", try_times);

		ret = SearchOilLevel();

		if (ret == NO_ERROR) {
			printf("液位流程\t液位搜索成功\r\n");
			break;
		} else if (ret == STATE_SWITCH) {
			printf("液位流程\t检测到状态切换，中止液位搜索\r\n");
			break;
		} else {
			printf("液位流程\t液位搜索失败，错误码:0x%lX\r\n", ret);
			HAL_Delay(1000);
		}
	}
	if (ret != NO_ERROR) {
		printf("液位流程\t液位搜索失败(尝试%d次)\r\n", try_times);
		CHECK_ERROR(ret);
	}
	// 修正液位位置
	if (g_measurement.device_status.device_state == STATE_CALIBRATIONOILING) {
		CorrectOilLevelProcess();
	}
	/*************** Step 2: 跟随液位 ***************/
	g_measurement.device_status.device_state = STATE_FLOWOIL; // 切换到液位跟随状态
	try_times = 0;
	while (try_times < 3) {
		try_times++;
		printf("液位流程\t液位跟随第%d次尝试\r\n", try_times);

		ret = FollowOilLevel();

		if (ret == NO_ERROR) {
			printf("液位流程\t液位跟随成功\r\n");
			break;
		} else if (ret == STATE_SWITCH) {
			printf("液位流程\t检测到状态切换，中止液位跟随\r\n");
			break;
		} else {
			printf("液位流程\t液位跟随失败，错误码:0x%lX\r\n", ret);
			HAL_Delay(1000);
		}
	}
	if (ret != NO_ERROR) {
		printf("液位流程\t液位跟随失败(尝试%d次)\r\n", try_times);
		CHECK_ERROR(ret);
	}

	printf("液位流程\t全部完成\r\n");
	return NO_ERROR;
}

/**
 * @brief 搜索液位高度
 *
 * 该函数通过粗找和精找两个阶段完成液位高度的测量。粗找阶段判断传感器当前状态（在油中或在空气中），
 * 并执行相应的搜索操作；精找阶段根据设定的目标频率进行精确液位定位。
 *
 * @return uint32_t 返回状态码：
 *                  - NO_ERROR: 液位测量成功
 *                  - MEASUREMENT_WEIGHT_DOWN_FAIL: 粗找液位失败（超过最大尝试次数）
 *                  - 其他错误码: 具体错误状态
 *
 * @note 函数执行流程：
 *       1. 启用液位模式（最多尝试3次）
 *       2. 粗找阶段：判断传感器在油中或空气中，执行SearchAir或SearchOil
 *       3. 精找阶段：根据测量方法计算目标频率，调用SearchOilPrecise
 *       4. 记录最终液位位置
 *
 * @note 依赖函数：
 *       - EnableLevelMode(): 启用液位模式
 *       - determine_level_status(): 判断液位状态
 *       - fault_info_init(): 清除故障信息
 *       - DSM_Get_LevelMode_Frequence_Avg(): 获取平均频率
 *       - motorMoveAndWaitUntilStopWithSpeed(): 电机移动
 *       - SearchAir(): 寻找空气
 *       - SearchOil(): 寻找液位
 *       - SearchOilPrecise(): 精确搜索液位
 */
uint32_t SearchOilLevel(void) {
	uint32_t ret;
	int try_times = 0;
	ret = EnableLevelMode();
	determine_level_status(); //第一次读取可能错误，临时用
	fault_info_init();  // 清除故障信息
	/*************** Step 1: 启用液位模式 ***************/
	try_times = 0;
	while (try_times < 3) {
		try_times++;
		printf("液位流程\t启用液位模式第%d次尝试\r\n", try_times);

		ret = EnableLevelMode();

		if (ret == NO_ERROR) {
			printf("液位流程\t液位模式启用成功\r\n");
			break;
		} else if (ret == STATE_SWITCH) {
			printf("液位流程\t检测到状态切换，中止液位模式启用\r\n");
			break;
		} else {
			printf("液位流程\t启用液位模式失败，错误码:0x%lX\r\n", ret);
			HAL_Delay(500);
		}
	}
	if (ret != NO_ERROR) {
		printf("液位流程\t液位模式启用失败(尝试%d次)\r\n", try_times);
		CHECK_ERROR(ret);
	}

	/*************** 粗找阶段 - 带重试机制 ***************/
	while (try_times < 3) {  //这里重试没有起作用
		try_times++;
		printf("液位测量\t粗找液位第%d次尝试\r\n", try_times);
		fault_info_init(); // 清除故障信息
		ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.current_frequency);
		if (INOIL) {
			//如果当前传感器在盲区以上100mm
			if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
				//向下运行保证传感器全部在油
				ret = motorMoveAndWaitUntilStopWithSpeed(100.0, MOTOR_DIRECTION_DOWN, motorGetDefaultSpeedX100());
				CHECK_ERROR(ret); // 检查下行是否成功
			}
			//取油中频率
			ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.oil_frequency);
			printf("液位测量\t油中频率：%ld\r\n", g_measurement.oil_measurement.oil_frequency);
			CHECK_ERROR(ret);  // 检查获取油中频率是否成功

			printf("液位测量\t传感器已在液位中，向上寻找空气\r\n");
			ret = SearchAir();
			CHECK_ERROR(ret); // 检查寻找空气是否成功
			break;
		} else if (INAIR) {
			printf("液位测量\t传感器在空气中，向下寻找液位\r\n");
			ret = SearchOil();
			CHECK_ERROR(ret); // 检查寻找液位是否成功
			break;
		}
	}
	if (try_times >= 3) {
		printf("液位测量\t粗找液位失败(尝试%d次)\r\n", try_times);
		RETURN_ERROR(MEASUREMENT_WEIGHT_DOWN_FAIL);
	}
	printf("液位测量\t粗找液位完成\r\n");
	/*************** 精找阶段  ***************/
	// 精确找液位
	if (g_deviceParams.liquidLevelMeasurementMethod == 0) {
		g_measurement.oil_measurement.follow_frequency = (g_measurement.oil_measurement.air_frequency + g_measurement.oil_measurement.oil_frequency) / 2.0;
	}
	else {
		g_measurement.oil_measurement.follow_frequency = g_deviceParams.oilLevelFrequency;
	}
	printf("液位测量\t目标频率：%ld Hz\r\n", g_measurement.oil_measurement.follow_frequency);

	ret = SearchOilPrecise(100);  // 执行精确搜索
	if (ret != NO_ERROR) {
		printf("液位测量\t精找失败:0x%lX\r\n", ret);
		CHECK_ERROR(ret);
	}
	/*************** 最终校验与记录 ***************/
	// 记录最终液位位置
	g_measurement.oil_measurement.oil_level = g_measurement.debug_data.sensor_position;  // 更新测量数据中的传感器位置
	g_measurement.density_distribution.Density_oil_level = g_measurement.oil_measurement.oil_level;
	// 打印测量结果
	printf("液位测量\t液位：%ld mm\r\n", g_measurement.oil_measurement.oil_level);

	return NO_ERROR;  // 返回成功状态
}
/**
 * @brief 液位跟随函数，用于持续监测并跟踪液位变化
 *
 * 该函数通过超声波频率信号实现液位的实时监测与跟随。当液位稳定时，电机保持静止；
 * 当检测到液位变动时，重新执行精确搜索并调整传感器位置，确保液位值准确更新。
 *
 * @return uint32_t 返回操作状态码：
 *             - NO_ERROR: 操作成功
 *             - 其他错误码: 具体错误状态
 *
 * @note 函数内部包含无限循环，持续监测液位状态，仅在发生错误时退出
 * @note 液位稳定判断基于频率波动阈值（oilLevelHysteresisThreshold）
 * @note 当液位过低进入盲区时，会调用 waitForTheLiquidLevelToExceedTheBlindZone 等待液位恢复
 */
uint32_t FollowOilLevel(void) {
	uint32_t ret;
	// 切换到跟随状态
	g_measurement.device_status.device_state = STATE_FLOWOIL;

	// 液位跟随主循环
	while (1) {
		printf("液位跟随\t");

		// 获取当前频率
		ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
		CHECK_ERROR(ret);  // 检查开启液位模式是否成功
		// 稳定性判断（频率波动在阈值内）
		if (fabs(frequency_difference) < g_deviceParams.oilLevelHysteresisThreshold) {
			printf("液位稳定,电机不动作\r\n");
			HAL_Delay(3000);  // 等待3秒
		} else {
			// 检测到液位变动，重新跟踪
			printf("识别到液位变动\t");
			ret = SearchOilPrecise(100);
			if (ret != NO_ERROR)
				return ret;
		}

		// 更新位置和液位值
		ret = determineTheSensorPositionAndUpdateTheLevelValue();
		if (ret == MEASUREMENT_OILLEVEL_LOW) {
			// 处理盲区状态
			ret = waitForTheLiquidLevelToExceedTheBlindZone();
			CHECK_ERROR(ret);
		} else {
			CHECK_ERROR(ret);
		}
	}
}

/**
 * @brief 搜索油液液位
 *
 * 该函数通过电机控制传感器移动，完成油液液位的测量。主要流程包括：
 * 1. 若尺带长度较长，先将电机上行到安全位置确保传感器在空气中。
 * 2. 长距离下行寻找油面，实时监控重量状态、检测丢步和碰撞。
 * 3. 检测到液位后快速停止电机。
 * 4. 若传感器未完全浸入油中，继续下行确保传感器全部在油中。
 * 5. 获取油中频率值。
 * 6. 调用 SearchAir() 向上寻找空气以完成测量。
 *
 * @return int 返回状态码：
 *             - NO_ERROR: 测量成功
 *             - MEASUREMENT_OILLEVEL_LOW: 到达位置下限（盲区）
 *             - 其他错误码: 电机操作、丢步检测或碰撞检测失败
 *
 * @note 函数内部会调用以下辅助函数：
 *       - determine_level_status(): 判断当前重量状态
 *       - motorMove_downWithSpeed(): 电机下行
 *       - Motor_CheckLostStep_AutoTiming(): 丢步检测
 *       - CheckWeightCollision(): 碰撞检测
 *       - DSM_Get_LevelMode_Frequence_Avg(): 获取油中频率
 *       - SearchAir(): 向上寻找空气
 *
 * @note 输出信息包括：
 *       - 初始重量
 *       - 寻找液位过程中的传感器位置和称重值
 *       - 油中频率值
 */
static int SearchOil() {
	uint32_t ret;

	printf("液位测量\t初始重量：%d\r\n", weight_parament.stable_weight);

	//向上运行保证传感器全部在空气
	if (g_measurement.debug_data.cable_length > 1000) { // 如果尺带长度大于200mm，先将电机上行到安全位置
		ret = motorMoveAndWaitUntilStopWithSpeed(100.0, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100());
		CHECK_ERROR(ret); // 检查上行是否成功
	}
	//长距离下行寻找油面
	MotorLostStep_Init();// 重置丢步检测计数器
	// 持续监控重量状态，直到检测到液位
	while (determine_level_status() != OIL) {
		ret = motorMove_downWithSpeed(motorGetDefaultSpeedX100());  // 启动电机向下运动
		CHECK_ERROR(ret); // 检查上行是否成功
		// 实时输出编码器位置和重量值（用于调试）
		printf("液位测量\t长距离寻找液位\t{传感器位置}%.1f\t{称重值}%d\r\n", (float) (g_measurement.debug_data.sensor_position) / 10.0, weight_parament.current_weight);
		CHECK_ERROR(ret);
		//丢步检测
		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功

		//称重检测
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); // 检查碰撞检测是否成功

		//盲区检测
		if (g_measurement.debug_data.sensor_position < g_deviceParams.blindZone) {
			printf("超声波找液位\t到达位置下限\r\n");
			return MEASUREMENT_OILLEVEL_LOW;
		}
	}
	ret = motorQuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功
	if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
		//向下运行保证传感器全部在油
		ret = motorMoveAndWaitUntilStopWithSpeed(100.0, MOTOR_DIRECTION_DOWN, motorGetDefaultSpeedX100());
		CHECK_ERROR(ret); // 检查下行是否成功
	}
	//取油中频率
	ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.oil_frequency);
	printf("液位测量\t油中频率：%ld\r\n", g_measurement.oil_measurement.oil_frequency);
	CHECK_ERROR(ret);  // 检查获取油中频率是否成功

	printf("液位测量\t传感器已在液位中，向上寻找空气\r\n");
	ret = SearchAir();
	CHECK_ERROR(ret); // 检查寻找空气是否成功

	return NO_ERROR;
}

/**
 * @brief 搜索空气中的零点位置并获取空气中频率值
 *
 * 该函数通过控制电机上行，持续监控重量状态，直到检测到液位（从油中进入空气）。
 * 到达零点后，快速停止电机，并确保传感器全部位于空气中，然后获取空气中频率值。
 * 如果传感器位置超出盲区范围，还会将传感器下移至油中进行后续操作。
 *
 * @return int 返回状态码：
 *             - NO_ERROR: 操作成功
 *             - 其他错误码: 具体错误状态
 *
 * @note 函数内部会调用以下辅助函数：
 *       - determine_level_status(): 判断当前液位状态（空气/油中）
 *       - motorMove_upWithSpeed(): 以指定速度控制电机上行
 *       - Motor_CheckLostStep_AutoTiming(): 自动定时检测电机丢步
 *       - CheckWeightCollision(): 检测称重碰撞
 *       - motorQuickStop(): 快速停止电机
 *       - motorMoveAndWaitUntilStopWithSpeed(): 移动电机并等待停止
 *       - DSM_Get_LevelMode_Frequence_Avg(): 获取平均频率值
 *
 * @note 输出信息包括：
 *       - 初始重量值
 *       - 传感器位置和称重值（实时调试信息）
 *       - 空气中频率值
 */
static int SearchAir() {
	uint32_t ret;
//	uint32_t frequency;	//当前频率
	printf("液位测量\t初始重量：%d\r\n", weight_parament.stable_weight);

	// 持续监控重量状态，直到检测到液位
    MotorLostStep_Init();// 重置丢步检测计数器
	while (determine_level_status() != AIR) {
		ret = motorMove_upWithSpeed(motorGetDefaultSpeedX100());  // 启动电机向下运动
		CHECK_ERROR(ret); // 检查上行是否成功
		// 实时输出编码器位置和重量值（用于调试）
		printf("液位测量\t长距离寻找空气\t{传感器位置}%.1f\t{称重值}%d\r\n", (float) (g_measurement.debug_data.sensor_position) / 10.0, weight_parament.current_weight);
		CHECK_ERROR(ret);
		//丢步检测
		ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.cable_length);
		CHECK_ERROR(ret); // 检查丢步检测是否成功
//		//称重检测
		ret = CheckWeightCollision();
		CHECK_ERROR(ret); // 检查碰撞检测是否成功
	}
	ret = motorQuickStop(); // 到达零点后快速停止电机
	CHECK_ERROR(ret); // 检查快速停止是否成功
	//向上运行保证传感器全部在空气
	if (g_measurement.debug_data.cable_length > 1000) { // 如果尺带长度大于200mm，先将电机上行到安全位置
		ret = motorMoveAndWaitUntilStopWithSpeed(100.0, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100());
		CHECK_ERROR(ret); // 检查上行是否成功
	}
	//取空气中频率
	ret = DSM_Get_LevelMode_Frequence_Avg(&g_measurement.oil_measurement.air_frequency);
	printf("液位测量\t空气中频率：%ld\r\n", g_measurement.oil_measurement.air_frequency);
	CHECK_ERROR(ret);  // 检查获取空气中频率是否成功
	if (g_measurement.debug_data.sensor_position > (g_deviceParams.blindZone + 1000)) {
		//向下运行保证传感器全部在油
		ret = motorMoveAndWaitUntilStopWithSpeed(100.0, MOTOR_DIRECTION_DOWN, motorGetDefaultSpeedX100());
		CHECK_ERROR(ret); // 检查下行是否成功
	}

	return NO_ERROR;
}

/**
 * @brief 确定液位状态
 *
 * 该函数通过获取当前液位模式的频率值，判断传感器当前所处的状态（空气中或油中）。
 * 根据频率值与阈值的比较，返回相应的液位状态枚举值。
 *
 * @return Level_StateTypeDef 返回液位状态：
 *             - AIR: 传感器在空气中（频率超过上限阈值）
 *             - OIL: 传感器在油中（频率低于下限阈值）
 *
 * @note 函数内部会调用 DSM_Get_LevelMode_Frequence 获取当前频率值。
 * @note 输出信息包括当前频率值和传感器状态（空气中或油中）。
 */
Level_StateTypeDef determine_level_status(void) {
	int ret;
	// 计算当前重量
	ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
	CHECK_ERROR(ret);
	printf("液位状态检测\t当前频率\t%ld\t", g_measurement.oil_measurement.current_frequency);

	// 判断是否超过上限阈值
	if (INAIR) {
		printf("传感器在空气中\r\n");
		return AIR; // 返回重量过大状态
	}
	// 判断是否低于下限阈值
	else {
		printf("传感器在油中\r\n");
		return OIL; // 返回重量过小状态
	}
}

/**
 * @func: SearchOilPrecise
 * @brief 通过频率跟随策略定位液位界面
 * @param per_mm_Frequency 每毫米对应的频率变化量（频率-位置换算系数）
 * @return 执行状态码
 *
 * 算法原理：
 * 1. 持续比较当前频率与目标频率(空气/油频率均值)
 * 2. 根据频率偏差计算电机移动距离：
 *    - 小偏差：按线性关系移动 (frequency_difference / per_mm_Frequency)
 *    - 大偏差：采用补偿步长 (4 * overTime + ...)
 * 3. 达到稳定条件（连续10次频率波动<阈值）时退出
 *
 * 特殊处理：
 * - 死循环保护：超过100次循环强制退出
 * - 超限保护：连续加速移动仍无法跟踪时报错
 */
static int SearchOilPrecise(float per_mm_Frequency) {
	// 初始化状态变量
	int ret;
	uint32_t dir;
	float runlenth;
	int followTime = 0, overTime = 0, lowerTime = 0, loopTime = 0;

	printf("进入频率跟随区间\r\n");

	// 主跟随循环（需满足连续10次稳定）
	while (followTime < 10) {
		// 延时保证传感器稳定性（总延时3秒）
		HAL_Delay(1000);
		HAL_Delay(1000);

		// 获取当前传感器频率
		ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
		if (ret != NO_ERROR)
			return ret;

		// 死循环保护（>100次循环退出）
		if (loopTime++ > 100) {
			printf("频率跟随可能陷入循环，跳出频率跟随\r\n");
			break;
		}

		// 计算当前频率与目标频率差值
//		float frequency_difference = g_measurement.oil_measurement.follow_frequency - g_measurement.oil_measurement.current_frequency;
		printf("当前频率%ld\t频率阈值%ld\t阈值差%f\r\n", g_measurement.oil_measurement.current_frequency, g_measurement.oil_measurement.follow_frequency,
		frequency_difference);

		// 方向决策树（基于频率偏差）
		if (frequency_difference > 500 ||  // 超大正偏差
				(g_measurement.oil_measurement.air_frequency - g_measurement.oil_measurement.current_frequency < 200)) { // 接近空气频率
			// 向下加速移动（补偿步长）
			overTime++;
			lowerTime = 0;
			runlenth = 4.0 * overTime + frequency_difference / per_mm_Frequency - 4.0;
			dir = MOTOR_DIRECTION_DOWN;
		} else if (frequency_difference > g_deviceParams.oilLevelThreshold) { // 可接受正偏差
			// 标准向下移动
			overTime = 0;
			lowerTime = 0;
			runlenth = frequency_difference / per_mm_Frequency;
			dir = MOTOR_DIRECTION_DOWN;
		} else if (frequency_difference < -500 ||  // 超大负偏差
				(g_measurement.oil_measurement.current_frequency - g_measurement.oil_measurement.oil_frequency < 200)) { // 接近油中频率
			// 向上加速移动
			lowerTime++;
			overTime = 0;
			runlenth = -(frequency_difference / per_mm_Frequency) + 4.0 * lowerTime - 4.0;
			dir = MOTOR_DIRECTION_UP;
		} else if (frequency_difference < -g_deviceParams.oilLevelThreshold) { // 可接受负偏差
			// 标准向上移动
			overTime = 0;
			lowerTime = 0;
			runlenth = -(frequency_difference / per_mm_Frequency);
			dir = MOTOR_DIRECTION_UP;
		} else {  // 频率在稳定范围内
			runlenth = 0;
			overTime = 0;
			lowerTime = 0;
		}

		// 稳定性检测（连续稳定计数）
		if ((abs(frequency_difference) <= g_deviceParams.oilLevelThreshold) && (g_measurement.oil_measurement.air_frequency - g_measurement.oil_measurement.current_frequency > 200)
				&& (g_measurement.oil_measurement.current_frequency - g_measurement.oil_measurement.oil_frequency > 200)) {
			followTime++;
			runlenth = 0;
			printf("频率跟随\t电机不动作\t等待频率稳定\t频率稳定次数%d\r\n", followTime);
		} else {
			followTime = 0;  // 重置连续稳定计数
		}

		// 电机移动控制
		if (runlenth != 0) {
			// 步长微调策略
			if (runlenth < 5)
				runlenth = runlenth / 2;
			if (runlenth < 0.3)
				runlenth = 0.1;
			printf("频率跟随\t电机移动\t距离%f\t方向%s\r\n", runlenth, (dir == MOTOR_DIRECTION_UP) ? "上" : "下");
			// 执行电机移动（带丢步检测）
			ret = motorMoveAndWaitUntilStopWithSpeed(runlenth, dir, motorGetDefaultSpeedX100());
			if (NO_ERROR != ret)
				return ret;
		}

		// 超限保护（连续多次加速仍无法跟踪）
		if (overTime > MAX_TIMES_WHEN_FRE_FOLLOW || lowerTime > MAX_TIMES_WHEN_FRE_FOLLOW) {
			printf("探头频率异常\r\n");
			return MEASUREMENT_OVERSPEED;
		}

		// 更新传感器位置及液位值
		ret = determineTheSensorPositionAndUpdateTheLevelValue();
		if (ret == MEASUREMENT_OILLEVEL_LOW) {
			// 处理液位过低（进入盲区）
			ret = waitForTheLiquidLevelToExceedTheBlindZone();
		}
		if (ret != NO_ERROR)
			return ret;
	}
	return NO_ERROR;  // 成功定位液位界面
}

/**
 * @func: int determineTheSensorPositionAndUpdateTheLevelValue(void)
 * @description: 判断传感器位置并更新液位值
 * 主要功能：
 *  1. 根据磁角度计算当前油位值
 *  2. 根据稳定性条件更新液位测量值
 *  3. 检查油位是否超出上下限范围
 *  4. 当超限时停止电机并返回状态
 * @return:
 *  正常范围：NO_ERROR
 *  到达下限：MEASURE_DOWNLIMIT
 *  到达上限：MEASURE_UPLIMIT
 */
static int determineTheSensorPositionAndUpdateTheLevelValue(void) {
	int32_t oil_level;         // 计算得到的当前油位高度（从参考点起算）
//	int32_t ret;    // ret: 操作返回值

	oil_level = g_measurement.debug_data.sensor_position; // 从传感器位置获取当前油位高度
	// 判断是否需要更新液位值
	// 条件1: 频率差在稳定阈值内（系统稳定）
	// 条件2: 新旧液位值差异大于100（需要强制更新）
	// 条件3：处在液位跟随状态
	// if (((abs(frequency_difference) < g_deviceParams.oilLevelThreshold) || (abs((int) oil_level - (int) g_measurement.oil_measurement.oil_level) > 100)) && (g_measurement.device_status.device_state == STATE_FLOWOIL)) {
	if  (g_measurement.device_status.device_state == STATE_FLOWOIL) {
		// 更新当前液位值
		g_measurement.oil_measurement.oil_level = oil_level;
		// 打印正常液位值信息
		printf("液位跟随\t液位值为%ld\r\n", g_measurement.oil_measurement.oil_level);
	} else {
		// 系统不稳定时仅打印动态液位值（不更新测量值）
		printf("液位跟随\t动态液位值为%ld\r\n", g_measurement.debug_data.sensor_position);
	}

	if (oil_level >= g_deviceParams.tankHeight-1000) {
		printf("超声波找液位\t到达位置上限\r\n");
		return MEASUREMENT_OILLEVEL_HIGH;
	}
	// 步骤4: 检查油位是否低于0（超出下限）
	else if (oil_level < g_deviceParams.blindZone) {
		printf("超声波找液位\t到达位置下限\r\n");
		return MEASUREMENT_OILLEVEL_LOW;
	}
	// 步骤5: 正常返回（无错误）
	return NO_ERROR;
}

/**
 * @func: int waitForTheLiquidLevelToExceedTheBlindZone(void)
 * @description:在盲区等待，直到液位超过盲区退出
 * @return NO_ERROR
 */
static int waitForTheLiquidLevelToExceedTheBlindZone(void) {
	int32_t ret;
	//运行到盲区

	while (1) {
		ret = DSM_Get_LevelMode_Frequence(&g_measurement.oil_measurement.current_frequency);
		CHECK_ERROR(ret);	// 检查开启液位模式是否成功
		printf("盲区等待\t频率阈值\t%ld\t当前频率\t%ld\t阈值差\t%f\r\n", g_measurement.oil_measurement.follow_frequency, g_measurement.oil_measurement.current_frequency,
		frequency_difference);
		if (g_measurement.oil_measurement.current_frequency > g_measurement.oil_measurement.follow_frequency) {
			break;
		}
		HAL_Delay(1000);
		//打断监测
	}
	return NO_ERROR;
}
/**
 * @brief 液位标定处理函数
 *
 * 该函数用于执行液位标定流程，包括：
 * - 根据缆线长度和标定液位计算并设置罐体高度
 * - 标定完成后清零标定液位值
 * - 更新传感器高度数据
 * - 保存设备参数
 *
 * @note 函数内部会调用以下辅助函数：
 *       - update_sensor_height_from_encoder(): 从编码器更新罐高数据
 *       - save_device_params(): 保存设备参数
 *
 * @note 输出信息包括：
 *       - 标定开始提示
 *       - 标定完成后的罐体高度值
 */
void CorrectOilLevelProcess(void) {
	printf("液位流程\t开始标定液位\r\n");
	g_deviceParams.tankHeight = g_measurement.debug_data.cable_length + g_deviceParams.calibrateOilLevel+1;
	g_deviceParams.calibrateOilLevel = 0; //标定完成后清零
	printf("液位流程\t标定完成，罐高设置为：%ld mm\r\n", g_deviceParams.tankHeight);
	update_sensor_height_from_encoder();	//更新罐高数据
	save_device_params();//保存参数
}
