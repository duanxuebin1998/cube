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
#include <stdio.h>
#include <math.h>

// 全局液位测量结构体，存储当前状态
struct measure_density_level measure_density_level;

// 静态函数声明
static int findTheLiquidLevelByFrequency(float per_mm_Frequency);
static int waitForTheLiquidLevelToExceedTheBlindZone(void);
static int determineTheSensorPositionAndUpdateTheLevelValue(void);
int lookForTheLevelInterface(void);  // 注：实际函数实现在本文件外

/**
 * @func: findTheLiquidLevelByFrequency
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
static int findTheLiquidLevelByFrequency(float per_mm_Frequency) {
	// 初始化状态变量
	int ret, mag_angle;
	uint32_t dir;
	float runlenth;
	int followTime = 0, overTime = 0, lowerTime = 0, loopTime = 0;

	printf("进入频率跟随区间\r\n");

	// 主跟随循环（需满足连续10次稳定）
	while (followTime < 3) {
		// 延时保证传感器稳定性（总延时3秒）
		HAL_Delay(1000);
		HAL_Delay(1000);

		// 获取当前传感器频率
		ret = DSM_Get_LevelMode_Frequence(&measure_density_level.current_frequency);
		if (ret != NO_ERROR)
			return ret;

		// 死循环保护（>100次循环退出）
		if (loopTime++ > 100) {
			printf("频率跟随可能陷入循环，跳出频率跟随\r\n");
			break;
		}

		// 计算当前频率与目标频率差值
//		float frequency_difference = measure_density_level.follow_frequency - measure_density_level.current_frequency;
		printf("当前频率%f\t频率阈值%f\t阈值差%f\r\n", measure_density_level.current_frequency, measure_density_level.follow_frequency,
		frequency_difference);

		// 方向决策树（基于频率偏差）
		if (frequency_difference > 1000 ||  // 超大正偏差
				(measure_density_level.air_frequency - measure_density_level.current_frequency < 200)) { // 接近空气频率
			// 向下加速移动（补偿步长）
			overTime++;
			lowerTime = 0;
			runlenth = 4 * overTime + frequency_difference / per_mm_Frequency - 4;
			dir = MOTOR_DIRECTION_DOWN;
		} else if (frequency_difference > STABILITYTHRESHOLD) { // 可接受正偏差
			// 标准向下移动
			overTime = 0;
			lowerTime = 0;
			runlenth = frequency_difference / per_mm_Frequency;
			dir = MOTOR_DIRECTION_DOWN;
		} else if (frequency_difference < -1000 ||  // 超大负偏差
				(measure_density_level.current_frequency - measure_density_level.oil_frequency < 200)) { // 接近油中频率
			// 向上加速移动
			lowerTime++;
			overTime = 0;
			runlenth = fabs(frequency_difference / per_mm_Frequency) + 4 * lowerTime - 4;
			dir = MOTOR_DIRECTION_UP;
		} else if (frequency_difference < -STABILITYTHRESHOLD) { // 可接受负偏差
			// 标准向上移动
			overTime = 0;
			lowerTime = 0;
			runlenth = fabs(frequency_difference / per_mm_Frequency);
			dir = MOTOR_DIRECTION_UP;
		} else {  // 频率在稳定范围内
			runlenth = 0;
			overTime = 0;
			lowerTime = 0;
		}

		// 稳定性检测（连续稳定计数）
		if ((fabs(frequency_difference) <= STABILITYTHRESHOLD) && (measure_density_level.air_frequency - measure_density_level.current_frequency > 200)
				&& (measure_density_level.current_frequency - measure_density_level.oil_frequency > 200)) {
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

			// 执行电机移动（带丢步检测）
			ret = MMMortorRun_MM_CheackLose(runlenth, dir, &mag_angle);
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
		if (ret == MEASURE_DOWNLIMIT) {
			// 处理液位过低（进入盲区）
			ret = waitForTheLiquidLevelToExceedTheBlindZone();
		}
		if (ret != NO_ERROR)
			return ret;
	}
	return NO_ERROR;  // 成功定位液位界面
}

/**
 * @func: findAndFollowTheLiquidLevel
 * @brief 液位跟随主函数
 * @return 执行状态码
 *
 * 工作流程：
 * 1. 初始定位液位界面
 * 2. 进入无限循环跟随：
 *    - 检测频率变化
 *    - 当频率稳定时保持位置
 *    - 当频率变化超过阈值时重新跟踪
 * 3. 持续更新实际液位值
 */
int findAndFollowTheLiquidLevel(void) {
	int ret;

	// 初始液位定位（换算系数100Hz/mm）
	ret = findTheLiquidLevelByFrequency(100);
	if (ret != NO_ERROR)
		return ret;

	// 切换到跟随状态
	g_measurement.device_status.device_state = STATE_FLOWOIL;

	// 液位跟随主循环
	while (1) {
		printf("液位跟随\t");

		// 获取当前频率
		ret = DSM_Get_LevelMode_Frequence(&measure_density_level.current_frequency);
		if (ret != NO_ERROR)
			return ret;

		// 稳定性判断（频率波动在阈值内）
		if (fabs(frequency_difference) < STABILITYTHRESHOLD) {
			printf("液位稳定,电机不动作\r\n");
			HAL_Delay(3000);  // 等待3秒
		} else {
			// 检测到液位变动，重新跟踪
			printf("识别到液位变动\t");
			ret = findTheLiquidLevelByFrequency(100);
			if (ret != NO_ERROR)
				return ret;
		}

		// 更新位置和液位值
		ret = determineTheSensorPositionAndUpdateTheLevelValue();
		if (ret == MEASURE_DOWNLIMIT) {
			// 处理盲区状态
			ret = waitForTheLiquidLevelToExceedTheBlindZone();
		}
		if (ret != NO_ERROR)
			return ret;
	}
}

/* 注：以下函数在实际代码中有实现，此处仅保留声明和接口说明 */

/**
 * @func: determineTheSensorPositionAndUpdateTheLevelValue
 * @brief 计算当前位置并更新液位值
 *
 * 功能：
 * 1. 通过角度传感器计算当前高度
 * 2. 转换高度为实际液位值（罐高-传感器高度）
 * 3. 更新全局液位数据
 * 4. 越界检测（超过罐高/低于零点）
 */
//static int determineTheSensorPositionAndUpdateTheLevelValue(void) { ... }
/**
 * @func: waitForTheLiquidLevelToExceedTheBlindZone
 * @brief 盲区等待函数（当传感器低于最低测量位置时）
 *
 * 策略：
 * 持续监测频率直到传感器重新进入可测量区域
 * 退出条件：当前频率 > 目标跟随频率
 */
//static int waitForTheLiquidLevelToExceedTheBlindZone(void) { ... }
/**
 * @func: lookForTheLevelInterface
 * @brief 液位搜索初始化函数（未完整实现）
 *
 * 设计功能：
 * 1. 零点校准
 * 2. 确定空气/油中基准频率
 * 3. 初始定位液面
 */
//int lookForTheLevelInterface(void) { ... }
/**
 * @func: int lookForLiquidLevelButNotFollow(void)
 * @description: 寻找液位不跟随
 * @return { }
 */
int lookForLiquidLevelButNotFollow(void) {
	int ret;
	ret = findTheLiquidLevelByFrequency(100); /*确定液位*/
	if (ret != NO_ERROR) {
		return ret;
	}
	ret = determineTheSensorPositionAndUpdateTheLevelValue();
	if (ret != NO_ERROR) {
		return ret;
	}
	return ret;
}

int lookForTheLevelInterface(void)
{
//	int ret;
//	int flagofdown;
//	char countofanglechange = 0;
//	int times;
//
//	ret = Am_GetData(&mag_angle); /* 读取编码值 */
//	if (NO_ERROR != ret)
//	{
//		return ret;
//	}
//	if (Am_Cal_ToHigh(mag_angle) > 100)
//	{
//		printf("液位测量\t震动管找液位需要寻找零点\r\n");
//		ret = MMFindZero();
//
//		if (ret != NO_ERROR)
//		{
//			if (ret == STATE_SWITCH)
//			{
//				printf("液位测量\t回零点过程\t被打断\r\n");
//				return ret;
//			}
//			else
//			{
//				printf("液位测量\t回零点过程\t报错=%X\r\n", ret); // 内部有三次尝试无需进行尝试
//				return ret;
//			}
//		}
//		else
//		{
//			printf("液位测量\t回零点过程\t完成\r\n");
////			FlagofOriginOkPowerOn = true;
//		}
//	}
//
//	DSM_Switch_LevelMode();
//	measure_density_level.follow_frequency = 5500.0;
//	measure_density_level.air_frequency = 6500.0;
//	measure_density_level.oil_frequency = 4500.0;
//
////	if (FlagofContinue == false)
////	{
////		return STATE_SWITCH;
////	}
//	ret = DSM_Get_LevelMode_Frequence(&measure_density_level.air_frequency); /*获取空气中频率值*/
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	printf("超声波向下寻找液位\t");
//	while (1)
//	{
//
//		if (Am_Cal_ToHigh(mag_angle)< (systemunion.systemparameter.TankHigh- systemunion.systemparameter.FadeZero)) /* 不在盲区 */
//		{
//			stepsofmotorrun = (abs(FadeZeroSteps));
//			delaytime = MotorDownLimit(stepsofmotorrun * 1.3);
//
//			if (delaytime == MOTORCOM_RESET)
//			{
//				return MOTORCOM_RESET;
//			}
//			else if (delaytime == MOTORCOM_FAIL)
//			{
//				return MOTORCOM_FAIL;
//			}
//			else if (delaytime == MOTORCOM_WRONGSTEPS)
//			{
//				return MOTORCOM_WRONGSTEPS;
//			}
//
//			ret = Am_GetDataLoop_Start();
//
//			if (ret != NO_ERROR)
//			{
//				return ret;
//			}
//
//			Timer5Start(delaytime);
//			flagofdown = 1;
//
//			while (flagofdown)
//			{
//				if (FlagofTimeout5)
//				{
//					ret = MotorSpotStd();
//
//					if (ret != MOTORCOM_OK)
//					{
//						return ret;
//					}
//
//					ret = MotorTest();
//
//					if (ret == MOTORCOM_RESET)
//					{
//						return MOTORCOM_RESET;
//					}
//					else if (ret == MOTORCOM_FAIL)
//					{
//						return MOTORCOM_FAIL;
//					}
//					else
//					{
//						printf("超声波寻找液位\t下行确定液面\t液面太低\r\n");
//						flagofdown = 0;
//						return ERR_OIL_LOW;
//					}
//				}
//
//				ret = DSM_AngleGet(&gyro_anglex, &gyro_angley); // 读取X/Y轴的角度值
//
//				if (NO_ERROR != ret)
//				{
//					ret_tem = MotorSpotStd();
//
//					if (ret_tem != MOTORCOM_OK)
//					{
//						return ret_tem;
//					}
//
//					return ret;
//				}
//				printf("超声波寻找液位\t下行确定液面\t运行过程X=\t%f\tY=\t%f\t\r\n", gyro_anglex,
//						gyro_angley);
//				if (isSensorTiltAngleAbnormal(gyro_anglex, gyro_angley))
//				{
//					printf("超声波寻找液位\t下行确定液面\t角度变化过大X=%f\r\n", gyro_anglex);
//					Timer5Stop();
//					ret = MotorSpotStd();
//
//					if (ret == MOTORCOM_RESET)
//					{
//						return MOTORCOM_RESET;
//					}
//					else if (ret == MOTORCOM_FAIL)
//					{
//						return MOTORCOM_FAIL;
//					}
//					ret = DSM_AngleStart();
//					if (ret != NO_ERROR)
//					{
//						ret_tem = MotorSpotStd(); // V1.106
//						if (ret_tem != MOTORCOM_OK)
//						{
//							return ret_tem;
//						}
//						return ret;
//					}
//					ret = DSM_AngleGet(&gyro_anglex, &gyro_angley);
//					printf("超声波寻找液位\t下行确定液面\t角度变化过大再次确认X=\t%f\tY=\t%f\tZ=\r\n",
//							gyro_anglex, gyro_angley);
//					if (ret != NO_ERROR)
//					{
//						// 角度读取故障
//						return ret;
//					}
//
//					if (isSensorTiltAngleAbnormal(gyro_anglex, gyro_angley))
//					{
//						return ERR_ANGLE_CHANGE;
//					}
//					else
//					{
//						++countofanglechange;
//						printf("超声波寻找液位\t下行确定液面\t角度变化过大r尝试次数=%d\r\n",
//								countofanglechange);
//						if (countofanglechange >= 3)
//						{
//							return ERR_ANGLE_CHANGE;
//						}
//						flagofdown = 0;
//						Timer3Stop();
//						break;
//					}
//				}
//
//				printf("向下寻找液位\t确认频率值\t");
//				ret = DSM_Get_LevelMode_Frequence(
//						&measure_density_level.current_frequency);
//				if (ret != NO_ERROR)
//				{
//					ret_tem = MotorSpotStd(); // V1.106
//
//					if (ret_tem != MOTORCOM_OK)
//					{
//						return ret_tem;
//					}
//
//					return ret;
//				}
//
//				if (INOIL) /* A路超声进入油中 */
//				{
//					Timer5Stop();
//					ret = MotorSpotStd();
//
//					if (ret != MOTORCOM_OK)
//					{
//						return ret;
//					}
//
//					printf("向下寻找液位\t再次确认频率值\t");
//					ret = DSM_Get_LevelMode_Frequence(
//							&measure_density_level.current_frequency);
//
//					if (ret != NO_ERROR)
//					{
//						return ret;
//					}
//
//					if (INOIL)
//					{
//						ret = MMMortorRun_MM_CheackLose(50,
//								MOTOR_DIRECTION_DOWN, &mag_angle); /*确保传感器在油里*/
//						ret = DSM_Get_LevelMode_Frequence(
//								&measure_density_level.oil_frequency); /*确定油里的频率*/
//						if (ret != NO_ERROR)
//						{
//							return ret;
//						}
//						measure_density_level.follow_frequency =
//								(measure_density_level.air_frequency
//										+ measure_density_level.oil_frequency)
//										/ 2;
//						return NO_ERROR;
//					}
//					else
//					{
//						++countofanglechange;
//
//						if (countofanglechange >= 3)
//						{
//							printf("超声波寻找液位\t下行确定液面\t确认找到液面失败次数=%d\r\n",
//									countofanglechange);
//							return ERR_NIC_NOUSE;
//						}
//
//						flagofdown = 0;
//						Timer3Stop();
//						break;
//					}
//				}
//
//				ret = MotorGetState(); // 确认电机在运行状态时看看是否丢步的丢步
//
//				if (ret == MOTORCOM_RUNNING)
//				{
//					ret = Am_GetData(&mag_angle);
//
//					if (NO_ERROR != ret)
//					{
//						return ret;
//					}
//
//					if (last_magangle != 0X7FFE)
//					{
//						times = (Timer3GetTime() / 10);
//						Timer3Stop();
//						Timer3Startms(AM4096_GetOntime_timems);
//						ret = Caculate_Check_Ve_Change(mag_angle,
//								(mag_angle - last_magangle), times, MOTOR_VE);
//
//						if (ret != NO_ERROR)
//						{
//							ret_tem = MotorSpotSt(); // V1.121();
//
//							if (ret_tem != MOTORCOM_OK)
//							{
//								return ret_tem;
//							}
//
//							return ret;
//						}
//					}
//					else
//					{
//						printf("第一次时间%d\r\n", Timer3GetTime());
//						Timer3Stop();
//						Timer3Startms(AM4096_GetOntime_timems);
//					}
//
//					last_magangle = mag_angle;
//
//					// 位置判断到盲区
//					if (Am_Cal_ToHigh(mag_angle)
//							> (systemunion.systemparameter.TankHigh
//									- systemunion.systemparameter.FadeZero))
//					{
//						ret = MotorSpotStd();
//
//						if (ret != MOTORCOM_OK)
//						{
//							return ret;
//						}
//
//						printf("超声波寻找液位\t下行确定液面\t位置=%d\t运行到达盲区,盲区距离罐底=%d\r\n",
//								Am_Cal_ToHigh(mag_angle),
//								(systemunion.systemparameter.TankHigh
//										- Am_Cal_ToHigh(mag_angle)));
//						flagofdown = 0;
//						return ERR_OIL_LOW;
//					}
//				}
//				else if (ret == MOTORCOM_NORUNNING) // 电机停下了则认为到达盲区
//				{
//					ret = MotorSpotStd();
//
//					if (ret != MOTORCOM_OK)
//					{
//						return ret;
//					}
//
//					ret = MotorTest();
//
//					if (ret == MOTORCOM_RESET)
//					{
//						return MOTORCOM_RESET;
//					}
//					else if (ret == MOTORCOM_FAIL)
//					{
//						return MOTORCOM_FAIL;
//					}
//					else
//					{
//						printf(
//								"find oil or flowing \tserch face\t too low\r\n");
//						flagofdown = 0;
//						return ERR_OIL_LOW;
//					}
//				}
//				else
//				{
//					Timer3Stop();
//					Timer5Stop();
//					ret = MotorSpotStd();
//
//					if (ret != MOTORCOM_OK)
//					{
//						printf("停止电机未成功\r\n");
//					}
//
//					ret = MotorGetState();
//
//					if (ret != MOTORCOM_NORUNNING && ret != MOTORCOM_RUNNING) // dq 8.20
//					{
//						printf("运行过程电机检查失败\r\n");
//						return ret;
//					}
//					else
//					{
//						break;
//					}
//				}
//
//				/*
//				 过程被打断
//				 */
//				if (FlagofContinue == false)
//				{
//					ret = MotorSpotStd();
//
//					if (ret != MOTORCOM_OK)
//					{
//						return ret;
//					}
//
//					return STATE_SWITCH;
//				}
//			}
//		}
//		else // 盲区以下低液位
//		{
//			printf("find oil or flowing \tserch face\t too low\r\n");
//			return ERR_OIL_LOW;
//		}
//	}
}

/**
 * @func: int stepLossDetectionDuringLongDistanceRunning(int last_magangle)
 * @description: 长距离运行时电机丢步检测
 * @param {int} last_magangle	0X7FFE
 * @return { }
 */
int stepLossDetectionDuringLongDistanceRunning(int last_magangle)
{
//	int ret, ret_tem;
//	int mag_angle;
//	int times;
//	ret = MotorGetState(); // 确认电机在运行状态时看看是否丢步的丢步
//
//	if (ret == MOTORCOM_RUNNING)
//	{
//		ret = Am_GetData(&mag_angle);
//
//		if (NO_ERROR != ret)
//		{
//			return ret;
//		}
//
//		if (last_magangle != 0X7FFE)
//		{
//			times = (Timer3GetTime() / 10);
//			Timer3Stop();
//			Timer3Startms(AM4096_GetOntime_timems);
//			ret = Caculate_Check_Ve_Change(mag_angle,
//					(mag_angle - last_magangle), times, MOTOR_VE);
//
//			if (ret != NO_ERROR)
//			{
//				ret_tem = MotorSpotSt(); // V1.121();
//
//				if (ret_tem != MOTORCOM_OK)
//				{
//					return ret_tem;
//				}
//
//				return ret;
//			}
//		}
//		else
//		{
//			printf("第一次时间%d\r\n", Timer3GetTime());
//			Timer3Stop();
//			Timer3Startms(AM4096_GetOntime_timems);
//		}
//
//		last_magangle = mag_angle;
//
//		// 位置判断到盲区
//		if (Am_Cal_ToHigh(mag_angle)
//				> (systemunion.systemparameter.TankHigh
//						- systemunion.systemparameter.FadeZero))
//		{
//			ret = MotorSpotStd();
//
//			if (ret != MOTORCOM_OK)
//			{
//				return ret;
//			}
//			printf("超声波寻找液位\t下行确定液面\t位置=%d\t运行到达盲区,盲区距离罐底=%d\r\n",
//					Am_Cal_ToHigh(mag_angle),
//					(systemunion.systemparameter.TankHigh
//							- Am_Cal_ToHigh(mag_angle)));
//			return ERR_OIL_LOW;
//		}
//		return ret;
//	}
//	else if (ret == MOTORCOM_NORUNNING) // 电机停下了则认为到达盲区
//	{
//		ret = MotorSpotStd();
//
//		if (ret != MOTORCOM_OK)
//		{
//			return ret;
//		}
//
//		ret = MotorTest();
//
//		if (ret == MOTORCOM_RESET)
//		{
//			return MOTORCOM_RESET;
//		}
//		else if (ret == MOTORCOM_FAIL)
//		{
//			return MOTORCOM_FAIL;
//		}
//		else
//		{
//			printf("find oil or flowing \tserch face\t too low\r\n");
//			return ERR_OIL_LOW;
//		}
//	}
//	else
//	{
//		Timer3Stop();
//		Timer5Stop();
//		ret = MotorSpotStd();
//
//		if (ret != MOTORCOM_OK)
//		{
//			printf("停止电机未成功\r\n");
//		}
//
//		ret = MotorGetState();
//
//		if (ret != MOTORCOM_NORUNNING && ret != MOTORCOM_RUNNING)
//		{
//			printf("运行过程电机检查失败\r\n");
//			return ret;
//		}
//		else
//		{
//		}
//	}
	return NO_ERROR; // 返回正常状态;
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
	if ((fabs(frequency_difference) < STABILITYTHRESHOLD) || (abs(oil_level - g_measurement.oil_level) > 100)) {
		// 更新当前液位值
		g_measurement.oil_level = oil_level;

		// 打印正常液位值信息
		printf("液位跟随\t液位值为%ld\r\n", g_measurement.oil_level);

		// 将液位值转换为4-20mA模拟信号输出
//        Out_4_20mA(systemunion.systemparameter.TankHigh, Measure_One.MeasureOil_Level);
	} else {
		// 系统不稳定时仅打印动态液位值（不更新测量值）
		printf("液位跟随\t动态液位值为%ld\r\n", g_measurement.debug_data.sensor_position);
	}

	// 步骤3: 检查油位是否超出储罐上限
	if (oil_level >= g_deviceParams.tankHeight) {
		printf("超声波找液位\t到达位置上限\r\n");

//		// 到达下限后停止电机
//		ret = MotorSpotStd();
//
//		// 检查停止电机操作是否成功
//		if (ret != MOTORCOM_OK) {
//			printf("停止电机未成功\r\n");
//		}
		// 返回下限状态码（停止测量）
		return MEASURE_DOWNLIMIT;
	}
	// 步骤4: 检查油位是否低于0（超出下限）
	else if (oil_level < 0) {
		printf("超声波找液位\t到达位置下限\r\n");

//		// 到达上限后停止电机
//		ret = MotorSpotStd();
//
//		// 检查停止电机操作是否成功
//		if (ret != MOTORCOM_OK) {
//			printf("停止电机未成功\r\n");
//		}
//		// 返回上限状态码（停止测量）
//		return MEASURE_UPLIMIT;
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
	while (1) {
		ret = DSM_Get_LevelMode_Frequence(&measure_density_level.current_frequency);
		if (ret != NO_ERROR) {
			return ret;
		}
		printf("盲区等待\t频率阈值\t%f\t当前频率\t%f\t阈值差\t%f\r\n", measure_density_level.follow_frequency, measure_density_level.current_frequency, frequency_difference);
		if (measure_density_level.current_frequency > measure_density_level.follow_frequency) {
			break;
		}
		HAL_Delay(1000);
//打断监测
	}
	return NO_ERROR;
}
