///*
// * measure_findOil.c
// *
// *  Created on: Mar 28, 2025
// *      Author: 1
// */
//
////#include "math.h"
////#include "stdlib.h"
//#include "main.h"
//#include "measure_oilLevel.h"
//#include "motor_ctrl.h"
//#include "sensor.h"
//#include <stdio.h>
//
//struct measure_density_level measure_density_level;
//
//static int findTheLiquidLevelByFrequency(float per_mm_Frequency);
//static int waitForTheLiquidLevelToExceedTheBlindZone(void);
//static int determineTheSensorPositionAndUpdateTheLevelValue(void);
//int lookForTheLevelInterface(void);
//
///**
// * @func: int findTheLiquidLevelByFrequency(float per_mm_Frequency)
// * @description: 通过密度传感器频率寻找液位
// * @param {float} per_mm_Frequency   初步定50mm
// * @param {u16} StabilityThreshold
// * @return NO_ERROR
// */
//static int findTheLiquidLevelByFrequency(float per_mm_Frequency)
//{
//	int ret, mag_angle;
//	uint32_t dir;
//	float runlenth;
//	int followTime = 0, overTime = 0, lowerTime = 0, loopTime = 0; /*跟随次数，超限次数，低限次数,死循环此处*/
//
//	printf("进入频率跟随区间\r\n");
//
//	while (followTime < 10) /*未进入小步上行区间*/
//	{
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		HAL_Delay(1000);
//		ret = DSM_Get_LevelMode_Frequence(
//				&measure_density_level.current_frequency);
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//		/* 最大跟随次数判断  */
//		loopTime++;
//		if (loopTime > 100)
//		{
//			printf("频率跟随可能陷入循环，跳出频率跟随\r\n");
//			break;
//		}
//
//		printf("当前频率%f\t频率阈值%f\t阈值差%f\r\n",
//				measure_density_level.current_frequency,
//				measure_density_level.follow_frequency, frequency_difference);
//
//		if ((frequency_difference > 1000)
//				|| (measure_density_level.air_frequency
//						- measure_density_level.current_frequency < 200))
//		{
//			overTime++;
//			lowerTime = 0;
//			runlenth = 4 * overTime + frequency_difference / per_mm_Frequency
//					- 4;
//			dir = MOTOR_DIRECTION_DOWN;
//		}
//		else if (frequency_difference > STABILITYTHRESHOLD)
//		{
//			overTime = 0;
//			lowerTime = 0;
//			runlenth = frequency_difference / per_mm_Frequency;
//			dir = MOTOR_DIRECTION_DOWN;
//		}
//		else if ((frequency_difference < -1000)
//				|| (measure_density_level.current_frequency
//						- measure_density_level.oil_frequency < 200))
//		{
//			lowerTime++;
//			overTime = 0;
//			runlenth = fabs(frequency_difference / per_mm_Frequency)
//					+ 4 * lowerTime - 4;
//			dir = MOTOR_DIRECTION_UP;
//		}
//		else if (frequency_difference < -STABILITYTHRESHOLD)
//		{
//			overTime = 0;
//			lowerTime = 0;
//			runlenth = fabs(frequency_difference / per_mm_Frequency);
//			dir = MOTOR_DIRECTION_UP;
//		}
//		else
//		{
//			runlenth = 0;
//			overTime = 0;
//			lowerTime = 0;
//		}
//		if ((fabs(frequency_difference) <= STABILITYTHRESHOLD)
//				&& (measure_density_level.air_frequency
//						- measure_density_level.current_frequency > 200)
//				&& (measure_density_level.current_frequency
//						- measure_density_level.oil_frequency > 200))
//		{
//			followTime++;
//			runlenth = 0;
//			printf("频率跟随\t电机不动作\t等待频率稳定\t频率稳定次数%d\r\n", followTime);
//		}
//		else
//		{
//			followTime = 0;
//		}
//		if (runlenth != 0)
//		{
//			if (runlenth < 5)
//			{
//				runlenth = runlenth / 2;
//			}
//			if (runlenth < 0.3)
//			{
//				runlenth = 0.1;
//			}
//			ret = MMMortorRun_MM_CheackLose(runlenth, dir, &mag_angle);
//			if (NO_ERROR != ret)
//			{
//				return ret;
//			}
//		}
//		if (overTime > MAX_TIMES_WHEN_FRE_FOLLOW
//				|| lowerTime > MAX_TIMES_WHEN_FRE_FOLLOW) /*多次加速仍然跟不上液位*/
//		{
//			printf("探头频率异常\r\n");
//			return MEASURE_FOLLOW_OVERRUN; /* 跟随不上*/
//		}
//		//
//		ret = determineTheSensorPositionAndUpdateTheLevelValue();
//		if (ret == MEASURE_DOWNLIMIT)
//		{
//			ret = waitForTheLiquidLevelToExceedTheBlindZone();
//		}
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//	}
//	return NO_ERROR;
//}
///**
// * @func: int findAndFollowTheLiquidLevel(void)
// * @description: 寻找并跟随液位
// * @return { }
// */
//int findAndFollowTheLiquidLevel(void)
//{
//	int ret;
//
//	ret = findTheLiquidLevelByFrequency(100); /*确定液位*/
//	if (ret != NO_ERROR)
//	{
//		return ret;
//	}
//	g_measurement.device_status.device_state = STATE_FLOWOIL; /* 置液位跟随状态 */
//
//	/**************************开启液位跟随********************************/
//	while (1)
//	{
//		printf("液位跟随\t");
//		ret = DSM_Get_LevelMode_Frequence(
//				&measure_density_level.current_frequency);
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//		if (fabs(frequency_difference) < STABILITYTHRESHOLD)
//		{
//			printf("液位稳定,电机不动作\r\n");
//			HAL_Delay(1000);
//			HAL_Delay(1000);
//			HAL_Delay(1000);
//		}
//		else
//		{
//			printf("识别到液位变动\t");
//			ret = findTheLiquidLevelByFrequency(100); /*识别到液位变动*/
//			if (ret != NO_ERROR)
//			{
//				return ret;
//			}
//		}
//		ret = determineTheSensorPositionAndUpdateTheLevelValue();
//		if (ret == MEASURE_DOWNLIMIT) //液位过低
//		{
//			ret = waitForTheLiquidLevelToExceedTheBlindZone();
//		}
//		if (ret != NO_ERROR)
//		{
//			return ret;
//		}
//	}
//}
//
/////**
//// * @func: int lookForLiquidLevelButNotFollow(void)
//// * @description: 寻找液位不跟随
//// * @return { }
//// */
////int lookForLiquidLevelButNotFollow(void)
////{
////	int ret;
////	ret = findTheLiquidLevelByFrequency(100); /*确定液位*/
////	if (ret != NO_ERROR)
////	{
////		return ret;
////	}
////	ret = determineTheSensorPositionAndUpdateTheLevelValue();
////	if (ret != NO_ERROR)
////	{
////		return ret;
////	}
////	return ret;
////}
////
////int lookForTheLevelInterface(void)
////{
////	int ret;
////	int flagofdown;
////	char countofanglechange = 0;
////	int times;
////
////	ret = Am_GetData(&mag_angle); /* 读取编码值 */
////	if (NO_ERROR != ret)
////	{
////		return ret;
////	}
////	if (Am_Cal_ToHigh(mag_angle) > 100)
////	{
////		printf("液位测量\t震动管找液位需要寻找零点\r\n");
////		ret = MMFindZero();
////
////		if (ret != NO_ERROR)
////		{
////			if (ret == STATE_SWITCH)
////			{
////				printf("液位测量\t回零点过程\t被打断\r\n");
////				return ret;
////			}
////			else
////			{
////				printf("液位测量\t回零点过程\t报错=%X\r\n", ret); // 内部有三次尝试无需进行尝试
////				return ret;
////			}
////		}
////		else
////		{
////			printf("液位测量\t回零点过程\t完成\r\n");
//////			FlagofOriginOkPowerOn = TRUE;
////		}
////	}
////
////	DSM_Switch_LevelMode();
////	measure_density_level.follow_frequency = 5500.0;
////	measure_density_level.air_frequency = 6500.0;
////	measure_density_level.oil_frequency = 4500.0;
////
//////	if (FlagofContinue == FALSE)
//////	{
//////		return STATE_SWITCH;
//////	}
////	ret = DSM_Get_LevelMode_Frequence(&measure_density_level.air_frequency); /*获取空气中频率值*/
////	if (ret != NO_ERROR)
////	{
////		return ret;
////	}
////	printf("超声波向下寻找液位\t");
////	while (1)
////	{
////
////		if (Am_Cal_ToHigh(mag_angle)< (systemunion.systemparameter.TankHigh- systemunion.systemparameter.FadeZero)) /* 不在盲区 */
////		{
////			stepsofmotorrun = (abs(FadeZeroSteps));
////			delaytime = MotorDownLimit(stepsofmotorrun * 1.3);
////
////			if (delaytime == MOTORCOM_RESET)
////			{
////				return MOTORCOM_RESET;
////			}
////			else if (delaytime == MOTORCOM_FAIL)
////			{
////				return MOTORCOM_FAIL;
////			}
////			else if (delaytime == MOTORCOM_WRONGSTEPS)
////			{
////				return MOTORCOM_WRONGSTEPS;
////			}
////
////			ret = Am_GetDataLoop_Start();
////
////			if (ret != NO_ERROR)
////			{
////				return ret;
////			}
////
////			Timer5Start(delaytime);
////			flagofdown = 1;
////
////			while (flagofdown)
////			{
////				if (FlagofTimeout5)
////				{
////					ret = MotorSpotStd();
////
////					if (ret != MOTORCOM_OK)
////					{
////						return ret;
////					}
////
////					ret = MotorTest();
////
////					if (ret == MOTORCOM_RESET)
////					{
////						return MOTORCOM_RESET;
////					}
////					else if (ret == MOTORCOM_FAIL)
////					{
////						return MOTORCOM_FAIL;
////					}
////					else
////					{
////						printf("超声波寻找液位\t下行确定液面\t液面太低\r\n");
////						flagofdown = 0;
////						return ERR_OIL_LOW;
////					}
////				}
////
////				ret = DSM_AngleGet(&gyro_anglex, &gyro_angley); // 读取X/Y轴的角度值
////
////				if (NO_ERROR != ret)
////				{
////					ret_tem = MotorSpotStd();
////
////					if (ret_tem != MOTORCOM_OK)
////					{
////						return ret_tem;
////					}
////
////					return ret;
////				}
////				printf("超声波寻找液位\t下行确定液面\t运行过程X=\t%f\tY=\t%f\t\r\n", gyro_anglex,
////						gyro_angley);
////				if (isSensorTiltAngleAbnormal(gyro_anglex, gyro_angley))
////				{
////					printf("超声波寻找液位\t下行确定液面\t角度变化过大X=%f\r\n", gyro_anglex);
////					Timer5Stop();
////					ret = MotorSpotStd();
////
////					if (ret == MOTORCOM_RESET)
////					{
////						return MOTORCOM_RESET;
////					}
////					else if (ret == MOTORCOM_FAIL)
////					{
////						return MOTORCOM_FAIL;
////					}
////					ret = DSM_AngleStart();
////					if (ret != NO_ERROR)
////					{
////						ret_tem = MotorSpotStd(); // V1.106
////						if (ret_tem != MOTORCOM_OK)
////						{
////							return ret_tem;
////						}
////						return ret;
////					}
////					ret = DSM_AngleGet(&gyro_anglex, &gyro_angley);
////					printf("超声波寻找液位\t下行确定液面\t角度变化过大再次确认X=\t%f\tY=\t%f\tZ=\r\n",
////							gyro_anglex, gyro_angley);
////					if (ret != NO_ERROR)
////					{
////						// 角度读取故障
////						return ret;
////					}
////
////					if (isSensorTiltAngleAbnormal(gyro_anglex, gyro_angley))
////					{
////						return ERR_ANGLE_CHANGE;
////					}
////					else
////					{
////						++countofanglechange;
////						printf("超声波寻找液位\t下行确定液面\t角度变化过大r尝试次数=%d\r\n",
////								countofanglechange);
////						if (countofanglechange >= 3)
////						{
////							return ERR_ANGLE_CHANGE;
////						}
////						flagofdown = 0;
////						Timer3Stop();
////						break;
////					}
////				}
////
////				printf("向下寻找液位\t确认频率值\t");
////				ret = DSM_Get_LevelMode_Frequence(
////						&measure_density_level.current_frequency);
////				if (ret != NO_ERROR)
////				{
////					ret_tem = MotorSpotStd(); // V1.106
////
////					if (ret_tem != MOTORCOM_OK)
////					{
////						return ret_tem;
////					}
////
////					return ret;
////				}
////
////				if (INOIL) /* A路超声进入油中 */
////				{
////					Timer5Stop();
////					ret = MotorSpotStd();
////
////					if (ret != MOTORCOM_OK)
////					{
////						return ret;
////					}
////
////					printf("向下寻找液位\t再次确认频率值\t");
////					ret = DSM_Get_LevelMode_Frequence(
////							&measure_density_level.current_frequency);
////
////					if (ret != NO_ERROR)
////					{
////						return ret;
////					}
////
////					if (INOIL)
////					{
////						ret = MMMortorRun_MM_CheackLose(50,
////								MOTOR_DIRECTION_DOWN, &mag_angle); /*确保传感器在油里*/
////						ret = DSM_Get_LevelMode_Frequence(
////								&measure_density_level.oil_frequency); /*确定油里的频率*/
////						if (ret != NO_ERROR)
////						{
////							return ret;
////						}
////						measure_density_level.follow_frequency =
////								(measure_density_level.air_frequency
////										+ measure_density_level.oil_frequency)
////										/ 2;
////						return NO_ERROR;
////					}
////					else
////					{
////						++countofanglechange;
////
////						if (countofanglechange >= 3)
////						{
////							printf("超声波寻找液位\t下行确定液面\t确认找到液面失败次数=%d\r\n",
////									countofanglechange);
////							return ERR_NIC_NOUSE;
////						}
////
////						flagofdown = 0;
////						Timer3Stop();
////						break;
////					}
////				}
////
////				ret = MotorGetState(); // 确认电机在运行状态时看看是否丢步的丢步
////
////				if (ret == MOTORCOM_RUNNING)
////				{
////					ret = Am_GetData(&mag_angle);
////
////					if (NO_ERROR != ret)
////					{
////						return ret;
////					}
////
////					if (last_magangle != 0X7FFE)
////					{
////						times = (Timer3GetTime() / 10);
////						Timer3Stop();
////						Timer3Startms(AM4096_GetOntime_timems);
////						ret = Caculate_Check_Ve_Change(mag_angle,
////								(mag_angle - last_magangle), times, MOTOR_VE);
////
////						if (ret != NO_ERROR)
////						{
////							ret_tem = MotorSpotSt(); // V1.121();
////
////							if (ret_tem != MOTORCOM_OK)
////							{
////								return ret_tem;
////							}
////
////							return ret;
////						}
////					}
////					else
////					{
////						printf("第一次时间%d\r\n", Timer3GetTime());
////						Timer3Stop();
////						Timer3Startms(AM4096_GetOntime_timems);
////					}
////
////					last_magangle = mag_angle;
////
////					// 位置判断到盲区
////					if (Am_Cal_ToHigh(mag_angle)
////							> (systemunion.systemparameter.TankHigh
////									- systemunion.systemparameter.FadeZero))
////					{
////						ret = MotorSpotStd();
////
////						if (ret != MOTORCOM_OK)
////						{
////							return ret;
////						}
////
////						printf("超声波寻找液位\t下行确定液面\t位置=%d\t运行到达盲区,盲区距离罐底=%d\r\n",
////								Am_Cal_ToHigh(mag_angle),
////								(systemunion.systemparameter.TankHigh
////										- Am_Cal_ToHigh(mag_angle)));
////						flagofdown = 0;
////						return ERR_OIL_LOW;
////					}
////				}
////				else if (ret == MOTORCOM_NORUNNING) // 电机停下了则认为到达盲区
////				{
////					ret = MotorSpotStd();
////
////					if (ret != MOTORCOM_OK)
////					{
////						return ret;
////					}
////
////					ret = MotorTest();
////
////					if (ret == MOTORCOM_RESET)
////					{
////						return MOTORCOM_RESET;
////					}
////					else if (ret == MOTORCOM_FAIL)
////					{
////						return MOTORCOM_FAIL;
////					}
////					else
////					{
////						printf(
////								"find oil or flowing \tserch face\t too low\r\n");
////						flagofdown = 0;
////						return ERR_OIL_LOW;
////					}
////				}
////				else
////				{
////					Timer3Stop();
////					Timer5Stop();
////					ret = MotorSpotStd();
////
////					if (ret != MOTORCOM_OK)
////					{
////						printf("停止电机未成功\r\n");
////					}
////
////					ret = MotorGetState();
////
////					if (ret != MOTORCOM_NORUNNING && ret != MOTORCOM_RUNNING) // dq 8.20
////					{
////						printf("运行过程电机检查失败\r\n");
////						return ret;
////					}
////					else
////					{
////						break;
////					}
////				}
////
////				/*
////				 过程被打断
////				 */
////				if (FlagofContinue == FALSE)
////				{
////					ret = MotorSpotStd();
////
////					if (ret != MOTORCOM_OK)
////					{
////						return ret;
////					}
////
////					return STATE_SWITCH;
////				}
////			}
////		}
////		else // 盲区以下低液位
////		{
////			printf("find oil or flowing \tserch face\t too low\r\n");
////			return ERR_OIL_LOW;
////		}
////	}
////}
////
/////**
//// * @func: int stepLossDetectionDuringLongDistanceRunning(int last_magangle)
//// * @description: 长距离运行时电机丢步检测
//// * @param {int} last_magangle	0X7FFE
//// * @return { }
//// */
////int stepLossDetectionDuringLongDistanceRunning(int last_magangle)
////{
////	int ret, ret_tem;
////	int mag_angle;
////	int times;
////	ret = MotorGetState(); // 确认电机在运行状态时看看是否丢步的丢步
////
////	if (ret == MOTORCOM_RUNNING)
////	{
////		ret = Am_GetData(&mag_angle);
////
////		if (NO_ERROR != ret)
////		{
////			return ret;
////		}
////
////		if (last_magangle != 0X7FFE)
////		{
////			times = (Timer3GetTime() / 10);
////			Timer3Stop();
////			Timer3Startms(AM4096_GetOntime_timems);
////			ret = Caculate_Check_Ve_Change(mag_angle,
////					(mag_angle - last_magangle), times, MOTOR_VE);
////
////			if (ret != NO_ERROR)
////			{
////				ret_tem = MotorSpotSt(); // V1.121();
////
////				if (ret_tem != MOTORCOM_OK)
////				{
////					return ret_tem;
////				}
////
////				return ret;
////			}
////		}
////		else
////		{
////			printf("第一次时间%d\r\n", Timer3GetTime());
////			Timer3Stop();
////			Timer3Startms(AM4096_GetOntime_timems);
////		}
////
////		last_magangle = mag_angle;
////
////		// 位置判断到盲区
////		if (Am_Cal_ToHigh(mag_angle)
////				> (systemunion.systemparameter.TankHigh
////						- systemunion.systemparameter.FadeZero))
////		{
////			ret = MotorSpotStd();
////
////			if (ret != MOTORCOM_OK)
////			{
////				return ret;
////			}
////			printf("超声波寻找液位\t下行确定液面\t位置=%d\t运行到达盲区,盲区距离罐底=%d\r\n",
////					Am_Cal_ToHigh(mag_angle),
////					(systemunion.systemparameter.TankHigh
////							- Am_Cal_ToHigh(mag_angle)));
////			return ERR_OIL_LOW;
////		}
////		return ret;
////	}
////	else if (ret == MOTORCOM_NORUNNING) // 电机停下了则认为到达盲区
////	{
////		ret = MotorSpotStd();
////
////		if (ret != MOTORCOM_OK)
////		{
////			return ret;
////		}
////
////		ret = MotorTest();
////
////		if (ret == MOTORCOM_RESET)
////		{
////			return MOTORCOM_RESET;
////		}
////		else if (ret == MOTORCOM_FAIL)
////		{
////			return MOTORCOM_FAIL;
////		}
////		else
////		{
////			printf("find oil or flowing \tserch face\t too low\r\n");
////			return ERR_OIL_LOW;
////		}
////	}
////	else
////	{
////		Timer3Stop();
////		Timer5Stop();
////		ret = MotorSpotStd();
////
////		if (ret != MOTORCOM_OK)
////		{
////			printf("停止电机未成功\r\n");
////		}
////
////		ret = MotorGetState();
////
////		if (ret != MOTORCOM_NORUNNING && ret != MOTORCOM_RUNNING)
////		{
////			printf("运行过程电机检查失败\r\n");
////			return ret;
////		}
////		else
////		{
////		}
////	}
////	return ret;
////}
//
////
/////**
//// * @func: int determineTheSensorPositionAndUpdateTheLevelValue(void)
//// * @description: 判断传感器位置并更新液位
//// * @return NO_ERROR
//// */
////static int determineTheSensorPositionAndUpdateTheLevelValue(void)
////{
////	int32_t oil_level;
////	int32_t ret, mag_angle;
////	ret = Am_GetData(&mag_angle);
////	if (NO_ERROR != ret)
////	{
////		return ret;
////	}
////
////	oil_level = Am_Cal_ToHigh(mag_angle);
////	if ((fabs(frequency_difference) < STABILITYTHRESHOLD)
////			|| (abs(
////					systemunion.systemparameter.TankHigh - oil_level
////							- Measure_One.MeasureOil_Level) > 100))
////	{
////		Measure_One.MeasureOil_Level = systemunion.systemparameter.TankHigh
////				- oil_level;
////		Measure_One.Spread_OilLevel = systemunion.systemparameter.TankHigh
////				- oil_level;
////		printf("液位跟随\t液位值为%d\r\n", Measure_One.MeasureOil_Level);
////		Out_4_20mA(systemunion.systemparameter.TankHigh,
////				Measure_One.MeasureOil_Level);
////	}
////	else
////	{
////		printf("液位跟随\t动态液位值为%d\r\n",
////				systemunion.systemparameter.TankHigh - oil_level);
////	}
////	if (oil_level >= systemunion.systemparameter.TankHigh)
////	{
////		printf("超声波找液位\t到达位置下限\r\n");
////		ret = MotorSpotStd();
////
////		if (ret != MOTORCOM_OK)
////		{
////			printf("停止电机未成功\r\n");
////		}
////		return MEASURE_DOWNLIMIT;
////	}
////	else if (oil_level < 0)
////	{
////		printf("超声波找液\t位到达位置上限\r\n");
////		ret = MotorSpotStd();
////
////		if (ret != MOTORCOM_OK)
////		{
////			printf("停止电机未成功\r\n");
////		}
////		return MEASURE_UPLIMIT;
////	}
////
////	if (FlagofContinue == FALSE)
////	{
////		ret = MotorSpotStd();
////
////		if (ret != MOTORCOM_OK)
////		{
////			return ret;
////		}
////		return STATE_SWITCH;
////	}
////	return NO_ERROR;
////}
////
/////**
//// * @func: int waitForTheLiquidLevelToExceedTheBlindZone(void)
//// * @description:在盲区等待，直到液位超过盲区退出
//// * @return NO_ERROR
//// */
////static int waitForTheLiquidLevelToExceedTheBlindZone(void)
////{
////	int32_t ret;
////	while (1)
////	{
////		ret = DSM_Get_LevelMode_Frequence(
////				&measure_density_level.current_frequency);
////		if (ret != NO_ERROR)
////		{
////			return ret;
////		}
////		printf("盲区等待\t频率阈值\t%f\t当前频率\t%f\t阈值差\t%f\r\n",
////				measure_density_level.follow_frequency,
////				measure_density_level.current_frequency, frequency_difference);
////		if (measure_density_level.current_frequency
////				> measure_density_level.follow_frequency)
////		{
////			break;
////		}
////		HAL_Delay(1000);
////		if (FlagofContinue == FALSE)
////		{
////			return STATE_SWITCH;
////		}
////	}
////	return NO_ERROR;
////}
