/*
 * system_parameter.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */
#include "system_parameter.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

volatile MeasurementResult g_measurement = { 0 }; // 测量结果
volatile DeviceParameters g_deviceParams = { 0 }; // 设备参数

///*********************** 参数存储逻辑 ***********************/
//void save_device_params(void) {
//	DeviceParameters params = g_deviceParams;
//
//// 计算CRC（不包含crc字段自身）
//	uint32_t crc_size = sizeof(params) - sizeof(params.crc);
//	params.crc = CRC32_HAL((uint8_t*) &params, crc_size);
//	printf("保存设备参数，CRC: 0x%08X\n", params.crc);
//// 通过现有接口写入
//	WriteMultiData((uint8_t*) &params, FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));
//	printf("设备参数已保存到FRAM。\n");
//	print_device_params(); // 打印当前参数
//}
//
///*********************** 参数加载逻辑 ***********************/
//int load_device_params(void) {
//	DeviceParameters temp;
//
//// 通过现有接口读取
//	ReadMultiData((uint8_t*) &temp,
//	FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));
//
//// 验证CRC
//	uint32_t saved_crc = temp.crc;
//	temp.crc = 0; // 清零后计算
//	// 计算CRC（不包含crc字段自身）
//	uint32_t crc_size = sizeof(temp) - sizeof(temp.crc);
//	uint32_t calc_crc = CRC32_HAL((uint8_t*) &temp, crc_size);
//	if (calc_crc == saved_crc) {
//		memcpy((void* volatile ) &g_deviceParams, &temp, sizeof(DeviceParameters));
//
//		return 1; // 校验成功
//	} else {
//		printf("设备参数校验失败，CRC不匹配。\n");
//		printf("校验计算的CRC: 0x%08X, 保存的CRC: 0x%08X\n", calc_crc, saved_crc);
//	}
//	return 0; // 校验失败
//}
//
///*********************** 参数初始化 ***********************/
//void init_device_params(void) {
//// 尝试加载参数
//	if (!load_device_params()) {
//// 加载失败时初始化默认值
//		memset((void* volatile ) &g_deviceParams, 0, sizeof(DeviceParameters));
//		RestoreFactoryParamsConfig(); //恢复出厂设置
//		printf("设备参数加载失败，已恢复出厂设置。\n");
//	}
//	print_device_params(); // 打印当前参数
//}
///*-------------------------------------------------------
// * 恢复出厂参数配置
// *-----------------------------------------------------*/
//void RestoreFactoryParamsConfig(void) {
//	/* —— 基础参数 —— */
//	g_deviceParams.tankHeight = 200000; // 20 000 mm
//	g_deviceParams.blindZone = 5000; // 500 mm
//	g_deviceParams.encoder_wheel_circumference_mm = 95000; // 95 mm ,1000倍缩放
//	g_deviceParams.sensorType = 1; // 默认传感器类型
//	g_deviceParams.softwareVersion = 0x0001001; // 1.001
//
//	/* —— 密度 / 温度 —— */
//	g_deviceParams.densityCorrection = 0;
//	g_deviceParams.temperatureCorrection = 0;
//
//	/* —— 单点/分布测量控制 —— */
//	g_deviceParams.requireBottomMeasurement = 0; //不需要测罐底
//	g_deviceParams.requireWaterMeasurement = 0; //不需要测水
//	g_deviceParams.requireSinglePointDensity = 0; //不需要单点监测
//	g_deviceParams.spreadMeasurementOrder = 0; // 0=由下向上
//	g_deviceParams.spreadMeasurementMode = 0; // 待定
//	g_deviceParams.spreadMeasurementCount = 5; //分布测量点位
//	g_deviceParams.spreadMeasurementDistance = 1000; // 1000 mm
//	g_deviceParams.spreadTopLimit = 300; // 距液面 300 mm
//	g_deviceParams.spreadBottomLimit = 300; // 距罐底 300 mm
//
//	/* —— 水位测量 —— */
//	g_deviceParams.waterLevelCorrection = 0; //水位修正值
//	g_deviceParams.maxDownDistance = 3000; // 300 mm
//
//	/* —— 实高测量 —— */
//	g_deviceParams.refreshTankHeightFlag = 0;
//	g_deviceParams.maxTankHeightDeviation = 100; // 10 mm
//	g_deviceParams.initialTankHeight = 0;
//	g_deviceParams.currentTankHeight = 0;
//
//	/* —— 液位测量 —— */
//	g_deviceParams.oilLevelThreshold = 3; // 0.3 mm
//	g_deviceParams.liquidLevelMeasurementMethod = 0; // 0默认测量方式
//
//	/* —— 持久化（可选） —— */
//	// 通过现有接口写入
//	save_device_params();
//}
/* 新增函数：打印所有设备参数 */
void print_device_params(void)
{
    // 创建参数副本以避免多次访问 volatile 变量
    DeviceParameters params;
    memcpy(&params, (void*)&g_deviceParams, sizeof(DeviceParameters));

    printf("========== Device Parameters ==========\r\n");

    /* 指令 */
    printf("[Command]\r\n");
    printf("  command                : %u\r\n", (unsigned)params.command);

    /* 基础参数 */
    printf("[Basic Params]\r\n");
    printf("  tankHeight             : %lu mm\r\n", (unsigned long)params.tankHeight);
    printf("  blindZone              : %lu mm\r\n", (unsigned long)params.blindZone);
    printf("  waterBlindZone         : %lu mm\r\n", (unsigned long)params.waterBlindZone);
    printf("  encoder_circumference  : %lu mm\r\n", (unsigned long)params.encoder_wheel_circumference_mm);
    printf("  sensorType             : %lu\r\n", (unsigned long)params.sensorType);
    printf("  sensorID               : %lu\r\n", (unsigned long)params.sensorID);
    printf("  softwareVersion        : %lu\r\n", (unsigned long)params.softwareVersion);

    /* 称重参数 */
    printf("[Weight Params]\r\n");
    printf("  empty_weight           : %lu\r\n", (unsigned long)params.empty_weight);
    printf("  full_weight            : %lu\r\n", (unsigned long)params.full_weight);
    printf("  weight_upper_limit_ratio : %lu\r\n", (unsigned long)params.weight_upper_limit_ratio);
    printf("  weight_lower_limit_ratio : %lu\r\n", (unsigned long)params.weight_lower_limit_ratio);
    printf("  empty_weight_upper_limit : %lu\r\n", (unsigned long)params.empty_weight_upper_limit);
    printf("  empty_weight_lower_limit : %lu\r\n", (unsigned long)params.empty_weight_lower_limit);
    printf("  full_weight_upper_limit  : %lu\r\n", (unsigned long)params.full_weight_upper_limit);
    printf("  full_weight_lower_limit  : %lu\r\n", (unsigned long)params.full_weight_lower_limit);
    printf("  findZeroDownDistance     : %lu mm\r\n", (unsigned long)params.findZeroDownDistance);

    /* 指令参数 */
    printf("[Command Params]\r\n");
    printf("  calibrateOilLevel            : %lu mm\r\n", (unsigned long)params.calibrateOilLevel);
    printf("  calibrateWaterLevel          : %lu mm\r\n", (unsigned long)params.calibrateWaterLevel);
    printf("  singlePointMeasurementPos    : %lu mm\r\n", (unsigned long)params.singlePointMeasurementPosition);
    printf("  singlePointMonitoringPos     : %lu mm\r\n", (unsigned long)params.singlePointMonitoringPosition);
    printf("  densityDistributionOilLevel  : %lu mm\r\n", (unsigned long)params.densityDistributionOilLevel);
    printf("  motorCommandDistance         : %lu mm\r\n", (unsigned long)params.motorCommandDistance);

    /* 密度和温度测量参数 + 分布测量参数 */
    printf("[Density / Temp / Spread]\r\n");
    printf("  densityCorrection        : %lu\r\n", (unsigned long)params.densityCorrection);
    printf("  temperatureCorrection    : %lu\r\n", (unsigned long)params.temperatureCorrection);
    printf("  requireBottomMeasurement : %s\r\n", params.requireBottomMeasurement ? "Yes" : "No");
    printf("  requireWaterMeasurement  : %s\r\n", params.requireWaterMeasurement ? "Yes" : "No");
    printf("  requireSinglePointDensity: %s\r\n", params.requireSinglePointDensity ? "Yes" : "No");
    printf("  spreadMeasurementOrder   : %lu (0=Bottom-Up)\r\n", (unsigned long)params.spreadMeasurementOrder);
    printf("  spreadMeasurementMode    : %lu\r\n", (unsigned long)params.spreadMeasurementMode);
    printf("  spreadMeasurementCount   : %lu points\r\n", (unsigned long)params.spreadMeasurementCount);
    printf("  spreadMeasurementDistance: %lu mm\r\n", (unsigned long)params.spreadMeasurementDistance);
    printf("  spreadTopLimit           : %lu mm below surface\r\n", (unsigned long)params.spreadTopLimit);
    printf("  spreadBottomLimit        : %lu mm above bottom\r\n", (unsigned long)params.spreadBottomLimit);
    printf("  spreadPointHoverTime     : %lu ms\r\n", (unsigned long)params.spreadPointHoverTime);

    /* 水位测量参数 */
    printf("[Water Level]\r\n");
    printf("  waterLevelCorrection     : %lu\r\n", (unsigned long)params.waterLevelCorrection);
    printf("  maxDownDistance          : %lu mm\r\n", (unsigned long)params.maxDownDistance);

    /* 实高测量 */
    printf("[Real Tank Height]\r\n");
    printf("  refreshTankHeightFlag    : %s\r\n", params.refreshTankHeightFlag ? "Yes" : "No");
    printf("  maxTankHeightDeviation   : %lu mm\r\n", (unsigned long)params.maxTankHeightDeviation);
    printf("  initialTankHeight        : %lu mm\r\n", (unsigned long)params.initialTankHeight);
    printf("  currentTankHeight        : %lu mm\r\n", (unsigned long)params.currentTankHeight);

    /* 液位测量 */
    printf("[Oil Level]\r\n");
    printf("  oilLevelThreshold        : %lu mm\r\n", (unsigned long)params.oilLevelThreshold);
    printf("  liquidLevelMeasureMethod : %lu\r\n", (unsigned long)params.liquidLevelMeasurementMethod);

    /* 报警 DO */
    printf("[Alarm DO]\r\n");
    printf("  AlarmHighDO              : %lu\r\n", (unsigned long)params.AlarmHighDO);
    printf("  AlarmLowDO               : %lu\r\n", (unsigned long)params.AlarmLowDO);
    printf("  ThirdStateThreshold      : %lu\r\n", (unsigned long)params.ThirdStateThreshold);

    /* 4-20mA 输出 / AO */
    printf("[4-20mA / AO]\r\n");
    printf("  CurrentRangeStart_mA     : %lu mA\r\n", (unsigned long)params.CurrentRangeStart_mA);
    printf("  CurrentRangeEnd_mA       : %lu mA\r\n", (unsigned long)params.CurrentRangeEnd_mA);
    printf("  AlarmHighAO              : %lu\r\n", (unsigned long)params.AlarmHighAO);
    printf("  AlarmLowAO               : %lu\r\n", (unsigned long)params.AlarmLowAO);
    printf("  InitialCurrent_mA        : %lu mA\r\n", (unsigned long)params.InitialCurrent_mA);
    printf("  AOHighCurrent_mA         : %lu mA\r\n", (unsigned long)params.AOHighCurrent_mA);
    printf("  AOLowCurrent_mA          : %lu mA\r\n", (unsigned long)params.AOLowCurrent_mA);
    printf("  FaultCurrent_mA          : %lu mA\r\n", (unsigned long)params.FaultCurrent_mA);
    printf("  DebugCurrent_mA          : %lu mA\r\n", (unsigned long)params.DebugCurrent_mA);

    /* CRC 校验 */
    printf("[CRC]\r\n");
    printf("  crc                      : 0x%08lX\r\n", (unsigned long)params.crc);

    printf("======================================\r\n");
}


