/*
 * system_parameter.c
 *
 *  Created on: Feb 27, 2025
 *      Author: 1
 */
#include "system_parameter.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

volatile MeasurementResult g_measurement = { 0 }; // 测量结果
volatile DeviceParameters g_deviceParams = { 0 }; // 设备参数

/*********************** 参数存储逻辑 ***********************/
void save_device_params(void) {
    DeviceParameters params = g_deviceParams;

    // 计算CRC（不包含crc字段自身）
    uint32_t crc_size = sizeof(params) - sizeof(params.crc);
    params.crc = CRC32_HAL((uint8_t*)&params, crc_size);
    printf("保存设备参数，CRC: 0x%08X\n", (unsigned int)params.crc);

    // 通过现有接口写入
    WriteMultiData((uint8_t*)&params, FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));
    printf("设备参数已保存到FRAM。\n");
    print_device_params(); // 打印当前参数
}

/*********************** 参数加载逻辑 ***********************/
int load_device_params(void) {
    DeviceParameters temp;

    // 通过现有接口读取
    ReadMultiData((uint8_t*)&temp, FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));

    // 验证CRC
    uint32_t saved_crc = temp.crc;
    temp.crc = 0; // 清零后计算

    // 计算CRC（不包含crc字段自身）
    uint32_t crc_size = sizeof(temp) - sizeof(temp.crc);
    uint32_t calc_crc = CRC32_HAL((uint8_t*)&temp, crc_size);
    if (calc_crc == saved_crc) {
        memcpy((void* volatile)&g_deviceParams, &temp, sizeof(DeviceParameters));
        return 1; // 校验成功
    } else {
        printf("设备参数校验失败，CRC不匹配。\n");
        printf("校验计算的CRC: 0x%08X, 保存的CRC: 0x%08X\n",
               (unsigned int)calc_crc, (unsigned int)saved_crc);
    }
    return 0; // 校验失败
}

/*********************** 参数初始化 ***********************/
void init_device_params(void) {
    // 尝试加载参数
    if (!load_device_params()) {
        // 加载失败时初始化默认值
         memset((void* volatile)&g_deviceParams, 0, sizeof(DeviceParameters));
         RestoreFactoryParamsConfig(); //恢复出厂设置
        // 置错误代码
        g_measurement.device_status.error_code = PARAM_UNINITIALIZED;
        printf("设备参数加载失败\n");
    }
    print_device_params(); // 打印当前参数
}

/*-------------------------------------------------------
 * 恢复出厂参数配置
 *-----------------------------------------------------*/
void RestoreFactoryParamsConfig(void) {
    /* - 指令 - */
    g_deviceParams.command = CMD_NONE;

    /* - 基础参数 - */
    g_deviceParams.tankHeight = 200000;                    // 罐高：20000 mm（×10）
    g_deviceParams.blindZone = 3000;                       // 液位盲区：500 mm（×10）
    g_deviceParams.waterBlindZone = 100;                   // 水位盲区：500 mm（×10）
    g_deviceParams.encoder_wheel_circumference_mm = 95000; // 编码轮周长：95 mm（×1000）
    g_deviceParams.sensorType = DSM_SENSOR;                // 传感器类型
    g_deviceParams.sensorID = 1234567;                     // 传感器编号
    g_deviceParams.softwareVersion = 0x0001012;            // 软件版本：1.001

    /* - 称重参数 - */
    g_deviceParams.empty_weight = 0;                       // 空载重量：kg×100
    g_deviceParams.full_weight = 5000;                     // 满载重量：kg×100
    g_deviceParams.weight_upper_limit_ratio = 70;          // 上限比例：120.00 %
    g_deviceParams.weight_lower_limit_ratio = 30;          // 下限比例：80.00 %
    g_deviceParams.empty_weight_upper_limit = 2000;        // 空载重量上限
    g_deviceParams.empty_weight_lower_limit = 0;           // 空载重量下限
    g_deviceParams.full_weight_upper_limit = 30000;        // 满载重量上限
    g_deviceParams.full_weight_lower_limit = 2000;         // 满载重量下限
    g_deviceParams.findZeroDownDistance = 1000;            // 找零点下行距离：100 mm（×10）

    /* - 指令参数 - */
    g_deviceParams.calibrateOilLevel = 0;                  // 标定液位(油)：mm（×10）
    g_deviceParams.calibrateWaterLevel = 0;                // 标定液位(水)：mm（×10）
    g_deviceParams.singlePointMeasurementPosition = 0;     // 单点测量位置：mm（×10）
    g_deviceParams.singlePointMonitoringPosition = 0;      // 单点监测位置：mm（×10）
    g_deviceParams.densityDistributionOilLevel = 0;        // 分布测量液位：mm（×10）
    g_deviceParams.motorCommandDistance = 0;               // 电机指令运行距离：mm（×10）

    /* - 修正参数（密度/温度）- */
    g_deviceParams.densityCorrection = 0;                  // 密度修正
    g_deviceParams.temperatureCorrection = 0;              // 温度修正

    /* - 分布测量参数 - */
    g_deviceParams.requireBottomMeasurement = 1;           // 是否测罐底：0-否 1-是
    g_deviceParams.requireWaterMeasurement = 0;            // 是否测水位：0-否 1-是
    g_deviceParams.requireSinglePointDensity = 0;          // 是否测单点密度：0-否 1-是
    g_deviceParams.spreadMeasurementOrder = 0;             // 测量顺序：0=由下向上
    g_deviceParams.spreadMeasurementMode = 0;              // 测量模式：枚举
    g_deviceParams.spreadMeasurementCount = 5;             // 测量点数
    g_deviceParams.spreadMeasurementDistance = 1000;       // 点间距：1000 mm（×10）
    g_deviceParams.spreadTopLimit = 300;                   // 距液面上限：300 mm（×10）
    g_deviceParams.spreadBottomLimit = 300;                // 距罐地下限：300 mm（×10）
    g_deviceParams.spreadPointHoverTime = 10;              // 第一测量点悬停时间：10 s

    /* - 水位测量参数 - */
    g_deviceParams.waterLevelCorrection = 0;               // 水位修正：mm（×10）
    g_deviceParams.maxDownDistance = 3000;                 // 最大下行距离：300 mm（×10）

    /* - 实高测量 - */
    g_deviceParams.refreshTankHeightFlag = 0;              // 是否更新罐高：0-否 1-是
    g_deviceParams.maxTankHeightDeviation = 100;           // 罐高最大变化范围：10 mm（×10）
    g_deviceParams.initialTankHeight = 0;                  // 初始实高：mm（×10）
    g_deviceParams.currentTankHeight = 0;                  // 当前实高：mm（×10）

    /* - 液位测量 - */
    g_deviceParams.oilLevelThreshold = 3;                  // 找油阈值：0.3 mm（×10）
    g_deviceParams.liquidLevelMeasurementMethod = 0;       // 液位测量方式：枚举

    /* - 报警（DO）- */
    g_deviceParams.AlarmHighDO = 0;                        // 高液位报警(DO)：阈值或使能
    g_deviceParams.AlarmLowDO = 0;                         // 低液位报警(DO)：阈值或使能
    g_deviceParams.ThirdStateThreshold = 0;                // 第三状态阈值

    /* - 4~20 mA 输出（AO）- */
    g_deviceParams.CurrentRangeStart_mA = 400;             // 量程起点：4.00 mA（×100）
    g_deviceParams.CurrentRangeEnd_mA = 2000;              // 量程终点：20.00 mA（×100）
    g_deviceParams.AlarmHighAO = 2000;                     // 高限报警电流：20.00 mA（×100）
    g_deviceParams.AlarmLowAO = 400;                       // 低限报警电流：4.00 mA（×100）
    g_deviceParams.InitialCurrent_mA = 400;                // 初始电流：4.00 mA（×100）
    g_deviceParams.AOHighCurrent_mA = 2000;                // 高位电流：20.00 mA（×100）
    g_deviceParams.AOLowCurrent_mA = 400;                  // 低位电流：4.00 mA（×100）
    g_deviceParams.FaultCurrent_mA = 2200;                 // 故障电流：22.00 mA（×100）
    g_deviceParams.DebugCurrent_mA = 1200;                 // 调试电流：12.00 mA（×100）

    /* - CRC - */
    g_deviceParams.crc = 0;                                // 参数 CRC32（按需计算/存储）

    /* - 持久化 - */
    save_device_params();
}

/* 新增函数: 打印所有设备参数 */
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
    printf("  empty_weight             : %lu\r\n", (unsigned long)params.empty_weight);
    printf("  full_weight              : %lu\r\n", (unsigned long)params.full_weight);
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
