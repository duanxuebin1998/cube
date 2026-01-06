/*
 * system_parameter.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Duan Xuebin
 */

#include "system_parameter.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>  /* 为了使用 offsetof */

volatile MeasurementResult g_measurement = { 0 };   /* 测量结果 */
volatile DeviceParameters g_deviceParams = { 0 };   /* 设备参数 */

/* 参数版本号和 magic 常量 */
#define DEVICE_PARAM_VERSION   (1u)
#define DEVICE_PARAM_MAGIC     (0x4C54444Du)  /* 'LTDM' */

/*
 * 持久化区域说明:
 *  - DeviceParameters 中第一个字段 command 仅用于当前指令，不参与掉电参数校验
 *  - 从 tankHeight 开始到 crc 之前的区域为持久化参数区
 */
#define DEVICE_PARAM_PERSIST_OFFSET   (offsetof(DeviceParameters, tankHeight))
#define DEVICE_PARAM_PERSIST_LEN      (offsetof(DeviceParameters, crc) - DEVICE_PARAM_PERSIST_OFFSET)

#define DEVICE_PARAM_PERSIST_START(p) ((uint8_t *)(p) + DEVICE_PARAM_PERSIST_OFFSET)

/*========================= 参数存储逻辑 =========================*/

/* 保存设备参数到 FRAM */
void save_device_params(void)
{
    DeviceParameters params = g_deviceParams;

    /* 版本和结构体大小字段 */
    params.param_version = DEVICE_PARAM_VERSION;
    params.struct_size   = sizeof(DeviceParameters);
    params.magic         = DEVICE_PARAM_MAGIC;

    /* 计算 CRC, 覆盖从 tankHeight 起到 crc 之前的区域, 不包含 command 和 crc 自身 */
    uint8_t *crc_base = DEVICE_PARAM_PERSIST_START(&params);
    uint32_t crc_size = DEVICE_PARAM_PERSIST_LEN;

    params.crc = CRC32_HAL(crc_base, crc_size);

    printf("保存设备参数: version=%lu, size=%lu, CRC=0x%08lX\r\n",
           (unsigned long)params.param_version,
           (unsigned long)params.struct_size,
           (unsigned long)params.crc);

    /* 整块写入 FRAM, 包含 command 和所有字段 */
    WriteMultiData((uint8_t *)&params, FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));

    printf("设备参数已保存到 FRAM\r\n");

    print_device_params();
}

/*========================= 参数加载逻辑 =========================*/

/* 从 FRAM 加载设备参数, 返回 1 表示成功, 0 表示失败 */
int load_device_params(void)
{
    DeviceParameters temp;

    /* 从 FRAM 读取整块结构体 */
    ReadMultiData((uint8_t *)&temp, FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));

    /* 检查 magic 字段 */
    if (temp.magic != DEVICE_PARAM_MAGIC)
    {
        printf("设备参数 magic 不匹配, 读取到: 0x%08lX, 使用默认参数\r\n",
               (unsigned long)temp.magic);
        return 0;
    }

    /* 检查结构体大小 */
    if (temp.struct_size != sizeof(DeviceParameters))
    {
        printf("设备参数 struct_size 不匹配, FRAM=%lu, 当前=%lu, 使用默认参数\r\n",
               (unsigned long)temp.struct_size,
               (unsigned long)sizeof(DeviceParameters));
        return 0;
    }

    /* 检查版本号 */
    if (temp.param_version != DEVICE_PARAM_VERSION)
    {
        printf("设备参数版本不匹配, FRAM=%lu, 当前=%lu, 使用默认参数\r\n",
               (unsigned long)temp.param_version,
               (unsigned long)DEVICE_PARAM_VERSION);
        return 0;
    }

    /* 计算 CRC */
    uint8_t *crc_base = DEVICE_PARAM_PERSIST_START(&temp);
    uint32_t crc_size = DEVICE_PARAM_PERSIST_LEN;
    uint32_t calc_crc = CRC32_HAL(crc_base, crc_size);

    if (calc_crc != temp.crc)
    {
        printf("设备参数 CRC 校验失败, 计算=0x%08lX, FRAM=0x%08lX\r\n",
               (unsigned long)calc_crc,
               (unsigned long)temp.crc);
        return 0;
    }

    /* 校验成功, 拷贝到全局变量 */
    memcpy((void * volatile)&g_deviceParams, &temp, sizeof(DeviceParameters));

    /* 上电时当前指令从默认指令恢复, 而不是沿用 FRAM 中旧的 command */
    g_deviceParams.command = g_deviceParams.powerOnDefaultCommand;

    printf("设备参数加载成功\r\n");
    return 1;
}

/*========================= 参数初始化 =========================*/

/* 初始化设备参数模块 */
void init_device_params(void)
{
    if (!load_device_params())
    {
        /* 如果加载失败, 恢复出厂参数并保存 */
        memset((void * volatile)&g_deviceParams, 0, sizeof(DeviceParameters));
        RestoreFactoryParamsConfig();    /* 函数内部会调用 save_device_params */

        g_measurement.device_status.error_code = PARAM_UNINITIALIZED;

        printf("设备参数加载失败, 已恢复出厂设置\r\n");
    }

    print_device_params();
}

/*========================= 恢复出厂参数 =========================*/

/*
 * 恢复出厂参数配置
 * 注意: 这里设置的是默认值, 可根据实际项目需要调整
 */
void RestoreFactoryParamsConfig(void)
{
    /* 整体清零, 保证保留字段等为 0 */
    memset((void * volatile)&g_deviceParams, 0, sizeof(DeviceParameters));

    /* 指令类字段 */
    g_deviceParams.command              = CMD_NONE;  /* 当前指令默认空闲 */
    g_deviceParams.powerOnDefaultCommand = CMD_NONE; /* 上电默认不自动执行命令 */

    /* 基础参数 */
    g_deviceParams.tankHeight                  = 200000;   /* 罐高, 单位: 0.1mm, 即 20000.0mm */
    g_deviceParams.blindZone                   = 3000;     /* 液位盲区, 0.1mm */
    g_deviceParams.waterBlindZone              = 100;      /* 水位盲区, 0.1mm */
    g_deviceParams.encoder_wheel_circumference_mm = 95000; /* 编码轮周长, 0.001mm */
    g_deviceParams.max_motor_speed             = 400;        /* 最大电机速度, r/s, 按需要调整 */
    g_deviceParams.sensorType                  = DSM_SENSOR;
    g_deviceParams.sensorID                    = 1234567;
    g_deviceParams.sensorSoftwareVersion       = 0x00010001;
    g_deviceParams.softwareVersion             = 0x00010001;

    g_deviceParams.findZeroDownDistance        = 1000;     /* 找零点下行距离, 0.1mm */
    g_deviceParams.first_loop_circumference_mm = 200000;   /* 尺带首圈周长, 0.1mm, 根据实际调整 */
    g_deviceParams.tape_thickness_mm           = 200;      /* 尺带厚度, 0.001mm */

    /* 称重参数 */
    g_deviceParams.empty_weight                = 0;
    g_deviceParams.full_weight                 = 5000;
    g_deviceParams.weight_upper_limit_ratio    = 50;    /* 120.00% */
    g_deviceParams.weight_lower_limit_ratio    = 20;     /* 80.00% */
    g_deviceParams.empty_weight_upper_limit    = 2000;
    g_deviceParams.empty_weight_lower_limit    = 0;
    g_deviceParams.full_weight_upper_limit     = 30000;
    g_deviceParams.full_weight_lower_limit     = 2000;

    /* 指令参数 */
    g_deviceParams.calibrateOilLevel               = 0;
    g_deviceParams.calibrateWaterLevel             = 0;
    g_deviceParams.singlePointMeasurementPosition  = 0;
    g_deviceParams.singlePointMonitoringPosition   = 0;
    g_deviceParams.densityDistributionOilLevel     = 0;
    g_deviceParams.motorCommandDistance            = 0;

    /* 密度和温度修正参数 */
    g_deviceParams.densityCorrection               = 0;
    g_deviceParams.temperatureCorrection           = 0;

    /* 分布测量参数 */
    g_deviceParams.requireBottomMeasurement        = 1;    /* 默认测罐底 */
    g_deviceParams.requireWaterMeasurement         = 0;
    g_deviceParams.requireSinglePointDensity       = 0;
    g_deviceParams.spreadMeasurementOrder          = 0;    /* 0 表示自下而上 */
    g_deviceParams.spreadMeasurementMode           = 0;    /* 根据项目定义 */
    g_deviceParams.spreadMeasurementCount          = 5;
    g_deviceParams.spreadMeasurementDistance       = 1000; /* 点间距, 0.1mm */
    g_deviceParams.spreadTopLimit                  = 300;  /* 距液面上限, 0.1mm */
    g_deviceParams.spreadBottomLimit               = 300;  /* 距罐地下限, 0.1mm */
    g_deviceParams.spreadPointHoverTime            = 10;   /* 首点悬停时间, 单位按实际定义 */

    /* Wartsila 密度区间测量参数 */
    g_deviceParams.wartsila_upper_density_limit      = 38000;
    g_deviceParams.wartsila_lower_density_limit      = 500;
    g_deviceParams.wartsila_density_interval         = 1000;
    g_deviceParams.wartsila_max_height_above_surface = 200; /* 0.1mm */

    /* 水位测量参数 */
    g_deviceParams.waterLevelCorrection           = 0;
    g_deviceParams.maxDownDistance                = 3000;  /* 最大下行距离, 0.1mm */

    /* 实高测量 */
    g_deviceParams.refreshTankHeightFlag          = 0;
    g_deviceParams.maxTankHeightDeviation         = 100;   /* 0.1mm */
    g_deviceParams.initialTankHeight              = 0;
    g_deviceParams.currentTankHeight              = 0;

    /* 液位测量 */
    g_deviceParams.oilLevelThreshold              = 15;     /* 0.1mm */
    g_deviceParams.oilLevelHysteresisThreshold    = 20;     /* 0.1mm */
    g_deviceParams.liquidLevelMeasurementMethod   = 0;     /* 根据项目定义 */

    /* 报警 DO */
    g_deviceParams.AlarmHighDO                    = 0;
    g_deviceParams.AlarmLowDO                     = 0;
    g_deviceParams.ThirdStateThreshold            = 0;

    /* 4~20mA 输出 AO */
    g_deviceParams.CurrentRangeStart_mA           = 400;   /* 4.00mA, ×0.01 */
    g_deviceParams.CurrentRangeEnd_mA             = 2000;  /* 20.00mA, ×0.01 */
    g_deviceParams.AlarmHighAO                    = 2000;
    g_deviceParams.AlarmLowAO                     = 400;
    g_deviceParams.InitialCurrent_mA              = 400;
    g_deviceParams.AOHighCurrent_mA               = 2000;
    g_deviceParams.AOLowCurrent_mA                = 400;
    g_deviceParams.FaultCurrent_mA                = 2200;  /* 22.00mA */
    g_deviceParams.DebugCurrent_mA                = 1200;  /* 12.00mA */

    /* 版本和校验相关字段, CRC 在 save_device_params 中更新 */
    g_deviceParams.param_version = DEVICE_PARAM_VERSION;
    g_deviceParams.struct_size   = sizeof(DeviceParameters);
    g_deviceParams.magic         = DEVICE_PARAM_MAGIC;
    g_deviceParams.crc           = 0;

    save_device_params();
}

/*========================= 参数打印 =========================*/

/* 打印所有设备参数, 便于调试 */
void print_device_params(void)
{
    DeviceParameters params;
    memcpy(&params, (void *)&g_deviceParams, sizeof(DeviceParameters));

    printf("========== Device Parameters ==========\r\n");

    /* 指令 */
    printf("[Command]\r\n");
    printf("  command                : %u\r\n", (unsigned)params.command);
    printf("  powerOnDefaultCommand  : %u\r\n", (unsigned)params.powerOnDefaultCommand);

    /* 基础参数 */
    printf("[Basic Params]\r\n");
    printf("  tankHeight(0.1mm)          : %lu\r\n", (unsigned long)params.tankHeight);
    printf("  blindZone(0.1mm)           : %lu\r\n", (unsigned long)params.blindZone);
    printf("  waterBlindZone(0.1mm)      : %lu\r\n", (unsigned long)params.waterBlindZone);
    printf("  encoder_circ(0.001mm)      : %lu\r\n", (unsigned long)params.encoder_wheel_circumference_mm);
    printf("  max_motor_speed(r/s)       : %lu\r\n", (unsigned long)params.max_motor_speed);
    printf("  sensorType                 : %lu\r\n", (unsigned long)params.sensorType);
    printf("  sensorID                   : %lu\r\n", (unsigned long)params.sensorID);
    printf("  sensorSoftwareVersion      : 0x%08lX\r\n", (unsigned long)params.sensorSoftwareVersion);
    printf("  softwareVersion            : 0x%08lX\r\n", (unsigned long)params.softwareVersion);
    printf("  findZeroDownDistance(0.1mm): %lu\r\n", (unsigned long)params.findZeroDownDistance);
    printf("  first_loop_circ(0.1mm)     : %lu\r\n", (unsigned long)params.first_loop_circumference_mm);
    printf("  tape_thickness(0.001mm)    : %lu\r\n", (unsigned long)params.tape_thickness_mm);

    /* 称重参数 */
    printf("[Weight Params]\r\n");
    printf("  empty_weight               : %lu\r\n", (unsigned long)params.empty_weight);
    printf("  full_weight                : %lu\r\n", (unsigned long)params.full_weight);
    printf("  weight_upper_limit_ratio   : %lu\r\n", (unsigned long)params.weight_upper_limit_ratio);
    printf("  weight_lower_limit_ratio   : %lu\r\n", (unsigned long)params.weight_lower_limit_ratio);
    printf("  empty_weight_upper_limit   : %lu\r\n", (unsigned long)params.empty_weight_upper_limit);
    printf("  empty_weight_lower_limit   : %lu\r\n", (unsigned long)params.empty_weight_lower_limit);
    printf("  full_weight_upper_limit    : %lu\r\n", (unsigned long)params.full_weight_upper_limit);
    printf("  full_weight_lower_limit    : %lu\r\n", (unsigned long)params.full_weight_lower_limit);

    /* 指令参数 */
    printf("[Command Params]\r\n");
    printf("  calibrateOilLevel(0.1mm)       : %lu\r\n", (unsigned long)params.calibrateOilLevel);
    printf("  calibrateWaterLevel(0.1mm)     : %lu\r\n", (unsigned long)params.calibrateWaterLevel);
    printf("  singlePointMeasurePos(0.1mm)   : %lu\r\n", (unsigned long)params.singlePointMeasurementPosition);
    printf("  singlePointMonitorPos(0.1mm)   : %lu\r\n", (unsigned long)params.singlePointMonitoringPosition);
    printf("  densityDistributionOilLevel    : %lu\r\n", (unsigned long)params.densityDistributionOilLevel);
    printf("  motorCommandDistance(0.1mm)    : %lu\r\n", (unsigned long)params.motorCommandDistance);

    /* 密度 / 温度 / 分布参数 */
    printf("[Density / Temp / Spread]\r\n");
    printf("  densityCorrection             : %lu\r\n", (unsigned long)params.densityCorrection);
    printf("  temperatureCorrection         : %lu\r\n", (unsigned long)params.temperatureCorrection);
    printf("  requireBottomMeasurement      : %s\r\n", params.requireBottomMeasurement ? "Yes" : "No");
    printf("  requireWaterMeasurement       : %s\r\n", params.requireWaterMeasurement ? "Yes" : "No");
    printf("  requireSinglePointDensity     : %s\r\n", params.requireSinglePointDensity ? "Yes" : "No");
    printf("  spreadMeasurementOrder        : %lu\r\n", (unsigned long)params.spreadMeasurementOrder);
    printf("  spreadMeasurementMode         : %lu\r\n", (unsigned long)params.spreadMeasurementMode);
    printf("  spreadMeasurementCount        : %lu\r\n", (unsigned long)params.spreadMeasurementCount);
    printf("  spreadMeasurementDistance(0.1mm): %lu\r\n", (unsigned long)params.spreadMeasurementDistance);
    printf("  spreadTopLimit(0.1mm)         : %lu\r\n", (unsigned long)params.spreadTopLimit);
    printf("  spreadBottomLimit(0.1mm)      : %lu\r\n", (unsigned long)params.spreadBottomLimit);
    printf("  spreadPointHoverTime          : %lu\r\n", (unsigned long)params.spreadPointHoverTime);

    /* Wartsila 密度区间 */
    printf("[Wartsila Density Range]\r\n");
    printf("  upper_density_limit           : %lu\r\n", (unsigned long)params.wartsila_upper_density_limit);
    printf("  lower_density_limit           : %lu\r\n", (unsigned long)params.wartsila_lower_density_limit);
    printf("  density_interval              : %lu\r\n", (unsigned long)params.wartsila_density_interval);
    printf("  max_height_above_surface(0.1mm): %lu\r\n", (unsigned long)params.wartsila_max_height_above_surface);

    /* 水位测量 */
    printf("[Water Level]\r\n");
    printf("  waterLevelCorrection          : %lu\r\n", (unsigned long)params.waterLevelCorrection);
    printf("  maxDownDistance(0.1mm)        : %lu\r\n", (unsigned long)params.maxDownDistance);

    /* 实高测量 */
    printf("[Real Tank Height]\r\n");
    printf("  refreshTankHeightFlag         : %s\r\n", params.refreshTankHeightFlag ? "Yes" : "No");
    printf("  maxTankHeightDeviation(0.1mm) : %lu\r\n", (unsigned long)params.maxTankHeightDeviation);
    printf("  initialTankHeight(0.1mm)      : %lu\r\n", (unsigned long)params.initialTankHeight);
    printf("  currentTankHeight(0.1mm)      : %lu\r\n", (unsigned long)params.currentTankHeight);

    /* 液位测量 */
    printf("[Oil Level]\r\n");
    printf("  oilLevelThreshold(0.1mm)      : %lu\r\n", (unsigned long)params.oilLevelThreshold);
    printf("  oilLevelHysteresis(0.1mm)     : %lu\r\n", (unsigned long)params.oilLevelHysteresisThreshold);
    printf("  liquidLevelMeasureMethod      : %lu\r\n", (unsigned long)params.liquidLevelMeasurementMethod);

    /* 报警 DO */
    printf("[Alarm DO]\r\n");
    printf("  AlarmHighDO                   : %lu\r\n", (unsigned long)params.AlarmHighDO);
    printf("  AlarmLowDO                    : %lu\r\n", (unsigned long)params.AlarmLowDO);
    printf("  ThirdStateThreshold           : %lu\r\n", (unsigned long)params.ThirdStateThreshold);

    /* 4~20mA / AO */
    printf("[4-20mA / AO]\r\n");
    printf("  CurrentRangeStart_mA (×0.01)  : %lu\r\n", (unsigned long)params.CurrentRangeStart_mA);
    printf("  CurrentRangeEnd_mA   (×0.01)  : %lu\r\n", (unsigned long)params.CurrentRangeEnd_mA);
    printf("  AlarmHighAO                   : %lu\r\n", (unsigned long)params.AlarmHighAO);
    printf("  AlarmLowAO                    : %lu\r\n", (unsigned long)params.AlarmLowAO);
    printf("  InitialCurrent_mA   (×0.01)   : %lu\r\n", (unsigned long)params.InitialCurrent_mA);
    printf("  AOHighCurrent_mA    (×0.01)   : %lu\r\n", (unsigned long)params.AOHighCurrent_mA);
    printf("  AOLowCurrent_mA     (×0.01)   : %lu\r\n", (unsigned long)params.AOLowCurrent_mA);
    printf("  FaultCurrent_mA     (×0.01)   : %lu\r\n", (unsigned long)params.FaultCurrent_mA);
    printf("  DebugCurrent_mA     (×0.01)   : %lu\r\n", (unsigned long)params.DebugCurrent_mA);

    /* 元信息和 CRC */
    printf("[Meta]\r\n");
    printf("  param_version                 : %lu\r\n", (unsigned long)params.param_version);
    printf("  struct_size                   : %lu\r\n", (unsigned long)params.struct_size);
    printf("  magic                         : 0x%08lX\r\n", (unsigned long)params.magic);
    printf("  crc                           : 0x%08lX\r\n", (unsigned long)params.crc);

    printf("======================================\r\n");
}

void PrintMeasurementResult(const MeasurementResult *m)
{
    if (!m) return;

    printf("\r\n=====================【设备实时测量结果】=====================\r\n");

    /* 1. 设备状态 */
    printf("【设备状态】\r\n");
    printf("  工作模式: %lu\r\n", m->device_status.work_mode);
    printf("  设备状态: %d\r\n",  m->device_status.device_state);
    printf("  错误代码: %lu\r\n", m->device_status.error_code);
    printf("  当前指令: %d\r\n",  m->device_status.current_command);
    printf("  零点状态: %lu\r\n", m->device_status.zero_point_status);

    printf("--------------------------------------------------------------\r\n");

    /* 2. 调试数据 */
    printf("【调试数据 DebugData】\r\n");
    printf("  编码值: %ld\r\n",       m->debug_data.current_encoder_value);
    printf("  传感器位置: %ld mm\r\n", m->debug_data.sensor_position);
    printf("  尺带长度: %ld mm\r\n",   m->debug_data.cable_length);

    printf("  当前频率: %lu Hz\r\n",   m->debug_data.frequency);
    printf("  温度: %0.2f ℃\r\n",      m->debug_data.temperature / 100.0f);
    printf("  空气中频率: %lu Hz\r\n", m->debug_data.air_frequency);
    printf("  幅值: %lu\r\n",          m->debug_data.current_amplitude);
    printf("  水位电压/电容值: %lu\r\n", m->debug_data.water_level_voltage);

    printf("  当前称重值: %lu\r\n",   m->debug_data.current_weight);
    printf("  称重参数: %lu\r\n",     m->debug_data.weight_param);

    printf("  X角度: %ld\r\n", m->debug_data.angle_x);
    printf("  Y角度: %ld\r\n", m->debug_data.angle_y);

    printf("  电机速度: %lu\r\n", m->debug_data.motor_speed);
    printf("  电机状态: %lu (%s)\r\n",
        m->debug_data.motor_state,
        (m->debug_data.motor_state == 0) ? "停止" :
        (m->debug_data.motor_state == 1) ? "上行" :
        (m->debug_data.motor_state == 2) ? "下行" : "未知");

    printf("--------------------------------------------------------------\r\n");

    /* 3. 液位测量 */
    printf("【液位测量 OilMeasurement】\r\n");
    printf("  跟随液位: %lu mm\r\n",     m->oil_measurement.oil_level);
    printf("  空气频率: %lu Hz\r\n",      m->oil_measurement.air_frequency);
    printf("  油中频率: %lu Hz\r\n",      m->oil_measurement.oil_frequency);
    printf("  跟随频率: %lu Hz\r\n",      m->oil_measurement.follow_frequency);
    printf("  当前频率: %lu Hz\r\n",      m->oil_measurement.current_frequency);

    printf("--------------------------------------------------------------\r\n");

    /* 4. 水位测量 */
    printf("【水位测量 WaterMeasurement】\r\n");
    printf("  水位值: %lu mm\r\n",     m->water_measurement.water_level);
    printf("  零点电容: %.3f\r\n",     m->water_measurement.zero_capacitance);
    printf("  油区电容: %.3f\r\n",     m->water_measurement.oil_capacitance);
    printf("  当前电容: %.3f\r\n",     m->water_measurement.current_capacitance);

    printf("--------------------------------------------------------------\r\n");

    /* 5. 实高测量 */
    printf("【实高测量 ActualHeight】\r\n");
    printf("  标定液位实高: %lu mm\r\n", m->height_measurement.calibrated_liquid_level);
    printf("  当前实高: %lu mm\r\n",     m->height_measurement.current_real_height);

    printf("--------------------------------------------------------------\r\n");

    /* 6. 单点密度测量 */
    printf("【单点密度测量 SinglePoint】\r\n");
    PrintDensity("单点测量", &m->single_point_measurement);

    /* 7. 单点监测 */
    printf("【单点监测 SinglePoint Monitoring】\r\n");
    PrintDensity("单点监测", &m->single_point_monitoring);

    printf("--------------------------------------------------------------\r\n");

    /* 8. 密度分布（概要） */
    printf("【密度分布 DensityDistribution】\r\n");
    printf("  平均温度: %lu\r\n",     m->density_distribution.average_temperature);
    printf("  平均密度: %lu\r\n",     m->density_distribution.average_density);
    printf("  平均计重密度: %lu\r\n", m->density_distribution.average_weight_density);
    printf("  测量点数: %lu\r\n",     m->density_distribution.measurement_points);
    printf("  测量时液位: %lu mm\r\n", m->density_distribution.Density_oil_level);

    printf("  --- 单点数据（仅打印前10个）---\r\n");
    for (uint32_t i = 0; i < 10 && i < m->density_distribution.measurement_points; i++) {
        const DensityMeasurement *d = &m->density_distribution.single_density_data[i];
        printf("    [%02lu] T=%lu ρ=%lu ρ15=%lu VCF=%lu WD=%lu Pos=%lu\r\n",
            i,
            d->temperature,
            d->density,
            d->standard_density,
            d->vcf20,
            d->weight_density,
            d->temperature_position
        );
    }

    printf("========================【打印结束】========================\r\n");
}

void PrintDensity(const char *title, const DensityMeasurement *d)
{
    printf("  [%s]\r\n", title);
    printf("    温度: %lu\r\n", d->temperature);
    printf("    密度: %lu\r\n", d->density);
    printf("    标准密度: %lu\r\n", d->standard_density);
    printf("    VCF20: %lu\r\n", d->vcf20);
    printf("    计重密度: %lu\r\n", d->weight_density);
    printf("    温度位置: %lu\r\n", d->temperature_position);
}

//#include "system_parameter.h"
//#include "mb85rs2m.h"
//#include "my_crc.h"
//#include <stdint.h>
//#include <string.h>
//#include <stdio.h>
//
//volatile MeasurementResult g_measurement = { 0 }; // 测量结果
//volatile DeviceParameters g_deviceParams = { 0 }; // 设备参数
//
///*********************** 参数存储逻辑 ***********************/
//void save_device_params(void) {
//    DeviceParameters params = g_deviceParams;
//
//    // 计算CRC（不包含crc字段自身）
//    uint32_t crc_size = sizeof(params) - sizeof(params.crc);
//    params.crc = CRC32_HAL((uint8_t*)&params, crc_size);
//    printf("保存设备参数，CRC: 0x%08X\n", (unsigned int)params.crc);
//
//    // 通过现有接口写入
//    WriteMultiData((uint8_t*)&params, FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));
//    printf("设备参数已保存到FRAM。\n");
//    print_device_params(); // 打印当前参数
//}
//
///*********************** 参数加载逻辑 ***********************/
//int load_device_params(void) {
//    DeviceParameters temp;
//
//    // 通过现有接口读取
//    ReadMultiData((uint8_t*)&temp, FRAM_PARAM_ADDRESS, sizeof(DeviceParameters));
//
//    // 验证CRC
//    uint32_t saved_crc = temp.crc;
//    temp.crc = 0; // 清零后计算
//
//    // 计算CRC（不包含crc字段自身）
//    uint32_t crc_size = sizeof(temp) - sizeof(temp.crc);
//    uint32_t calc_crc = CRC32_HAL((uint8_t*)&temp, crc_size);
//    if (calc_crc == saved_crc) {
//        memcpy((void* volatile)&g_deviceParams, &temp, sizeof(DeviceParameters));
//        return 1; // 校验成功
//    } else {
//        printf("设备参数校验失败，CRC不匹配。\n");
//        printf("校验计算的CRC: 0x%08X, 保存的CRC: 0x%08X\n",
//               (unsigned int)calc_crc, (unsigned int)saved_crc);
//    }
//    return 0; // 校验失败
//}
//
///*********************** 参数初始化 ***********************/
//void init_device_params(void) {
//    // 尝试加载参数
//    if (!load_device_params()) {
//        // 加载失败时初始化默认值
//         memset((void* volatile)&g_deviceParams, 0, sizeof(DeviceParameters));
//         RestoreFactoryParamsConfig(); //恢复出厂设置
//        // 置错误代码
//        g_measurement.device_status.error_code = PARAM_UNINITIALIZED;
//        printf("设备参数加载失败\n");
//    }
//    print_device_params(); // 打印当前参数
//}
//
///*-------------------------------------------------------
// * 恢复出厂参数配置
// *-----------------------------------------------------*/
//void RestoreFactoryParamsConfig(void) {
//    /* - 指令 - */
//    g_deviceParams.command = CMD_NONE;
//
//    /* - 基础参数 - */
//    g_deviceParams.tankHeight = 200000;                    // 罐高：20000 mm（×10）
//    g_deviceParams.blindZone = 3000;                       // 液位盲区：500 mm（×10）
//    g_deviceParams.waterBlindZone = 100;                   // 水位盲区：500 mm（×10）
//    g_deviceParams.encoder_wheel_circumference_mm = 95000; // 编码轮周长：95 mm（×1000）
//    g_deviceParams.sensorType = DSM_SENSOR;                // 传感器类型
//    g_deviceParams.sensorID = 1234567;                     // 传感器编号
//    g_deviceParams.softwareVersion = 0x0001012;            // 软件版本：1.001
//
//    /* - 称重参数 - */
//    g_deviceParams.empty_weight = 0;                       // 空载重量：kg×100
//    g_deviceParams.full_weight = 5000;                     // 满载重量：kg×100
//    g_deviceParams.weight_upper_limit_ratio = 70;          // 上限比例：120.00 %
//    g_deviceParams.weight_lower_limit_ratio = 30;          // 下限比例：80.00 %
//    g_deviceParams.empty_weight_upper_limit = 2000;        // 空载重量上限
//    g_deviceParams.empty_weight_lower_limit = 0;           // 空载重量下限
//    g_deviceParams.full_weight_upper_limit = 30000;        // 满载重量上限
//    g_deviceParams.full_weight_lower_limit = 2000;         // 满载重量下限
//    g_deviceParams.findZeroDownDistance = 1000;            // 找零点下行距离：100 mm（×10）
//
//    /* - 指令参数 - */
//    g_deviceParams.calibrateOilLevel = 0;                  // 标定液位(油)：mm（×10）
//    g_deviceParams.calibrateWaterLevel = 0;                // 标定液位(水)：mm（×10）
//    g_deviceParams.singlePointMeasurementPosition = 0;     // 单点测量位置：mm（×10）
//    g_deviceParams.singlePointMonitoringPosition = 0;      // 单点监测位置：mm（×10）
//    g_deviceParams.densityDistributionOilLevel = 0;        // 分布测量液位：mm（×10）
//    g_deviceParams.motorCommandDistance = 0;               // 电机指令运行距离：mm（×10）
//
//    /* - 修正参数（密度/温度）- */
//    g_deviceParams.densityCorrection = 0;                  // 密度修正
//    g_deviceParams.temperatureCorrection = 0;              // 温度修正
//
//    /* - 分布测量参数 - */
//    g_deviceParams.requireBottomMeasurement = 1;           // 是否测罐底：0-否 1-是
//    g_deviceParams.requireWaterMeasurement = 0;            // 是否测水位：0-否 1-是
//    g_deviceParams.requireSinglePointDensity = 0;          // 是否测单点密度：0-否 1-是
//    g_deviceParams.spreadMeasurementOrder = 0;             // 测量顺序：0=由下向上
//    g_deviceParams.spreadMeasurementMode = 0;              // 测量模式：枚举
//    g_deviceParams.spreadMeasurementCount = 5;             // 测量点数
//    g_deviceParams.spreadMeasurementDistance = 1000;       // 点间距：1000 mm（×10）
//    g_deviceParams.spreadTopLimit = 300;                   // 距液面上限：300 mm（×10）
//    g_deviceParams.spreadBottomLimit = 300;                // 距罐地下限：300 mm（×10）
//    g_deviceParams.spreadPointHoverTime = 10;              // 第一测量点悬停时间：10 s
//
//    /* - 水位测量参数 - */
//    g_deviceParams.waterLevelCorrection = 0;               // 水位修正：mm（×10）
//    g_deviceParams.maxDownDistance = 3000;                 // 最大下行距离：300 mm（×10）
//
//    /* - 实高测量 - */
//    g_deviceParams.refreshTankHeightFlag = 0;              // 是否更新罐高：0-否 1-是
//    g_deviceParams.maxTankHeightDeviation = 100;           // 罐高最大变化范围：10 mm（×10）
//    g_deviceParams.initialTankHeight = 0;                  // 初始实高：mm（×10）
//    g_deviceParams.currentTankHeight = 0;                  // 当前实高：mm（×10）
//
//    /* - 液位测量 - */
//    g_deviceParams.oilLevelThreshold = 3;                  // 找油阈值：0.3 mm（×10）
//    g_deviceParams.liquidLevelMeasurementMethod = 0;       // 液位测量方式：枚举
//
//    /* - 报警（DO）- */
//    g_deviceParams.AlarmHighDO = 0;                        // 高液位报警(DO)：阈值或使能
//    g_deviceParams.AlarmLowDO = 0;                         // 低液位报警(DO)：阈值或使能
//    g_deviceParams.ThirdStateThreshold = 0;                // 第三状态阈值
//
//    /* - 4~20 mA 输出（AO）- */
//    g_deviceParams.CurrentRangeStart_mA = 400;             // 量程起点：4.00 mA（×100）
//    g_deviceParams.CurrentRangeEnd_mA = 2000;              // 量程终点：20.00 mA（×100）
//    g_deviceParams.AlarmHighAO = 2000;                     // 高限报警电流：20.00 mA（×100）
//    g_deviceParams.AlarmLowAO = 400;                       // 低限报警电流：4.00 mA（×100）
//    g_deviceParams.InitialCurrent_mA = 400;                // 初始电流：4.00 mA（×100）
//    g_deviceParams.AOHighCurrent_mA = 2000;                // 高位电流：20.00 mA（×100）
//    g_deviceParams.AOLowCurrent_mA = 400;                  // 低位电流：4.00 mA（×100）
//    g_deviceParams.FaultCurrent_mA = 2200;                 // 故障电流：22.00 mA（×100）
//    g_deviceParams.DebugCurrent_mA = 1200;                 // 调试电流：12.00 mA（×100）
//
//    /* - CRC - */
//    g_deviceParams.crc = 0;                                // 参数 CRC32（按需计算/存储）
//
//    /* - 持久化 - */
//    save_device_params();
//}
//
///* 新增函数: 打印所有设备参数 */
//void print_device_params(void)
//{
//    // 创建参数副本以避免多次访问 volatile 变量
//    DeviceParameters params;
//    memcpy(&params, (void*)&g_deviceParams, sizeof(DeviceParameters));
//
//    printf("========== Device Parameters ==========\r\n");
//
//    /* 指令 */
//    printf("[Command]\r\n");
//    printf("  command                : %u\r\n", (unsigned)params.command);
//
//    /* 基础参数 */
//    printf("[Basic Params]\r\n");
//    printf("  tankHeight             : %lu mm\r\n", (unsigned long)params.tankHeight);
//    printf("  blindZone              : %lu mm\r\n", (unsigned long)params.blindZone);
//    printf("  waterBlindZone         : %lu mm\r\n", (unsigned long)params.waterBlindZone);
//    printf("  encoder_circumference  : %lu mm\r\n", (unsigned long)params.encoder_wheel_circumference_mm);
//    printf("  sensorType             : %lu\r\n", (unsigned long)params.sensorType);
//    printf("  sensorID               : %lu\r\n", (unsigned long)params.sensorID);
//    printf("  softwareVersion        : %lu\r\n", (unsigned long)params.softwareVersion);
//
//    /* 称重参数 */
//    printf("[Weight Params]\r\n");
//    printf("  empty_weight             : %lu\r\n", (unsigned long)params.empty_weight);
//    printf("  full_weight              : %lu\r\n", (unsigned long)params.full_weight);
//    printf("  weight_upper_limit_ratio : %lu\r\n", (unsigned long)params.weight_upper_limit_ratio);
//    printf("  weight_lower_limit_ratio : %lu\r\n", (unsigned long)params.weight_lower_limit_ratio);
//    printf("  empty_weight_upper_limit : %lu\r\n", (unsigned long)params.empty_weight_upper_limit);
//    printf("  empty_weight_lower_limit : %lu\r\n", (unsigned long)params.empty_weight_lower_limit);
//    printf("  full_weight_upper_limit  : %lu\r\n", (unsigned long)params.full_weight_upper_limit);
//    printf("  full_weight_lower_limit  : %lu\r\n", (unsigned long)params.full_weight_lower_limit);
//    printf("  findZeroDownDistance     : %lu mm\r\n", (unsigned long)params.findZeroDownDistance);
//
//    /* 指令参数 */
//    printf("[Command Params]\r\n");
//    printf("  calibrateOilLevel            : %lu mm\r\n", (unsigned long)params.calibrateOilLevel);
//    printf("  calibrateWaterLevel          : %lu mm\r\n", (unsigned long)params.calibrateWaterLevel);
//    printf("  singlePointMeasurementPos    : %lu mm\r\n", (unsigned long)params.singlePointMeasurementPosition);
//    printf("  singlePointMonitoringPos     : %lu mm\r\n", (unsigned long)params.singlePointMonitoringPosition);
//    printf("  densityDistributionOilLevel  : %lu mm\r\n", (unsigned long)params.densityDistributionOilLevel);
//    printf("  motorCommandDistance         : %lu mm\r\n", (unsigned long)params.motorCommandDistance);
//
//    /* 密度和温度测量参数 + 分布测量参数 */
//    printf("[Density / Temp / Spread]\r\n");
//    printf("  densityCorrection        : %lu\r\n", (unsigned long)params.densityCorrection);
//    printf("  temperatureCorrection    : %lu\r\n", (unsigned long)params.temperatureCorrection);
//    printf("  requireBottomMeasurement : %s\r\n", params.requireBottomMeasurement ? "Yes" : "No");
//    printf("  requireWaterMeasurement  : %s\r\n", params.requireWaterMeasurement ? "Yes" : "No");
//    printf("  requireSinglePointDensity: %s\r\n", params.requireSinglePointDensity ? "Yes" : "No");
//    printf("  spreadMeasurementOrder   : %lu (0=Bottom-Up)\r\n", (unsigned long)params.spreadMeasurementOrder);
//    printf("  spreadMeasurementMode    : %lu\r\n", (unsigned long)params.spreadMeasurementMode);
//    printf("  spreadMeasurementCount   : %lu points\r\n", (unsigned long)params.spreadMeasurementCount);
//    printf("  spreadMeasurementDistance: %lu mm\r\n", (unsigned long)params.spreadMeasurementDistance);
//    printf("  spreadTopLimit           : %lu mm below surface\r\n", (unsigned long)params.spreadTopLimit);
//    printf("  spreadBottomLimit        : %lu mm above bottom\r\n", (unsigned long)params.spreadBottomLimit);
//    printf("  spreadPointHoverTime     : %lu ms\r\n", (unsigned long)params.spreadPointHoverTime);
//
//    /* 水位测量参数 */
//    printf("[Water Level]\r\n");
//    printf("  waterLevelCorrection     : %lu\r\n", (unsigned long)params.waterLevelCorrection);
//    printf("  maxDownDistance          : %lu mm\r\n", (unsigned long)params.maxDownDistance);
//
//    /* 实高测量 */
//    printf("[Real Tank Height]\r\n");
//    printf("  refreshTankHeightFlag    : %s\r\n", params.refreshTankHeightFlag ? "Yes" : "No");
//    printf("  maxTankHeightDeviation   : %lu mm\r\n", (unsigned long)params.maxTankHeightDeviation);
//    printf("  initialTankHeight        : %lu mm\r\n", (unsigned long)params.initialTankHeight);
//    printf("  currentTankHeight        : %lu mm\r\n", (unsigned long)params.currentTankHeight);
//
//    /* 液位测量 */
//    printf("[Oil Level]\r\n");
//    printf("  oilLevelThreshold        : %lu mm\r\n", (unsigned long)params.oilLevelThreshold);
//    printf("  liquidLevelMeasureMethod : %lu\r\n", (unsigned long)params.liquidLevelMeasurementMethod);
//
//    /* 报警 DO */
//    printf("[Alarm DO]\r\n");
//    printf("  AlarmHighDO              : %lu\r\n", (unsigned long)params.AlarmHighDO);
//    printf("  AlarmLowDO               : %lu\r\n", (unsigned long)params.AlarmLowDO);
//    printf("  ThirdStateThreshold      : %lu\r\n", (unsigned long)params.ThirdStateThreshold);
//
//    /* 4-20mA 输出 / AO */
//    printf("[4-20mA / AO]\r\n");
//    printf("  CurrentRangeStart_mA     : %lu mA\r\n", (unsigned long)params.CurrentRangeStart_mA);
//    printf("  CurrentRangeEnd_mA       : %lu mA\r\n", (unsigned long)params.CurrentRangeEnd_mA);
//    printf("  AlarmHighAO              : %lu\r\n", (unsigned long)params.AlarmHighAO);
//    printf("  AlarmLowAO               : %lu\r\n", (unsigned long)params.AlarmLowAO);
//    printf("  InitialCurrent_mA        : %lu mA\r\n", (unsigned long)params.InitialCurrent_mA);
//    printf("  AOHighCurrent_mA         : %lu mA\r\n", (unsigned long)params.AOHighCurrent_mA);
//    printf("  AOLowCurrent_mA          : %lu mA\r\n", (unsigned long)params.AOLowCurrent_mA);
//    printf("  FaultCurrent_mA          : %lu mA\r\n", (unsigned long)params.FaultCurrent_mA);
//    printf("  DebugCurrent_mA          : %lu mA\r\n", (unsigned long)params.DebugCurrent_mA);
//
//    /* CRC 校验 */
//    printf("[CRC]\r\n");
//    printf("  crc                      : 0x%08lX\r\n", (unsigned long)params.crc);
//
//    printf("======================================\r\n");
//}
