/*
 * system_parameter.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Duan Xuebin
 *
 * 说明:
 *  - DeviceParameters 中 command 仅用于当前指令，不参与掉电参数校验
 *  - 从 sensorType 开始到 crc 之前的区域为持久化参数区
 */

#include "system_parameter.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h> /* offsetof */

volatile MeasurementResult g_measurement = {0};   /* 测量结果 */
volatile DeviceParameters  g_deviceParams = {0};  /* 设备参数 */

/* 参数版本号和 magic 常量 */
#define DEVICE_PARAM_VERSION   (1u)
#define DEVICE_PARAM_MAGIC     (0x4C54444Du)  /* 'LTDM' */

/*
 * 持久化区域说明:
 *  - command 不参与掉电参数校验
 *  - 从 sensorType 起到 crc 前为持久化参数区
 */
#define DEVICE_PARAM_PERSIST_OFFSET   (offsetof(DeviceParameters, sensorType))
#define DEVICE_PARAM_PERSIST_LEN      (offsetof(DeviceParameters, crc) - DEVICE_PARAM_PERSIST_OFFSET)
#define DEVICE_PARAM_PERSIST_START(p) ((uint8_t *)(p) + DEVICE_PARAM_PERSIST_OFFSET)

/*========================= 参数存储逻辑 =========================*/

/* 保存设备参数到 FRAM */
void save_device_params(void)
{
    DeviceParameters params = g_deviceParams;

    /* 元信息字段 */
    params.param_version = DEVICE_PARAM_VERSION;
    params.struct_size   = (uint32_t)sizeof(DeviceParameters);
    params.magic         = DEVICE_PARAM_MAGIC;

    /* CRC 覆盖从 sensorType 起到 crc 之前的区域, 不包含 command 和 crc 自身 */
    uint8_t  *crc_base = DEVICE_PARAM_PERSIST_START(&params);
    uint32_t  crc_size = (uint32_t)DEVICE_PARAM_PERSIST_LEN;

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
    {
        uint8_t *crc_base = DEVICE_PARAM_PERSIST_START(&temp);
        uint32_t crc_size = (uint32_t)DEVICE_PARAM_PERSIST_LEN;
        uint32_t calc_crc = CRC32_HAL(crc_base, crc_size);

        if (calc_crc != temp.crc)
        {
            printf("设备参数 CRC 校验失败, 计算=0x%08lX, FRAM=0x%08lX\r\n",
                   (unsigned long)calc_crc,
                   (unsigned long)temp.crc);
            return 0;
        }
    }

    /* 校验成功, 拷贝到全局变量 */
    memcpy((void * volatile)&g_deviceParams, &temp, sizeof(DeviceParameters));

    /* 上电时当前指令从默认指令恢复, 而不是沿用 FRAM 中旧的 command */
    g_deviceParams.command = g_deviceParams.powerOnDefaultCommand;
    save_device_params();
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
        RestoreFactoryParamsConfig(); /* 内部会调用 save_device_params */

        g_measurement.device_status.error_code = PARAM_UNINITIALIZED;

        printf("设备参数加载失败, 已恢复出厂设置\r\n");
    }
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

    /* ---------------- 指令类字段 ---------------- */
    g_deviceParams.command               = CMD_NONE;
    g_deviceParams.powerOnDefaultCommand = CMD_NONE;

    /* ---------------- 基础参数 ---------------- */
    g_deviceParams.sensorType            = DSM_SENSOR;
    g_deviceParams.sensorID              = 1234567;
    g_deviceParams.sensorSoftwareVersion = 0x00010001;
    g_deviceParams.softwareVersion       = 0x00010001;
    g_deviceParams.error_auto_back_zero  = 1;   /* 默认: 报错回零 */
    g_deviceParams.error_stop_measurement= 1;   /* 默认: 报错停止测量 */

    /* ---------------- 电机与编码器参数 ---------------- */
    g_deviceParams.encoder_wheel_circumference_mm = 95000;  /* 0.001mm */
    g_deviceParams.max_motor_speed                = 400;    /* r/s */
    g_deviceParams.first_loop_circumference_mm    = 6000; /* 0.1mm */
    g_deviceParams.tape_thickness_mm              = 200;    /* 0.001mm */

    /* ---------------- 称重参数 ---------------- */
    g_deviceParams.empty_weight             = 0;
    g_deviceParams.empty_weight_upper_limit = 2000;
    g_deviceParams.empty_weight_lower_limit = 0;

    g_deviceParams.full_weight              = 5000;
    g_deviceParams.full_weight_upper_limit  = 30000;
    g_deviceParams.full_weight_lower_limit  = 2000;

    g_deviceParams.weight_upper_limit_ratio = 50;
    g_deviceParams.weight_lower_limit_ratio = 20;

    /* ---------------- 零点测量 ---------------- */
    g_deviceParams.zero_weight_threshold_ratio = 50;   /* 按算法需要调整 */
    g_deviceParams.weight_ignore_zone          = 1000; /* 0.1mm => 100mm */
    g_deviceParams.max_zero_deviation_distance = 1000;  /* 零点区域最大偏差值 0.1mm => 20mm */
    g_deviceParams.findZeroDownDistance        = 1000; /* 0.1mm => 100mm */

    /* ---------------- 液位测量 ---------------- */
    g_deviceParams.tankHeight                  = 200000; /* 0.1mm => 20000mm */
    g_deviceParams.liquid_sensor_distance_diff = 0;      /* 0.1mm */
    g_deviceParams.blindZone                   = 3000;   /* 0.1mm => 300mm */

    g_deviceParams.oilLevelThreshold           = 15;     /* 项目自定义倍率/单位 */
    g_deviceParams.oilLevelHysteresisThreshold = 20;     /* 项目自定义倍率/单位 */
    g_deviceParams.liquidLevelMeasurementMethod= 0;		/* 0 空气+液体频率/2 1：根据设置跟随频率跟随 2 根据设置密度跟随 3.根据振动管跟随 */
    g_deviceParams.oilLevelFrequency          = 5500;      /* 液位跟随频率 */
    g_deviceParams.oilLevelDensity            = 0;      /* 液位跟随密度 */

    /* ---------------- 水位测量参数 ---------------- */
    g_deviceParams.water_tank_height                = 200000; /* 0.1mm */
    g_deviceParams.water_level_sensor_distance_diff = 0;      /* 0.1mm */
    g_deviceParams.waterBlindZone                   = 100;    /* 0.1mm */

    g_deviceParams.water_cap_threshold              = 50000;      /* 建议明确倍率后再设默认 */
    g_deviceParams.water_cap_hysteresis             = 5000;      /* 建议明确倍率后再设默认 */
    g_deviceParams.maxDownDistance                  = 3000;   /* 0.1mm => 300mm */

    /* ---------------- 罐高/罐底测量 ---------------- */
    g_deviceParams.bottom_detect_mode      = 0;    /* 0=按项目定义 */
    g_deviceParams.bottom_angle_threshold  = 12;    /* 单位(度）/倍率*1 */
    g_deviceParams.bottom_weight_threshold = 2000;    /* 按现场经验再设默认 */

    g_deviceParams.refreshTankHeightFlag   = 0;  /* 不自动刷新 */
    g_deviceParams.maxTankHeightDeviation  = 100;  /* 0.1mm => 10mm */
    g_deviceParams.initialTankHeight       = 0;
    g_deviceParams.currentTankHeight       = 0;

    /* ---------------- 密度/温度修正 ---------------- */
    g_deviceParams.densityCorrection       = 10000;
    g_deviceParams.temperatureCorrection   = 1000;

    /* ---------------- 分布/区间测量参数 ---------------- */
    g_deviceParams.requireBottomMeasurement    = 0;
    g_deviceParams.requireWaterMeasurement     = 0;
    g_deviceParams.requireSinglePointDensity   = 0;

    g_deviceParams.spreadMeasurementOrder      = 0;
    g_deviceParams.spreadMeasurementMode       = 0;
    g_deviceParams.spreadMeasurementCount      = 5;
    g_deviceParams.spreadMeasurementDistance   = 10000; /* 0.1mm */
    g_deviceParams.spreadTopLimit              = 300;  /* 0.1mm */
    g_deviceParams.spreadBottomLimit           = 300;  /* 0.1mm */
    g_deviceParams.spreadPointHoverTime        = 10;

    g_deviceParams.intervalMeasurementTopLimit    = 300; /* 0.1mm */
    g_deviceParams.intervalMeasurementBottomLimit = 300; /* 0.1mm */

    /* ---------------- Wartsila 密度区间 ---------------- */
    g_deviceParams.wartsila_upper_density_limit      = 38000;
    g_deviceParams.wartsila_lower_density_limit      = 500;
    g_deviceParams.wartsila_density_interval         = 1000;
    g_deviceParams.wartsila_max_height_above_surface = 200; /* 0.1mm 或按定义 */

    /* ---------------- 继电器报警输出 ---------------- */
    g_deviceParams.AlarmHighDO         = 0;
    g_deviceParams.AlarmLowDO          = 0;
    g_deviceParams.ThirdStateThreshold = 0;

    /* ---------------- 4~20mA 输出 ---------------- */
    g_deviceParams.CurrentRangeStart_mA = 400;   /* 4.00mA (×0.01) */
    g_deviceParams.CurrentRangeEnd_mA   = 2000;  /* 20.00mA (×0.01) */
    g_deviceParams.AlarmHighAO          = 2000;
    g_deviceParams.AlarmLowAO           = 400;
    g_deviceParams.InitialCurrent_mA    = 400;
    g_deviceParams.AOHighCurrent_mA     = 2000;
    g_deviceParams.AOLowCurrent_mA      = 400;
    g_deviceParams.FaultCurrent_mA      = 2200;  /* 22.00mA */
    g_deviceParams.DebugCurrent_mA      = 1200;  /* 12.00mA */

    /* ---------------- 指令参数 ---------------- */
    g_deviceParams.calibrateOilLevel              = 0;
    g_deviceParams.calibrateWaterLevel            = 0;
    g_deviceParams.singlePointMeasurementPosition = 0;
    g_deviceParams.singlePointMonitoringPosition  = 0;
    g_deviceParams.densityDistributionOilLevel    = 0;
    g_deviceParams.motorCommandDistance           = 0;

    /* ---------------- 元信息与校验字段 ---------------- */
    g_deviceParams.param_version = DEVICE_PARAM_VERSION;
    g_deviceParams.struct_size   = (uint32_t)sizeof(DeviceParameters);
    g_deviceParams.magic         = DEVICE_PARAM_MAGIC;
    g_deviceParams.crc           = 0; /* save_device_params 内更新 */

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
    printf("  command                    : %u\r\n", (unsigned)params.command);
    printf("  powerOnDefaultCommand      : %u\r\n", (unsigned)params.powerOnDefaultCommand);

    /* 基础参数 */
    printf("[Basic]\r\n");
    printf("  sensorType                 : %lu\r\n", (unsigned long)params.sensorType);
    printf("  sensorID                   : %lu\r\n", (unsigned long)params.sensorID);
    printf("  sensorSoftwareVersion      : 0x%08lX\r\n", (unsigned long)params.sensorSoftwareVersion);
    printf("  softwareVersion            : 0x%08lX\r\n", (unsigned long)params.softwareVersion);
    printf("  error_auto_back_zero       : %lu\r\n", (unsigned long)params.error_auto_back_zero);
    printf("  error_stop_measurement     : %lu\r\n", (unsigned long)params.error_stop_measurement);

    /* 电机与编码器 */
    printf("[Motor / Encoder]\r\n");
    printf("  encoder_circ(0.001mm)      : %lu\r\n", (unsigned long)params.encoder_wheel_circumference_mm);
    printf("  max_motor_speed(r/s)       : %lu\r\n", (unsigned long)params.max_motor_speed);
    printf("  first_loop_circ(0.1mm)     : %lu\r\n", (unsigned long)params.first_loop_circumference_mm);
    printf("  tape_thickness(0.001mm)    : %lu\r\n", (unsigned long)params.tape_thickness_mm);

    /* 称重 */
    printf("[Weight]\r\n");
    printf("  empty_weight               : %lu\r\n", (unsigned long)params.empty_weight);
    printf("  empty_weight_upper_limit   : %lu\r\n", (unsigned long)params.empty_weight_upper_limit);
    printf("  empty_weight_lower_limit   : %lu\r\n", (unsigned long)params.empty_weight_lower_limit);
    printf("  full_weight                : %lu\r\n", (unsigned long)params.full_weight);
    printf("  full_weight_upper_limit    : %lu\r\n", (unsigned long)params.full_weight_upper_limit);
    printf("  full_weight_lower_limit    : %lu\r\n", (unsigned long)params.full_weight_lower_limit);
    printf("  weight_upper_limit_ratio   : %lu\r\n", (unsigned long)params.weight_upper_limit_ratio);
    printf("  weight_lower_limit_ratio   : %lu\r\n", (unsigned long)params.weight_lower_limit_ratio);

    /* 零点 */
    printf("[Zero]\r\n");
    printf("  zero_weight_threshold_ratio: %lu\r\n", (unsigned long)params.zero_weight_threshold_ratio);
    printf("  weight_ignore_zone(0.1mm)  : %lu\r\n", (unsigned long)params.weight_ignore_zone);
    printf("  max_zero_deviation(0.1mm)  : %lu\r\n", (unsigned long)params.max_zero_deviation_distance);
    printf("  findZeroDownDistance(0.1mm): %lu\r\n", (unsigned long)params.findZeroDownDistance);

    /* 液位 */
    printf("[Oil Level]\r\n");
    printf("  tankHeight(0.1mm)          : %lu\r\n", (unsigned long)params.tankHeight);
    printf("  liquid_sensor_diff(0.1mm)  : %lu\r\n", (unsigned long)params.liquid_sensor_distance_diff);
    printf("  blindZone(0.1mm)           : %lu\r\n", (unsigned long)params.blindZone);
    printf("  oilLevelThreshold          : %lu\r\n", (unsigned long)params.oilLevelThreshold);
    printf("  oilLevelHysteresis         : %lu\r\n", (unsigned long)params.oilLevelHysteresisThreshold);
    printf("  liquidLevelMethod          : %lu\r\n", (unsigned long)params.liquidLevelMeasurementMethod);
    printf("  oilLevelFrequency          : %lu\r\n", (unsigned long)params.oilLevelFrequency);
    printf("  oilLevelDensity            : %lu\r\n", (unsigned long)params.oilLevelDensity);

    /* 水位 */
    printf("[Water Level]\r\n");
    printf("  water_tank_height(0.1mm)   : %lu\r\n", (unsigned long)params.water_tank_height);
    printf("  water_sensor_diff(0.1mm)   : %lu\r\n", (unsigned long)params.water_level_sensor_distance_diff);
    printf("  waterBlindZone(0.1mm)      : %lu\r\n", (unsigned long)params.waterBlindZone);
    printf("  water_cap_threshold        : %lu\r\n", (unsigned long)params.water_cap_threshold);
    printf("  water_cap_hysteresis       : %lu\r\n", (unsigned long)params.water_cap_hysteresis);
    printf("  maxDownDistance(0.1mm)     : %lu\r\n", (unsigned long)params.maxDownDistance);
    printf("  zero_point_capacitance     : %lu\r\n", (unsigned long)params.zero_cap);

    /* 罐底/罐高 */
    printf("[Bottom / Tank Height]\r\n");
    printf("  bottom_detect_mode         : %lu\r\n", (unsigned long)params.bottom_detect_mode);
    printf("  bottom_angle_threshold     : %lu\r\n", (unsigned long)params.bottom_angle_threshold);
    printf("  bottom_weight_threshold    : %lu\r\n", (unsigned long)params.bottom_weight_threshold);
    printf("  refreshTankHeightFlag      : %lu\r\n", (unsigned long)params.refreshTankHeightFlag);
    printf("  maxTankHeightDeviation     : %lu\r\n", (unsigned long)params.maxTankHeightDeviation);
    printf("  initialTankHeight          : %lu\r\n", (unsigned long)params.initialTankHeight);
    printf("  currentTankHeight          : %lu\r\n", (unsigned long)params.currentTankHeight);

    /* 修正 */
    printf("[Correction]\r\n");
    printf("  densityCorrection          : %lu\r\n", (unsigned long)params.densityCorrection);
    printf("  temperatureCorrection      : %lu\r\n", (unsigned long)params.temperatureCorrection);

    /* 分布/区间 */
    printf("[Spread / Interval]\r\n");
    printf("  requireBottomMeasurement   : %lu\r\n", (unsigned long)params.requireBottomMeasurement);
    printf("  requireWaterMeasurement    : %lu\r\n", (unsigned long)params.requireWaterMeasurement);
    printf("  requireSinglePointDensity  : %lu\r\n", (unsigned long)params.requireSinglePointDensity);
    printf("  spreadMeasurementOrder     : %lu\r\n", (unsigned long)params.spreadMeasurementOrder);
    printf("  spreadMeasurementMode      : %lu\r\n", (unsigned long)params.spreadMeasurementMode);
    printf("  spreadMeasurementCount     : %lu\r\n", (unsigned long)params.spreadMeasurementCount);
    printf("  spreadMeasurementDistance  : %lu\r\n", (unsigned long)params.spreadMeasurementDistance);
    printf("  spreadTopLimit(0.1mm)      : %lu\r\n", (unsigned long)params.spreadTopLimit);
    printf("  spreadBottomLimit(0.1mm)   : %lu\r\n", (unsigned long)params.spreadBottomLimit);
    printf("  spreadPointHoverTime       : %lu\r\n", (unsigned long)params.spreadPointHoverTime);
    printf("  intervalTopLimit(0.1mm)    : %lu\r\n", (unsigned long)params.intervalMeasurementTopLimit);
    printf("  intervalBottomLimit(0.1mm) : %lu\r\n", (unsigned long)params.intervalMeasurementBottomLimit);

    /* Wartsila */
    printf("[Wartsila]\r\n");
    printf("  upper_density_limit        : %lu\r\n", (unsigned long)params.wartsila_upper_density_limit);
    printf("  lower_density_limit        : %lu\r\n", (unsigned long)params.wartsila_lower_density_limit);
    printf("  density_interval           : %lu\r\n", (unsigned long)params.wartsila_density_interval);
    printf("  max_height_above_surface   : %lu\r\n", (unsigned long)params.wartsila_max_height_above_surface);

    /* DO */
    printf("[Alarm DO]\r\n");
    printf("  AlarmHighDO                : %lu\r\n", (unsigned long)params.AlarmHighDO);
    printf("  AlarmLowDO                 : %lu\r\n", (unsigned long)params.AlarmLowDO);
    printf("  ThirdStateThreshold        : %lu\r\n", (unsigned long)params.ThirdStateThreshold);

    /* AO */
    printf("[4-20mA / AO]\r\n");
    printf("  CurrentRangeStart_mA       : %lu\r\n", (unsigned long)params.CurrentRangeStart_mA);
    printf("  CurrentRangeEnd_mA         : %lu\r\n", (unsigned long)params.CurrentRangeEnd_mA);
    printf("  AlarmHighAO                : %lu\r\n", (unsigned long)params.AlarmHighAO);
    printf("  AlarmLowAO                 : %lu\r\n", (unsigned long)params.AlarmLowAO);
    printf("  InitialCurrent_mA          : %lu\r\n", (unsigned long)params.InitialCurrent_mA);
    printf("  AOHighCurrent_mA           : %lu\r\n", (unsigned long)params.AOHighCurrent_mA);
    printf("  AOLowCurrent_mA            : %lu\r\n", (unsigned long)params.AOLowCurrent_mA);
    printf("  FaultCurrent_mA            : %lu\r\n", (unsigned long)params.FaultCurrent_mA);
    printf("  DebugCurrent_mA            : %lu\r\n", (unsigned long)params.DebugCurrent_mA);

    /* 指令参数 */
    printf("[Command Params]\r\n");
    printf("  calibrateOilLevel          : %lu\r\n", (unsigned long)params.calibrateOilLevel);
    printf("  calibrateWaterLevel        : %lu\r\n", (unsigned long)params.calibrateWaterLevel);
    printf("  singlePointMeasurePos      : %lu\r\n", (unsigned long)params.singlePointMeasurementPosition);
    printf("  singlePointMonitorPos      : %lu\r\n", (unsigned long)params.singlePointMonitoringPosition);
    printf("  densityDistributionOilLevel: %lu\r\n", (unsigned long)params.densityDistributionOilLevel);
    printf("  motorCommandDistance       : %lu\r\n", (unsigned long)params.motorCommandDistance);

    /* 元信息/CRC */
    printf("[Meta]\r\n");
    printf("  param_version              : %lu\r\n", (unsigned long)params.param_version);
    printf("  struct_size                : %lu\r\n", (unsigned long)params.struct_size);
    printf("  magic                      : 0x%08lX\r\n", (unsigned long)params.magic);
    printf("  crc                        : 0x%08lX\r\n", (unsigned long)params.crc);

    printf("======================================\r\n");
}

/*========================= 测量结果打印（可选） =========================*/
/* 注: 该部分与参数结构无强耦合，仅保留你现有打印习惯；如果不需要可移除 */

void PrintDensity(const char *title, const DensityMeasurement *d)
{
    if (!d) return;

    printf("  [%s]\r\n", title);
    printf("    温度: %lu\r\n", (unsigned long)d->temperature);
    printf("    密度: %lu\r\n", (unsigned long)d->density);
    printf("    标准密度: %lu\r\n", (unsigned long)d->standard_density);
    printf("    VCF20: %lu\r\n", (unsigned long)d->vcf20);
    printf("    计重密度: %lu\r\n", (unsigned long)d->weight_density);
    printf("    温度位置: %lu\r\n", (unsigned long)d->temperature_position);
}

void PrintMeasurementResult(const MeasurementResult *m)
{
    if (!m) return;

    printf("\r\n=====================【设备实时测量结果】=====================\r\n");

    /* 1. 设备状态 */
    printf("【设备状态】\r\n");
    printf("  工作模式: %lu\r\n", (unsigned long)m->device_status.work_mode);
    printf("  设备状态: %d\r\n",  (int)m->device_status.device_state);
    printf("  错误代码: %lu\r\n", (unsigned long)m->device_status.error_code);
    printf("  当前指令: %d\r\n",  (int)m->device_status.current_command);
    printf("  零点状态: %lu\r\n", (unsigned long)m->device_status.zero_point_status);

    printf("--------------------------------------------------------------\r\n");

    /* 2. 调试数据 */
    printf("【调试数据 DebugData】\r\n");
    printf("  编码值: %ld\r\n",        (long)m->debug_data.current_encoder_value);
    printf("  传感器位置: %ld mm\r\n", (long)m->debug_data.sensor_position);
    printf("  尺带长度: %ld mm\r\n",   (long)m->debug_data.cable_length);
    printf("  电机步进: %ld\r\n",      (long)m->debug_data.motor_step);
    printf("  电机距离: %ld (0.1mm)\r\n",(long)m->debug_data.motor_distance);

    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->debug_data.frequency);
    printf("  温度: %.2f ℃\r\n",       m->debug_data.temperature / 100.0f);
    printf("  空气中频率: %lu Hz\r\n", (unsigned long)m->debug_data.air_frequency);
    printf("  幅值: %lu\r\n",          (unsigned long)m->debug_data.current_amplitude);
    printf("  水位电压/电容值: %lu\r\n",(unsigned long)m->debug_data.water_level_voltage);

    printf("  当前称重值: %lu\r\n",    (unsigned long)m->debug_data.current_weight);
    printf("  称重参数: %lu\r\n",      (unsigned long)m->debug_data.weight_param);

    printf("  X角度: %ld\r\n", (long)m->debug_data.angle_x);
    printf("  Y角度: %ld\r\n", (long)m->debug_data.angle_y);

    printf("  电机速度: %lu\r\n", (unsigned long)m->debug_data.motor_speed);
    printf("  电机状态: %lu (%s)\r\n",
           (unsigned long)m->debug_data.motor_state,
           (m->debug_data.motor_state == 0) ? "停止" :
           (m->debug_data.motor_state == 1) ? "上行" :
           (m->debug_data.motor_state == 2) ? "下行" : "未知");

    printf("--------------------------------------------------------------\r\n");

    /* 3. 液位测量 */
    printf("【液位测量 OilMeasurement】\r\n");
    printf("  跟随液位: %lu mm\r\n",   (unsigned long)m->oil_measurement.oil_level);
    printf("  空气频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.air_frequency);
    printf("  油中频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.oil_frequency);
    printf("  跟随频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.follow_frequency);
    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.current_frequency);

    printf("--------------------------------------------------------------\r\n");

    /* 4. 水位测量 */
    printf("【水位测量 WaterMeasurement】\r\n");
    printf("  水位值: %lu mm\r\n", (unsigned long)m->water_measurement.water_level);
    printf("  零点电容: %.3f\r\n", m->water_measurement.zero_capacitance);
    printf("  油区电容: %.3f\r\n", m->water_measurement.oil_capacitance);
    printf("  当前电容: %.3f\r\n", m->water_measurement.current_capacitance);

    printf("--------------------------------------------------------------\r\n");

    /* 5. 实高测量 */
    printf("【实高测量 ActualHeight】\r\n");
    printf("  标定液位实高: %lu mm\r\n", (unsigned long)m->height_measurement.calibrated_liquid_level);
    printf("  当前实高: %lu mm\r\n",     (unsigned long)m->height_measurement.current_real_height);

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
    printf("  平均温度: %lu\r\n",     (unsigned long)m->density_distribution.average_temperature);
    printf("  平均密度: %lu\r\n",     (unsigned long)m->density_distribution.average_density);
    printf("  平均计重密度: %lu\r\n", (unsigned long)m->density_distribution.average_weight_density);
    printf("  测量点数: %lu\r\n",     (unsigned long)m->density_distribution.measurement_points);
    printf("  测量时液位: %lu mm\r\n",(unsigned long)m->density_distribution.Density_oil_level);

    printf("  --- 单点数据（仅打印前10个）---\r\n");
    for (uint32_t i = 0; i < 10 && i < m->density_distribution.measurement_points; i++)
    {
        const DensityMeasurement *d = &m->density_distribution.single_density_data[i];
        printf("    [%02lu] T=%lu ρ=%lu ρ15=%lu VCF=%lu WD=%lu Pos=%lu\r\n",
               (unsigned long)i,
               (unsigned long)d->temperature,
               (unsigned long)d->density,
               (unsigned long)d->standard_density,
               (unsigned long)d->vcf20,
               (unsigned long)d->weight_density,
               (unsigned long)d->temperature_position);
    }

    printf("========================【打印结束】========================\r\n");
}
/**
 * @brief 默认指令 -> 测量命令 映射
 * @param def_cmd  DefaultCommandType
 * @return         CommandType（测量命令）；无匹配返回 CMD_UNKNOWN
 */
CommandType DefaultCmd_To_MeasureCmd(DefaultCommandType def_cmd)
{
    switch (def_cmd) {

    case CMD_NONE_DEF:
        return CMD_NONE;

    case CMD_BACK_ZERO_DEF:
        return CMD_BACK_ZERO;

    case CMD_FIND_OIL_DEF:
        return CMD_FIND_OIL;

    case CMD_MONITOR_SINGLE_DEF:
        return CMD_MONITOR_SINGLE;

    case CMD_FOLLOW_WATER_DEF:
        return CMD_FOLLOW_WATER;

    default:
        return CMD_NONE;
    }
}
