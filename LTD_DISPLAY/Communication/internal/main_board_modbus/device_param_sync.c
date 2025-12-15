/*
 * device_param_sync.c
 *
 *  Created on: 2025年12月12日
 *      Author: Duan Xuebin
 */


#include "device_param_sync.h"
#include <stdio.h>

extern const int holdValueAmount;

/* ==================== 内部：operanum → g_deviceParams 字段映射 ==================== */

/* 根据 operanum(COM_NUM_xxx) 找到 DeviceParameters 中对应字段的指针 */
static volatile uint32_t* get_deviceparam_ptr_by_operanum(int operanum)
{
    switch (operanum) {

    /* ===== 指令 ===== */
    case COM_NUM_DEVICEPARAM_COMMAND:
        return (volatile uint32_t*)&g_deviceParams.command;

    /* ===== 基础参数 ===== */
    case COM_NUM_DEVICEPARAM_TANKHEIGHT:
        return &g_deviceParams.tankHeight;
    case COM_NUM_DEVICEPARAM_BLINDZONE:
        return &g_deviceParams.blindZone;
    case COM_NUM_DEVICEPARAM_WATER_BLINDZONE:
        return &g_deviceParams.waterBlindZone;
    case COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM:
        return &g_deviceParams.encoder_wheel_circumference_mm;
    case COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED:
        return &g_deviceParams.max_motor_speed;
    case COM_NUM_DEVICEPARAM_SENSORTYPE:
        return &g_deviceParams.sensorType;
    case COM_NUM_DEVICEPARAM_SENSORID:
        return &g_deviceParams.sensorID;
    case COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION:
        return &g_deviceParams.sensorSoftwareVersion;
    case COM_NUM_DEVICEPARAM_SOFTWAREVERSION:
        return &g_deviceParams.softwareVersion;
    case COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND:
        return (volatile uint32_t*)&g_deviceParams.powerOnDefaultCommand;
    case COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE:
        return &g_deviceParams.findZeroDownDistance;
    case COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM:
        return &g_deviceParams.first_loop_circumference_mm;
    case COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM:
        return &g_deviceParams.tape_thickness_mm;

    /* ===== 称重参数 ===== */
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT:
        return &g_deviceParams.empty_weight;
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT:
        return &g_deviceParams.full_weight;
    case COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO:
        return &g_deviceParams.weight_upper_limit_ratio;
    case COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO:
        return &g_deviceParams.weight_lower_limit_ratio;
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT:
        return &g_deviceParams.empty_weight_upper_limit;
    case COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT:
        return &g_deviceParams.empty_weight_lower_limit;
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT:
        return &g_deviceParams.full_weight_upper_limit;
    case COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT:
        return &g_deviceParams.full_weight_lower_limit;

    /* ===== 指令参数 ===== */
    case COM_NUM_CAL_OIL:
        return &g_deviceParams.calibrateOilLevel;
    case COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL:
        return &g_deviceParams.calibrateWaterLevel;
    case COM_NUM_SINGLE_POINT:
        return &g_deviceParams.singlePointMeasurementPosition;
    case COM_NUM_SP_TEST:
        return &g_deviceParams.singlePointMonitoringPosition;
//    case COM_NUM_SPREADPOINTS:
//        return &g_deviceParams.densityDistributionOilLevel;
    case COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE:
        return &g_deviceParams.motorCommandDistance;

    /* ===== 修正参数 ===== */
    case COM_NUM_DEVICEPARAM_DENSITYCORRECTION:
        return &g_deviceParams.densityCorrection;
    case COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION:
        return &g_deviceParams.temperatureCorrection;

    /* ===== 分布测量参数 ===== */
    case COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT:
        return &g_deviceParams.requireBottomMeasurement;
    case COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT:
        return &g_deviceParams.requireWaterMeasurement;
    case COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY:
        return &g_deviceParams.requireSinglePointDensity;
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER:
        return &g_deviceParams.spreadMeasurementOrder;
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE:
        return &g_deviceParams.spreadMeasurementMode;
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT:
        return &g_deviceParams.spreadMeasurementCount;
    case COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE:
        return &g_deviceParams.spreadMeasurementDistance;
    case COM_NUM_DEVICEPARAM_SPREADTOPLIMIT:
        return &g_deviceParams.spreadTopLimit;
    case COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT:
        return &g_deviceParams.spreadBottomLimit;
    case COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME:
        return &g_deviceParams.spreadPointHoverTime;

    /* ===== 瓦锡兰区间密度 ===== */
    case COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT:
        return &g_deviceParams.wartsila_upper_density_limit;
    case COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT:
        return &g_deviceParams.wartsila_lower_density_limit;
    case COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL:
        return &g_deviceParams.wartsila_density_interval;
    case COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE:
        return &g_deviceParams.wartsila_max_height_above_surface;

    /* ===== 水位测量 ===== */
    case COM_NUM_DEVICEPARAM_WATERLEVELCORRECTION:
        return &g_deviceParams.waterLevelCorrection;
    case COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE:
        return &g_deviceParams.maxDownDistance;

    /* ===== 实高测量 ===== */
    case COM_NUM_DEVICEPARAM_REFRESHTANKHEIGHTFLAG:
        return &g_deviceParams.refreshTankHeightFlag;
    case COM_NUM_DEVICEPARAM_MAXTANKHEIGHTDEVIATION:
        return &g_deviceParams.maxTankHeightDeviation;
    case COM_NUM_DEVICEPARAM_INITIALTANKHEIGHT:
        return &g_deviceParams.initialTankHeight;
    case COM_NUM_DEVICEPARAM_CURRENTTANKHEIGHT:
        return &g_deviceParams.currentTankHeight;

    /* ===== 液位测量 ===== */
    case COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD:
        return &g_deviceParams.oilLevelThreshold;
    case COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD:
        return &g_deviceParams.oilLevelHysteresisThreshold;
    case COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD:
        return &g_deviceParams.liquidLevelMeasurementMethod;

    /* ===== 报警 DO ===== */
    case COM_NUM_DEVICEPARAM_ALARM_HIGH_DO:
        return &g_deviceParams.AlarmHighDO;
    case COM_NUM_DEVICEPARAM_ALARM_LOW_DO:
        return &g_deviceParams.AlarmLowDO;
    case COM_NUM_DEVICEPARAM_THIRD_STATE_THRESHOLD:
        return &g_deviceParams.ThirdStateThreshold;

    /* ===== 4–20mA AO ===== */
    case COM_NUM_DEVICEPARAM_CURRENT_RANGE_START_mA:
        return &g_deviceParams.CurrentRangeStart_mA;
    case COM_NUM_DEVICEPARAM_CURRENT_RANGE_END_mA:
        return &g_deviceParams.CurrentRangeEnd_mA;
    case COM_NUM_DEVICEPARAM_ALARM_HIGH_AO:
        return &g_deviceParams.AlarmHighAO;
    case COM_NUM_DEVICEPARAM_ALARM_LOW_AO:
        return &g_deviceParams.AlarmLowAO;
    case COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA:
        return &g_deviceParams.InitialCurrent_mA;
    case COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA:
        return &g_deviceParams.AOHighCurrent_mA;
    case COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA:
        return &g_deviceParams.AOLowCurrent_mA;
    case COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA:
        return &g_deviceParams.FaultCurrent_mA;
    case COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA:
        return &g_deviceParams.DebugCurrent_mA;

    /* ===== 结构版本信息 ===== */
    case COM_NUM_DEVICEPARAM_PARAM_VERSION:
        return &g_deviceParams.param_version;
    case COM_NUM_DEVICEPARAM_STRUCT_SIZE:
        return &g_deviceParams.struct_size;
    case COM_NUM_DEVICEPARAM_MAGIC:
        return &g_deviceParams.magic;
    case COM_NUM_DEVICEPARAM_CRC:
        return &g_deviceParams.crc;

    default:
        return NULL;
    }
}

/* ==================== 内部：把 holdValue[i].val 下发到 CPU2（10 功能码） ==================== */

static void DeviceParams_SendHoldValueToCPU2(volatile struct HoldRegisterData *h)
{
    if (h == NULL) return;

    if (!h->authority_write) {
        // 屏幕不可写的参数通常不需要同步到 CPU2
        return;
    }

    // CPU2_CombinatePackage_Send 是按 32bit + word swap 来发的，
    // 每个参数占两个寄存器，因此这里只支持 rgstcnt == 2 的情况。
    if (h->rgstcnt != 2) {
        // 如果以后有 1 寄存器参数，再单独处理
        printf("DeviceParam warn: %s rgstcnt=%u not supported by sync\n",
               h->name ? (char*)h->name : "noname", h->rgstcnt);
        return;
    }

    // holdValue[i].val 就是与 CPU2 通信的“寄存器值”（之前已经说明）
    int32_t val = h->val;

    // 直接把这个 32 位值作为 holddata 传给 CPU2_CombinatePackage_Send
    uint32_t u32_temp = (uint32_t)val;

    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                               h->startadd,
                               h->rgstcnt,
                               &u32_temp);
}

/* ==================== 内部：同步一个 HoldRegisterData 项 ==================== */

static void DeviceParams_SyncOneHold(volatile struct HoldRegisterData *h)
{
    if (h == NULL) return;

    volatile uint32_t *p_dev = get_deviceparam_ptr_by_operanum(h->operanum);
    if (p_dev == NULL) {
        // 不属于 DeviceParameters 的项（例如测量结果），跳过
        return;
    }

    // g_deviceParams 中的源值
    int32_t dev_val = (int32_t)(*p_dev);

    if (h->val == dev_val) {
        // 与 CPU2 当前值一致，不需要更新
        return;
    }

    printf("DeviceParam diff: %s, CPU2=%d, LOCAL=%ld -> update & send\r\n",
           h->name ? (char*)h->name : "noname",
           h->val, dev_val);

    // 1) 把 g_deviceParams 的值写回 holdValue[i].val
    h->val = dev_val;

    // 2) 按寄存器信息下发 10 指令给 CPU2
    DeviceParams_SendHoldValueToCPU2(h);
}

/* ==================== 对外接口 ==================== */

/* 同步所有 DeviceParameters → CPU2 */
void DeviceParams_SyncAllToCPU2(void)
{
    for (uint32_t i = 0; i < holdValueAmount; ++i) {
    	//需要判定一下是否是CPU2的可写参数
        DeviceParams_SyncOneHold(&holdValue[i]);
    }
}

/* 只同步一个 operanum 对应的参数 */
void DeviceParams_SyncOneToCPU2(int operanum)
{
    for (uint32_t i = 0; i < holdValueAmount; ++i) {
        if (holdValue[i].operanum == operanum) {
            DeviceParams_SyncOneHold(&holdValue[i]);
            break;
        }
    }
}
