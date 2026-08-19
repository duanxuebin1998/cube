/*
 * system_parameter.c
 *  CPU2参数读取与管理
 *  Created on: Feb 27, 2025
 *      Author: 1
 */
#include "system_parameter.h"
#include "display_tankopera.h"
#include "cpu2_communicate.h"
#include "cpu3_comm_display_params.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/** CPU3 当前测量结果快照，由 CPU2 通信解析流程更新，显示与外部协议只读使用。 */
volatile MeasurementResult g_measurement = { 0 };

/** CPU2 下发并经 CPU3 同步确认的设备参数快照，供菜单、显示和协议映射共同使用。 */
volatile DeviceParameters g_deviceParams = { 0 };

/*
 * 函数用途：校验CPU2共享的传感器类型枚举。
 * 调用场景：CPU3接收参数快照或准备把参数同步回CPU2之前。
 * 关键约束：类型14表示DM4物理传感器，Safe只属于运行模式；未知值不能按既有类型解释。
 */
bool DeviceParam_IsSensorTypeSupported(uint32_t value)
{
    return (value == (uint32_t)DSM_SENSOR) ||
           (value == (uint32_t)LTD_SENSOR) ||
           (value == (uint32_t)DM4_SENSOR);
}

/*
 * 函数用途：把CPU2共享的传感器类型值转换为现场可读名称。
 * 调用场景：CPU3设备参数诊断打印。
 * 关键约束：未知值只显示为非法配置，不擅自按LTD或多参数V4解释。
 */
static const char * sensor_type_str(uint32_t value)
{
    switch ((SENSOR_TYPE)value) {
    case DSM_SENSOR:
        return "一体机传感器";
    case LTD_SENSOR:
        return "多参数V3传感器";
    case DM4_SENSOR:
        return "DM4传感器";
    default:
        return "非法配置";
    }
}

/**
 * @brief 将继电器报警输出工作模式转换为中文名称。
 *
 * @param value `RelayAlarmOperatingMode` 枚举值。
 * @return 对应的中文名称；枚举值不在当前协议定义范围内时返回“未定义”。
 */
static const char * relay_operating_mode_str(uint32_t value)
{
    switch ((RelayAlarmOperatingMode)value) {
    case RELAY_ALARM_OPERATING_DISABLED:
        return "禁用";
    case RELAY_ALARM_OPERATING_OUTPUT_PASSIVE:
        return "无源输出";
    default:
        return "未定义";
    }
}

/**
 * @brief 将继电器数字报警源组合转换为中文名称。
 *
 * @param value `RelayAlarmDigitalSource` 枚举值，可表示单个报警位或组合报警位。
 * @return 对应的中文名称；枚举值不在当前协议定义范围内时返回“未定义”。
 */
static const char * relay_digital_source_str(uint32_t value)
{
    /* 按协议枚举逐项映射，组合报警位必须保持与 CPU2 输出语义一致。 */
    switch ((RelayAlarmDigitalSource)value) {
    case RELAY_ALARM_DIGITAL_NONE:
        return "无";
    case RELAY_ALARM_DIGITAL_H:
        return "高报";
    case RELAY_ALARM_DIGITAL_HH:
        return "高高报";
    case RELAY_ALARM_DIGITAL_H_OR_HH:
        return "高/高高";
    case RELAY_ALARM_DIGITAL_L:
        return "低报";
    case RELAY_ALARM_DIGITAL_LL:
        return "低低报";
    case RELAY_ALARM_DIGITAL_L_OR_LL:
        return "低/低低";
    case RELAY_ALARM_DIGITAL_ANY:
        return "任意报警";
    default:
        return "未定义";
    }
}

/**
 * @brief 将继电器接点类型转换为中文名称。
 *
 * @param value `RelayAlarmContactType` 枚举值。
 * @return “常开”或“常闭”；枚举值无效时返回“未定义”。
 */
static const char * relay_contact_type_str(uint32_t value)
{
    switch ((RelayAlarmContactType)value) {
    case RELAY_ALARM_CONTACT_NORMALLY_OPEN:
        return "常开";
    case RELAY_ALARM_CONTACT_NORMALLY_CLOSED:
        return "常闭";
    default:
        return "未定义";
    }
}

/**
 * @brief 将继电器报警动作模式转换为中文名称。
 *
 * @param value `RelayAlarmMode` 枚举值。
 * @return “关”“开”或“锁存”；枚举值无效时返回“未定义”。
 */
static const char * relay_alarm_mode_str(uint32_t value)
{
    switch ((RelayAlarmMode)value) {
    case RELAY_ALARM_MODE_OFF:
        return "关";
    case RELAY_ALARM_MODE_ON:
        return "开";
    case RELAY_ALARM_MODE_LATCHING:
        return "锁存";
    default:
        return "未定义";
    }
}

/**
 * @brief 将继电器报警测量源转换为中文名称。
 *
 * @param value `RelayAlarmSource` 枚举值，表示液位、温度、水位、浮子位置或无测量源。
 * @return 对应的中文名称；枚举值不在当前协议定义范围内时返回“未定义”。
 */
static const char * relay_alarm_source_str(uint32_t value)
{
    /* 这里的名称直接用于设备参数打印，必须与 CPU2 报警源定义保持一致。 */
    switch ((RelayAlarmSource)value) {
    case RELAY_ALARM_SOURCE_TANK_LEVEL:
        return "储罐液位";
    case RELAY_ALARM_SOURCE_LIQUID_TEMP:
        return "液相温度";
    case RELAY_ALARM_SOURCE_WATER_LEVEL:
        return "水位";
    case RELAY_ALARM_SOURCE_DISPLACER_POS:
        return "浮子位置";
    case RELAY_ALARM_SOURCE_NONE:
        return "无";
    default:
        return "未定义";
    }
}

/**
 * @brief 将继电器报警无效值策略转换为中文名称。
 *
 * @param value `RelayAlarmErrorValue` 枚举值，表示测量值无效时需要置位的报警组合。
 * @return 对应的中文名称；枚举值不在当前协议定义范围内时返回“未定义”。
 * @note 该文本仅用于参数打印，不改变继电器报警状态或锁存状态。
 */
static const char * relay_error_value_str(uint32_t value)
{
    /* 按协议定义返回无效测量值对应的报警组合名称。 */
    switch ((RelayAlarmErrorValue)value) {
    case RELAY_ALARM_ERROR_NO_ALARM:
        return "无报警";
    case RELAY_ALARM_ERROR_HH_H:
        return "高高/高";
    case RELAY_ALARM_ERROR_H:
        return "高";
    case RELAY_ALARM_ERROR_L:
        return "低";
    case RELAY_ALARM_ERROR_LL_L:
        return "低低/低";
    case RELAY_ALARM_ERROR_ALL_ALARMS:
        return "全部报警";
    default:
        return "未定义";
    }
}

/**
 * @brief 将继电器报警阈值的 32 位原始位模式还原为单精度浮点数。
 *
 * @param raw CPU2 参数结构中保存的继电器报警阈值 IEEE 754 Float32 原始 32 位位模式。
 * @return 返回与 raw 的 32 位 IEEE 754 位模式完全一致的 float 数值；不执行缩放或范围修正。
 * @note 使用 `memcpy` 避免类型双关和严格别名问题，不执行数值缩放或范围修正。
 */
static float relay_alarm_raw_to_float(uint32_t raw)
{
    /* value 保存从 raw 原样复制得到的 IEEE 754 单精度数值。 */
    float value;
    /* 按原始位转换 IEEE754 float，避免字段打印改变协议解释口径。 */
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/* 名称 数值 数据号 起始地址 寄存器数 是否检范围 最小 最大 单位 小数 偏移 写权 类型 显示 隐藏 英文 */
/**
 * @brief CPU3 参数菜单和保持寄存器共用的参数元数据表。
 *
 * 每个表项依次描述中文名称、数据号、寄存器起始地址与数量、范围检查、
 * 最小值、最大值、单位、小数位、显示偏移、写权限、数据类型、显示宽度、
 * 可选值文本和英文短名称。大量同构表项统一由本说明描述，各表项自身的
 * 中文名称、寄存器宏和范围字段用于区分具体参数语义。
 */
struct ParameterMetadata param_meta[] = {

{(uint8_t*)"设备指令",	0,	COM_NUM_DEVICEPARAM_COMMAND,	HOLDREGISTER_DEVICEPARAM_COMMAND,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Cmd"},

{(uint8_t*)"传感器类型",	0,	COM_NUM_DEVICEPARAM_SENSORTYPE,	HOLDREGISTER_DEVICEPARAM_SENSORTYPE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	3,	NULL,	(uint8_t*)"SensorType"},
{(uint8_t*)"传感器编号",	0,	COM_NUM_DEVICEPARAM_SENSORID,	HOLDREGISTER_DEVICEPARAM_SENSORID,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"SensorID"},
{(uint8_t*)"传感器软件版本",	0,	COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"SenSWVer"},
{(uint8_t*)"CPU2程序版本",	0,	COM_NUM_DEVICEPARAM_SOFTWAREVERSION,	HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION,	2,	false,	0,	0,	NULL,	3,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"CPU2FWVer"},
{(uint8_t*)"CPU3程序版本",	0,	COM_NUM_PARA_LOCAL_LEDVERSION,	HOLDREGISTER_CPU3_LED_VERSION,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"CPU3FWVer"},
{(uint8_t*)"协议版本",	0,	COM_NUM_DEVICEPARAM_PROTOCOL_VERSION,	HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	3,	NULL,	(uint8_t*)"ProtoVer"},
{(uint8_t*)"上电默认指令",	0,	COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,	HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	3,	ret_arr_word,	(uint8_t*)"PwrOnCmd"},
{(uint8_t*)"故障自动回零",	0,	COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO,	HOLDREGISTER_DEVICEPARAM_ERROR_AUTO_BACK_ZERO,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"ErrAutoBackZero"},
{(uint8_t*)"故障停止测量",	0,	COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT,	HOLDREGISTER_DEVICEPARAM_ERROR_STOP_MEASUREMENT,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"ErrStopMeas"},

{(uint8_t*)"故障自动恢复重跑次数",	0,	COM_NUM_DEVICEPARAM_RESERVED2,	HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT,	2,	true,	0,	10,	(uint8_t*)"次",	0,	0,	true,	TYPE_INT,	2,	NULL,	(uint8_t*)"AutoRecover"},
{(uint8_t*)"水位滞后时间",	0,	COM_NUM_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME,	HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME,	2,	true,	0,	3600,	(uint8_t*)"s",	0,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterHysTime"},

{(uint8_t*)"电机运行电流", 0, COM_NUM_DEVICEPARAM_MOTOR_CURRENT, HOLDREGISTER_DEVICEPARAM_MOTOR_CURRENT, 2, true, 1, 31, NULL, 0, 0, true, TYPE_INT, 2, NULL, (uint8_t*)"MotorCur"},
{(uint8_t*)"编码轮周长",	0,	COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM,	2,	false,	0,	0,	(uint8_t*)"mm",	3,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EncWheelCirc"},
{(uint8_t*)"电机限速",	0,	COM_NUM_DEVICEPARAM_MAX_MOTOR_SPEED,	HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED,	2,	false,	0,	0,	(uint8_t*)"m/min",	2,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"MaxMotorSpd"},
{(uint8_t*)"首圈周长",	0,	COM_NUM_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FirstLoop"},
{(uint8_t*)"尺带厚度",	0,	COM_NUM_DEVICEPARAM_TAPE_THICKNESS_MM,	HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM,	2,	false,	0,	0,	(uint8_t*)"mm",	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"TapeThick"},
{(uint8_t*)"位置源自动切换",	0,	COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH,	HOLDREGISTER_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"PosAutoSw"},
{(uint8_t*)"记步模式",	0,	COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE,	HOLDREGISTER_DEVICEPARAM_POSITION_COUNT_MODE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"CntMode"},
{(uint8_t*)"电机局部周长",	0,	COM_NUM_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC,	HOLDREGISTER_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC,	2,	true,	50000,	5000000,	(uint8_t*)"mm",	3,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorCirc"},

{(uint8_t*)"空载扭力",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyTq"},
{(uint8_t*)"空载扭力上限",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyTqHi"},
{(uint8_t*)"空载扭力下限",	0,	COM_NUM_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,	HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"EmptyTqLo"},
{(uint8_t*)"满载扭力",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullTq"},
{(uint8_t*)"满载扭力上限",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullTqHi"},
{(uint8_t*)"满载扭力下限",	0,	COM_NUM_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,	HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"FullTqLo"},
{(uint8_t*)"碰撞上限比率",	0,	COM_NUM_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,	HOLDREGISTER_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO,	2,	false,	0,	0,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"UpperRatio"},
{(uint8_t*)"碰撞下限比率",	0,	COM_NUM_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,	HOLDREGISTER_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO,	2,	false,	0,	0,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"LowerRatio"},
{(uint8_t*)"保留8",	0,	COM_NUM_DEVICEPARAM_RESERVED8,	HOLDREGISTER_DEVICEPARAM_RESERVED8,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv8"},
{(uint8_t*)"保留9",	0,	COM_NUM_DEVICEPARAM_RESERVED9,	HOLDREGISTER_DEVICEPARAM_RESERVED9,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv9"},

{(uint8_t*)"零点阈值比例",	0,	COM_NUM_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO,	HOLDREGISTER_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO,	2,	true,	0,	100,	(uint8_t*)"%",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"ZeroThRatio"},
{(uint8_t*)"扭力忽略区",	0,	COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE,	HOLDREGISTER_DEVICEPARAM_WEIGHT_IGNORE_ZONE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"TorqueIgnore"},
{(uint8_t*)"零点最大偏差",	0,	COM_NUM_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE,	HOLDREGISTER_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"MaxZeroDev"},
{(uint8_t*)"找零下行距离",	0,	COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,	HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"FindZeroDown"},
{(uint8_t*)"保留10",	0,	COM_NUM_DEVICEPARAM_RESERVED10,	HOLDREGISTER_DEVICEPARAM_RESERVED10,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv10"},
{(uint8_t*)"保留11",	0,	COM_NUM_DEVICEPARAM_RESERVED11,	HOLDREGISTER_DEVICEPARAM_RESERVED11,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv11"},

{(uint8_t*)"液位测量方式",	0,	COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,	HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"LvlMeasMode"},
{(uint8_t*)"液位罐高",	0,	COM_NUM_DEVICEPARAM_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_TANKHEIGHT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"TankHeight"},
{(uint8_t*)"液位探头距差",	0,	COM_NUM_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF,	HOLDREGISTER_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF,	2,	false,	-999999,	999999,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"LvlSenDiff"},
{(uint8_t*)"液位盲区",	0,	COM_NUM_DEVICEPARAM_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_BLINDZONE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"BlindZone"},
{(uint8_t*)"液位找液阈值",	0,	COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD,	HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD,	2,	false,	0,	0,	(uint8_t*)"Hz",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"OilLvlTh"},
{(uint8_t*)"液位滞后阈值",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,	2,	false,	0,	0,	(uint8_t*)"Hz",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"HysTh"},
{(uint8_t*)"液位跟随频率",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_FREQUENCY,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_FREQUENCY,	2,	false,	0,	0,	(uint8_t*)"Hz",	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"FollowFreq"},
{(uint8_t*)"液位跟随密度",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_DENSITY,	2,	false,	0,	0,	(uint8_t*)"kg/m3",	2,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"FollowDens"},
{(uint8_t*)"液位滞后时间",	0,	COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME,	HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME,	2,	false,	0,	0,	(uint8_t*)"s",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"OilHysTime"},

{(uint8_t*)"水位测量方式",	0,	COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE,	HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_MODE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"WaterMode"},
{(uint8_t*)"水位罐高",	0,	COM_NUM_DEVICEPARAM_WATER_TANK_HEIGHT,	HOLDREGISTER_DEVICEPARAM_WATER_TANK_HEIGHT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WaterTankH"},
{(uint8_t*)"水位盲区",	0,	COM_NUM_DEVICEPARAM_WATER_BLINDZONE,	HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterBlind"},
{(uint8_t*)"水位跟随电容阈值",	0,	COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_WATER_CAP_THRESHOLD,	2,	false,	0,	0,	(uint8_t*)"pF",	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterCapTh"},
{(uint8_t*)"水位寻找电容阈值",	0,	COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD,	2,	false,	0,	0,	(uint8_t*)"pF",	3,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WaterFind"},
{(uint8_t*)"水位滞后电容阈值", 0, COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD, HOLDREGISTER_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD, 2, false, 0, 0, (uint8_t*)"pF", 3, 0, true, TYPE_INT, 6, NULL, (uint8_t*)"WaterLagCap"},
{(uint8_t*)"水位稳定距离",	0,	COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_WATER_STABLE_THRESHOLD,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"WaterStable"},
{(uint8_t*)"水位修正值",	0,	COM_NUM_DEVICEPARAM_WATER_LEVEL_CORRECTION,	HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_CORRECTION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WaterCorr"},
{(uint8_t*)"水位最大下行距离",	0,	COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE,	HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"MaxDown"},
{(uint8_t*)"零点电容",	0,	COM_NUM_DEVICEPARAM_ZERO_CAP,	HOLDREGISTER_DEVICEPARAM_ZERO_CAP,	2,	false,	0,	0,	(uint8_t*)"pF",	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"ZeroCap"},

{(uint8_t*)"罐底检测模式",	0,	COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE,	HOLDREGISTER_DEVICEPARAM_BOTTOM_DETECT_MODE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"BottomMode"},
{(uint8_t*)"探底角度阈值",	0,	COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD,	2,	false,	0,	0,	(uint8_t*)"°",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"BottomAngTh"},
{(uint8_t*)"探底扭力阈值",	0,	COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD,	HOLDREGISTER_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"BottomTqTh"},
{(uint8_t*)"更新罐高标志",	0,	COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG,	HOLDREGISTER_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"UpdTankHeight"},
{(uint8_t*)"实测罐高最大偏差",	0,	COM_NUM_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION,	HOLDREGISTER_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"MaxDev"},
{(uint8_t*)"罐底后编码器修正",	0,	COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE,	HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"BotEncFix"},
{(uint8_t*)"初始罐高",	0,	COM_NUM_DEVICEPARAM_INITIAL_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_INITIAL_TANKHEIGHT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"InitTankH"},
{(uint8_t*)"当前罐高",	0,	COM_NUM_DEVICEPARAM_CURRENT_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_CURRENT_TANKHEIGHT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	false,	TYPE_INT,	7,	NULL,	(uint8_t*)"CurrTankH"},

{(uint8_t*)"磁通量D",	0,	COM_NUM_DEVICEPARAM_DENSITYCORRECTION,	HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION,	2,	false,	-100000,	100000,	(uint8_t*)"kg/m3",	2,	-100000,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"DensityCorr"},
{(uint8_t*)"磁通量T",	0,	COM_NUM_DEVICEPARAM_TEMPERATURECORRECTION,	HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION,	2,	false,	-1000,	1000,	(uint8_t*)"℃",	1,	-1000,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"TempCorr"},
{(uint8_t*)"保留18",	0,	COM_NUM_DEVICEPARAM_RESERVED18,	HOLDREGISTER_DEVICEPARAM_RESERVED18,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv18"},
{(uint8_t*)"保留19",	0,	COM_NUM_DEVICEPARAM_RESERVED19,	HOLDREGISTER_DEVICEPARAM_RESERVED19,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv19"},

{(uint8_t*)"是否测罐底",	0,	COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,	HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"NeedBottom"},
{(uint8_t*)"是否测水位",	0,	COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT,	HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"NeedWater"},
{(uint8_t*)"是否测单点密度",	0,	COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,	HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"NeedSingleDens"},
{(uint8_t*)"分布测顺序",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"SpreadOrder"},
{(uint8_t*)"分布测模式",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE,	2,	false,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"SpreadMode"},
{(uint8_t*)"分布测点数",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTCOUNT,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT,	2,	true,	1,	200,	(uint8_t*)"点",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"SpreadCount"},
{(uint8_t*)"分布点间距",	0,	COM_NUM_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,	HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SpreadDist"},
{(uint8_t*)"最高点距液面",	0,	COM_NUM_DEVICEPARAM_SPREADTOPLIMIT,	HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Top2Level"},
{(uint8_t*)"最低点距罐底",	0,	COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT,	HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Bot2Tank"},
{(uint8_t*)"分布点悬停(s)",	0,	COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,	HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,	2,	true,	0,	600,	(uint8_t*)"s",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"PtHoverTime"},
{(uint8_t*)"区间测量上限",	0,	COM_NUM_DEVICEPARAM_INTERVAL_TOPLIMIT,	HOLDREGISTER_DEVICEPARAM_INTERVAL_TOPLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"IntTop"},
{(uint8_t*)"区间测量下限",	0,	COM_NUM_DEVICEPARAM_INTERVAL_BOTTOMLIMIT,	HOLDREGISTER_DEVICEPARAM_INTERVAL_BOTTOMLIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"IntBottom"},
{(uint8_t*)"保留20",	0,	COM_NUM_DEVICEPARAM_RESERVED20,	HOLDREGISTER_DEVICEPARAM_RESERVED20,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv20"},
{(uint8_t*)"保留21",	0,	COM_NUM_DEVICEPARAM_RESERVED21,	HOLDREGISTER_DEVICEPARAM_RESERVED21,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv21"},

{(uint8_t*)"最高密度点",	0,	COM_NUM_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WUpperDen"},
{(uint8_t*)"最低密度点",	0,	COM_NUM_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"WLowerDen"},
{(uint8_t*)"密度点间距",	0,	COM_NUM_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WDenStep"},
{(uint8_t*)"最高点液面距",	0,	COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,	2,	false,	0,	0,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"WMaxHeight"},
{(uint8_t*)"瓦锡兰探底间隔",	0,	COM_NUM_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL,	HOLDREGISTER_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL,	2,	true,	0,	100,	(uint8_t*)"次",	0,	0,	true,	TYPE_INT,	3,	NULL,	(uint8_t*)"WBotFreq"},
{(uint8_t*)"探底修正罐高",	0,	COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT,	HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"BotFixH"},

{(uint8_t*)"工作模式",	0,	COM_NUM_DEVICEPARAM_AO_WORK_MODE,	HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"WorkMode"},
{(uint8_t*)"电流模式",	0,	COM_NUM_DEVICEPARAM_AO_CURRENT_MODE,	HOLDREGISTER_DEVICEPARAM_AO_CURRENT_MODE,	2,	true,	0,	3,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"CurrentMode"},
{(uint8_t*)"输出源",	0,	COM_NUM_DEVICEPARAM_AO_OUTPUT_SOURCE,	HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_SOURCE,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"Source"},
{(uint8_t*)"电流修正",	0,	COM_NUM_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000,	HOLDREGISTER_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000,	2,	true,	AO_CURRENT_CORRECTION_MIN_MA_X1000,	AO_CURRENT_CORRECTION_MAX_MA_X1000,	(uint8_t*)"mA",	3,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"AOTrim"},
{(uint8_t*)"固定电流",	0,	COM_NUM_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100,	HOLDREGISTER_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100,	2,	true,	AO_FIXED_CURRENT_MIN_MA_X100,	AO_FIXED_CURRENT_MAX_MA_X100,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FixedCur"},
{(uint8_t*)"0%对应值",	0,	COM_NUM_DEVICEPARAM_AO_RANGE_0_01MM,	HOLDREGISTER_DEVICEPARAM_AO_RANGE_0_01MM,	2,	true,	0,	2147483647,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Range0"},
{(uint8_t*)"100%对应值",	0,	COM_NUM_DEVICEPARAM_AO_RANGE_100_01MM,	HOLDREGISTER_DEVICEPARAM_AO_RANGE_100_01MM,	2,	true,	0,	2147483647,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"Range100"},
{(uint8_t*)"阻尼系数",	0,	COM_NUM_DEVICEPARAM_AO_DAMPING_X10_S,	HOLDREGISTER_DEVICEPARAM_AO_DAMPING_X10_S,	2,	true,	0,	AO_DAMPING_MAX_X10_S,	(uint8_t*)"s",	1,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"Damping"},
{(uint8_t*)"故障动作",	0,	COM_NUM_DEVICEPARAM_AO_FAULT_MODE,	HOLDREGISTER_DEVICEPARAM_AO_FAULT_MODE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"FaultAction"},
{(uint8_t*)"故障电流",	0,	COM_NUM_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100,	HOLDREGISTER_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100,	2,	true,	AO_FAULT_CURRENT_MIN_MA_X100,	AO_FAULT_CURRENT_MAX_MA_X100,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"FaultCur"},
{(uint8_t*)"保留AO错误等级",	0,	COM_NUM_DEVICEPARAM_AO_ERROR_LEVEL,	HOLDREGISTER_DEVICEPARAM_AO_ERROR_LEVEL,	2,	true,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	1,	NULL,	(uint8_t*)"AOErrLvlRsv"},
{(uint8_t*)"初始电流",	0,	COM_NUM_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100,	HOLDREGISTER_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100,	2,	true,	AO_INITIAL_CURRENT_MIN_MA_X100,	AO_INITIAL_CURRENT_MAX_MA_X100,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"InitialCur"},
{(uint8_t*)"模拟电流",	0,	COM_NUM_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100,	HOLDREGISTER_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100,	2,	true,	AO_SIMULATION_CURRENT_MIN_MA_X100,	AO_SIMULATION_CURRENT_MAX_MA_X100,	(uint8_t*)"mA",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SimCur"},

{(uint8_t*)"标定液位值",	0,	COM_NUM_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CalOilLvl"},
{(uint8_t*)"标定水位值",	0,	COM_NUM_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CalWaterLvl"},
{(uint8_t*)"标定罐高值",	0,	COM_NUM_DEVICEPARAM_CALIBRATE_TANK_HEIGHT,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_TANK_HEIGHT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"CalTankH"},
{(uint8_t*)"单点测量位置",	0,	COM_NUM_DEVICEPARAM_SP_MEAS_POSITION,	HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MeasPos"},
{(uint8_t*)"单点监测位置",	0,	COM_NUM_DEVICEPARAM_SP_MONITOR_POSITION,	HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MonPos"},
{(uint8_t*)"电机运行位置",	0,	COM_NUM_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,	HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"DistOilLvl"},
{(uint8_t*)"电机上行距离",	0,	COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"保留28",	0,	COM_NUM_DEVICEPARAM_RESERVED28,	HOLDREGISTER_DEVICEPARAM_RESERVED28,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv28"},
{(uint8_t*)"保留29",	0,	COM_NUM_DEVICEPARAM_RESERVED29,	HOLDREGISTER_DEVICEPARAM_RESERVED29,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Rsv29"},

{(uint8_t*)"上次液位修正液位",	0,	COM_NUM_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL,	HOLDREGISTER_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"LastOilCorrLvl"},
{(uint8_t*)"气相温度",	0,	COM_NUM_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE,	HOLDREGISTER_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE,	2,	false,	0,	0,	(uint8_t*)"℃",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"GasTemp"},
{(uint8_t*)"尺带伸缩率",	0,	COM_NUM_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT,	HOLDREGISTER_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT,	2,	false,	0,	0,	NULL,	6,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"TapeExpCoeff"},
{(uint8_t*)"标定尺带温度",	0,	COM_NUM_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE,	HOLDREGISTER_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE,	2,	false,	0,	0,	(uint8_t*)"℃",	1,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"TapeCalTemp"},
{(uint8_t*)"SI首点",	0,	COM_NUM_DEVICEPARAM_SI_PROFILE_FIRST_POINT,	HOLDREGISTER_DEVICEPARAM_SI_PROFILE_FIRST_POINT,	2,	true,	10,	655350,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SI1stPt"},
{(uint8_t*)"SI步距",	0,	COM_NUM_DEVICEPARAM_SI_PROFILE_INCREMENT,	HOLDREGISTER_DEVICEPARAM_SI_PROFILE_INCREMENT,	2,	true,	10,	655350,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SIInc"},
{(uint8_t*)"SI停留",	0,	COM_NUM_DEVICEPARAM_SI_PROFILE_DWELL_TIME,	HOLDREGISTER_DEVICEPARAM_SI_PROFILE_DWELL_TIME,	2,	true,	1,	3600,	(uint8_t*)"s",	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"SIDwell"},
{(uint8_t*)"SI探底频次",	0,	COM_NUM_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL,	HOLDREGISTER_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL,	2,	true,	1,	1000,	(uint8_t*)"次",	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"SIBtmInt"},

/* ==================== 继电器报警输出配置（四路） ==================== */
{(uint8_t*)"K1工作模式",	0,	COM_NUM_DEVICEPARAM_RELAY1_OPERATING_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(0U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K1WorkMode"},
{(uint8_t*)"K1输出报警组合",	0,	COM_NUM_DEVICEPARAM_RELAY1_DIGITAL_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(0U),	2,	true,	0,	7,	NULL,	0,	0,	true,	TYPE_INT,	2,	ret_arr_word,	(uint8_t*)"K1OutAlarm"},
{(uint8_t*)"K1接点类型",	0,	COM_NUM_DEVICEPARAM_RELAY1_CONTACT_TYPE,	HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(0U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K1Contact"},
{(uint8_t*)"K1报警模式",	0,	COM_NUM_DEVICEPARAM_RELAY1_ALARM_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(0U),	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K1AlarmMode"},
{(uint8_t*)"K1无效值报警策略",	0,	COM_NUM_DEVICEPARAM_RELAY1_ERROR_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(0U),	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K1Invalid"},
{(uint8_t*)"K1报警取值源",	0,	COM_NUM_DEVICEPARAM_RELAY1_ALARM_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(0U),	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K1ValueSrc"},
{(uint8_t*)"K1高高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY1_HH_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(0U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K1HHValue"},
{(uint8_t*)"K1高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY1_H_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(0U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K1HValue"},
{(uint8_t*)"K1低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY1_L_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(0U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K1LValue"},
{(uint8_t*)"K1低低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY1_LL_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(0U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K1LLValue"},
{(uint8_t*)"K1报警滞回",	0,	COM_NUM_DEVICEPARAM_RELAY1_ALARM_HYSTERESIS,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(0U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K1Hysteresis"},
{(uint8_t*)"K1阻尼系数",	0,	COM_NUM_DEVICEPARAM_RELAY1_DAMPING_FACTOR,	HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(0U),	2,	true,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"K1Damping"},
{(uint8_t*)"K1清除锁存报警",	0,	COM_NUM_DEVICEPARAM_RELAY1_CLEAR_ALARM,	HOLDREGISTER_DEVICEPARAM_RELAY_CLEAR_ALARM(0U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K1Clear"},

{(uint8_t*)"K2工作模式",	0,	COM_NUM_DEVICEPARAM_RELAY2_OPERATING_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(1U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K2WorkMode"},
{(uint8_t*)"K2输出报警组合",	0,	COM_NUM_DEVICEPARAM_RELAY2_DIGITAL_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(1U),	2,	true,	0,	7,	NULL,	0,	0,	true,	TYPE_INT,	2,	ret_arr_word,	(uint8_t*)"K2OutAlarm"},
{(uint8_t*)"K2接点类型",	0,	COM_NUM_DEVICEPARAM_RELAY2_CONTACT_TYPE,	HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(1U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K2Contact"},
{(uint8_t*)"K2报警模式",	0,	COM_NUM_DEVICEPARAM_RELAY2_ALARM_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(1U),	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K2AlarmMode"},
{(uint8_t*)"K2无效值报警策略",	0,	COM_NUM_DEVICEPARAM_RELAY2_ERROR_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(1U),	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K2Invalid"},
{(uint8_t*)"K2报警取值源",	0,	COM_NUM_DEVICEPARAM_RELAY2_ALARM_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(1U),	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K2ValueSrc"},
{(uint8_t*)"K2高高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY2_HH_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(1U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K2HHValue"},
{(uint8_t*)"K2高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY2_H_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(1U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K2HValue"},
{(uint8_t*)"K2低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY2_L_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(1U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K2LValue"},
{(uint8_t*)"K2低低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY2_LL_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(1U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K2LLValue"},
{(uint8_t*)"K2报警滞回",	0,	COM_NUM_DEVICEPARAM_RELAY2_ALARM_HYSTERESIS,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(1U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K2Hysteresis"},
{(uint8_t*)"K2阻尼系数",	0,	COM_NUM_DEVICEPARAM_RELAY2_DAMPING_FACTOR,	HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(1U),	2,	true,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"K2Damping"},
{(uint8_t*)"K2清除锁存报警",	0,	COM_NUM_DEVICEPARAM_RELAY2_CLEAR_ALARM,	HOLDREGISTER_DEVICEPARAM_RELAY_CLEAR_ALARM(1U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K2Clear"},

{(uint8_t*)"K3工作模式",	0,	COM_NUM_DEVICEPARAM_RELAY3_OPERATING_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(2U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K3WorkMode"},
{(uint8_t*)"K3输出报警组合",	0,	COM_NUM_DEVICEPARAM_RELAY3_DIGITAL_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(2U),	2,	true,	0,	7,	NULL,	0,	0,	true,	TYPE_INT,	2,	ret_arr_word,	(uint8_t*)"K3OutAlarm"},
{(uint8_t*)"K3接点类型",	0,	COM_NUM_DEVICEPARAM_RELAY3_CONTACT_TYPE,	HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(2U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K3Contact"},
{(uint8_t*)"K3报警模式",	0,	COM_NUM_DEVICEPARAM_RELAY3_ALARM_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(2U),	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K3AlarmMode"},
{(uint8_t*)"K3无效值报警策略",	0,	COM_NUM_DEVICEPARAM_RELAY3_ERROR_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(2U),	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K3Invalid"},
{(uint8_t*)"K3报警取值源",	0,	COM_NUM_DEVICEPARAM_RELAY3_ALARM_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(2U),	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K3ValueSrc"},
{(uint8_t*)"K3高高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY3_HH_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(2U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K3HHValue"},
{(uint8_t*)"K3高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY3_H_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(2U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K3HValue"},
{(uint8_t*)"K3低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY3_L_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(2U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K3LValue"},
{(uint8_t*)"K3低低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY3_LL_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(2U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K3LLValue"},
{(uint8_t*)"K3报警滞回",	0,	COM_NUM_DEVICEPARAM_RELAY3_ALARM_HYSTERESIS,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(2U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K3Hysteresis"},
{(uint8_t*)"K3阻尼系数",	0,	COM_NUM_DEVICEPARAM_RELAY3_DAMPING_FACTOR,	HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(2U),	2,	true,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"K3Damping"},
{(uint8_t*)"K3清除锁存报警",	0,	COM_NUM_DEVICEPARAM_RELAY3_CLEAR_ALARM,	HOLDREGISTER_DEVICEPARAM_RELAY_CLEAR_ALARM(2U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K3Clear"},

{(uint8_t*)"K4工作模式",	0,	COM_NUM_DEVICEPARAM_RELAY4_OPERATING_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(3U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K4WorkMode"},
{(uint8_t*)"K4输出报警组合",	0,	COM_NUM_DEVICEPARAM_RELAY4_DIGITAL_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(3U),	2,	true,	0,	7,	NULL,	0,	0,	true,	TYPE_INT,	2,	ret_arr_word,	(uint8_t*)"K4OutAlarm"},
{(uint8_t*)"K4接点类型",	0,	COM_NUM_DEVICEPARAM_RELAY4_CONTACT_TYPE,	HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(3U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K4Contact"},
{(uint8_t*)"K4报警模式",	0,	COM_NUM_DEVICEPARAM_RELAY4_ALARM_MODE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(3U),	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K4AlarmMode"},
{(uint8_t*)"K4无效值报警策略",	0,	COM_NUM_DEVICEPARAM_RELAY4_ERROR_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(3U),	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K4Invalid"},
{(uint8_t*)"K4报警取值源",	0,	COM_NUM_DEVICEPARAM_RELAY4_ALARM_SOURCE,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(3U),	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K4ValueSrc"},
{(uint8_t*)"K4高高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY4_HH_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(3U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K4HHValue"},
{(uint8_t*)"K4高报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY4_H_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(3U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K4HValue"},
{(uint8_t*)"K4低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY4_L_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(3U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K4LValue"},
{(uint8_t*)"K4低低报警阈值",	0,	COM_NUM_DEVICEPARAM_RELAY4_LL_ALARM_VALUE,	HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(3U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K4LLValue"},
{(uint8_t*)"K4报警滞回",	0,	COM_NUM_DEVICEPARAM_RELAY4_ALARM_HYSTERESIS,	HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(3U),	2,	false,	0,	0,	NULL,	1,	0,	true,	TYPE_FLOAT,	7,	NULL,	(uint8_t*)"K4Hysteresis"},
{(uint8_t*)"K4阻尼系数",	0,	COM_NUM_DEVICEPARAM_RELAY4_DAMPING_FACTOR,	HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(3U),	2,	true,	0,	0,	NULL,	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"K4Damping"},
{(uint8_t*)"K4清除锁存报警",	0,	COM_NUM_DEVICEPARAM_RELAY4_CLEAR_ALARM,	HOLDREGISTER_DEVICEPARAM_RELAY_CLEAR_ALARM(3U),	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"K4Clear"},


{(uint8_t*)"参数版本号",	0,	COM_NUM_DEVICEPARAM_PARAM_VERSION,	HOLDREGISTER_DEVICEPARAM_PARAM_VERSION,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"ParamVer"},
{(uint8_t*)"结构体大小",	0,	COM_NUM_DEVICEPARAM_STRUCT_SIZE,	HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	4,	NULL,	(uint8_t*)"StructSize"},
{(uint8_t*)"魔术字",	0,	COM_NUM_DEVICEPARAM_MAGIC,	HOLDREGISTER_DEVICEPARAM_MAGIC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"Magic"},
{(uint8_t*)"参数CRC32",	0,	COM_NUM_DEVICEPARAM_CRC,	HOLDREGISTER_DEVICEPARAM_CRC,	2,	false,	0,	0,	NULL,	0,	0,	false,	TYPE_INT,	8,	NULL,	(uint8_t*)"ParamCRC"},


/* ==================== 下发指令参数（保持你原来的寄存器宏）==================== */
{(uint8_t*)"标定液位值",	0,	COM_NUM_CAL_OIL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalOilLvl"},
{(uint8_t*)"修正液位值",	0,	COM_NUM_CORRECTION_OIL,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalOilLvl"},
{(uint8_t*)"标定水位值",	0,	COM_NUM_CALIBRATE_WATER,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalWaterLvl"},
{(uint8_t*)"标定罐高值",	0,	COM_NUM_CALIBRATE_TANKHEIGHT,	HOLDREGISTER_DEVICEPARAM_CALIBRATE_TANK_HEIGHT,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"CalTankH"},
{(uint8_t*)"单点测量位置",	0,	COM_NUM_SINGLE_POINT,	HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MeasPos"},
{(uint8_t*)"单点监测位置",	0,	COM_NUM_SP_TEST,	HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"SP_MonPos"},
{(uint8_t*)"电机运行位置",	0,	COM_NUM_RUN_TO_POSITION,	HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"DistOilLvl"},
{(uint8_t*)"电机上行距离",	0,	COM_NUM_RUNUP,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"电机下行距离",	0,	COM_NUM_RUNDOWN,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"电机上行距离",	0,	COM_NUM_FORCE_RUNUP,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},
{(uint8_t*)"电机下行距离",	0,	COM_NUM_FORCE_RUNDOWN,	HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE,	2,	false,	0,	0,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"MotorDist"},

/* ==================== CPU3 本机参数（保持你原来的寄存器宏）==================== */
{(uint8_t*)"小数点位数",	0,	COM_NUM_SCREEN_DECIMAL,	HOLDREGISTER_CPU3_DECIMAL,	2,	true,	0,	4,	(uint8_t*)"位",	0,	0,	true,	TYPE_INT,	1,	NULL,	(uint8_t*)"Decimal"},
{(uint8_t*)"屏幕密码",	0,	COM_NUM_SCREEN_PASSWARD,	HOLDREGISTER_CPU3_PASSWORD,	2,	true,	0,	9999,	NULL,	0,	0,	true,	TYPE_INT,	4,	NULL,	(uint8_t*)"ScrPwd"},
{(uint8_t*)"息屏开关",	0,	COM_NUM_SCREEN_OFF,	HOLDREGISTER_CPU3_OFF_TIME,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"ScreenOff"},
{(uint8_t*)"屏幕亮度",	0,	COM_NUM_SCREEN_BRIGHTNESS,	HOLDREGISTER_CPU3_BRIGHTNESS,	2,	true,	0,	4,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"Brightness"},
{(uint8_t*)"SI周期",	0,	COM_NUM_CPU3_SI_AUTO_PROFILE_INTERVAL,	HOLDREGISTER_CPU3_SI_AUTO_PROFILE_INTERVAL,	2,	true,	1,	65535,	(uint8_t*)"min",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SIAutoInt"},
{(uint8_t*)"SI自动",	0,	COM_NUM_CPU3_SI_AUTO_PROFILE_ENABLE,	HOLDREGISTER_CPU3_SI_AUTO_PROFILE_ENABLE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"SIAutoEn"},
{(uint8_t*)"SI时",	0,	COM_NUM_CPU3_SI_AUTO_PROFILE_HOUR,	HOLDREGISTER_CPU3_SI_AUTO_PROFILE_HOUR,	2,	true,	0,	23,	(uint8_t*)"h",	0,	0,	true,	TYPE_INT,	2,	NULL,	(uint8_t*)"SIAutoHr"},
{(uint8_t*)"SI分",	0,	COM_NUM_CPU3_SI_AUTO_PROFILE_MINUTE,	HOLDREGISTER_CPU3_SI_AUTO_PROFILE_MINUTE,	2,	true,	0,	59,	(uint8_t*)"min",	0,	0,	true,	TYPE_INT,	2,	NULL,	(uint8_t*)"SIAutoMin"},
{(uint8_t*)"SI低密限",	0,	COM_NUM_CPU3_SI_LOW_DENSITY_SETPOINT,	HOLDREGISTER_CPU3_SI_LOW_DENSITY_SETPOINT,	2,	true,	0,	65535,	NULL,	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SILowDen"},
{(uint8_t*)"SI高密限",	0,	COM_NUM_CPU3_SI_HIGH_DENSITY_SETPOINT,	HOLDREGISTER_CPU3_SI_HIGH_DENSITY_SETPOINT,	2,	true,	0,	65535,	NULL,	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SIHighDen"},
{(uint8_t*)"SI低温限",	0,	COM_NUM_CPU3_SI_LOW_TEMPERATURE_SETPOINT,	HOLDREGISTER_CPU3_SI_LOW_TEMPERATURE_SETPOINT,	2,	true,	-32768,	32767,	(uint8_t*)"℃",	2,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"SILowTemp"},
{(uint8_t*)"SI高温限",	0,	COM_NUM_CPU3_SI_HIGH_TEMPERATURE_SETPOINT,	HOLDREGISTER_CPU3_SI_HIGH_TEMPERATURE_SETPOINT,	2,	true,	-32768,	32767,	(uint8_t*)"℃",	2,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"SIHighTemp"},
{(uint8_t*)"SI LL液位",	0,	COM_NUM_CPU3_SI_LL_LEVEL_SETPOINT,	HOLDREGISTER_CPU3_SI_LL_LEVEL_SETPOINT,	2,	true,	0,	65535,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SILLLevel"},
{(uint8_t*)"SI HH液位",	0,	COM_NUM_CPU3_SI_HH_LEVEL_SETPOINT,	HOLDREGISTER_CPU3_SI_HH_LEVEL_SETPOINT,	2,	true,	0,	65535,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SIHHLevel"},
{(uint8_t*)"SI低液位",	0,	COM_NUM_CPU3_SI_LOW_LEVEL_SETPOINT,	HOLDREGISTER_CPU3_SI_LOW_LEVEL_SETPOINT,	2,	true,	0,	65535,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SILowLvl"},
{(uint8_t*)"SI高液位",	0,	COM_NUM_CPU3_SI_HIGH_LEVEL_SETPOINT,	HOLDREGISTER_CPU3_SI_HIGH_LEVEL_SETPOINT,	2,	true,	0,	65535,	(uint8_t*)"mm",	0,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SIHighLvl"},
{(uint8_t*)"SI温差限",	0,	COM_NUM_CPU3_SI_TEMP_DEVIATION_SETPOINT,	HOLDREGISTER_CPU3_SI_TEMP_DEVIATION_SETPOINT,	2,	true,	0,	65535,	(uint8_t*)"℃",	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SITempDev"},
{(uint8_t*)"SI密差限",	0,	COM_NUM_CPU3_SI_DENSITY_DEVIATION_SETPOINT,	HOLDREGISTER_CPU3_SI_DENSITY_DEVIATION_SETPOINT,	2,	true,	0,	65535,	NULL,	2,	0,	true,	TYPE_INT,	5,	NULL,	(uint8_t*)"SIDenDev"},
{(uint8_t*)"语言",	0,	COM_NUM_PARA_LANG,	HOLDREGISTER_CPU3_LANGUAGE,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"Lang"},

{(uint8_t*)"液位数据源",	0,	COM_NUM_SCREEN_SOURCE_OIL,	HOLDREGISTER_CPU3_SRC_OIL,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"SrcOil"},
{(uint8_t*)"水位数据源",	0,	COM_NUM_SCREEN_SOURCE_WATER,	HOLDREGISTER_CPU3_SRC_WATER,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"SrcWater"},
{(uint8_t*)"密度数据源",	0,	COM_NUM_SCREEN_SOURCE_D,	HOLDREGISTER_CPU3_SRC_D,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"SrcD"},
{(uint8_t*)"温度数据源",	0,	COM_NUM_SCREEN_SOURCE_T,	HOLDREGISTER_CPU3_SRC_T,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"SrcT"},

{(uint8_t*)"液位手输值",	0,	COM_NUM_SCREEN_INPUT_OIL,	HOLDREGISTER_CPU3_IN_OIL,	2,	true,	0,	999999,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"InOil"},
{(uint8_t*)"水位手输值",	0,	COM_NUM_SCREEN_INPUT_WATER,	HOLDREGISTER_CPU3_IN_WATER,	2,	true,	0,	999999,	(uint8_t*)"mm",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"InWater"},
{(uint8_t*)"密度手输值",	0,	COM_NUM_SCREEN_INPUT_D,	HOLDREGISTER_CPU3_IN_D,	2,	true,	0,	300000,	(uint8_t*)"kg/m3",	2,	0,	true,	TYPE_INT,	6,	NULL,	(uint8_t*)"InD"},
{(uint8_t*)"上传手输密度",	0,	COM_NUM_SCREEN_INPUT_D_SWITCH,	HOLDREGISTER_CPU3_IN_D_SW,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"InDSw"},
{(uint8_t*)"温度手输值",	0,	COM_NUM_SCREEN_INPUT_T,	HOLDREGISTER_CPU3_IN_T,	2,	true,	-500,	2000,	(uint8_t*)"℃",	1,	0,	true,	TYPE_INT,	7,	NULL,	(uint8_t*)"InT"},

/* 协议菜单最大值 5 包含 SI；修改协议时自动带出协议默认串口参数，后续允许单独覆盖。 */
{(uint8_t*)"COM1协议",	0,	COM_NUM_CPU3_COM1_PROTOCOL,	HOLDREGISTER_CPU3_COM1_PROTO,	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Proto"},
{(uint8_t*)"COM1波特率",	0,	COM_NUM_CPU3_COM1_BAUDRATE,	HOLDREGISTER_CPU3_COM1_BAUD,	2,	true,	0,	7,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Baud"},
{(uint8_t*)"COM1数据位",	0,	COM_NUM_CPU3_COM1_DATABITS,	HOLDREGISTER_CPU3_COM1_DATABITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Data"},
{(uint8_t*)"COM1校验",	0,	COM_NUM_CPU3_COM1_PARITY,	HOLDREGISTER_CPU3_COM1_PARITY,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Parity"},
{(uint8_t*)"COM1停止位",	0,	COM_NUM_CPU3_COM1_STOPBITS,	HOLDREGISTER_CPU3_COM1_STOPBITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C1Stop"},

{(uint8_t*)"COM2协议",	0,	COM_NUM_CPU3_COM2_PROTOCOL,	HOLDREGISTER_CPU3_COM2_PROTO,	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Proto"},
{(uint8_t*)"COM2波特率",	0,	COM_NUM_CPU3_COM2_BAUDRATE,	HOLDREGISTER_CPU3_COM2_BAUD,	2,	true,	0,	7,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Baud"},
{(uint8_t*)"COM2数据位",	0,	COM_NUM_CPU3_COM2_DATABITS,	HOLDREGISTER_CPU3_COM2_DATABITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Data"},
{(uint8_t*)"COM2校验",	0,	COM_NUM_CPU3_COM2_PARITY,	HOLDREGISTER_CPU3_COM2_PARITY,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Parity"},
{(uint8_t*)"COM2停止位",	0,	COM_NUM_CPU3_COM2_STOPBITS,	HOLDREGISTER_CPU3_COM2_STOPBITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C2Stop"},

{(uint8_t*)"COM3协议",	0,	COM_NUM_CPU3_COM3_PROTOCOL,	HOLDREGISTER_CPU3_COM3_PROTO,	2,	true,	0,	5,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Proto"},
{(uint8_t*)"COM3波特率",	0,	COM_NUM_CPU3_COM3_BAUDRATE,	HOLDREGISTER_CPU3_COM3_BAUD,	2,	true,	0,	7,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Baud"},
{(uint8_t*)"COM3数据位",	0,	COM_NUM_CPU3_COM3_DATABITS,	HOLDREGISTER_CPU3_COM3_DATABITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Data"},
{(uint8_t*)"COM3校验",	0,	COM_NUM_CPU3_COM3_PARITY,	HOLDREGISTER_CPU3_COM3_PARITY,	2,	true,	0,	2,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Parity"},
{(uint8_t*)"COM3停止位",	0,	COM_NUM_CPU3_COM3_STOPBITS,	HOLDREGISTER_CPU3_COM3_STOPBITS,	2,	true,	0,	1,	NULL,	0,	0,	true,	TYPE_INT,	1,	ret_arr_word,	(uint8_t*)"C3Stop"},
};




/** `param_meta` 元数据表的有效表项数量，供索引查找和菜单遍历统一使用。 */
const int param_metaAmount = sizeof(param_meta) / sizeof(param_meta[0]);

/**
 * @brief 根据操作号查找对应的参数元数据索引。
 *
 * @param operanum 菜单或参数操作号，对应 `ParameterMetadata.operanum`。
 * @return 找到时返回 `param_meta` 的零基索引；未找到时返回 -1。
 */
int getHoldValueNum(int operanum)
{
    /* i 为元数据表的当前扫描索引，循环结束后同时用于判断是否命中。 */
    int i;
    for(i = 0;i < param_metaAmount;i++)
    {
        if(operanum == param_meta[i].operanum)
            break;
    }
    if(i >= param_metaAmount)
        return -1;
    else
        return i;
}

/**
 * @brief 初始化 CPU3 本地显示参数和首包同步前的测量运行态。
 *
 * @note 本函数在 CPU3 启动初始化阶段调用。它先应用本地显示参数，再将尚未从
 * CPU2 收到的数据设置为明确的无效值或未激活态，避免界面把零初始化误当成实测结果。
 */
void InputValueInit(void)
{
	Cpu3Local_ApplyDisplayRuntimeParams();

	g_measurement.oil_measurement.oil_level = UNVALID_LEVEL;
	g_measurement.water_measurement.water_level = LEVEL_DOWNLIMITWATER;
	g_measurement.device_status.device_state = STATE_INIT;
	g_measurement.single_point_monitoring.density = UNVALID_DENSITY;
	g_measurement.single_point_monitoring.temperature = UNVALID_TEMPERATURE_WIRELESS;

	/* channel 为零基继电器通道索引；CPU2 首包到达前逐通道初始化只读运行态。 */
	for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
		g_measurement.relay_alarm_runtime[channel].alarm_value = ((float)UNVALID_LEVEL) / 10.0f;
		g_measurement.relay_alarm_runtime[channel].HH_alarm = RELAY_ALARM_STATE_INACTIVE;
		g_measurement.relay_alarm_runtime[channel].H_alarm = RELAY_ALARM_STATE_INACTIVE;
		g_measurement.relay_alarm_runtime[channel].HH_H_alarm = RELAY_ALARM_STATE_INACTIVE;
		g_measurement.relay_alarm_runtime[channel].L_alarm = RELAY_ALARM_STATE_INACTIVE;
		g_measurement.relay_alarm_runtime[channel].LL_alarm = RELAY_ALARM_STATE_INACTIVE;
		g_measurement.relay_alarm_runtime[channel].LL_L_alarm = RELAY_ALARM_STATE_INACTIVE;
		g_measurement.relay_alarm_runtime[channel].any_error = RELAY_ALARM_STATE_INACTIVE;
		g_measurement.relay_alarm_runtime[channel].clear_alarm = RELAY_ALARM_CLEAR_NO;
	}
}
/**
 * @brief 将当前 CPU2 设备参数快照按业务分组完整打印到调试串口。
 *
 * @note 打印使用局部快照，避免输出过程中 CPU2 同步更新全局参数造成同一次
 * 打印前后字段不一致；本函数只读取参数，不修改运行态或持久化内容。
 */
void print_device_params(void)
{
    /* params 为本次打印使用的一致性快照，后续所有字段均从该副本读取。 */
    DeviceParameters params;
    /* 将全局设备参数复制到局部快照，本次整页打印后续只读取该副本，避免打印过程中反复访问全局结构。 */
    memcpy(&params, (void *)&g_deviceParams, sizeof(DeviceParameters));

    printf("\r\n========================================\r\n");
    printf("              设备参数\r\n");
    printf("========================================\r\n");

    /* 指令 */
    printf("\r\n-- 指令 --\r\n");
    printf("  %-32s : %u\r\n", "当前指令", (unsigned)params.command);
    printf("  %-32s : %u\r\n", "上电默认指令", (unsigned)params.powerOnDefaultCommand);

    /* 基础参数 */
    printf("\r\n-- 基础参数 --\r\n");
    printf("  %-32s : %s(%lu)\r\n",
           "传感器类型",
           sensor_type_str(params.sensorType),
           (unsigned long)params.sensorType);
    printf("  %-32s : %lu\r\n", "传感器编号", (unsigned long)params.sensorID);
    printf("  %-32s : 0x%08lX\r\n", "传感器软件版本", (unsigned long)params.sensorSoftwareVersion);
    printf("  %-32s : 0x%08lX\r\n", "CPU2程序版本", (unsigned long)params.softwareVersion);
    printf("  %-32s : %lu\r\n", "协议版本", (unsigned long)params.protocolVersion);
    printf("  %-32s : %lu\r\n", "故障自动回零", (unsigned long)params.error_auto_back_zero);
    printf("  %-32s : %lu\r\n", "故障停止测量", (unsigned long)params.error_stop_measurement);
    printf("  %-32s : %lu\r\n", "故障自动恢复重跑次数", (unsigned long)params.fault_auto_recovery_retry_limit);
    printf("  %-32s : %lu\r\n", "位置源自动切换", (unsigned long)params.position_source_auto_switch);

    /* 电机与编码器 */
    printf("\r\n-- 电机与编码器 --\r\n");
    printf("  %-32s : %lu\r\n", "编码轮周长(0.001mm)", (unsigned long)params.encoder_wheel_circumference_mm);
    printf("  %-32s : %lu\r\n", "电机运行电流(IRUN)", (unsigned long)params.motor_current);
    printf("  %-32s : %lu\r\n", "电机限速(0.01m/min)", (unsigned long)params.max_motor_speed);
    printf("  %-32s : %lu\r\n", "首圈周长(0.1mm)", (unsigned long)params.first_loop_circumference_mm);
    printf("  %-32s : %lu\r\n", "尺带厚度(0.001mm)", (unsigned long)params.tape_thickness_mm);
    printf("  %-32s : %lu\r\n", "记步模式", (unsigned long)params.position_count_mode);
    printf("  %-32s : %lu\r\n", "电机局部周长(0.001mm)", (unsigned long)params.motor_count_first_loop_circumference_mm);

    /* 扭力 */
    printf("\r\n-- 扭力参数 --\r\n");
    printf("  %-32s : %ld\r\n", "空载扭力", (long)params.empty_weight);
    printf("  %-32s : %lu\r\n", "空载扭力上限", (unsigned long)params.empty_weight_upper_limit);
    printf("  %-32s : %lu\r\n", "空载扭力下限", (unsigned long)params.empty_weight_lower_limit);
    printf("  %-32s : %lu\r\n", "满载扭力", (unsigned long)params.full_weight);
    printf("  %-32s : %lu\r\n", "满载扭力上限", (unsigned long)params.full_weight_upper_limit);
    printf("  %-32s : %lu\r\n", "满载扭力下限", (unsigned long)params.full_weight_lower_limit);
    printf("  %-32s : %lu\r\n", "碰撞上限比率", (unsigned long)params.weight_upper_limit_ratio);
    printf("  %-32s : %lu\r\n", "碰撞下限比率", (unsigned long)params.weight_lower_limit_ratio);

    /* 零点 */
    printf("\r\n-- 零点参数 --\r\n");
    printf("  %-32s : %lu\r\n", "零点阈值比例", (unsigned long)params.zero_weight_threshold_ratio);
    printf("  %-32s : %lu\r\n", "扭力忽略区(0.1mm)", (unsigned long)params.weight_ignore_zone);
    printf("  %-32s : %lu\r\n", "零点最大偏差(0.1mm)", (unsigned long)params.max_zero_deviation_distance);
    printf("  %-32s : %lu\r\n", "找零下行距离(0.1mm)", (unsigned long)params.findZeroDownDistance);

    /* 液位 */
    printf("\r\n-- 液位参数 --\r\n");
    printf("  %-32s : %lu\r\n", "液位罐高(0.1mm)", (unsigned long)params.tankHeight);
    printf("  %-32s : %ld\r\n", "液位探头距差(0.1mm)", (long)(int32_t)params.liquid_sensor_distance_diff);
    printf("  %-32s : %lu\r\n", "液位盲区(0.1mm)", (unsigned long)params.blindZone);
    printf("  %-32s : %lu\r\n", "液位找液阈值", (unsigned long)params.oilLevelThreshold);
    printf("  %-32s : %lu\r\n", "液位滞后阈值", (unsigned long)params.oilLevelHysteresisThreshold);
    printf("  %-32s : %lu\r\n", "液位测量方式", (unsigned long)params.liquidLevelMeasurementMethod);
    printf("  %-32s : %lu\r\n", "液位跟随频率", (unsigned long)params.oilLevelFrequency);
    printf("  %-32s : %lu\r\n", "液位跟随密度", (unsigned long)params.oilLevelDensity);
    printf("  %-32s : %lu\r\n", "液位滞后时间", (unsigned long)params.oilLevelHysteresisTime);

    /* 水位 */
    printf("\r\n-- 水位参数 --\r\n");
    printf("  %-32s : %lu\r\n", "水位罐高(0.1mm)", (unsigned long)params.water_tank_height);
    printf("  %-32s : %lu\r\n", "水位测量方式", (unsigned long)params.water_level_mode);
    printf("  %-32s : %lu\r\n", "水位盲区(0.1mm)", (unsigned long)params.waterBlindZone);
    printf("  %-32s : %lu\r\n", "水位电容阈值", (unsigned long)params.water_cap_threshold);
    printf("  %-32s : %lu\r\n", "水位寻找电容阈值", (unsigned long)params.water_find_cap_threshold);
    printf("  %-32s : %lu\r\n", "水位最大下行距离(0.1mm)", (unsigned long)params.maxDownDistance);
    printf("  %-32s : %lu\r\n", "水位零点电容", (unsigned long)params.zero_cap);
    printf("  %-32s : %lu\r\n", "水位稳定距离", (unsigned long)params.water_stable_threshold);
    printf("  %-32s : %lu\r\n", "水位滞后电容阈值", (unsigned long)params.water_lag_cap_threshold);
    printf("  %-32s : %lu\r\n", "水位滞后时间(s)", (unsigned long)params.water_level_hysteresis_time_s);
    printf("  %-32s : %lu\r\n", "水位修正值", (unsigned long)params.waterLevelCorrection);

    /* 罐底/罐高 */
    printf("\r\n-- 罐底/罐高参数 --\r\n");
    printf("  %-32s : %lu\r\n", "罐底检测模式", (unsigned long)params.bottom_detect_mode);
    printf("  %-32s : %lu\r\n", "探底角度阈值", (unsigned long)params.bottom_angle_threshold);
    printf("  %-32s : %lu\r\n", "探底扭力阈值", (unsigned long)params.bottom_weight_threshold);
    printf("  %-32s : %lu\r\n", "更新罐高标志", (unsigned long)params.refreshTankHeightFlag);
    printf("  %-32s : %lu\r\n", "实测罐高最大偏差", (unsigned long)params.maxTankHeightDeviation);
    printf("  %-32s : %lu\r\n", "初始罐高", (unsigned long)params.initialTankHeight);
    printf("  %-32s : %lu\r\n", "当前罐高", (unsigned long)params.currentTankHeight);
    printf("  %-32s : %lu\r\n", "罐底后编码器修正", (unsigned long)params.bottom_encoder_correction_enable);

    /* 修正 */
    printf("\r\n-- 修正参数 --\r\n");
    printf("  %-32s : %lu\r\n", "密度修正值", (unsigned long)params.densityCorrection);
    printf("  %-32s : %lu\r\n", "温度修正值", (unsigned long)params.temperatureCorrection);

    /* 分布/区间 */
    printf("\r\n-- 分布/区间参数 --\r\n");
    printf("  %-32s : %lu\r\n", "是否测罐底", (unsigned long)params.requireBottomMeasurement);
    printf("  %-32s : %lu\r\n", "是否测水位", (unsigned long)params.requireWaterMeasurement);
    printf("  %-32s : %lu\r\n", "是否测单点密度", (unsigned long)params.requireSinglePointDensity);
    printf("  %-32s : %lu\r\n", "分布测顺序", (unsigned long)params.spreadMeasurementOrder);
    printf("  %-32s : %lu\r\n", "分布测模式", (unsigned long)params.spreadMeasurementMode);
    printf("  %-32s : %lu\r\n", "分布测点数", (unsigned long)params.spreadMeasurementCount);
    printf("  %-32s : %lu\r\n", "分布点间距", (unsigned long)params.spreadMeasurementDistance);
    printf("  %-32s : %lu\r\n", "最高点距液面(0.1mm)", (unsigned long)params.spreadTopLimit);
    printf("  %-32s : %lu\r\n", "最低点距罐底(0.1mm)", (unsigned long)params.spreadBottomLimit);
    printf("  %-32s : %lu\r\n", "分布点悬停时间", (unsigned long)params.spreadPointHoverTime);
    printf("  %-32s : %lu\r\n", "区间测量上限(0.1mm)", (unsigned long)params.intervalMeasurementTopLimit);
    printf("  %-32s : %lu\r\n", "区间测量下限(0.1mm)", (unsigned long)params.intervalMeasurementBottomLimit);

    /* Wartsila */
    printf("\r\n-- 瓦锡兰参数 --\r\n");
    printf("  %-32s : %lu\r\n", "最高密度点", (unsigned long)params.wartsila_upper_density_limit);
    printf("  %-32s : %lu\r\n", "最低密度点", (unsigned long)params.wartsila_lower_density_limit);
    printf("  %-32s : %lu\r\n", "密度点间距", (unsigned long)params.wartsila_density_interval);
    printf("  %-32s : %lu\r\n", "最高点液面距", (unsigned long)params.wartsila_max_height_above_surface);
    printf("  %-32s : %lu\r\n", "瓦锡兰探底间隔", (unsigned long)params.wartsila_bottom_detect_interval);
    printf("  %-32s : %lu\r\n", "探底修正罐高", (unsigned long)params.bottom_encoder_correction_tank_height);

    /* channel 为零基通道索引，逐项打印各继电器的配置和浮点阈值。 */
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        /* cfg 指向当前通道在局部参数快照中的报警配置。 */
        const RelayAlarmConfig *cfg = &params.relayAlarm[channel];
        printf("  继电器%lu报警: 模式=%s(%lu) 报警源=%s(%lu) 报警位=%s(%lu) 接点=%s(%lu) 报警模式=%s(%lu) 无效值=%s(%lu)\r\n",
               (unsigned long)(channel + 1U),
               relay_operating_mode_str(cfg->operating_mode),
               (unsigned long)cfg->operating_mode,
               relay_alarm_source_str(cfg->alarm_source),
               (unsigned long)cfg->alarm_source,
               relay_digital_source_str(cfg->digital_source),
               (unsigned long)cfg->digital_source,
               relay_contact_type_str(cfg->contact_type),
               (unsigned long)cfg->contact_type,
               relay_alarm_mode_str(cfg->alarm_mode),
               (unsigned long)cfg->alarm_mode,
               relay_error_value_str(cfg->error_value),
               (unsigned long)cfg->error_value);
        printf("    阈值: HH=%.1f H=%.1f L=%.1f LL=%.1f 滞回=%.1f 阻尼=%lu 清锁存=%lu\r\n",
               relay_alarm_raw_to_float(cfg->HH_alarm_value),
               relay_alarm_raw_to_float(cfg->H_alarm_value),
               relay_alarm_raw_to_float(cfg->L_alarm_value),
               relay_alarm_raw_to_float(cfg->LL_alarm_value),
               relay_alarm_raw_to_float(cfg->alarm_hysteresis),
               (unsigned long)cfg->damping_factor,
               (unsigned long)cfg->clear_alarm);
    }

    /* AO */
    printf("\r\n-- 4-20mA/AO参数 --\r\n");
    printf("  %-32s : %lu\r\n", "AO工作模式", (unsigned long)params.ao_output.work_mode);
    printf("  %-32s : %lu\r\n", "AO电流模式", (unsigned long)params.ao_output.current_mode);
    printf("  %-32s : %lu\r\n", "AO输出源", (unsigned long)params.ao_output.output_source);
    printf("  %-32s : %ld\r\n", "AO电流修正(x0.001mA)", (long)params.ao_output.current_correction_mA_x1000);
    printf("  %-32s : %lu\r\n", "AO固定电流(x0.01mA)", (unsigned long)params.ao_output.fixed_current_mA_x100);
    printf("  %-32s : %ld\r\n", "AO 0%对应值(x0.1mm)", (long)params.ao_output.range_0_01mm);
    printf("  %-32s : %ld\r\n", "AO 100%对应值(x0.1mm)", (long)params.ao_output.range_100_01mm);
    printf("  %-32s : %lu\r\n", "AO阻尼(x0.1s)", (unsigned long)params.ao_output.damping_x10_s);
    printf("  %-32s : %lu\r\n", "AO故障动作", (unsigned long)params.ao_output.fault_mode);
    printf("  %-32s : %lu\r\n", "AO故障电流(x0.01mA)", (unsigned long)params.ao_output.fault_current_mA_x100);
    printf("  %-32s : %lu\r\n", "AO预留槽位", (unsigned long)params.ao_output.error_level);
    printf("  %-32s : %lu\r\n", "AO初始电流(x0.01mA)", (unsigned long)params.ao_output.power_on_current_mA_x100);
    printf("  %-32s : %lu\r\n", "AO仿真电流(x0.01mA)", (unsigned long)params.ao_output.simulation_current_mA_x100);

    /* 指令参数 */
    printf("\r\n-- 指令参数 --\r\n");
    printf("  %-32s : %lu\r\n", "标定液位值", (unsigned long)params.calibrateOilLevel);
    printf("  %-32s : %lu\r\n", "标定水位值", (unsigned long)params.calibrateWaterLevel);
    printf("  %-32s : %lu\r\n", "单点测量位置", (unsigned long)params.singlePointMeasurementPosition);
    printf("  %-32s : %lu\r\n", "单点监测位置", (unsigned long)params.singlePointMonitoringPosition);
    printf("  %-32s : %lu\r\n", "密度分布测量液位", (unsigned long)params.densityDistributionOilLevel);
    printf("  %-32s : %lu\r\n", "电机指令距离", (unsigned long)params.motorCommandDistance);

    printf("\r\n-- 尺带补偿参数 --\r\n");
    printf("  %-32s : %lu\r\n", "上次液位修正液位", (unsigned long)params.lastOilCorrectionLevel);
    printf("  %-32s : %lu\r\n", "气相温度", (unsigned long)params.tankGasPhaseTemperature);
    printf("  %-32s : %lu\r\n", "尺带伸缩率", (unsigned long)params.tapeExpansionCoefficient);
    printf("  %-32s : %lu\r\n", "尺带标定温度", (unsigned long)params.tapeCalibrationTemperature);

    /* 元信息/CRC */
    printf("\r\n-- 元信息/CRC --\r\n");
    printf("  %-32s : %lu\r\n", "参数版本号", (unsigned long)params.param_version);
    printf("  %-32s : %lu\r\n", "结构体大小", (unsigned long)params.struct_size);
    printf("  %-32s : 0x%08lX\r\n", "魔术字", (unsigned long)params.magic);
    printf("  %-32s : 0x%08lX\r\n", "参数CRC32", (unsigned long)params.crc);

    printf("========================================\r\n");
}
/* ========================= 测量结果打印（可选） ========================= */
/* 注: 该部分与参数结构无强耦合，仅保留你现有打印习惯；如果不需要可移除 */

/**
 * @brief 打印一组密度测量结果的温度、密度及位置字段。
 *
 * @param title 本组数据的显示标题，用于区分单点测量和单点监测。
 * @param d 待打印的密度测量结构；传入 NULL 时直接返回且不输出。
 * @note 本函数按结构体保存的原始整数值打印，不在此处执行单位换算或有效性修正。
 */
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
/**
 * @brief 按业务分区完整打印当前设备测量结果快照。
 *
 * @param m 待打印的测量结果结构；传入 NULL 时直接返回且不输出。
 * @note 输出包括设备状态、调试数据、液位、水位、罐高、密度分布和继电器运行态。
 * 密度分布单点数据最多打印前 10 项，避免调试串口被 200 点完整数据长时间占用。
 */
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
    printf("【调试数据】\r\n");
    printf("  编码值: %ld\r\n",        (long)m->debug_data.current_encoder_value);
    printf("  传感器位置: %ld mm\r\n", (long)m->debug_data.sensor_position);
    printf("  尺带长度: %ld mm\r\n",   (long)m->debug_data.cable_length);
    printf("  电机步进: %ld\r\n",      (long)m->debug_data.motor_step);
    printf("  电机距离: %ld (0.1mm)\r\n",(long)m->debug_data.motor_distance);

    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->debug_data.frequency);
    printf("  温度: %.2f ℃\r\n",       (float)m->debug_data.temperature / 100.0f);
    printf("  空气中频率: %lu Hz\r\n", (unsigned long)m->debug_data.air_frequency);
    printf("  幅值: %lu\r\n",          (unsigned long)m->debug_data.current_amplitude);
    printf("  水位电容快照(0.1pF): %lu\r\n",(unsigned long)m->debug_data.water_capacitance_x10);

    printf("  当前扭力值: %lu\r\n",    (unsigned long)m->debug_data.current_weight);
    printf("  扭力模块温度位模式: 0x%08lX\r\n",
           (unsigned long)m->debug_data.torque_temperature_bits);

    printf("  X角度: %ld\r\n", (long)m->debug_data.angle_x);
    printf("  Y角度: %ld\r\n", (long)m->debug_data.angle_y);

    printf("  电机速度(0.01m/min): %lu\r\n", (unsigned long)m->debug_data.motor_speed);
    printf("  电机状态: %lu (%s)\r\n",
           (unsigned long)m->debug_data.motor_state,
           (m->debug_data.motor_state == 0) ? "停止" :
           (m->debug_data.motor_state == 1) ? "上行" :
           (m->debug_data.motor_state == 2) ? "下行" : "未知");
    printf("  磁零点电压: %.6f V\r\n", (double)m->debug_data.magnetic_zero_voltage);
    printf("  动力黏度: %.6f cP\r\n", (double)m->debug_data.dynamic_viscosity_cp);
    printf("  运动黏度: %.6f cSt\r\n", (double)m->debug_data.kinematic_viscosity_cst);
    printf("  传感器供电电压: %.6f V\r\n", (double)m->debug_data.supply_voltage_v);

    printf("--------------------------------------------------------------\r\n");

    /* 3. 液位测量 */
    printf("【液位测量】\r\n");
    printf("  跟随液位: %lu mm\r\n",   (unsigned long)m->oil_measurement.oil_level);
    printf("  空气频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.air_frequency);
    printf("  油中频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.oil_frequency);
    printf("  跟随频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.follow_frequency);
    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.current_frequency);

    printf("--------------------------------------------------------------\r\n");

    /* 4. 水位测量 */
    printf("【水位测量】\r\n");
    printf("  水位值: %lu mm\r\n", (unsigned long)m->water_measurement.water_level);
    printf("  零点电容: %.3f\r\n",(float) m->water_measurement.zero_capacitance);
    printf("  油区电容: %.3f\r\n",(float) m->water_measurement.oil_capacitance);
    printf("  当前电容: %.3f\r\n",(float) m->water_measurement.current_capacitance);

    printf("--------------------------------------------------------------\r\n");

    /* 5. 实高测量 */
    printf("【实高测量】\r\n");
    printf("  标定液位实高: %lu mm\r\n", (unsigned long)m->height_measurement.calibrated_liquid_level);
    printf("  当前实高: %lu mm\r\n",     (unsigned long)m->height_measurement.current_real_height);

    printf("--------------------------------------------------------------\r\n");

    /* 6. 单点密度测量 */
    printf("【单点密度测量】\r\n");
    PrintDensity("单点测量", &m->single_point_measurement);

    /* 7. 单点监测 */
    printf("【单点监测】\r\n");
    PrintDensity("单点监测", &m->single_point_monitoring);

    printf("--------------------------------------------------------------\r\n");

    /* 8. 密度分布（概要） */
    printf("【密度分布】\r\n");
    printf("  平均温度: %lu\r\n",     (unsigned long)m->density_distribution.average_temperature);
    printf("  平均密度: %lu\r\n",     (unsigned long)m->density_distribution.average_density);
    printf("  平均计重密度: %lu\r\n", (unsigned long)m->density_distribution.average_weight_density);
    printf("  测量点数: %lu\r\n",     (unsigned long)m->density_distribution.measurement_points);
    printf("  测量时液位: %lu mm\r\n",(unsigned long)m->density_distribution.Density_oil_level);

    printf("  --- 单点数据（仅打印前10个）---\r\n");
    /* i 为密度分布点索引；最多输出前 10 点且不超过本次有效测量点数。 */
    for (uint32_t i = 0; i < 10 && i < m->density_distribution.measurement_points; i++)
    {
        /* d 指向当前分布测量点，字段按结构体中的原始整数格式打印。 */
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


    printf("--------------------------------------------------------------\r\n");
    printf("【继电器报警输出运行态】\r\n");
    /* channel 为零基继电器通道索引，逐通道输出 CPU2 同步的只读报警运行态。 */
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        /* state 指向当前通道的报警判定、组合状态和清锁存状态。 */
        const RelayAlarmRuntimeState *state = &m->relay_alarm_runtime[channel];
        printf("  继电器%lu: value=%.1f HH=%lu H=%lu HH_H=%lu L=%lu LL=%lu LL_L=%lu any=%lu clear=%lu\r\n",
               (unsigned long)(channel + 1U),
               (double)state->alarm_value,
               (unsigned long)state->HH_alarm,
               (unsigned long)state->H_alarm,
               (unsigned long)state->HH_H_alarm,
               (unsigned long)state->L_alarm,
               (unsigned long)state->LL_alarm,
               (unsigned long)state->LL_L_alarm,
               (unsigned long)state->any_error,
               (unsigned long)state->clear_alarm);
    }
    printf("========================【打印结束】========================\r\n");
}
