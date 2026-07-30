#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "dataanalysis_modbus.h"
#include "usart.h"
#include "stateformodbus.h"
#include <ctype.h>
#include <float.h>
#include "system_parameter.h"
#include "encoder.h"
#include "motor_ctrl.h"
#include "AoOutput/ao_output.h"

/* ===================== 通用寄存器读写函数 ===================== */

/**
 * @brief 写入 uint32_t 到寄存器数组（高 16 位在前，低 16 位在后）。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param addr 目标 32 位值高 16 位所在的零基寄存器下标；低 16 位写入 addr + 1。
 * @param value 待编码写入线格式的 32 位数值。
 */
static inline void write_u32_to_regs(uint16_t *regs, uint16_t addr, uint32_t value) {
	regs[addr] = (uint16_t) ((value >> 16) & 0xFFFFu);
	regs[addr + 1] = (uint16_t) (value & 0xFFFFu);
}

/**
 * @brief 从寄存器数组读取 uint32_t。
 *
 * @param regs 连续寄存器值缓冲区。函数按既定寄存器数量只读 regs[]，并按高低字、字段偏移或协议映射解析业务值。
 * @param addr 目标 32 位值高 16 位所在的零基寄存器下标；低 16 位从 addr + 1 读取。
 * @return 返回寄存器数组中高字在前的两个 16 位寄存器组合得到的 32 位无符号值。
 */
static inline uint32_t read_u32_from_regs(const uint16_t *regs, uint16_t addr) {
	return ((uint32_t) regs[addr] << 16) | (uint32_t) regs[addr + 1];
}

/**
 * @brief 保护电机运行电流，避免 0 或越界值被保存并用于初始化 TMC5130。
 *
 * @param value 准备写入 TMC5130 IRUN 的电流档位，合法范围为 MOTOR_CURRENT_MIN 至 MOTOR_CURRENT_MAX。
 * @return value 位于 MOTOR_CURRENT_MIN 至 MOTOR_CURRENT_MAX 时原样返回；0 或越界值返回 MOTOR_CURRENT_DEFAULT，当前默认
 *         IRUN 档位为 12。
 */
static inline uint32_t normalize_motor_current(uint32_t value) {
    if ((value < MOTOR_CURRENT_MIN) || (value > MOTOR_CURRENT_MAX)) {
        return MOTOR_CURRENT_DEFAULT;
    }
    return value;
}
/**
 * @brief 写入 int32_t 到寄存器数组。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param addr 目标有符号 32 位值高 16 位所在的零基寄存器下标；低 16 位写入 addr + 1。
 * @param value 待写入协议缓冲区或寄存器的16 位数值。
 */
static inline void write_i32_to_regs(uint16_t *regs, uint16_t addr, int32_t value) {
	write_u32_to_regs(regs, addr, (uint32_t) value);
}

/**
 * @brief 从寄存器数组读取 int32_t。
 *
 * @param regs 连续寄存器值缓冲区。函数按既定寄存器数量只读 regs[]，并按高低字、字段偏移或协议映射解析业务值。
 * @param addr 目标有符号 32 位值高 16 位所在的零基寄存器下标；低 16 位从 addr + 1 读取。
 * @return 返回寄存器数组中两个 16 位寄存器组合并按补码解释的 32 位有符号值。
 */
static inline int32_t read_i32_from_regs(const uint16_t *regs, uint16_t addr) {
	return (int32_t) read_u32_from_regs(regs, addr);
}

/**
 * @brief 写入 float 到寄存器数组（按 IEEE754 的 uint32_t 比特位存放）。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param addr 目标 Float32 位模式高 16 位所在的零基寄存器下标；低 16 位写入 addr + 1。
 * @param value 待写入协议缓冲区或寄存器的16 位数值。
 */
static inline void write_float_to_regs(uint16_t *regs, uint16_t addr, float value) {
	uint32_t temp;
	/* 把 float 的 IEEE-754 Float32 位模式复制为 32 位整数，再由 write_u32_to_regs 按约定寄存器字序拆分。 */
	memcpy(&temp, &value, sizeof(float));
	write_u32_to_regs(regs, addr, temp);
}



/**
 * @brief 写入单路继电器报警输出配置。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param channel 零基通道号。合法范围为 0～3，用于选择 CPU2 设备参数中的对应继电器配置块。
 * @param cfg 共享设备参数区中的只读单路继电器报警配置；读取时保持 volatile 语义，用于生成寄存器镜像或运行态配置快照。
 */
static void write_relay_alarm_config_to_regs(uint16_t *regs, uint32_t channel, const volatile RelayAlarmConfig *cfg)
{
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(channel), cfg->operating_mode);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(channel), cfg->digital_source);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(channel), cfg->contact_type);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(channel), cfg->alarm_mode);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(channel), cfg->error_value);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(channel), cfg->alarm_source);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(channel), cfg->HH_alarm_value);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(channel), cfg->H_alarm_value);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(channel), cfg->L_alarm_value);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(channel), cfg->LL_alarm_value);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(channel), cfg->alarm_hysteresis);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(channel), cfg->damping_factor);
    write_u32_to_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_CLEAR_ALARM(channel), cfg->clear_alarm);
}

/**
 * @brief 从保持寄存器读取单路继电器报警输出配置。
 *
 * @param regs 连续寄存器值缓冲区。函数按既定寄存器数量只读 regs[]，并按高低字、字段偏移或协议映射解析业务值。
 * @param channel 零基通道号。合法范围为 0～3，用于选择 CPU2 设备参数中的对应继电器配置块。
 * @param cfg 共享设备参数区中的可写单路继电器报警配置；写入时保持 volatile 语义，并只更新当前函数负责的模式、阈值、滞回或锁存命令字段。
 */
static void read_relay_alarm_config_from_regs(const uint16_t *regs, uint32_t channel, volatile RelayAlarmConfig *cfg)
{
    cfg->operating_mode = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(channel));
    cfg->digital_source = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(channel));
    cfg->contact_type = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(channel));
    cfg->alarm_mode = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(channel));
    cfg->error_value = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(channel));
    cfg->alarm_source = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(channel));
    cfg->HH_alarm_value = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(channel));
    cfg->H_alarm_value = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(channel));
    cfg->L_alarm_value = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(channel));
    cfg->LL_alarm_value = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(channel));
    cfg->alarm_hysteresis = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(channel));
    cfg->damping_factor = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(channel));
    cfg->clear_alarm = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RELAY_CLEAR_ALARM(channel));
}


/**
 * @brief 写入单路继电器报警输出运行态。运行态寄存器按参考程序只读区布局：报警值占2个寄存器，其余状态各占1个寄存器。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param channel 零基通道号。合法范围为 0～3，用于选择 CPU2 设备参数中的对应继电器配置块。
 * @param state 待编码到 CPU2 输入寄存器的单路继电器运行快照，只读使用其报警值和各状态字段。
 */
static void write_relay_alarm_runtime_to_regs(uint16_t *regs, uint32_t channel, const volatile RelayAlarmRuntimeState *state)
{
    write_float_to_regs(regs, REG_RELAY_ALARM_RUNTIME_ALARM_VALUE(channel), state->alarm_value);
    regs[REG_RELAY_ALARM_RUNTIME_HH_ALARM(channel)] = (uint16_t)(state->HH_alarm & 0xFFFFU);
    regs[REG_RELAY_ALARM_RUNTIME_H_ALARM(channel)] = (uint16_t)(state->H_alarm & 0xFFFFU);
    regs[REG_RELAY_ALARM_RUNTIME_HH_H_ALARM(channel)] = (uint16_t)(state->HH_H_alarm & 0xFFFFU);
    regs[REG_RELAY_ALARM_RUNTIME_L_ALARM(channel)] = (uint16_t)(state->L_alarm & 0xFFFFU);
    regs[REG_RELAY_ALARM_RUNTIME_LL_ALARM(channel)] = (uint16_t)(state->LL_alarm & 0xFFFFU);
    regs[REG_RELAY_ALARM_RUNTIME_LL_L_ALARM(channel)] = (uint16_t)(state->LL_L_alarm & 0xFFFFU);
    regs[REG_RELAY_ALARM_RUNTIME_ANY_ERROR(channel)] = (uint16_t)(state->any_error & 0xFFFFU);
    regs[REG_RELAY_ALARM_RUNTIME_CLEAR_ALARM(channel)] = (uint16_t)(state->clear_alarm & 0xFFFFU);
}


/* ===================== 参数结构体 <-> 保持寄存器映射 ===================== */

/**
 * @brief 将 g_deviceParams 写入保持寄存器数组。
 *
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t。
 *
 * @param HoldingRegisterArray 外部保持寄存器缓存区, 元素类型为 uint16_t。
 */
void WriteDeviceParamsToHoldingRegisters(uint16_t *HoldingRegisterArray)
{
    if (HoldingRegisterArray == NULL) {
        return;
    }

    /* 指令 */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_COMMAND, (uint32_t)g_deviceParams.command);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_PROTOCOL_CAPABILITIES,
                      LTD_CAPABILITY_SUPPORTED_MASK);

    /* ===================== 基础参数 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SENSORTYPE,            g_deviceParams.sensorType);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SENSORID,              g_deviceParams.sensorID);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION,g_deviceParams.sensorSoftwareVersion);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION,       g_deviceParams.softwareVersion);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND,(uint32_t)g_deviceParams.powerOnDefaultCommand);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ERROR_AUTO_BACK_ZERO,  g_deviceParams.error_auto_back_zero);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ERROR_STOP_MEASUREMENT,g_deviceParams.error_stop_measurement);

    /* 协议版本是CPU2发布给CPU3的本机能力常量，不接受外部值覆盖。 */
    g_deviceParams.protocolVersion = DEVICE_PROTOCOL_VERSION;
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION, g_deviceParams.protocolVersion);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT, g_deviceParams.fault_auto_recovery_retry_limit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME, g_deviceParams.water_level_hysteresis_time_s);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH, g_deviceParams.position_source_auto_switch);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MOTOR_CURRENT, g_deviceParams.motor_current);

    /* ===================== 电机与编码器参数 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, g_deviceParams.encoder_wheel_circumference_mm);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED,                g_deviceParams.max_motor_speed);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM,    g_deviceParams.first_loop_circumference_mm);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM,              g_deviceParams.tape_thickness_mm);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_POSITION_COUNT_MODE, g_deviceParams.position_count_mode);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC, g_deviceParams.motor_count_first_loop_circumference_mm);

    /* ===================== 扭力参数 ===================== */
    write_i32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT,             g_deviceParams.empty_weight);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT, g_deviceParams.empty_weight_upper_limit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT, g_deviceParams.empty_weight_lower_limit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT,              g_deviceParams.full_weight);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT,  g_deviceParams.full_weight_upper_limit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT,  g_deviceParams.full_weight_lower_limit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO, g_deviceParams.weight_upper_limit_ratio);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO, g_deviceParams.weight_lower_limit_ratio);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED8, g_deviceParams.reserved8);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED9, g_deviceParams.reserved9);

    /* ===================== 零点测量 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO, g_deviceParams.zero_weight_threshold_ratio);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WEIGHT_IGNORE_ZONE,          g_deviceParams.weight_ignore_zone);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE, g_deviceParams.max_zero_deviation_distance);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE,      g_deviceParams.findZeroDownDistance);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED10, g_deviceParams.reserved10);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED11, g_deviceParams.reserved11);

    /* ===================== 液位测量 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TANKHEIGHT,                  g_deviceParams.tankHeight);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF, g_deviceParams.liquid_sensor_distance_diff);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_BLINDZONE,                   g_deviceParams.blindZone);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD,           g_deviceParams.oilLevelThreshold);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD,g_deviceParams.oilLevelHysteresisThreshold);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD,g_deviceParams.liquidLevelMeasurementMethod);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_OILLEVEL_FREQUENCY, g_deviceParams.oilLevelFrequency);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_OILLEVEL_DENSITY, g_deviceParams.oilLevelDensity);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME,    g_deviceParams.oilLevelHysteresisTime);

    /* ===================== 水位测量参数 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_TANK_HEIGHT,                g_deviceParams.water_tank_height);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_MODE,                 g_deviceParams.water_level_mode);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE,                  g_deviceParams.waterBlindZone);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_CAP_THRESHOLD,              g_deviceParams.water_cap_threshold);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD,             g_deviceParams.water_find_cap_threshold);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE,                  g_deviceParams.maxDownDistance);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ZERO_CAP,                         g_deviceParams.zero_cap);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_STABLE_THRESHOLD, g_deviceParams.water_stable_threshold);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_CORRECTION,      g_deviceParams.calibrateWaterLevel);

    /* ===================== 罐高/罐底测量 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_BOTTOM_DETECT_MODE,     g_deviceParams.bottom_detect_mode);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD, g_deviceParams.bottom_angle_threshold);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD,g_deviceParams.bottom_weight_threshold);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG,  g_deviceParams.refreshTankHeightFlag);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION, g_deviceParams.maxTankHeightDeviation);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_INITIAL_TANKHEIGHT,       g_deviceParams.initialTankHeight);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CURRENT_TANKHEIGHT,       g_deviceParams.currentTankHeight);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE, g_deviceParams.bottom_encoder_correction_enable);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD, g_deviceParams.water_lag_cap_threshold);

    /* ===================== 密度和温度修正参数 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION,     g_deviceParams.densityCorrection);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION, g_deviceParams.temperatureCorrection);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED18, g_deviceParams.reserved18);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED19, g_deviceParams.reserved19);

    /* ===================== 分布/区间测量参数 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT,    g_deviceParams.requireBottomMeasurement);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT,     g_deviceParams.requireWaterMeasurement);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY,   g_deviceParams.requireSinglePointDensity);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER,      g_deviceParams.spreadMeasurementOrder);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE,       g_deviceParams.spreadMeasurementMode);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT,      g_deviceParams.spreadMeasurementCount);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,   g_deviceParams.spreadMeasurementDistance);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT,              g_deviceParams.spreadTopLimit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT,           g_deviceParams.spreadBottomLimit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,     g_deviceParams.spreadPointHoverTime);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_INTERVAL_TOPLIMIT,         g_deviceParams.intervalMeasurementTopLimit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_INTERVAL_BOTTOMLIMIT,      g_deviceParams.intervalMeasurementBottomLimit);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED20, g_deviceParams.reserved20);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED21, g_deviceParams.reserved21);

    /* ===================== Wartsila 密度区间测量参数 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT,       g_deviceParams.wartsila_upper_density_limit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT,       g_deviceParams.wartsila_lower_density_limit);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL,          g_deviceParams.wartsila_density_interval);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE,  g_deviceParams.wartsila_max_height_above_surface);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL, g_deviceParams.wartsila_bottom_detect_interval);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT, g_deviceParams.bottom_encoder_correction_tank_height);


    /* ===================== 4-20mA 输出 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE, g_deviceParams.ao_output.work_mode);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_CURRENT_MODE, g_deviceParams.ao_output.current_mode);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_SOURCE, g_deviceParams.ao_output.output_source);
    write_i32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X100, g_deviceParams.ao_output.current_correction_mA_x100);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100, g_deviceParams.ao_output.fixed_current_mA_x100);
    write_i32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_RANGE_0_01MM, g_deviceParams.ao_output.range_0_01mm);
    write_i32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_RANGE_100_01MM, g_deviceParams.ao_output.range_100_01mm);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_DAMPING_X10_S, g_deviceParams.ao_output.damping_x10_s);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_FAULT_MODE, g_deviceParams.ao_output.fault_mode);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100, g_deviceParams.ao_output.fault_current_mA_x100);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_ERROR_LEVEL, g_deviceParams.ao_output.error_level);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100, g_deviceParams.ao_output.power_on_current_mA_x100);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100, g_deviceParams.ao_output.simulation_current_mA_x100);

    /* ===================== 指令参数 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL,    g_deviceParams.calibrateOilLevel);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL,  g_deviceParams.calibrateWaterLevel);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CALIBRATE_TANK_HEIGHT,  g_deviceParams.calibrateTankHeight);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION,       g_deviceParams.singlePointMeasurementPosition);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION,    g_deviceParams.singlePointMonitoringPosition);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL, g_deviceParams.densityDistributionOilLevel);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE, g_deviceParams.motorCommandDistance);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED28, g_deviceParams.reserved28);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED29, g_deviceParams.reserved29);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL,   g_deviceParams.lastOilCorrectionLevel);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE,  g_deviceParams.tankGasPhaseTemperature);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT,  g_deviceParams.tapeExpansionCoefficient);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE,g_deviceParams.tapeCalibrationTemperature);

    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_FIRST_POINT, g_deviceParams.si_profile_first_point);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_INCREMENT, g_deviceParams.si_profile_increment);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_DWELL_TIME, g_deviceParams.si_profile_dwell_time);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL, g_deviceParams.si_profile_bottom_detect_interval);

    /* ===================== 继电器报警输出配置（四路） ===================== */
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        write_relay_alarm_config_to_regs(HoldingRegisterArray, channel, &g_deviceParams.relayAlarm[channel]);
    }

    /* ===================== 元信息与校验 ===================== */
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_PARAM_VERSION, g_deviceParams.param_version);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE,   g_deviceParams.struct_size);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAGIC,         g_deviceParams.magic);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CRC,           g_deviceParams.crc);
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_AO_SIMULATION_ENABLE,
                      AoOutput_IsSimulationEnabled());
}

/**
 * @brief 从保持寄存器数组读取数据到 g_deviceParams。
 *
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t。
 *
 * @param HoldingRegisterArray 外部保持寄存器缓存区, 元素类型为 uint16_t。
 * @note command 一般由线圈或功能码触发, 这里按照保持寄存器映射也支持读回。
 */
void ReadDeviceParamsFromHoldingRegisters(uint16_t *HoldingRegisterArray)
{
    if (HoldingRegisterArray == NULL) {
        return;
    }

    const uint16_t *regs = (const uint16_t*)HoldingRegisterArray;
    uint32_t tmp32;
    uint32_t previous_motor_current = g_deviceParams.motor_current;

    /* 指令 */
    tmp32 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_COMMAND);
    g_deviceParams.command = (CommandType)tmp32;

    /* ===================== 基础参数 ===================== */
    g_deviceParams.sensorType            = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSORTYPE);
    g_deviceParams.sensorID              = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSORID);
    g_deviceParams.sensorSoftwareVersion = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION);
    g_deviceParams.softwareVersion       = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION);

    tmp32 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND);
    g_deviceParams.powerOnDefaultCommand = (CommandType)tmp32;

    g_deviceParams.error_auto_back_zero   = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ERROR_AUTO_BACK_ZERO);
    g_deviceParams.error_stop_measurement = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ERROR_STOP_MEASUREMENT);

    /* 协议版本由CPU2固件决定；即使寄存器缓存被写错，也立即恢复为当前协议。 */
    g_deviceParams.protocolVersion = DEVICE_PROTOCOL_VERSION;
    g_deviceParams.fault_auto_recovery_retry_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT);
    g_deviceParams.water_level_hysteresis_time_s = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME);
    g_deviceParams.position_source_auto_switch = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH);
    g_deviceParams.motor_current = normalize_motor_current(read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MOTOR_CURRENT));
    write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MOTOR_CURRENT, g_deviceParams.motor_current);

    /* ===================== 电机与编码器参数 ===================== */
    g_deviceParams.encoder_wheel_circumference_mm = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM);
    g_deviceParams.max_motor_speed                = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED);
    g_deviceParams.first_loop_circumference_mm    = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM);
    g_deviceParams.tape_thickness_mm              = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM);

    g_deviceParams.position_count_mode = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_POSITION_COUNT_MODE);
    g_deviceParams.motor_count_first_loop_circumference_mm = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MOTOR_COUNT_FIRST_LOOP_CIRC);

    /* ===================== 扭力参数 ===================== */
    g_deviceParams.empty_weight             = read_i32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT);
    g_deviceParams.empty_weight_upper_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT);
    g_deviceParams.empty_weight_lower_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT);
    g_deviceParams.full_weight              = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT);
    g_deviceParams.full_weight_upper_limit  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT);
    g_deviceParams.full_weight_lower_limit  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT);
    g_deviceParams.weight_upper_limit_ratio = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO);
    g_deviceParams.weight_lower_limit_ratio = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO);

    g_deviceParams.reserved8 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED8);
    g_deviceParams.reserved9 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED9);

    /* ===================== 零点测量 ===================== */
    g_deviceParams.zero_weight_threshold_ratio = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ZERO_WEIGHT_THRESHOLD_RATIO);
    g_deviceParams.weight_ignore_zone          = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WEIGHT_IGNORE_ZONE);
    g_deviceParams.max_zero_deviation_distance = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAX_ZERO_DEVIATION_DISTANCE);
    g_deviceParams.findZeroDownDistance        = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE);

    g_deviceParams.reserved10 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED10);
    g_deviceParams.reserved11 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED11);

    /* ===================== 液位测量 ===================== */
    g_deviceParams.tankHeight                  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TANKHEIGHT);
    g_deviceParams.liquid_sensor_distance_diff = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF);
    g_deviceParams.blindZone                   = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_BLINDZONE);
    g_deviceParams.oilLevelThreshold           = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD);
    g_deviceParams.oilLevelHysteresisThreshold = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD);
    g_deviceParams.liquidLevelMeasurementMethod= read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD);

    g_deviceParams.oilLevelFrequency = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_OILLEVEL_FREQUENCY);
    g_deviceParams.oilLevelDensity = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_OILLEVEL_DENSITY);

    g_deviceParams.oilLevelHysteresisTime  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_TIME);
    /* ===================== 水位测量参数 ===================== */
    g_deviceParams.water_tank_height                = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_TANK_HEIGHT);
    g_deviceParams.water_level_mode                 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_MODE);
    g_deviceParams.waterBlindZone                   = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE);
    g_deviceParams.water_cap_threshold              = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_CAP_THRESHOLD);
    g_deviceParams.water_find_cap_threshold             = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD);
    g_deviceParams.maxDownDistance                  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE);
    g_deviceParams.zero_cap                         = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ZERO_CAP);

    g_deviceParams.water_stable_threshold = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_STABLE_THRESHOLD);

    g_deviceParams.calibrateWaterLevel     = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_CORRECTION);
    /* ===================== 罐高/罐底测量 ===================== */
    g_deviceParams.bottom_detect_mode      = (read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_BOTTOM_DETECT_MODE) == 0U) ? 0U : 1U;
    g_deviceParams.bottom_angle_threshold  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD);
    g_deviceParams.bottom_weight_threshold = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD);
    g_deviceParams.refreshTankHeightFlag  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG);
    g_deviceParams.maxTankHeightDeviation = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAX_TANKHEIGHT_DEVIATION);
    g_deviceParams.initialTankHeight      = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_INITIAL_TANKHEIGHT);
    g_deviceParams.currentTankHeight      = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CURRENT_TANKHEIGHT);

    g_deviceParams.bottom_encoder_correction_enable = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE);
    g_deviceParams.water_lag_cap_threshold = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD);

    /* ===================== 密度和温度修正参数 ===================== */
    g_deviceParams.densityCorrection     = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION);
    g_deviceParams.temperatureCorrection = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION);

    g_deviceParams.reserved18 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED18);
    g_deviceParams.reserved19 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED19);

    /* ===================== 分布/区间测量参数 ===================== */
    g_deviceParams.requireBottomMeasurement       = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT);
    g_deviceParams.requireWaterMeasurement        = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT);
    g_deviceParams.requireSinglePointDensity      = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY);
    g_deviceParams.spreadMeasurementOrder         = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER);
    g_deviceParams.spreadMeasurementMode          = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE);
    g_deviceParams.spreadMeasurementCount         = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT);
    g_deviceParams.spreadMeasurementDistance      = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE);
    g_deviceParams.spreadTopLimit                 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT);
    g_deviceParams.spreadBottomLimit              = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT);
    g_deviceParams.spreadPointHoverTime           = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME);
    g_deviceParams.intervalMeasurementTopLimit    = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_INTERVAL_TOPLIMIT);
    g_deviceParams.intervalMeasurementBottomLimit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_INTERVAL_BOTTOMLIMIT);

    g_deviceParams.reserved20 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED20);
    g_deviceParams.reserved21 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED21);

    /* ===================== Wartsila 密度区间测量参数 ===================== */
    g_deviceParams.wartsila_upper_density_limit      = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT);
    g_deviceParams.wartsila_lower_density_limit      = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT);
    g_deviceParams.wartsila_density_interval         = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL);
    g_deviceParams.wartsila_max_height_above_surface = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE);

    g_deviceParams.wartsila_bottom_detect_interval = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_BOTTOM_DETECT_INTERVAL);
    g_deviceParams.bottom_encoder_correction_tank_height = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT);


    /* ===================== 4-20mA 输出 ===================== */
    g_deviceParams.ao_output.work_mode = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE);
    g_deviceParams.ao_output.current_mode = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_CURRENT_MODE);
    g_deviceParams.ao_output.output_source = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_SOURCE);
    g_deviceParams.ao_output.current_correction_mA_x100 = read_i32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X100);
    g_deviceParams.ao_output.fixed_current_mA_x100 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100);
    g_deviceParams.ao_output.range_0_01mm = read_i32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_RANGE_0_01MM);
    g_deviceParams.ao_output.range_100_01mm = read_i32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_RANGE_100_01MM);
    g_deviceParams.ao_output.damping_x10_s = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_DAMPING_X10_S);
    g_deviceParams.ao_output.fault_mode = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_FAULT_MODE);
    g_deviceParams.ao_output.fault_current_mA_x100 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100);
    g_deviceParams.ao_output.error_level = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_ERROR_LEVEL);
    g_deviceParams.ao_output.power_on_current_mA_x100 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100);
    g_deviceParams.ao_output.simulation_current_mA_x100 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_SIMULATION_CURRENT_MA_X100);
    AoOutput_SetSimulationEnabled(
        (read_u32_from_regs(regs, HOLDREGISTER_AO_SIMULATION_ENABLE) == 0U) ? 0U : 1U);

    /* ===================== 指令参数 ===================== */
    g_deviceParams.calibrateOilLevel            = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL);
    g_deviceParams.calibrateWaterLevel          = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL);
    g_deviceParams.calibrateTankHeight          = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CALIBRATE_TANK_HEIGHT);
    g_deviceParams.singlePointMeasurementPosition = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION);
    g_deviceParams.singlePointMonitoringPosition  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION);
    g_deviceParams.densityDistributionOilLevel    = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL);
    g_deviceParams.motorCommandDistance           = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE);

    g_deviceParams.reserved28 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED28);
    g_deviceParams.reserved29 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED29);

    g_deviceParams.lastOilCorrectionLevel  = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_LAST_OIL_CORRECTION_LEVEL);
    g_deviceParams.tankGasPhaseTemperature = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TANK_GAS_PHASE_TEMPERATURE);
    g_deviceParams.tapeExpansionCoefficient= read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT);
    g_deviceParams.tapeCalibrationTemperature = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TAPE_CALIBRATION_TEMPERATURE);

    g_deviceParams.si_profile_first_point = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_FIRST_POINT);
    g_deviceParams.si_profile_increment = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_INCREMENT);
    g_deviceParams.si_profile_dwell_time = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_DWELL_TIME);
    g_deviceParams.si_profile_bottom_detect_interval = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL);

    /* ===================== 继电器报警输出配置（四路） ===================== */
    for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
        read_relay_alarm_config_from_regs(regs, channel, &g_deviceParams.relayAlarm[channel]);
    }

    /* ===================== 元信息与校验 ===================== */
    g_deviceParams.param_version = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_PARAM_VERSION);
    g_deviceParams.struct_size   = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE);
    g_deviceParams.magic         = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAGIC);
    g_deviceParams.crc           = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CRC);

    if (g_deviceParams.motor_current != previous_motor_current) {
        (void)MotorCtrl_SetCurrent(g_deviceParams.motor_current);
    }

    MotorCtrl_ApplyPositionSourceParams();

    /* todo: */
}

/* ===================== MeasurementResult <-> 输入寄存器映射 ===================== */

/**
 * @brief 将全局测量结果写入输入寄存器数组。
 *
 * 函数先清零完整输入寄存器镜像，避免本轮未覆盖字段保留旧值，再按 CPU2/CPU3 共享地址表编码设备状态、调试数据、液位、水位、罐高和协议辅助状态。
 * 单点测量、单点监测和密度分布平均值逐字段写入；同质的密度点阵统一循环写入 MAX_MEASUREMENT_POINTS 个点，每点包含六个 32 位测量字段。
 * 后续固定块包含无线配对与连接状态、四路继电器报警运行态、AO 运行快照、SI Profile 生命周期、固定点代际计数器以及维护和继电器状态槽。
 * 所有 32 位无符号量、有符号量和 float 都通过共享编码函数写入，保持高低字和原始位模式一致；电机状态只使用主循环预先刷新的缓存，不在可能由 UART5 请求触发的路径中访问 TMC5130。
 * 固定点数据字段先写入，完成或采样代际计数器最后写入，供 CPU3 在读取前后执行一致性校验。
 *
 * @param regs 输入寄存器数组（uint16_t 数组）；用于接收 CPU2 当前 g_measurement 的编码结果，容量至少为 INPUTREGISTER_AMOUNT 个元素。
 * @note 调用方必须提供至少 INPUTREGISTER_AMOUNT 个 uint16_t 元素；函数会清零并重写整个镜像，不保留调用方原内容。
 */
void write_measurement_result_to_InputRegisters(uint16_t *regs) {
	if (regs == NULL) {
		return;
	}

	/* 先清零，避免未写字段残留旧值 */
	memset(regs, 0, INPUTREGISTER_AMOUNT * sizeof(uint16_t));

	/* ==== DeviceStatus ==== */
	write_u32_to_regs(regs, REG_DEVICE_STATUS_WORK_MODE, g_measurement.device_status.work_mode);
	write_u32_to_regs(regs, REG_DEVICE_STATUS_DEVICE_STATE, (uint32_t) g_measurement.device_status.device_state);
	write_u32_to_regs(regs, REG_DEVICE_STATUS_ERROR_CODE, g_measurement.device_status.error_code);
	write_u32_to_regs(regs, REG_DEVICE_STATUS_CURRENT_COMMAND, (uint32_t) g_measurement.device_status.current_command);

	/* zero_point_status 在地址宏里已经按 REG_SIZE_U32 计算，必须用 u32 写 */
	write_u32_to_regs(regs, REG_DEVICE_STATUS_ZERO_POINT_STATUS, (uint32_t) g_measurement.device_status.zero_point_status);
	write_u32_to_regs(regs, REG_DEVICE_STATUS_PARAM_UPDATE_FLAG, g_measurement.device_status.parameter_update_flag);

	/* ==== DebugData ==== */
	write_i32_to_regs(regs, REG_DEBUG_CURRENT_ENCODER, g_measurement.debug_data.current_encoder_value);
	write_i32_to_regs(regs, REG_DEBUG_SENSOR_POSITION, g_measurement.debug_data.sensor_position);
	write_i32_to_regs(regs, REG_DEBUG_CABLE_LENGTH, g_measurement.debug_data.cable_length);
	write_i32_to_regs(regs, REG_DEBUG_MOTOR_STEP,     g_measurement.debug_data.motor_step);
	write_i32_to_regs(regs, REG_DEBUG_MOTOR_DISTANCE, g_measurement.debug_data.motor_distance);

	write_u32_to_regs(regs, REG_DEBUG_FREQUENCY, g_measurement.debug_data.frequency);
	write_u32_to_regs(regs, REG_DEBUG_TEMPERATURE, g_measurement.debug_data.temperature);
	write_u32_to_regs(regs, REG_DEBUG_AIR_FREQUENCY, g_measurement.debug_data.air_frequency);
	write_u32_to_regs(regs, REG_DEBUG_CURRENT_AMPLITUDE, g_measurement.debug_data.current_amplitude);
	write_u32_to_regs(regs, REG_DEBUG_WATER_CAPACITANCE_X10, g_measurement.debug_data.water_capacitance_x10);

	/* 扭力相关 */
	write_u32_to_regs(regs, REG_DEBUG_CURRENT_WEIGHT, g_measurement.debug_data.current_weight);
	write_u32_to_regs(regs,
	                  REG_DEBUG_TORQUE_TEMPERATURE_BITS,
	                  g_measurement.debug_data.torque_temperature_bits);

	/* 姿态角 */
	write_i32_to_regs(regs, REG_DEBUG_ANGLE_X, g_measurement.debug_data.angle_x);
	write_i32_to_regs(regs, REG_DEBUG_ANGLE_Y, g_measurement.debug_data.angle_y);

	/* 电机状态相关 */
	write_u32_to_regs(regs, REG_DEBUG_MOTOR_SPEED, g_measurement.debug_data.motor_speed);
	/* Modbus读输入寄存器可能发生在UART5中断上下文，不能在这里实时读取TMC5130。
	 * 电机状态由主循环MotorCtrl_PollRuntimePosition()统一刷新，这里只上报缓存值。 */
	write_u32_to_regs(regs, REG_DEBUG_MOTOR_STATE, g_measurement.debug_data.motor_state);

	/* ==== OilMeasurement ==== */
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_OIL_LEVEL, g_measurement.oil_measurement.oil_level);
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_AIR_FREQUENCY, g_measurement.oil_measurement.air_frequency);
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_OIL_FREQUENCY, g_measurement.oil_measurement.oil_frequency);
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_FOLLOW_FREQUENCY, g_measurement.oil_measurement.follow_frequency);
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_CURRENT_FREQUENCY, g_measurement.oil_measurement.current_frequency);

	/* ==== WaterMeasurement ==== */
	write_u32_to_regs(regs, REG_WATER_MEASUREMENT_WATER_LEVEL, g_measurement.water_measurement.water_level);
	write_float_to_regs(regs, REG_WATER_MEASUREMENT_ZERO_CAPACITANCE, g_measurement.water_measurement.zero_capacitance);
	write_float_to_regs(regs, REG_WATER_MEASUREMENT_OIL_CAPACITANCE, g_measurement.water_measurement.oil_capacitance);
	write_float_to_regs(regs, REG_WATER_MEASUREMENT_CURRENT_CAPACITANCE, g_measurement.water_measurement.current_capacitance);

	/* ==== ActualHeightMeasurement ==== */
	write_u32_to_regs(regs, REG_HEIGHT_MEASUREMENT_CAL_LIQUID_LEVEL, g_measurement.height_measurement.calibrated_liquid_level);
	write_u32_to_regs(regs, REG_HEIGHT_MEASUREMENT_CURRENT_REAL, g_measurement.height_measurement.current_real_height);

	/* ==== SI shared status ====
	 * CPU2 将协议辅助状态写入共享输入寄存器，CPU3 再把它翻译成 SI 状态位。
	 * 顺序必须和 CPU3 侧保持一致，协议契约脚本会检查这段顺序。
	 */
	write_u32_to_regs(regs, REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID, g_measurement.height_measurement.bottom_reference_valid);
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL, g_measurement.oil_measurement.probe_at_liquid_level);
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_LIQUID_STABLE, g_measurement.oil_measurement.liquid_stable);
	write_u32_to_regs(regs, REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED, g_measurement.density_distribution.profile_complete_latched);
	write_u32_to_regs(regs, REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER, g_measurement.density_distribution.profile_complete_counter);
	write_u32_to_regs(regs, REG_DENSITY_DIST_PROFILE_SOURCE, g_measurement.density_distribution.profile_source);
	write_u32_to_regs(regs, REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS, g_measurement.density_distribution.profile_blocked_by_process);
	write_u32_to_regs(regs, REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE, g_measurement.device_status.loading_unloading_active);
	write_u32_to_regs(regs, REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT, g_measurement.device_status.manual_alarm_inhibit);
	write_u32_to_regs(regs, REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT, g_measurement.oil_measurement.manual_level_update_inhibit);
	write_u32_to_regs(regs, REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM, g_measurement.density_distribution.profile_temp_deviation_alarm);
	write_u32_to_regs(regs, REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM, g_measurement.density_distribution.profile_density_deviation_alarm);

	/* ==== Single Point Measurement ==== */
	write_u32_to_regs(regs, REG_SINGLE_POINT_MEAS_TEMP, g_measurement.single_point_measurement.temperature);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MEAS_DENSITY, g_measurement.single_point_measurement.density);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MEAS_TEMP_POS, g_measurement.single_point_measurement.temperature_position);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MEAS_STD_DENSITY, g_measurement.single_point_measurement.standard_density);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MEAS_VCF20, g_measurement.single_point_measurement.vcf20);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MEAS_WEIGHT_DENSITY, g_measurement.single_point_measurement.weight_density);

	/* ==== Single Point Monitoring ==== */
	write_u32_to_regs(regs, REG_SINGLE_POINT_MON_TEMP, g_measurement.single_point_monitoring.temperature);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MON_DENSITY, g_measurement.single_point_monitoring.density);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MON_TEMP_POS, g_measurement.single_point_monitoring.temperature_position);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MON_STD_DENSITY, g_measurement.single_point_monitoring.standard_density);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MON_VCF20, g_measurement.single_point_monitoring.vcf20);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MON_WEIGHT_DENSITY, g_measurement.single_point_monitoring.weight_density);

	/* ==== Density Distribution ==== */
	write_u32_to_regs(regs, REG_DENSITY_DIST_AVG_TEMP, g_measurement.density_distribution.average_temperature);
	write_u32_to_regs(regs, REG_DENSITY_DIST_AVG_DENSITY, g_measurement.density_distribution.average_density);
	write_u32_to_regs(regs, REG_DENSITY_DIST_AVG_STD_DENSITY, g_measurement.density_distribution.average_standard_density);
	write_u32_to_regs(regs, REG_DENSITY_DIST_AVG_VCF20, g_measurement.density_distribution.average_vcf20);
	write_u32_to_regs(regs, REG_DENSITY_DIST_AVG_WEIGHT_DENSITY, g_measurement.density_distribution.average_weight_density);
	write_u32_to_regs(regs, REG_DENSITY_DIST_MEAS_POINTS, g_measurement.density_distribution.measurement_points);
	write_u32_to_regs(regs, REG_DENSITY_DIST_OIL_LEVEL, g_measurement.density_distribution.Density_oil_level);

	/* ==== Density Distribution Points ==== */
	for (int i = 0; i < MAX_MEASUREMENT_POINTS; i++) {
		const volatile DensityMeasurement *p = &g_measurement.density_distribution.single_density_data[i];

		write_u32_to_regs(regs, REG_DENSITY_POINT_TEMP(i), p->temperature);
		write_u32_to_regs(regs, REG_DENSITY_POINT_DENSITY(i), p->density);
		write_u32_to_regs(regs, REG_DENSITY_POINT_TEMP_POS(i), p->temperature_position);
		write_u32_to_regs(regs, REG_DENSITY_POINT_STD_DENSITY(i), p->standard_density);
		write_u32_to_regs(regs, REG_DENSITY_POINT_VCF20(i), p->vcf20);
		write_u32_to_regs(regs, REG_DENSITY_POINT_WEIGHT_DENSITY(i), p->weight_density);
	}

	/* ==== 无线滑环匹配状态 ==== */
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_RESULT, g_measurement.wireless_pairing_status.result);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_MAC_VALID, g_measurement.wireless_pairing_status.mac_valid);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_MAC_HIGH, g_measurement.wireless_pairing_status.mac_high);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_MAC_MID, g_measurement.wireless_pairing_status.mac_mid);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_MAC_LOW, g_measurement.wireless_pairing_status.mac_low);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_ERROR_CODE, g_measurement.wireless_pairing_status.error_code);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_UPDATE_COUNTER, g_measurement.wireless_pairing_status.update_counter);

	/* ==== 继电器报警输出运行态 ==== */
	for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
		write_relay_alarm_runtime_to_regs(regs, channel, &g_measurement.relay_alarm_runtime[channel]);
	}

	/* ==== 无线运行状态固定块：0x1600 ==== */
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_CONNECTION_VALID, g_measurement.wireless_pairing_status.connection_valid);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_RSSI_VALID, g_measurement.wireless_pairing_status.rssi_valid);
	write_i32_to_regs(regs, REG_WIRELESS_PAIRING_RSSI, g_measurement.wireless_pairing_status.rssi);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_CONNECTION_ERROR_CODE, g_measurement.wireless_pairing_status.connection_error_code);
	write_u32_to_regs(regs, REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER, g_measurement.wireless_pairing_status.rssi_update_counter);

	/* ==== AO运行状态固定块：0x1400 ==== */
	AoOutputRuntime ao_runtime;
	AoOutput_GetRuntimeSnapshot(&ao_runtime);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_TARGET_MA_X100, ao_runtime.target_mA_x100);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_SENT_MA_X100, ao_runtime.last_sent_mA_x100);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_SOURCE, ao_runtime.source);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_FLAGS, ao_runtime.driver_fault_flags);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_REGISTER, ao_runtime.driver_fault_register);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_ERROR_CODE, ao_runtime.last_error_code);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_UPDATE_COUNTER, ao_runtime.update_counter);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_UPDATE_TICK, ao_runtime.last_update_tick);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_SENT_TICK, ao_runtime.last_sent_tick);

	/* 固定结果块中的SI Profile生命周期：0x115A~0x115F。 */
	write_u32_to_regs(regs, REG_DENSITY_DIST_SI_PROFILE_PHASE, g_measurement.si_profile_runtime.phase);
	write_u32_to_regs(regs, REG_DENSITY_DIST_SI_PROFILE_CYCLE_COUNTER, g_measurement.si_profile_runtime.cycle_counter);
	write_u32_to_regs(regs, REG_DENSITY_DIST_SI_PROFILE_PROGRESS_POINTS, g_measurement.si_profile_runtime.progress_points);

	/* AO扩展运行态与基础运行态统一位于0x1400固定块。 */
	write_i32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_PROCESS_VALUE_01MM, ao_runtime.process_value_01mm);
	write_i32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_PERCENT_X100, ao_runtime.percent_x100);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_PROCESS_VALID, ao_runtime.process_valid);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_SIMULATION_ENABLED, ao_runtime.simulation_enabled);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_DAC_READBACK_MA_X100, ao_runtime.dac_readback_mA_x100);
	write_u32_to_regs(regs, REG_AO_OUTPUT_RUNTIME_DAC_READBACK_VALID, ao_runtime.dac_readback_valid);

	/* 固定点六字段先发布，代际计数器最后映射，供CPU3执行前后双读一致性校验。 */
	write_u32_to_regs(regs, REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER, g_measurement.measurement_complete_counter);
	write_u32_to_regs(regs, REG_SINGLE_POINT_MON_SAMPLE_COUNTER, g_measurement.monitoring_sample_counter);

	/* 维护与继电器状态槽随同一CPU2快照发布；当前预留值保持0。 */
	write_u32_to_regs(regs, REG_MAINTENANCE_MODE_ACTIVE, g_measurement.device_status.maintenance_mode_active);
	write_u32_to_regs(regs, REG_RELAY_ALARM_INHIBIT_EFFECTIVE, g_measurement.device_status.relay_alarm_inhibit_effective);
	write_u32_to_regs(regs, REG_RELAY_ALARM_ACTION_CHANNEL1, (g_measurement.device_status.relay_alarm_action_mask >> 0U) & 1U);
	write_u32_to_regs(regs, REG_RELAY_ALARM_ACTION_CHANNEL2, (g_measurement.device_status.relay_alarm_action_mask >> 1U) & 1U);
	write_u32_to_regs(regs, REG_RELAY_ALARM_ACTION_CHANNEL3, (g_measurement.device_status.relay_alarm_action_mask >> 2U) & 1U);
	write_u32_to_regs(regs, REG_RELAY_ALARM_ACTION_CHANNEL4, (g_measurement.device_status.relay_alarm_action_mask >> 3U) & 1U);
}
