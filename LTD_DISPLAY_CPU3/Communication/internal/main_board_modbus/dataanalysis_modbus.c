#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "dataanalysis_modbus.h"
#include "usart.h"
#include "stateformodbus.h"
#include <ctype.h>
#include <float.h>
#include "system_parameter.h"
#include "cpu2_communicate.h"
#include "cpu3_comm_display_params.h"
#include "cpu3_debug_log.h"
#include "param_float32.h"
#include <math.h>     /* for pow() */

/**
 * @brief 将保持寄存器原始值解析为有符号 32 位参数值。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param rgscnt 参与本次保持寄存器解析的寄存器数量。
 * @return 返回保持寄存器原始值按目标字段语义转换得到的 32 位有符号参数值。
 */
int32_t ywj_hold_analysis_data(int startadd,int rgscnt);

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
 * @brief 从寄存器数组读取 float（按 IEEE754 编码）。
 *
 * @param regs 包含 IEEE 754 单精度位模式的连续 16 位寄存器数组。
 * @param addr 浮点值高 16 位所在的零基寄存器下标；低 16 位位于 addr + 1。
 * @return 返回由 regs[addr] 高 16 位和 regs[addr+1] 低 16 位拼成的 IEEE 754 单精度位模式；函数只重解释位模式，不执行数值缩放。
 */
static inline float read_float_from_regs(const uint16_t *regs, uint16_t addr) {
	uint32_t temp = read_u32_from_regs(regs, addr);
	float value;
	/* 按 IEEE-754 Float32 位模式解释两个寄存器拼成的 32 位值；不做数值强制转换，寄存器高低字序由 read_u32_from_regs 统一处理。 */
	memcpy(&value, &temp, sizeof(float));
	return value;
}



/**
 * @brief 写入单路继电器报警输出配置。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param channel 零基通道号。合法范围为 0～3，用于选择 CPU3 共享参数中的对应继电器配置块。
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
 * @param channel 零基通道号。合法范围为 0～3，用于选择 CPU3 共享参数中的对应继电器配置块。
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
 * @brief 从输入寄存器读取单路继电器报警输出运行态。
 *
 * @param regs 连续寄存器值缓冲区。函数按既定寄存器数量只读 regs[]，并按高低字、字段偏移或协议映射解析业务值。
 * @param channel 零基通道号。合法范围为 0～3，用于选择 CPU3 共享参数中的对应继电器配置块。
 * @param state 单路继电器运行态输出对象；函数从 CPU2 输入寄存器解析并逐字段写入。
 */
static void read_relay_alarm_runtime_from_regs(const uint16_t *regs, uint32_t channel, volatile RelayAlarmRuntimeState *state)
{
    state->alarm_value = read_float_from_regs(regs, REG_RELAY_ALARM_RUNTIME_ALARM_VALUE(channel));
    state->HH_alarm = regs[REG_RELAY_ALARM_RUNTIME_HH_ALARM(channel)] & 0xFFFFU;
    state->H_alarm = regs[REG_RELAY_ALARM_RUNTIME_H_ALARM(channel)] & 0xFFFFU;
    state->HH_H_alarm = regs[REG_RELAY_ALARM_RUNTIME_HH_H_ALARM(channel)] & 0xFFFFU;
    state->L_alarm = regs[REG_RELAY_ALARM_RUNTIME_L_ALARM(channel)] & 0xFFFFU;
    state->LL_alarm = regs[REG_RELAY_ALARM_RUNTIME_LL_ALARM(channel)] & 0xFFFFU;
    state->LL_L_alarm = regs[REG_RELAY_ALARM_RUNTIME_LL_L_ALARM(channel)] & 0xFFFFU;
    state->any_error = regs[REG_RELAY_ALARM_RUNTIME_ANY_ERROR(channel)] & 0xFFFFU;
    state->clear_alarm = regs[REG_RELAY_ALARM_RUNTIME_CLEAR_ALARM(channel)] & 0xFFFFU;
}

/**
 * @brief 从 CPU2 输入寄存器解析 4～20 mA 模拟输出运行快照。
 *
 * @param regs 连续寄存器值缓冲区。函数按既定寄存器数量只读 regs[]，并按高低字、字段偏移或协议映射解析业务值。
 * @param state AO 运行快照输出对象；函数从 CPU2 输入寄存器解析模式、电流、百分比、故障和仿真字段。
 */
static void read_ao_output_runtime_from_regs(const uint16_t *regs, volatile AoOutputRuntime *state)
{
    state->target_mA_x100 = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_TARGET_MA_X100);
    state->last_sent_mA_x100 = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_SENT_MA_X100);
    state->source = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_SOURCE);
    state->driver_fault_flags = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_FLAGS);
    state->driver_fault_register = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_REGISTER);
    state->last_error_code = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_ERROR_CODE);
    state->update_counter = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_UPDATE_COUNTER);
    state->last_update_tick = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_UPDATE_TICK);
    state->last_sent_tick = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_LAST_SENT_TICK);
    state->process_value_01mm = read_i32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_PROCESS_VALUE_01MM);
    state->percent_x100 = read_i32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_PERCENT_X100);
    state->process_valid = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_PROCESS_VALID);
    state->simulation_enabled = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_SIMULATION_ENABLED);
    state->dac_readback_mA_x100 = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_DAC_READBACK_MA_X100);
    state->dac_readback_valid = read_u32_from_regs(regs, REG_AO_OUTPUT_RUNTIME_DAC_READBACK_VALID);
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
                      g_measurement.ao_output_runtime.simulation_enabled);
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

    /* 指令 */
    tmp32 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_COMMAND);
    g_deviceParams.command = (CommandType)tmp32;

    /* ===================== 基础参数 ===================== */
    g_deviceParams.sensorType            = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSORTYPE);
    if (!DeviceParam_IsSensorTypeSupported(g_deviceParams.sensorType)) {
        /* 保留CPU2原始值用于诊断，但明确报警，禁止后续把未知值按LTD或V4解释。 */
        CPU3_LOG_WARNING("CPU2参数",
                         "收到未知传感器类型 值=%lu",
                         (unsigned long)g_deviceParams.sensorType);
    }
    g_deviceParams.sensorID              = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSORID);
    g_deviceParams.sensorSoftwareVersion = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION);
    g_deviceParams.softwareVersion       = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION);

    tmp32 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND);
    g_deviceParams.powerOnDefaultCommand = (CommandType)tmp32;

    g_deviceParams.error_auto_back_zero   = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ERROR_AUTO_BACK_ZERO);
    g_deviceParams.error_stop_measurement = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ERROR_STOP_MEASUREMENT);

    g_deviceParams.protocolVersion = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION);
    g_deviceParams.fault_auto_recovery_retry_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT);
    g_deviceParams.water_level_hysteresis_time_s = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_LEVEL_HYSTERESIS_TIME);
    g_deviceParams.position_source_auto_switch = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH);
    g_deviceParams.motor_current = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MOTOR_CURRENT);

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
    g_measurement.ao_output_runtime.simulation_enabled =
        (read_u32_from_regs(regs, HOLDREGISTER_AO_SIMULATION_ENABLE) == 0U) ? 0U : 1U;

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

/* update_sensor_height_from_encoder(); / / 如果修改了罐高，需要更新传感器高度测量 */
/* save_device_params();/ /保存设备参数 */
}

/* ===================== MeasurementResult <-> 输入寄存器映射 ===================== */


/**
 * @brief 将输入寄存器数组解析回 MeasurementResult 结构体。
 *
 * 函数按 CPU2/CPU3 共享输入寄存器地址表，依次恢复设备状态、调试数据、液位、水位、罐高、单点测量、单点监测和密度分布平均值。
 * 同质的密度分布点阵统一循环解析 MAX_MEASUREMENT_POINTS 个点，每点按温度、密度、位置、标准密度、VCF20 和重量密度六个 32 位字段恢复。
 * 随后恢复无线配对与连接状态、四路继电器报警运行态、AO 固定运行块、SI Profile 生命周期、固定点代际计数器以及维护和继电器状态槽。
 * 无符号量、有符号量和 float 分别使用共享寄存器解码函数，保持 CPU2 写入端的高低字顺序和原始位模式不变。
 *
 * @param regs 输入寄存器数组（uint16_t 数组）；内容是已经通过板间响应校验的 CPU2 快照，至少包含共享地址表要求的完整范围，函数从中恢复 CPU3 的
 *             g_measurement。
 * @note 调用方必须提供已完成长度、CRC、功能码和代际一致性校验的完整输入寄存器快照；本函数只做字段映射，不重新验证帧或缓冲区长度。
 */
void read_measurement_result_from_InputRegisters(uint16_t *regs) {
	if (regs == NULL) {
		return;
	}

	const uint16_t *cregs = (const uint16_t*) regs;

	/* ==== DeviceStatus ==== */
	g_measurement.device_status.work_mode = read_u32_from_regs(cregs, REG_DEVICE_STATUS_WORK_MODE);
	g_measurement.device_status.device_state = (DeviceState) read_u32_from_regs(cregs, REG_DEVICE_STATUS_DEVICE_STATE);
	g_measurement.device_status.error_code = read_u32_from_regs(cregs, REG_DEVICE_STATUS_ERROR_CODE);
	g_measurement.device_status.current_command = (CommandType) read_u32_from_regs(cregs, REG_DEVICE_STATUS_CURRENT_COMMAND);
	/* zero_point_status 映射为 uint32，占 2 个寄存器，直接按 u32 读 */
	g_measurement.device_status.zero_point_status = read_u32_from_regs(cregs, REG_DEVICE_STATUS_ZERO_POINT_STATUS);
	g_measurement.device_status.parameter_update_flag = read_u32_from_regs(cregs, REG_DEVICE_STATUS_PARAM_UPDATE_FLAG);

	/* ==== DebugData ==== */
	g_measurement.debug_data.current_encoder_value = read_i32_from_regs(cregs, REG_DEBUG_CURRENT_ENCODER);
	g_measurement.debug_data.sensor_position = read_i32_from_regs(cregs, REG_DEBUG_SENSOR_POSITION);
	g_measurement.debug_data.cable_length = read_i32_from_regs(cregs, REG_DEBUG_CABLE_LENGTH);
	g_measurement.debug_data.motor_step     = read_i32_from_regs(cregs, REG_DEBUG_MOTOR_STEP);
	g_measurement.debug_data.motor_distance = read_i32_from_regs(cregs, REG_DEBUG_MOTOR_DISTANCE);

	g_measurement.debug_data.frequency = read_u32_from_regs(cregs, REG_DEBUG_FREQUENCY);
	g_measurement.debug_data.temperature = read_u32_from_regs(cregs, REG_DEBUG_TEMPERATURE);
	g_measurement.debug_data.air_frequency = read_u32_from_regs(cregs, REG_DEBUG_AIR_FREQUENCY);
	g_measurement.debug_data.current_amplitude = read_u32_from_regs(cregs, REG_DEBUG_CURRENT_AMPLITUDE);
	g_measurement.debug_data.water_capacitance_x10 = read_u32_from_regs(cregs, REG_DEBUG_WATER_CAPACITANCE_X10);

	/* 扭力相关 */
	g_measurement.debug_data.current_weight = read_u32_from_regs(cregs, REG_DEBUG_CURRENT_WEIGHT);
	g_measurement.debug_data.torque_temperature_bits =
	    read_u32_from_regs(cregs, REG_DEBUG_TORQUE_TEMPERATURE_BITS);

	/* 姿态角 */
	g_measurement.debug_data.angle_x = read_i32_from_regs(cregs, REG_DEBUG_ANGLE_X);
	g_measurement.debug_data.angle_y = read_i32_from_regs(cregs, REG_DEBUG_ANGLE_Y);

	/* 电机状态 */
	g_measurement.debug_data.motor_speed = read_u32_from_regs(cregs, REG_DEBUG_MOTOR_SPEED);
	g_measurement.debug_data.motor_state = read_u32_from_regs(cregs, REG_DEBUG_MOTOR_STATE);

	/* 多参数传感器调试测量结果 */
	g_measurement.debug_data.magnetic_zero_voltage =
	    read_float_from_regs(cregs, REG_DEBUG_MAGNETIC_ZERO_VOLTAGE);
	g_measurement.debug_data.dynamic_viscosity_cp =
	    read_float_from_regs(cregs, REG_DEBUG_DYNAMIC_VISCOSITY_CP);
	g_measurement.debug_data.kinematic_viscosity_cst =
	    read_float_from_regs(cregs, REG_DEBUG_KINEMATIC_VISCOSITY_CST);
	g_measurement.debug_data.supply_voltage_v =
	    read_float_from_regs(cregs, REG_DEBUG_SUPPLY_VOLTAGE_V);

	/* ==== OilMeasurement ==== */
	g_measurement.oil_measurement.oil_level = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_OIL_LEVEL);
	g_measurement.oil_measurement.air_frequency = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_AIR_FREQUENCY);
	g_measurement.oil_measurement.oil_frequency = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_OIL_FREQUENCY);
	g_measurement.oil_measurement.follow_frequency = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_FOLLOW_FREQUENCY);
	g_measurement.oil_measurement.current_frequency = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_CURRENT_FREQUENCY);

	/* ==== WaterMeasurement ==== */
	g_measurement.water_measurement.water_level = read_u32_from_regs(cregs, REG_WATER_MEASUREMENT_WATER_LEVEL);
	g_measurement.water_measurement.zero_capacitance = read_float_from_regs(cregs, REG_WATER_MEASUREMENT_ZERO_CAPACITANCE);
	g_measurement.water_measurement.oil_capacitance = read_float_from_regs(cregs, REG_WATER_MEASUREMENT_OIL_CAPACITANCE);
	g_measurement.water_measurement.current_capacitance = read_float_from_regs(cregs, REG_WATER_MEASUREMENT_CURRENT_CAPACITANCE);

	/* ==== 实高测量 ==== */
	g_measurement.height_measurement.calibrated_liquid_level = read_u32_from_regs(cregs, REG_HEIGHT_MEASUREMENT_CAL_LIQUID_LEVEL);
	g_measurement.height_measurement.current_real_height = read_u32_from_regs(cregs, REG_HEIGHT_MEASUREMENT_CURRENT_REAL);

	/* ==== SI shared status ====
	 * 该段从 CPU2 输入寄存器恢复到 CPU3 本地 g_measurement，供外部协议模块统一读取。
	 */
	g_measurement.height_measurement.bottom_reference_valid = read_u32_from_regs(cregs, REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID);
	g_measurement.oil_measurement.probe_at_liquid_level = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL);
	g_measurement.oil_measurement.liquid_stable = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_LIQUID_STABLE);
	g_measurement.density_distribution.profile_complete_latched = read_u32_from_regs(cregs, REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED);
	g_measurement.density_distribution.profile_complete_counter = read_u32_from_regs(cregs, REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER);
	g_measurement.density_distribution.profile_source = read_u32_from_regs(cregs, REG_DENSITY_DIST_PROFILE_SOURCE);
	g_measurement.density_distribution.profile_blocked_by_process = read_u32_from_regs(cregs, REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS);
	g_measurement.device_status.loading_unloading_active = read_u32_from_regs(cregs, REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE);
	g_measurement.device_status.manual_alarm_inhibit = read_u32_from_regs(cregs, REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT);
	g_measurement.oil_measurement.manual_level_update_inhibit = read_u32_from_regs(cregs, REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT);
	g_measurement.density_distribution.profile_temp_deviation_alarm = read_u32_from_regs(cregs, REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM);
	g_measurement.density_distribution.profile_density_deviation_alarm = read_u32_from_regs(cregs, REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM);

	/* ==== 单点密度测量 ==== */
	g_measurement.single_point_measurement.temperature = read_u32_from_regs(cregs, REG_SINGLE_POINT_MEAS_TEMP);
	g_measurement.single_point_measurement.density = read_u32_from_regs(cregs, REG_SINGLE_POINT_MEAS_DENSITY);
	g_measurement.single_point_measurement.temperature_position = read_u32_from_regs(cregs, REG_SINGLE_POINT_MEAS_TEMP_POS);
	g_measurement.single_point_measurement.standard_density = read_u32_from_regs(cregs, REG_SINGLE_POINT_MEAS_STD_DENSITY);
	g_measurement.single_point_measurement.vcf20 = read_u32_from_regs(cregs, REG_SINGLE_POINT_MEAS_VCF20);
	g_measurement.single_point_measurement.weight_density = read_u32_from_regs(cregs, REG_SINGLE_POINT_MEAS_WEIGHT_DENSITY);

	/* ==== 单点监测 ==== */
	g_measurement.single_point_monitoring.temperature = read_u32_from_regs(cregs, REG_SINGLE_POINT_MON_TEMP);
	g_measurement.single_point_monitoring.density = read_u32_from_regs(cregs, REG_SINGLE_POINT_MON_DENSITY);
	g_measurement.single_point_monitoring.temperature_position = read_u32_from_regs(cregs, REG_SINGLE_POINT_MON_TEMP_POS);
	g_measurement.single_point_monitoring.standard_density = read_u32_from_regs(cregs, REG_SINGLE_POINT_MON_STD_DENSITY);
	g_measurement.single_point_monitoring.vcf20 = read_u32_from_regs(cregs, REG_SINGLE_POINT_MON_VCF20);
	g_measurement.single_point_monitoring.weight_density = read_u32_from_regs(cregs, REG_SINGLE_POINT_MON_WEIGHT_DENSITY);

	/* ==== 密度分布（平均值） ==== */
	g_measurement.density_distribution.average_temperature = read_u32_from_regs(cregs, REG_DENSITY_DIST_AVG_TEMP);
	g_measurement.density_distribution.average_density = read_u32_from_regs(cregs, REG_DENSITY_DIST_AVG_DENSITY);
	g_measurement.density_distribution.average_standard_density = read_u32_from_regs(cregs, REG_DENSITY_DIST_AVG_STD_DENSITY);
	g_measurement.density_distribution.average_vcf20 = read_u32_from_regs(cregs, REG_DENSITY_DIST_AVG_VCF20);
	g_measurement.density_distribution.average_weight_density = read_u32_from_regs(cregs, REG_DENSITY_DIST_AVG_WEIGHT_DENSITY);
	g_measurement.density_distribution.measurement_points = read_u32_from_regs(cregs, REG_DENSITY_DIST_MEAS_POINTS);
	g_measurement.density_distribution.Density_oil_level = read_u32_from_regs(cregs, REG_DENSITY_DIST_OIL_LEVEL);

	/* ==== 密度分布单点数据 ==== */
	for (int i = 0; i < MAX_MEASUREMENT_POINTS; i++) {
		g_measurement.density_distribution.single_density_data[i].temperature = read_u32_from_regs(cregs, REG_DENSITY_POINT_TEMP(i));
		g_measurement.density_distribution.single_density_data[i].density = read_u32_from_regs(cregs, REG_DENSITY_POINT_DENSITY(i));
		g_measurement.density_distribution.single_density_data[i].temperature_position = read_u32_from_regs(cregs, REG_DENSITY_POINT_TEMP_POS(i));
		g_measurement.density_distribution.single_density_data[i].standard_density = read_u32_from_regs(cregs, REG_DENSITY_POINT_STD_DENSITY(i));
		g_measurement.density_distribution.single_density_data[i].vcf20 = read_u32_from_regs(cregs, REG_DENSITY_POINT_VCF20(i));
		g_measurement.density_distribution.single_density_data[i].weight_density = read_u32_from_regs(cregs, REG_DENSITY_POINT_WEIGHT_DENSITY(i));
	}

	/* ==== 无线滑环匹配状态 ==== */
	g_measurement.wireless_pairing_status.result = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_RESULT);
	g_measurement.wireless_pairing_status.mac_valid = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_MAC_VALID);
	g_measurement.wireless_pairing_status.mac_high = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_MAC_HIGH);
	g_measurement.wireless_pairing_status.mac_mid = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_MAC_MID);
	g_measurement.wireless_pairing_status.mac_low = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_MAC_LOW);
	g_measurement.wireless_pairing_status.error_code = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_ERROR_CODE);
	g_measurement.wireless_pairing_status.update_counter = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_UPDATE_COUNTER);

	/* ==== 继电器报警输出运行态 ==== */
	for (uint32_t channel = 0U; channel < RELAY_ALARM_CHANNEL_COUNT; channel++) {
		read_relay_alarm_runtime_from_regs(cregs, channel, &g_measurement.relay_alarm_runtime[channel]);
	}

	/* ==== 无线运行状态固定块：0x1600 ==== */
	g_measurement.wireless_pairing_status.connection_valid = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_CONNECTION_VALID);
	g_measurement.wireless_pairing_status.rssi_valid = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_RSSI_VALID);
	g_measurement.wireless_pairing_status.rssi = read_i32_from_regs(cregs, REG_WIRELESS_PAIRING_RSSI);
	g_measurement.wireless_pairing_status.connection_error_code = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_CONNECTION_ERROR_CODE);
	g_measurement.wireless_pairing_status.rssi_update_counter = read_u32_from_regs(cregs, REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER);

	/* ==== AO运行状态固定块：0x1400 ==== */
	read_ao_output_runtime_from_regs(cregs, &g_measurement.ao_output_runtime);

	/* ==== 固定结果块中的SI Profile生命周期：0x115A~0x115F ==== */
	g_measurement.si_profile_runtime.phase = read_u32_from_regs(cregs, REG_DENSITY_DIST_SI_PROFILE_PHASE);
	g_measurement.si_profile_runtime.cycle_counter = read_u32_from_regs(cregs, REG_DENSITY_DIST_SI_PROFILE_CYCLE_COUNTER);
	g_measurement.si_profile_runtime.progress_points = read_u32_from_regs(cregs, REG_DENSITY_DIST_SI_PROFILE_PROGRESS_POINTS);

	/* 协议21固定点代际只从已通过前后双读校验的公开缓存恢复。 */
	g_measurement.measurement_complete_counter = read_u32_from_regs(cregs, REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER);
	g_measurement.monitoring_sample_counter = read_u32_from_regs(cregs, REG_SINGLE_POINT_MON_SAMPLE_COUNTER);

	/* 维护与继电器状态槽均来自同一CPU2权威输入快照。 */
	g_measurement.device_status.maintenance_mode_active =
		read_u32_from_regs(cregs, REG_MAINTENANCE_MODE_ACTIVE);
	g_measurement.device_status.relay_alarm_inhibit_effective =
		read_u32_from_regs(cregs, REG_RELAY_ALARM_INHIBIT_EFFECTIVE);
	g_measurement.device_status.relay_alarm_action_mask =
		((read_u32_from_regs(cregs, REG_RELAY_ALARM_ACTION_CHANNEL1) & 1U) << 0U) |
		((read_u32_from_regs(cregs, REG_RELAY_ALARM_ACTION_CHANNEL2) & 1U) << 1U) |
		((read_u32_from_regs(cregs, REG_RELAY_ALARM_ACTION_CHANNEL3) & 1U) << 2U) |
		((read_u32_from_regs(cregs, REG_RELAY_ALARM_ACTION_CHANNEL4) & 1U) << 3U);
}

/**
 * @brief 解析03功能码保持寄存器数据。
 */
void AnalysisHoldRegister(void)
{
    int index = 0;

    for(index = 0;index < param_metaAmount;index++)
    {
        /* 关键：CPU3 本机参数不参与 CPU2 HOLD 轮询解析 */
        if (Cpu3Local_IsParam(param_meta[index].operanum)) {
            continue;
        }
        if(param_meta[index].data_type == TYPE_INT)
        {
            param_meta[index].val = ywj_hold_analysis_data(param_meta[index].startadd,param_meta[index].rgstcnt);
            param_meta[index].val += param_meta[index].offset;
/* printf("Hold Reg %s: %d\n",param_meta[index].name,param_meta[index].val); */
        }
        else if(param_meta[index].data_type == TYPE_FLOAT)
        {
            int32_t scaled_value;
            uint32_t raw_value = (uint32_t)ywj_hold_analysis_data(
                param_meta[index].startadd,
                param_meta[index].rgstcnt);

            if (ParamFloat32_TryRawToScaledInt(raw_value,
                                               param_meta[index].point,
                                               &scaled_value)) {
                param_meta[index].val = scaled_value;
            }
        }
        else if(param_meta[index].data_type == TYPE_DOUBLE)
        {
            union utod tmp_d;
            tmp_d.u[1] = ywj_hold_analysis_data(param_meta[index].startadd,param_meta[index].rgstcnt / 2);
            tmp_d.u[0] = ywj_hold_analysis_data(param_meta[index].startadd + 2,param_meta[index].rgstcnt / 2);
            tmp_d.d *= pow(10,param_meta[index].point);
            param_meta[index].val = tmp_d.d;
        }
    }
}

/**
 * @brief 将保持寄存器原始值解析为有符号 32 位参数值。
 *
 * @param startadd 本次 Modbus 访问的起始寄存器地址。
 * @param rgscnt 参与本次保持寄存器解析的寄存器数量。
 * @return 返回保持寄存器原始值按目标字段语义转换得到的 32 位有符号参数值。
 */
int32_t ywj_hold_analysis_data(int startadd,int rgscnt)
{
    uint32_t value = 0U;
    int i;

/* startadd *= 2; */
    /* 解析数据 */
    for(i = 0;i < rgscnt;i++)
    {
        value <<= 16;
        value |= (uint32_t)HoldingRegisterArray[startadd + i];
    }
    return (int32_t)value;
}

