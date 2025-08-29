#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "dataanalysis_modbus.h"
#include "usart.h"
#include "stateformodbus.h"	 
#include <ctype.h>
#include <float.h>
#include "system_parameter.h"
#define WRITE_UINT32_TO_HOLDING(addr, val)                  \
		HoldingRegisterArray[(addr)]     = ((val) >> 16) & 0xFFFF; \
    HoldingRegisterArray[(addr) + 1] = (val) & 0xFFFF;
#define READ_UINT32_FROM_HOLDING(addr, dest)                             \
    dest = (((uint32_t)HoldingRegisterArray[(addr)] << 16) |             \
             (uint32_t)HoldingRegisterArray[(addr) + 1])


void WriteDeviceParamsToHoldingRegisters(int *HoldingRegisterArray) {
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_COMMAND, g_deviceParams.command);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_TANKHEIGHT, g_deviceParams.tankHeight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_BLINDZONE, g_deviceParams.blindZone);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, g_deviceParams.encoder_wheel_circumference_mm);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SENSORTYPE, g_deviceParams.sensorType);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION, g_deviceParams.softwareVersion);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT, g_deviceParams.empty_weight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION, g_deviceParams.densityCorrection);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION, g_deviceParams.temperatureCorrection);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, g_deviceParams.requireBottomMeasurement);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT, g_deviceParams.requireWaterMeasurement);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, g_deviceParams.requireSinglePointDensity);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER, g_deviceParams.spreadMeasurementOrder);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE, g_deviceParams.spreadMeasurementMode);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT, g_deviceParams.spreadMeasurementCount);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, g_deviceParams.spreadMeasurementDistance);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT, g_deviceParams.spreadTopLimit);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT, g_deviceParams.spreadBottomLimit);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION, g_deviceParams.waterLevelCorrection);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE, g_deviceParams.maxDownDistance);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_REFRESHTANKHEIGHTFLAG, g_deviceParams.refreshTankHeightFlag);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXTANKHEIGHTDEVIATION, g_deviceParams.maxTankHeightDeviation);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_INITIALTANKHEIGHT, g_deviceParams.initialTankHeight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_CURRENTTANKHEIGHT, g_deviceParams.currentTankHeight);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD, g_deviceParams.oilLevelThreshold);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, g_deviceParams.liquidLevelMeasurementMethod);
	WRITE_UINT32_TO_HOLDING(HOLDREGISTER_DEVICEPARAM_CRC, g_deviceParams.crc);
}

void ReadDeviceParamsFromHoldingRegisters(int *HoldingRegisterArray) {
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_COMMAND, g_deviceParams.command);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_TANKHEIGHT, g_deviceParams.tankHeight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_BLINDZONE, g_deviceParams.blindZone);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, g_deviceParams.encoder_wheel_circumference_mm);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SENSORTYPE, g_deviceParams.sensorType);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION, g_deviceParams.softwareVersion);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT, g_deviceParams.empty_weight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION, g_deviceParams.densityCorrection);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION, g_deviceParams.temperatureCorrection);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, g_deviceParams.requireBottomMeasurement);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT, g_deviceParams.requireWaterMeasurement);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, g_deviceParams.requireSinglePointDensity);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER, g_deviceParams.spreadMeasurementOrder);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE, g_deviceParams.spreadMeasurementMode);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT, g_deviceParams.spreadMeasurementCount);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, g_deviceParams.spreadMeasurementDistance);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT, g_deviceParams.spreadTopLimit);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT, g_deviceParams.spreadBottomLimit);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION, g_deviceParams.waterLevelCorrection);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE, g_deviceParams.maxDownDistance);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_REFRESHTANKHEIGHTFLAG, g_deviceParams.refreshTankHeightFlag);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_MAXTANKHEIGHTDEVIATION, g_deviceParams.maxTankHeightDeviation);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_INITIALTANKHEIGHT, g_deviceParams.initialTankHeight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_CURRENTTANKHEIGHT, g_deviceParams.currentTankHeight);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD, g_deviceParams.oilLevelThreshold);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, g_deviceParams.liquidLevelMeasurementMethod);
	READ_UINT32_FROM_HOLDING(HOLDREGISTER_DEVICEPARAM_CRC, g_deviceParams.crc);
}

/* 写入 uint32_t 到寄存器数组 */
static void write_u32_to_regs(uint16_t *regs, uint16_t addr, uint32_t value) {
    regs[addr]     = (uint16_t)((value >> 16) & 0xFFFF);
    regs[addr + 1] = (uint16_t)(value & 0xFFFF);
}

/* 写入 int32_t 到寄存器数组 */
static void write_i32_to_regs(uint16_t *regs, uint16_t addr, int32_t value) {
    write_u32_to_regs(regs, addr, (uint32_t)value);
}

/* 写入 float 到寄存器数组（IEEE754 按 uint32_t 方式存） */
static void write_float_to_regs(uint16_t *regs, uint16_t addr, float value) {
    uint32_t temp;
    memcpy(&temp, &value, sizeof(float));
    write_u32_to_regs(regs, addr, temp);
}

/* 写入 uint8_t 到寄存器数组（占 2 个寄存器，低 16 位写值，高 16 位清零） */
static void write_u8_to_regs(uint16_t *regs, uint16_t addr, uint8_t value) {
    regs[addr] = 0;           // 高 16 位占位
    regs[addr + 1] = value;   // 低 16 位存实际值
}

/**
 * @brief 将全局测量结果写入输入寄存器数组
 * @param regs 输入寄存器数组（uint16_t 数组）
 */
void write_measurement_result_to_IputerRegisters(uint16_t *regs)
{
    /* ==== DeviceStatus ==== */
    write_u32_to_regs(regs, REG_DEVICE_STATUS_WORK_MODE, 1000);
//    write_u32_to_regs(regs, REG_DEVICE_STATUS_WORK_MODE, g_measurement.device_status.work_mode);
    write_u32_to_regs(regs, REG_DEVICE_STATUS_DEVICE_STATE, (uint32_t)g_measurement.device_status.device_state);
    write_u32_to_regs(regs, REG_DEVICE_STATUS_ERROR_CODE, g_measurement.device_status.error_code);
    write_u32_to_regs(regs, REG_DEVICE_STATUS_CURRENT_COMMAND, (uint32_t)g_measurement.device_status.current_command);
    write_u8_to_regs(regs, REG_DEVICE_STATUS_ZERO_POINT_STATUS, g_measurement.device_status.zero_point_status);

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

    /* ==== ZeroCalibration ==== */
    write_u32_to_regs(regs, REG_ZERO_CALIBRATION_TURNS, g_measurement.zero_calibration.zero_encoder_turns);
    write_u32_to_regs(regs, REG_ZERO_CALIBRATION_VALUE, g_measurement.zero_calibration.zero_encoder_value);

    /* ==== ActualHeightMeasurement ==== */
    write_u32_to_regs(regs, REG_HEIGHT_MEASUREMENT_CAL_LIQUID_LEVEL, g_measurement.height_measurement.calibrated_liquid_level);
    write_u32_to_regs(regs, REG_HEIGHT_MEASUREMENT_CURRENT_REAL, g_measurement.height_measurement.current_real_height);

    /* ==== DebugData ==== */
    write_i32_to_regs(regs, REG_DEBUG_CURRENT_ENCODER, g_measurement.debug_data.current_encoder_value);
    write_i32_to_regs(regs, REG_DEBUG_SENSOR_POSITION, g_measurement.debug_data.sensor_position);
    write_i32_to_regs(regs, REG_DEBUG_CABLE_LENGTH, g_measurement.debug_data.cable_length);
    write_u32_to_regs(regs, REG_DEBUG_FREQUENCY, g_measurement.debug_data.frequency);
    write_u32_to_regs(regs, REG_DEBUG_TEMPERATURE, g_measurement.debug_data.temperature);
    write_u32_to_regs(regs, REG_DEBUG_AIR_FREQUENCY, g_measurement.debug_data.air_frequency);
    write_u32_to_regs(regs, REG_DEBUG_CURRENT_AMPLITUDE, g_measurement.debug_data.current_amplitude);
    write_u32_to_regs(regs, REG_DEBUG_WATER_LEVEL_VOLTAGE, g_measurement.debug_data.water_level_voltage);

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
        write_u32_to_regs(regs, REG_DENSITY_POINT_TEMP(i), g_measurement.density_distribution.single_density_data[i].temperature);
        write_u32_to_regs(regs, REG_DENSITY_POINT_DENSITY(i), g_measurement.density_distribution.single_density_data[i].density);
        write_u32_to_regs(regs, REG_DENSITY_POINT_TEMP_POS(i), g_measurement.density_distribution.single_density_data[i].temperature_position);
        write_u32_to_regs(regs, REG_DENSITY_POINT_STD_DENSITY(i), g_measurement.density_distribution.single_density_data[i].standard_density);
        write_u32_to_regs(regs, REG_DENSITY_POINT_VCF20(i), g_measurement.density_distribution.single_density_data[i].vcf20);
        write_u32_to_regs(regs, REG_DENSITY_POINT_WEIGHT_DENSITY(i), g_measurement.density_distribution.single_density_data[i].weight_density);
    }
}

/* 读取 uint32_t 从寄存器数组 */
static uint32_t read_u32_from_regs(uint16_t *regs, uint16_t addr) {
    return ((uint32_t)regs[addr] << 16) | regs[addr + 1];
}

/* 读取 int32_t 从寄存器数组 */
static int32_t read_i32_from_regs(uint16_t *regs, uint16_t addr) {
    return (int32_t)read_u32_from_regs(regs, addr);
}

/* 读取 float 从寄存器数组 */
static float read_float_from_regs(uint16_t *regs, uint16_t addr) {
    uint32_t temp = read_u32_from_regs(regs, addr);
    float value;
    memcpy(&value, &temp, sizeof(float));
    return value;
}

/* 读取 uint8_t 从寄存器数组（低16位存值） */
static uint8_t read_u8_from_regs(uint16_t *regs, uint16_t addr) {
    return (uint8_t)(regs[addr + 1] & 0xFF);
}

/**
 * @brief 将输入寄存器数组解析回 MeasurementResult 结构体
 * @param regs 输入寄存器数组（uint16_t 数组）
 */
void read_measurement_result_from_inputRegisters(uint16_t *regs)
{
    /* ==== DeviceStatus ==== */
    g_measurement.device_status.work_mode      = read_u32_from_regs(regs, REG_DEVICE_STATUS_WORK_MODE);
    g_measurement.device_status.device_state   = (DeviceState)read_u32_from_regs(regs, REG_DEVICE_STATUS_DEVICE_STATE);
    printf("Device State: %d\n", g_measurement.device_status.device_state);
    g_measurement.device_status.error_code     = read_u32_from_regs(regs, REG_DEVICE_STATUS_ERROR_CODE);
    g_measurement.device_status.current_command = (CommandType)read_u32_from_regs(regs, REG_DEVICE_STATUS_CURRENT_COMMAND);
    g_measurement.device_status.zero_point_status = read_u8_from_regs(regs, REG_DEVICE_STATUS_ZERO_POINT_STATUS);

    /* ==== OilMeasurement ==== */
    g_measurement.oil_measurement.oil_level        = read_u32_from_regs(regs, REG_OIL_MEASUREMENT_OIL_LEVEL);
    g_measurement.oil_measurement.air_frequency   = read_u32_from_regs(regs, REG_OIL_MEASUREMENT_AIR_FREQUENCY);
    g_measurement.oil_measurement.oil_frequency   = read_u32_from_regs(regs, REG_OIL_MEASUREMENT_OIL_FREQUENCY);
    g_measurement.oil_measurement.follow_frequency= read_u32_from_regs(regs, REG_OIL_MEASUREMENT_FOLLOW_FREQUENCY);
    g_measurement.oil_measurement.current_frequency = read_u32_from_regs(regs, REG_OIL_MEASUREMENT_CURRENT_FREQUENCY);

    /* ==== WaterMeasurement ==== */
    g_measurement.water_measurement.water_level       = read_u32_from_regs(regs, REG_WATER_MEASUREMENT_WATER_LEVEL);
    g_measurement.water_measurement.zero_capacitance  = read_float_from_regs(regs, REG_WATER_MEASUREMENT_ZERO_CAPACITANCE);
    g_measurement.water_measurement.oil_capacitance   = read_float_from_regs(regs, REG_WATER_MEASUREMENT_OIL_CAPACITANCE);
    g_measurement.water_measurement.current_capacitance = read_float_from_regs(regs, REG_WATER_MEASUREMENT_CURRENT_CAPACITANCE);

    /* ==== ZeroCalibration ==== */
    g_measurement.zero_calibration.zero_encoder_turns = read_u32_from_regs(regs, REG_ZERO_CALIBRATION_TURNS);
    g_measurement.zero_calibration.zero_encoder_value = read_u32_from_regs(regs, REG_ZERO_CALIBRATION_VALUE);

    /* ==== ActualHeightMeasurement ==== */
    g_measurement.height_measurement.calibrated_liquid_level = read_u32_from_regs(regs, REG_HEIGHT_MEASUREMENT_CAL_LIQUID_LEVEL);
    g_measurement.height_measurement.current_real_height     = read_u32_from_regs(regs, REG_HEIGHT_MEASUREMENT_CURRENT_REAL);

    /* ==== DebugData ==== */
    g_measurement.debug_data.sensor_position   = read_i32_from_regs(regs, REG_DEBUG_SENSOR_POSITION);
    g_measurement.debug_data.cable_length      = read_i32_from_regs(regs, REG_DEBUG_CABLE_LENGTH);
    g_measurement.debug_data.frequency         = read_u32_from_regs(regs, REG_DEBUG_FREQUENCY);
    g_measurement.debug_data.temperature       = read_u32_from_regs(regs, REG_DEBUG_TEMPERATURE);
    g_measurement.debug_data.air_frequency     = read_u32_from_regs(regs, REG_DEBUG_AIR_FREQUENCY);
    g_measurement.debug_data.current_amplitude = read_u32_from_regs(regs, REG_DEBUG_CURRENT_AMPLITUDE);
    g_measurement.debug_data.water_level_voltage = read_u32_from_regs(regs, REG_DEBUG_WATER_LEVEL_VOLTAGE);

    /* ==== Single Point Measurement ==== */
    g_measurement.single_point_measurement.temperature        = read_u32_from_regs(regs, REG_SINGLE_POINT_MEAS_TEMP);
    g_measurement.single_point_measurement.density            = read_u32_from_regs(regs, REG_SINGLE_POINT_MEAS_DENSITY);
    g_measurement.single_point_measurement.temperature_position = read_u32_from_regs(regs, REG_SINGLE_POINT_MEAS_TEMP_POS);
    g_measurement.single_point_measurement.standard_density   = read_u32_from_regs(regs, REG_SINGLE_POINT_MEAS_STD_DENSITY);
    g_measurement.single_point_measurement.vcf20              = read_u32_from_regs(regs, REG_SINGLE_POINT_MEAS_VCF20);
    g_measurement.single_point_measurement.weight_density     = read_u32_from_regs(regs, REG_SINGLE_POINT_MEAS_WEIGHT_DENSITY);

    /* ==== Single Point Monitoring ==== */
    g_measurement.single_point_monitoring.temperature        = read_u32_from_regs(regs, REG_SINGLE_POINT_MON_TEMP);
    g_measurement.single_point_monitoring.density            = read_u32_from_regs(regs, REG_SINGLE_POINT_MON_DENSITY);
    g_measurement.single_point_monitoring.temperature_position = read_u32_from_regs(regs, REG_SINGLE_POINT_MON_TEMP_POS);
    g_measurement.single_point_monitoring.standard_density   = read_u32_from_regs(regs, REG_SINGLE_POINT_MON_STD_DENSITY);
    g_measurement.single_point_monitoring.vcf20              = read_u32_from_regs(regs, REG_SINGLE_POINT_MON_VCF20);
    g_measurement.single_point_monitoring.weight_density     = read_u32_from_regs(regs, REG_SINGLE_POINT_MON_WEIGHT_DENSITY);

    /* ==== Density Distribution ==== */
    g_measurement.density_distribution.average_temperature  = read_u32_from_regs(regs, REG_DENSITY_DIST_AVG_TEMP);
    g_measurement.density_distribution.average_density      = read_u32_from_regs(regs, REG_DENSITY_DIST_AVG_DENSITY);
    g_measurement.density_distribution.average_standard_density = read_u32_from_regs(regs, REG_DENSITY_DIST_AVG_STD_DENSITY);
    g_measurement.density_distribution.average_vcf20        = read_u32_from_regs(regs, REG_DENSITY_DIST_AVG_VCF20);
    g_measurement.density_distribution.average_weight_density = read_u32_from_regs(regs, REG_DENSITY_DIST_AVG_WEIGHT_DENSITY);
    g_measurement.density_distribution.measurement_points   = read_u32_from_regs(regs, REG_DENSITY_DIST_MEAS_POINTS);
    g_measurement.density_distribution.Density_oil_level    = read_u32_from_regs(regs, REG_DENSITY_DIST_OIL_LEVEL);

    for (int i = 0; i < MAX_MEASUREMENT_POINTS; i++) {
        g_measurement.density_distribution.single_density_data[i].temperature       = read_u32_from_regs(regs, REG_DENSITY_POINT_TEMP(i));
        g_measurement.density_distribution.single_density_data[i].density           = read_u32_from_regs(regs, REG_DENSITY_POINT_DENSITY(i));
        g_measurement.density_distribution.single_density_data[i].temperature_position = read_u32_from_regs(regs, REG_DENSITY_POINT_TEMP_POS(i));
        g_measurement.density_distribution.single_density_data[i].standard_density = read_u32_from_regs(regs, REG_DENSITY_POINT_STD_DENSITY(i));
        g_measurement.density_distribution.single_density_data[i].vcf20            = read_u32_from_regs(regs, REG_DENSITY_POINT_VCF20(i));
        g_measurement.density_distribution.single_density_data[i].weight_density   = read_u32_from_regs(regs, REG_DENSITY_POINT_WEIGHT_DENSITY(i));
    }
}

