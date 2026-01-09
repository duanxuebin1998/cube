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

/* ===================== 通用寄存器读写函数 ===================== */

/* 写入 uint32_t 到寄存器数组（高 16 位在前，低 16 位在后） */
static inline void write_u32_to_regs(uint16_t *regs, uint16_t addr, uint32_t value) {
	regs[addr] = (uint16_t) ((value >> 16) & 0xFFFFu);
	regs[addr + 1] = (uint16_t) (value & 0xFFFFu);
}

/* 从寄存器数组读取 uint32_t */
static inline uint32_t read_u32_from_regs(const uint16_t *regs, uint16_t addr) {
	return ((uint32_t) regs[addr] << 16) | (uint32_t) regs[addr + 1];
}

/* 写入 int32_t 到寄存器数组 */
static inline void write_i32_to_regs(uint16_t *regs, uint16_t addr, int32_t value) {
	write_u32_to_regs(regs, addr, (uint32_t) value);
}

/* 从寄存器数组读取 int32_t */
static inline int32_t read_i32_from_regs(const uint16_t *regs, uint16_t addr) {
	return (int32_t) read_u32_from_regs(regs, addr);
}

/* 写入 float 到寄存器数组（按 IEEE754 的 uint32_t 比特位存放） */
static inline void write_float_to_regs(uint16_t *regs, uint16_t addr, float value) {
	uint32_t temp;
	memcpy(&temp, &value, sizeof(float));
	write_u32_to_regs(regs, addr, temp);
}

/* 从寄存器数组读取 float（按 IEEE754 编码） */
static inline float read_float_from_regs(const uint16_t *regs, uint16_t addr) {
	uint32_t temp = read_u32_from_regs(regs, addr);
	float value;
	memcpy(&value, &temp, sizeof(float));
	return value;
}

/* ===================== 参数结构体 <-> 保持寄存器映射 ===================== */

/*----------------------------------------------------------------
 * 将 g_deviceParams 写入保持寄存器数组
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t
 *---------------------------------------------------------------*/
void WriteDeviceParamsToHoldingRegisters(uint16_t *HoldingRegisterArray) {
	if (HoldingRegisterArray == NULL) {
		return;
	}

	/* 指令 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_COMMAND, (uint32_t) g_deviceParams.command);

	/* 基础参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TANKHEIGHT, g_deviceParams.tankHeight);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_BLINDZONE, g_deviceParams.blindZone);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE, g_deviceParams.waterBlindZone);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM, g_deviceParams.encoder_wheel_circumference_mm);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED, g_deviceParams.max_motor_speed);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SENSORTYPE, g_deviceParams.sensorType);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SENSORID, g_deviceParams.sensorID);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION, g_deviceParams.sensorSoftwareVersion);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION, g_deviceParams.softwareVersion);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND, (uint32_t) g_deviceParams.powerOnDefaultCommand);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE, g_deviceParams.findZeroDownDistance);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM, g_deviceParams.first_loop_circumference_mm);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM, g_deviceParams.tape_thickness_mm);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED1, g_deviceParams.reserved1);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED2, g_deviceParams.reserved2);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED3, g_deviceParams.reserved3);

	/* 称重参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT, g_deviceParams.empty_weight);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT, g_deviceParams.full_weight);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO, g_deviceParams.weight_upper_limit_ratio);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO, g_deviceParams.weight_lower_limit_ratio);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT, g_deviceParams.empty_weight_upper_limit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT, g_deviceParams.empty_weight_lower_limit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT, g_deviceParams.full_weight_upper_limit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT, g_deviceParams.full_weight_lower_limit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED4, g_deviceParams.reserved4);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED5, g_deviceParams.reserved5);

	/* 指令参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL, g_deviceParams.calibrateOilLevel);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL, g_deviceParams.calibrateWaterLevel);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION, g_deviceParams.singlePointMeasurementPosition);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION, g_deviceParams.singlePointMonitoringPosition);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL, g_deviceParams.densityDistributionOilLevel);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE, g_deviceParams.motorCommandDistance);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED6, g_deviceParams.reserved6);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED7, g_deviceParams.reserved7);

	/* 密度与温度修正 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION, g_deviceParams.densityCorrection);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION, g_deviceParams.temperatureCorrection);

	/* 分布测量参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT, g_deviceParams.requireBottomMeasurement);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT, g_deviceParams.requireWaterMeasurement);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY, g_deviceParams.requireSinglePointDensity);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER, g_deviceParams.spreadMeasurementOrder);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE, g_deviceParams.spreadMeasurementMode);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT, g_deviceParams.spreadMeasurementCount);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE, g_deviceParams.spreadMeasurementDistance);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT, g_deviceParams.spreadTopLimit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT, g_deviceParams.spreadBottomLimit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME, g_deviceParams.spreadPointHoverTime);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED8, g_deviceParams.reserved8);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED9, g_deviceParams.reserved9);

	/* Wartsila 密度区间测量参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT, g_deviceParams.wartsila_upper_density_limit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT, g_deviceParams.wartsila_lower_density_limit);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL, g_deviceParams.wartsila_density_interval);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE, g_deviceParams.wartsila_max_height_above_surface);

	/* 水位测量参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION, g_deviceParams.waterLevelCorrection);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE, g_deviceParams.maxDownDistance);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_RESERVED10, g_deviceParams.reserved10);

	/* 实高测量参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_REFRESHTANKHEIGHTFLAG, g_deviceParams.refreshTankHeightFlag);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAXTANKHEIGHTDEVIATION, g_deviceParams.maxTankHeightDeviation);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_INITIALTANKHEIGHT, g_deviceParams.initialTankHeight);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CURRENTTANKHEIGHT, g_deviceParams.currentTankHeight);

	/* 液位测量参数 */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD, g_deviceParams.oilLevelThreshold);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD, g_deviceParams.oilLevelHysteresisThreshold);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD, g_deviceParams.liquidLevelMeasurementMethod);

	/* 报警 DO */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_DO, g_deviceParams.AlarmHighDO);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ALARM_LOW_DO, g_deviceParams.AlarmLowDO);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_THIRD_STATE_THRESHOLD, g_deviceParams.ThirdStateThreshold);

	/* 4-20mA 输出 AO */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_START_mA, g_deviceParams.CurrentRangeStart_mA);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_END_mA, g_deviceParams.CurrentRangeEnd_mA);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_AO, g_deviceParams.AlarmHighAO);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_ALARM_LOW_AO, g_deviceParams.AlarmLowAO);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_INITIAL_CURRENT_mA, g_deviceParams.InitialCurrent_mA);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_HIGH_CURRENT_mA, g_deviceParams.AOHighCurrent_mA);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_AO_LOW_CURRENT_mA, g_deviceParams.AOLowCurrent_mA);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_FAULT_CURRENT_mA, g_deviceParams.FaultCurrent_mA);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA, g_deviceParams.DebugCurrent_mA);

	/* 元信息和 CRC */
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_PARAM_VERSION, g_deviceParams.param_version);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE, g_deviceParams.struct_size);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_MAGIC, g_deviceParams.magic);
	write_u32_to_regs(HoldingRegisterArray, HOLDREGISTER_DEVICEPARAM_CRC, g_deviceParams.crc);
}

/* 解析03功能码保持寄存器数据 */
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
//            printf("Hold Reg %s: %d\n",param_meta[index].name,param_meta[index].val);
        }
        else if(param_meta[index].data_type == TYPE_FLOAT)
        {
            union utof tmp_f;
            tmp_f.u = ywj_hold_analysis_data(param_meta[index].startadd,param_meta[index].rgstcnt);
            tmp_f.f *= pow(10,param_meta[index].point);
            param_meta[index].val = tmp_f.f;
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

/* 单个数据解析 - 保持寄存器 */
int ywj_hold_analysis_data(int startadd,int rgscnt)
{
    int value = 0;
    int i;

//    startadd *= 2;
    //解析数据
    for(i = 0;i < rgscnt;i++)
    {
        value <<= 16;
        value += HoldingRegisterArray[startadd + i];
    }
    return value;
}



/*----------------------------------------------------------------
 * 从保持寄存器数组读取数据到 g_deviceParams
 * HoldingRegisterArray: 外部保持寄存器缓存区, 元素类型为 uint16_t
 * 注意: command 一般由线圈或功能码触发, 这里按照保持寄存器映射也支持读回
 *---------------------------------------------------------------*/
void ReadDeviceParamsFromHoldingRegisters(uint16_t *HoldingRegisterArray) {
	if (HoldingRegisterArray == NULL) {
		return;
	}

	const uint16_t *regs = (const uint16_t*) HoldingRegisterArray;
	uint32_t tmp32;

	/* 指令 */
	tmp32 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_COMMAND);
	g_deviceParams.command = (CommandType) tmp32;

	/* 基础参数 */
	g_deviceParams.tankHeight = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TANKHEIGHT);
	g_deviceParams.tankHeight = ywj_hold_analysis_data(HOLDREGISTER_DEVICEPARAM_TANKHEIGHT, 2);
	g_deviceParams.blindZone = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_BLINDZONE);
	g_deviceParams.waterBlindZone = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATER_BLINDZONE);
	g_deviceParams.encoder_wheel_circumference_mm = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM);
	g_deviceParams.max_motor_speed = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED);
	g_deviceParams.sensorType = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSORTYPE);
	g_deviceParams.sensorID = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSORID);
	g_deviceParams.sensorSoftwareVersion = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SENSOR_SOFTWARE_VERSION);
	g_deviceParams.softwareVersion = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SOFTWAREVERSION);

	tmp32 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND);
	g_deviceParams.powerOnDefaultCommand = (CommandType) tmp32;

	g_deviceParams.findZeroDownDistance = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FINDZERO_DOWN_DISTANCE);
	g_deviceParams.first_loop_circumference_mm = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FIRST_LOOP_CIRCUMFERENCE_MM);
	g_deviceParams.tape_thickness_mm = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TAPE_THICKNESS_MM);
	g_deviceParams.reserved1 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED1);
	g_deviceParams.reserved2 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED2);
	g_deviceParams.reserved3 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED3);

	/* 称重参数 */
	g_deviceParams.empty_weight = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT);
	g_deviceParams.full_weight = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT);
	g_deviceParams.weight_upper_limit_ratio = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WEIGHT_UPPER_LIMIT_RATIO);
	g_deviceParams.weight_lower_limit_ratio = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WEIGHT_LOWER_LIMIT_RATIO);
	g_deviceParams.empty_weight_upper_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_UPPER_LIMIT);
	g_deviceParams.empty_weight_lower_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT_LOWER_LIMIT);
	g_deviceParams.full_weight_upper_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_UPPER_LIMIT);
	g_deviceParams.full_weight_lower_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FULL_WEIGHT_LOWER_LIMIT);
	g_deviceParams.reserved4 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED4);
	g_deviceParams.reserved5 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED5);

	/* 指令参数 */
	g_deviceParams.calibrateOilLevel = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL);
	g_deviceParams.calibrateWaterLevel = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL);
	g_deviceParams.singlePointMeasurementPosition = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SP_MEAS_POSITION);
	g_deviceParams.singlePointMonitoringPosition = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SP_MONITOR_POSITION);
	g_deviceParams.densityDistributionOilLevel = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_DENSITY_DISTRIBUTION_OIL_LEVEL);
	g_deviceParams.motorCommandDistance = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE);
	g_deviceParams.reserved6 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED6);
	g_deviceParams.reserved7 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED7);

	/* 密度与温度修正 */
	g_deviceParams.densityCorrection = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_DENSITYCORRECTION);
	g_deviceParams.temperatureCorrection = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_TEMPERATURECORRECTION);

	/* 分布测量参数 */
	g_deviceParams.requireBottomMeasurement = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT);
	g_deviceParams.requireWaterMeasurement = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REQUIREWATERMEASUREMENT);
	g_deviceParams.requireSinglePointDensity = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY);
	g_deviceParams.spreadMeasurementOrder = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTORDER);
	g_deviceParams.spreadMeasurementMode = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTMODE);
	g_deviceParams.spreadMeasurementCount = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTCOUNT);
	g_deviceParams.spreadMeasurementDistance = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE);
	g_deviceParams.spreadTopLimit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT);
	g_deviceParams.spreadBottomLimit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREADBOTTOMLIMIT);
	g_deviceParams.spreadPointHoverTime = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME);
	g_deviceParams.reserved8 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED8);
	g_deviceParams.reserved9 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED9);

	/* Wartsila 密度区间测量参数 */
	g_deviceParams.wartsila_upper_density_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_UPPER_DENSITY_LIMIT);
	g_deviceParams.wartsila_lower_density_limit = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_LOWER_DENSITY_LIMIT);
	g_deviceParams.wartsila_density_interval = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_DENSITY_INTERVAL);
	g_deviceParams.wartsila_max_height_above_surface = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE);

	/* 水位测量参数 */
	g_deviceParams.waterLevelCorrection = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_WATERLEVELCORRECTION);
	g_deviceParams.maxDownDistance = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE);
	g_deviceParams.reserved10 = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_RESERVED10);

	/* 实高测量参数 */
	g_deviceParams.refreshTankHeightFlag = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_REFRESHTANKHEIGHTFLAG);
	g_deviceParams.maxTankHeightDeviation = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAXTANKHEIGHTDEVIATION);
	g_deviceParams.initialTankHeight = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_INITIALTANKHEIGHT);
	g_deviceParams.currentTankHeight = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CURRENTTANKHEIGHT);

	/* 液位测量参数 */
	g_deviceParams.oilLevelThreshold = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_OILLEVELTHRESHOLD);
	g_deviceParams.oilLevelHysteresisThreshold = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD);
	g_deviceParams.liquidLevelMeasurementMethod = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD);

	/* 报警 DO */
	g_deviceParams.AlarmHighDO = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_DO);
	g_deviceParams.AlarmLowDO = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ALARM_LOW_DO);
	g_deviceParams.ThirdStateThreshold = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_THIRD_STATE_THRESHOLD);

	/* 4-20mA 输出 AO */
	g_deviceParams.CurrentRangeStart_mA = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_START_mA);
	g_deviceParams.CurrentRangeEnd_mA = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CURRENT_RANGE_END_mA);
	g_deviceParams.AlarmHighAO = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ALARM_HIGH_AO);
	g_deviceParams.AlarmLowAO = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_ALARM_LOW_AO);
	g_deviceParams.InitialCurrent_mA = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_INITIAL_CURRENT_mA);
	g_deviceParams.AOHighCurrent_mA = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_HIGH_CURRENT_mA);
	g_deviceParams.AOLowCurrent_mA = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_AO_LOW_CURRENT_mA);
	g_deviceParams.FaultCurrent_mA = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_FAULT_CURRENT_mA);
	g_deviceParams.DebugCurrent_mA = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA);

	/* 元信息和 CRC */
	g_deviceParams.param_version = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_PARAM_VERSION);
	g_deviceParams.struct_size = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_STRUCT_SIZE);
	g_deviceParams.magic = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_MAGIC);
	g_deviceParams.crc = read_u32_from_regs(regs, HOLDREGISTER_DEVICEPARAM_CRC);
}

/* ===================== MeasurementResult <-> 输入寄存器映射 ===================== */

/**
 * @brief 将全局测量结果写入输入寄存器数组
 * @param regs 输入寄存器数组（uint16_t 数组）
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

	/* ==== DebugData ==== */
	write_i32_to_regs(regs, REG_DEBUG_CURRENT_ENCODER, g_measurement.debug_data.current_encoder_value);
	write_i32_to_regs(regs, REG_DEBUG_SENSOR_POSITION, g_measurement.debug_data.sensor_position);
	write_i32_to_regs(regs, REG_DEBUG_CABLE_LENGTH, g_measurement.debug_data.cable_length);

	write_u32_to_regs(regs, REG_DEBUG_FREQUENCY, g_measurement.debug_data.frequency);
	write_u32_to_regs(regs, REG_DEBUG_TEMPERATURE, g_measurement.debug_data.temperature);
	write_u32_to_regs(regs, REG_DEBUG_AIR_FREQUENCY, g_measurement.debug_data.air_frequency);
	write_u32_to_regs(regs, REG_DEBUG_CURRENT_AMPLITUDE, g_measurement.debug_data.current_amplitude);
	write_u32_to_regs(regs, REG_DEBUG_WATER_LEVEL_VOLTAGE, g_measurement.debug_data.water_level_voltage);

	/* 称重相关 */
	write_u32_to_regs(regs, REG_DEBUG_CURRENT_WEIGHT, g_measurement.debug_data.current_weight);
	write_u32_to_regs(regs, REG_DEBUG_WEIGHT_PARAM, g_measurement.debug_data.weight_param);

	/* 姿态角 */
	write_i32_to_regs(regs, REG_DEBUG_ANGLE_X, g_measurement.debug_data.angle_x);
	write_i32_to_regs(regs, REG_DEBUG_ANGLE_Y, g_measurement.debug_data.angle_y);

	/* 电机状态相关 */
	write_u32_to_regs(regs, REG_DEBUG_MOTOR_SPEED, g_measurement.debug_data.motor_speed);
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
		const DensityMeasurement *p = &g_measurement.density_distribution.single_density_data[i];

		write_u32_to_regs(regs, REG_DENSITY_POINT_TEMP(i), p->temperature);
		write_u32_to_regs(regs, REG_DENSITY_POINT_DENSITY(i), p->density);
		write_u32_to_regs(regs, REG_DENSITY_POINT_TEMP_POS(i), p->temperature_position);
		write_u32_to_regs(regs, REG_DENSITY_POINT_STD_DENSITY(i), p->standard_density);
		write_u32_to_regs(regs, REG_DENSITY_POINT_VCF20(i), p->vcf20);
		write_u32_to_regs(regs, REG_DENSITY_POINT_WEIGHT_DENSITY(i), p->weight_density);
	}
}

/**
 * @brief 将输入寄存器数组解析回 MeasurementResult 结构体
 * @param regs 输入寄存器数组（uint16_t 数组）
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

	/* ==== DebugData ==== */
	g_measurement.debug_data.current_encoder_value = read_i32_from_regs(cregs, REG_DEBUG_CURRENT_ENCODER);
	g_measurement.debug_data.sensor_position = read_i32_from_regs(cregs, REG_DEBUG_SENSOR_POSITION);
	g_measurement.debug_data.cable_length = read_i32_from_regs(cregs, REG_DEBUG_CABLE_LENGTH);

	g_measurement.debug_data.frequency = read_u32_from_regs(cregs, REG_DEBUG_FREQUENCY);
	g_measurement.debug_data.temperature = read_u32_from_regs(cregs, REG_DEBUG_TEMPERATURE);
	g_measurement.debug_data.air_frequency = read_u32_from_regs(cregs, REG_DEBUG_AIR_FREQUENCY);
	g_measurement.debug_data.current_amplitude = read_u32_from_regs(cregs, REG_DEBUG_CURRENT_AMPLITUDE);
	g_measurement.debug_data.water_level_voltage = read_u32_from_regs(cregs, REG_DEBUG_WATER_LEVEL_VOLTAGE);

	/* 称重相关 */
	g_measurement.debug_data.current_weight = read_u32_from_regs(cregs, REG_DEBUG_CURRENT_WEIGHT);
	g_measurement.debug_data.weight_param = read_u32_from_regs(cregs, REG_DEBUG_WEIGHT_PARAM);

	/* 姿态角 */
	g_measurement.debug_data.angle_x = read_i32_from_regs(cregs, REG_DEBUG_ANGLE_X);
	g_measurement.debug_data.angle_y = read_i32_from_regs(cregs, REG_DEBUG_ANGLE_Y);

	/* 电机状态 */
	g_measurement.debug_data.motor_speed = read_u32_from_regs(cregs, REG_DEBUG_MOTOR_SPEED);
	g_measurement.debug_data.motor_state = read_u32_from_regs(cregs, REG_DEBUG_MOTOR_STATE);

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
}
