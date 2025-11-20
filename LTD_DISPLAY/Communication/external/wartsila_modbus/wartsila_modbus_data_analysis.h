/*
 * wartsila_modbus_data_analysis.h
 *
 *  Created on: 2025年11月20日
 *      Author: admin
 */

#ifndef WARTSILA_MODBUS_WARTSILA_MODBUS_DATA_ANALYSIS_H_
#define WARTSILA_MODBUS_WARTSILA_MODBUS_DATA_ANALYSIS_H_
#include "system_parameter.h"
typedef struct
{
    // 浮子
    int32_t float_pos_mm;
    uint16_t float_state;

    // 温度1
    int32_t temp1_c_x100;
    uint16_t temp1_state;

    // 密度
    int32_t density_kgm3_x10;
    uint16_t density_state;

    // 指令 / 工作状态
    uint16_t down_command;
    uint16_t device_work_state;

    uint16_t amplitude_unknown;
    uint16_t freq_value;
    uint16_t unknown_state_bit;

    // 温度2
    int32_t temp2_c_x10;
    uint16_t temp2_state;

    // 液位
    int32_t liquid_level_mm;
    uint16_t liquid_state;

    uint16_t position_mm;

    // 分布测量
    uint16_t spread_point_count;
    uint16_t spread_oillevel_mm;
    uint16_t spread_unknown;

    uint16_t spread_lowest_mm;
    uint16_t spread_highest_mm;
    uint16_t spread_interval_mm;
    uint16_t spread_dist_to_surface_mm;

    // 密度点 100 组
    struct {
        int16_t pos_mm;       // scale 1
        int16_t density_x10;  // scale 10
        int16_t temp_x100;    // scale 100
    } dens_points[100];

} wartsila_DeviceParameters;

void DeviceParams_LoadFromRegisters(uint16_t *reg) ;
void DeviceParams_StoreToRegisters(uint16_t *reg) ;

#endif /* WARTSILA_MODBUS_WARTSILA_MODBUS_DATA_ANALYSIS_H_ */
