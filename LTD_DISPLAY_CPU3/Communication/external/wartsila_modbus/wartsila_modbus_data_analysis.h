/*
 * wartsila_modbus_data_analysis.h
 *
 *  Created on: 2025年11月20日
 *      Author: Duan Xuebin
 */

#ifndef WARTSILA_MODBUS_WARTSILA_MODBUS_DATA_ANALYSIS_H_
/* WARTSILA_MODBUS_WARTSILA_MODBUS_DATA_ANALYSIS_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define WARTSILA_MODBUS_WARTSILA_MODBUS_DATA_ANALYSIS_H_
#include "system_parameter.h"
typedef struct
{
    /* 浮子 */
    int32_t float_pos_mm;
    uint16_t float_state;

    /* 温度1 */
    int32_t temp1_c_x100;
    uint16_t temp1_state;

    /* 密度 */
    int32_t density_kgm3_x10;
    uint16_t density_state;

    /* 指令 / 工作状态 */
    uint16_t down_command;
    uint16_t device_work_state;

    uint16_t amplitude_unknown;
    uint16_t freq_value;
    uint16_t unknown_state_bit;

    /* 温度2 */
    int32_t temp2_c_x10;
    uint16_t temp2_state;

    /* 液位 */
    int32_t liquid_level_mm;
    uint16_t liquid_state;

    uint16_t position_mm;

    /* 分布测量 */
    uint16_t spread_point_count;
    uint16_t spread_oillevel_mm;
    uint16_t spread_unknown;

    uint16_t spread_lowest_mm;
    uint16_t spread_highest_mm;
    uint16_t spread_interval_mm;
    uint16_t spread_dist_to_surface_mm;

    /* 密度点 100 组 */
    struct {
        int16_t pos_mm;       /* scale 1 */
        int16_t density_x10;  /* scale 10 */
        int16_t temp_x100;    /* scale 100 */
    } dens_points[100];

} wartsila_DeviceParameters;

/**
 * @brief 从瓦锡兰保持寄存器读取命令和分布测量参数，并转换为 CPU2 参数镜像。
 *
 * @param reg 目标寄存器地址或寄存器值。该指针指向瓦锡兰连续寄存器镜像，函数按固定地址表装载或写入设备参数字段。
 */
void DeviceParams_LoadFromRegisters(uint16_t *reg) ;
/**
 * @brief 将当前设备参数同步到瓦锡兰保持寄存器缓存。
 * @param reg 目标寄存器地址或寄存器值。该指针指向瓦锡兰连续寄存器镜像，函数按固定地址表装载或写入设备参数字段。
 */
void DeviceParams_StoreToRegisters(uint16_t *reg) ;

/**
 * @brief 刷新 Wärtsilä 可离线读取的 CPU3 本地静态字段。
 *
 * @details 调用场景：FC03 完成整帧新鲜度分类后、复制寄存器池之前调用。
 *
 * @param reg Wärtsilä 保持寄存器池。
 *
 * @note 关键约束：不读取 g_measurement 或 g_deviceParams，点表预留 lane 始终返回零。
 *
 * @param reg 目标寄存器地址或寄存器值。该指针指向瓦锡兰连续寄存器镜像，函数按固定地址表装载或写入设备参数字段。
 */
void Wartsila_StoreLocalStaticRegisters(uint16_t *reg);

#endif /* WARTSILA_MODBUS_WARTSILA_MODBUS_DATA_ANALYSIS_H_ */
