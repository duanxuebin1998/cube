/*
 * wartsila_modbus_data_analysis.c
 *
 *  Created on: 2025年11月20日
 *      Author: Duan Xuebin
 */
#include "wartsila_modbus_data_analysis.h"
#include "system_parameter.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
wartsila_DeviceParameters wartsila_deviceParams = { 0 };

#include <stdint.h>

static inline uint16_t WXL_STATE(uint8_t hi, uint8_t lo)
{
    return (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

/**
 * @brief  澳邦 DeviceState -> Wartsila 设备工作状态（寄存器16bit，两字节复合码）
 * @note   映射依据：你提供的“设备工作状态对应表”
 *         0x00 44  固定点监测中（传感器不动）
 *         0x0C 64  开始密度分布测量
 *         0x0C 60  向上运行，密度分布测量中
 *         0x28 40  密度分布测量完成（到空气、液位更新）
 *         0x08 40  向下运动会固定点监测
 *         0x08 64  接收到提浮子指令
 *         0x08 60  提浮子，向上运行
 */
uint16_t AubonState_To_WartsilaWorkState(DeviceState s)
{
    switch (s)
    {
        /* 传感器不动，固定点监测中 */
        case STATE_SPTESTING:
            return WXL_STATE(0x00, 44);

        /* 开始密度分布测量 */
        case STATE_WARTSILA_DENSITY_START:
            return WXL_STATE(0x0C, 64);

        /* 向上运行，密度分布测量中 */
        case STATE_WARTSILA_DENSITY_MEASURING:
            return WXL_STATE(0x0C, 60);

        /* 密度分布测量完成 */
        case STATE_WARTSILA_DENSITY_OVER:
            return WXL_STATE(0x28, 40);

        /* 向下运动会固定点监测 */
        case STATE_RUNTOPOINTING:
            return WXL_STATE(0x08, 40);

//        /* 接收到提浮子指令 */
//        case STATE_BACKZEROING:
//            return WXL_STATE(0x08, 64);

        /* 提浮子，向上运行（你前面也说过这里等价“回零点中”） */
        case STATE_BACKZEROING:
            return WXL_STATE(0x08, 60);

        default:
            /* Wartsila 表里没给“故障码”，这里保守回到“不动监测”或你也可改成 0xFF FF */
            return WXL_STATE(0x00, 00);
    }
}


static void WartsilaToDSM(const wartsila_DeviceParameters *wxl, DeviceParameters *dsm) {

	if (wxl->down_command != 0) {
		switch (wxl->down_command) {
		case 5:
			dsm->command = CMD_WARTSILA_DENSITY_RANGE; //瓦西莱密度区间测量
			break;
		case 3:
			dsm->command = CMD_MONITOR_SINGLE; //固定点监测
			break;
		case 4:
			dsm->command = CMD_BACK_ZERO; //回零点
			break;
			//其它指令
		default:
			dsm->command = CMD_NONE;
			break;
		}
	}

	// ===== 分布测量参数 =====
	dsm->wartsila_lower_density_limit = wxl->spread_lowest_mm;     // 0x005A
	dsm->wartsila_upper_density_limit = wxl->spread_highest_mm;    // 0x005B
	dsm->wartsila_density_interval = wxl->spread_interval_mm;   // 0x005C
	dsm->wartsila_max_height_above_surface = wxl->spread_dist_to_surface_mm; // 0x005D
	//下发参数
}
void DeviceParams_LoadFromRegisters(uint16_t *reg) {

	wartsila_deviceParams.down_command = reg[0x0006]; //下发指令

	/*分布测量参数*/
	wartsila_deviceParams.spread_lowest_mm = reg[0x005A];    //分布测量起始点
	wartsila_deviceParams.spread_highest_mm = reg[0x005B];			//分布测量上限，这里用罐高代替
	wartsila_deviceParams.spread_interval_mm = reg[0x005C];			//分布测量间距
	wartsila_deviceParams.spread_dist_to_surface_mm = reg[0x005D];			//分布测量点位与液面的最小距离

	WartsilaToDSM(&wartsila_deviceParams, &g_deviceParams);
}
static void DSMToWartsila(const MeasurementResult *DSM, wartsila_DeviceParameters *WXL) {
	if ((DSM == NULL) || (WXL == NULL)) {
		return;
	}

	// 先清零，避免残留
	memset(WXL, 0, sizeof(*WXL));

	// 浮子位置
	WXL->float_pos_mm = (int32_t) ((DSM->debug_data.sensor_position)/10);
	WXL->float_state = 0;

	// 温度1（这里假设 DSM->single_point_monitoring.temperature 已经是 “×100 后的整型”，
	// 你之前寄存器里是 (temperature - 20000)，所以 WXL 里先保留 ×100 值）
	WXL->temp1_c_x100 = (int32_t) DSM->single_point_monitoring.temperature;
	WXL->temp1_state = 0;

	// 密度（同理，直接拿 DSM 里的数值）
	WXL->density_kgm3_x10 = (int32_t) DSM->single_point_monitoring.density;
	WXL->density_state = 0;

	// 指令/状态
	WXL->down_command = 0;
	WXL->device_work_state = AubonState_To_WartsilaWorkState(DSM->device_status.device_state);

	// 幅值/频率等
	WXL->amplitude_unknown = 0x4896;
	WXL->freq_value = (uint16_t) DSM->debug_data.frequency;
	WXL->unknown_state_bit = 0;

	// 温度2：你现在是用同一个温度
	WXL->temp2_c_x10 = (int32_t) DSM->single_point_monitoring.temperature;
	WXL->temp2_state = 0;

	// 液位
//	WXL->liquid_level_mm = (int32_t) (DSM->oil_measurement.oil_level/10);
	WXL->liquid_level_mm = (uint16_t) (DSM->density_distribution.Density_oil_level/10);
	WXL->liquid_state = 0;

	// 位置标志
	WXL->position_mm = 1;
	printf("分布测量上限: %lu mm\r\n", g_deviceParams.wartsila_lower_density_limit);
	WXL->spread_lowest_mm = g_deviceParams.wartsila_lower_density_limit;
	WXL->spread_highest_mm = g_deviceParams.wartsila_upper_density_limit;
	WXL->spread_interval_mm = g_deviceParams.wartsila_density_interval;
	WXL->spread_dist_to_surface_mm = g_deviceParams.wartsila_max_height_above_surface;
	// 分布测量
//	printf("分布测量起始点：%d\r\n", DSM->density_distribution.measurement_points);
	WXL->spread_point_count = (uint16_t) DSM->density_distribution.measurement_points;
	WXL->spread_oillevel_mm = (uint16_t) (DSM->density_distribution.Density_oil_level/10);
	WXL->spread_unknown = 0;

	// 密度点 100 组
	for (int i = 0; i < 100; ++i) {
		WXL->dens_points[i].pos_mm = (int16_t) (DSM->density_distribution.single_density_data[i].temperature_position/10);
		// 这里保持你当前的缩放方式：直接用 DSM 里的 density 值，不再额外 *10
		WXL->dens_points[i].density_x10 = (int16_t) DSM->density_distribution.single_density_data[i].density;
		// 温度同上，保持 ×100 的原始值
		WXL->dens_points[i].temp_x100 = (int16_t) DSM->density_distribution.single_density_data[i].temperature;
	}
}

void DeviceParams_StoreToRegisters(uint16_t *reg) {
	wartsila_DeviceParameters wxl;
	DSMToWartsila(&g_measurement, &wxl);   // 先把 DSM/g_measurement -> WXL

	// ===== 0x0000 ~ 0x000F =====
	reg[0x0000] = (uint16_t) wxl.float_pos_mm;
	reg[0x0001] = wxl.float_state;

	// 温度寄存器：你之前是 (temperature - 20000)，
	// 现在 temp1_c_x100 已经是 “×100 的整型温度”，这里保持同样偏移方式：
	reg[0x0002] = (uint16_t) (wxl.temp1_c_x100 - 20000);
	reg[0x0003] = wxl.temp1_state;

	// 密度：你之前直接写 g_measurement.single_point_monitoring.density
	reg[0x0004] = (uint16_t) wxl.density_kgm3_x10;
	reg[0x0005] = wxl.density_state;

	reg[0x0006] = wxl.down_command;
	reg[0x0007] = wxl.device_work_state;

	reg[0x0008] = wxl.amplitude_unknown;
	reg[0x0009] = wxl.freq_value;
	reg[0x000A] = wxl.unknown_state_bit;

	// 温度2，同样使用 (×100 - 20000) 方式
	reg[0x000B] = (uint16_t)((wxl.temp2_c_x10 - 20000)/10);
	reg[0x000C] = wxl.temp2_state;

	reg[0x000D] = (uint16_t) wxl.liquid_level_mm;
	reg[0x000E] = wxl.liquid_state;

	reg[0x000F] = wxl.position_mm;

	// ===== 分布测量 0x50 ~ 0x5D =====
	reg[0x0050] = wxl.spread_point_count;
	printf("分布测量点数：%d\r\n", wxl.spread_point_count);
	reg[0x0051] = wxl.spread_oillevel_mm;
	reg[0x0052] = 1;
	reg[0x0053] = wxl.spread_unknown; // 保留

	reg[0x005A] = wxl.spread_lowest_mm;
	reg[0x005B] = wxl.spread_highest_mm;
	reg[0x005C] = wxl.spread_interval_mm;
	reg[0x005D] = wxl.spread_dist_to_surface_mm;

	// ===== 密度点 100 组，每组 6 寄存器 =====
	for (int i = 0; i < 100; ++i) {
		uint16_t base = 0x0064 + i * 6;

		reg[base + 0] = (uint16_t) wxl.dens_points[i].pos_mm;
		reg[base + 1] = (uint16_t) wxl.dens_points[i].density_x10;

		// 温度保持同样的偏移方式： (×100 - 20000)
		reg[base + 2] = (uint16_t) (wxl.dens_points[i].temp_x100 - 20000);

		// base + 3、4、5 预留 => 清零（安全）
		reg[base + 3] = 0;
		reg[base + 4] = 0;
		reg[base + 5] = 0;
	}
}

