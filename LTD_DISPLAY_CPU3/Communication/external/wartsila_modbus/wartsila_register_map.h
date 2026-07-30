/*
 * wartsila_register_map.h
 *
 *  Created on: 2025年11月20日
 *      Author: Duan Xuebin
 */

#ifndef WARTSILA_MODBUS_WARTSILA_REGISTER_MAP_H_
/* WARTSILA_MODBUS_WARTSILA_REGISTER_MAP_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define WARTSILA_MODBUS_WARTSILA_REGISTER_MAP_H_
#pragma once
#include <stdint.h>
#include "../external_read_freshness.h"

/*
 * ==========================================================
 *              Modbus  Holding Register Map
 * ==========================================================
 * 说明：
 * 1. 所有寄存器均为 16-bit；
 * 2. 单位与缩放比参考你的 Excel 表；
 * 3. 支持 100 个密度点（每点 6 个寄存器）；
 * ==========================================================
 */

/****************************************************
 * 0x0000 ~ 0x000F
 * 浮子/液位/温度/密度/状态测量区
 ****************************************************/

#define REG_FLOAT_POS                  0x0000   /* 浮子位置 (mm), scale = 1 */
#define REG_FLOAT_STATE                0x0001   /* 浮子状态, scale = 1 */

#define REG_TEMP1                      0x0002   /* 温度1 (°C, 有符号), scale = 100 */
#define REG_TEMP1_STATE                0x0003   /* 温度1 状态, scale = 1 */

#define REG_DENSITY_VALUE              0x0004   /* 密度值 (kg/m3), scale = 10 */
#define REG_DENSITY_STATE              0x0005   /* 密度状态, scale = 1 */

#define REG_DOWN_COMMAND               0x0006   /* 下发指令, scale = 1 */
#define REG_DEVICE_WORK_STATE          0x0007   /* 设备工作状态, scale = 1 */

#define REG_AMPLITUDE_UNKNOWN          0x0008   /* (幅值) 未知, scale = 1 */
#define REG_FREQ_VALUE                 0x0009   /* 频率值, scale = 1 */

#define REG_UNKNOWN_STATE_BIT          0x000A   /* 未知状态位, scale = 1 */

#define REG_TEMP2                      0x000B   /* 温度2 (°C, 有符号), scale = 10 */
#define REG_TEMP2_STATE                0x000C   /* 温度2 状态, scale = 1 */

#define REG_LIQUID_LEVEL               0x000D   /* 液位值 (mm), scale = 1 */
#define REG_LIQUID_STATE               0x000E   /* 液位状态, scale = 1 */

#define REG_POSITION                   0x000F   /* 位置, scale = 1 */


/****************************************************
 * 0x0050 ~ 0x005D
 * 分布测量参数区
 ****************************************************/

#define REG_SPREAD_POINT_COUNT         0x0050   /* 分布测量点数, scale = 1 */
#define REG_SPREAD_OILLEVEL            0x0051   /* 分布测量液位值 (mm), scale = 1 */
#define REG_SPREAD_UNKNOWN             0x0052   /* 未知，随液位变化 */
/* Wartsila 分布测量摘要区地址 0x0053 的协议保留字段；当前不承载业务语义，发送端保持兼容占位。 */
#define REG_SPREAD_RESERVED0           0x0053

#define REG_SPREAD_LOWEST_POINT        0x005A   /* 分布测量最低点 (mm) */
#define REG_SPREAD_HIGHEST_POINT       0x005B   /* 分布测量最高点 (mm) */
#define REG_SPREAD_INTERVAL            0x005C   /* 分布测量间隔 */
#define REG_SPREAD_DIST_TO_SURFACE     0x005D   /* 分布测量距离液面距离 (mm) */


/****************************************************
 * 0x0064 起：100 个密度点
 * 每点 6 个寄存器：位置/密度/温度/预留×3
 ****************************************************/

#define REG_DENSITY_POINT_BASE         0x0064     /* 密度点1起始地址 */
#define REG_DENSITY_POINT_STRIDE       6          /* 每点占 6 个寄存器 */
#define REG_DENSITY_POINT_COUNT        CPU3_EXTERNAL_WARTSILA_POINT_COUNT /* 第一版可靠支持 100 点 */

/* i = 0..REG_DENSITY_POINT_COUNT-1 对应 点1..点N */
#define REG_DENS_PT_POS(i)   (uint16_t)(REG_DENSITY_POINT_BASE + (i) * REG_DENSITY_POINT_STRIDE + 0)
/* 位置 (mm), scale = 1 */

/* 返回第 i 个密度点的密度字段地址；单位为 kg/m3，协议缩放系数为 10。 */
#define REG_DENS_PT_VALUE(i) (uint16_t)(REG_DENSITY_POINT_BASE + (i) * REG_DENSITY_POINT_STRIDE + 1)

/* 返回第 i 个密度点的温度字段地址；单位为摄氏度，协议缩放系数为 100。 */
#define REG_DENS_PT_TEMP(i)  (uint16_t)(REG_DENSITY_POINT_BASE + (i) * REG_DENSITY_POINT_STRIDE + 2)

/* 返回第 i 个密度点的第 1 个保留寄存器地址；当前无业务语义，但仍计入固定步长。 */
#define REG_DENS_PT_RSVD1(i) (uint16_t)(REG_DENSITY_POINT_BASE + (i) * REG_DENSITY_POINT_STRIDE + 3)
/* 返回第 i 个密度点的第 2 个保留寄存器地址；当前无业务语义，但仍计入固定步长。 */
#define REG_DENS_PT_RSVD2(i) (uint16_t)(REG_DENSITY_POINT_BASE + (i) * REG_DENSITY_POINT_STRIDE + 4)
/* 返回第 i 个密度点的第 3 个保留寄存器地址；该字段也是本点布局的最后一个地址。 */
#define REG_DENS_PT_RSVD3(i) (uint16_t)(REG_DENSITY_POINT_BASE + (i) * REG_DENSITY_POINT_STRIDE + 5)


/****************************************************
 * 全寄存器范围（供 0x03/0x10 边界检查）
 ****************************************************/
/* Wartsila 保持寄存器表的包含式起始地址 0x0000；与结束地址共同用于单地址和连续区间边界校验。 */
#define HOLDREG_START_ADDR   0x0000
/* Wartsila 保持寄存器表的包含式结束地址；由最后一个密度点的第三个保留字段计算，点数变化时自动同步。 */
#define HOLDREG_END_ADDR     ( REG_DENS_PT_RSVD3(REG_DENSITY_POINT_COUNT - 1) )
/* Wartsila 保持寄存器镜像元素数量；按包含式首末地址计算，必须用于声明 g_holding_regs 的长度。 */
#define HOLDREG_COUNT        (HOLDREG_END_ADDR - HOLDREG_START_ADDR + 1)


/****************************************************
 * 辅助访问宏（可选）
 ****************************************************/
extern uint16_t g_holding_regs[HOLDREG_COUNT];

/* 判断地址 addr 是否落在 Wartsila 保持寄存器闭区间内；该宏只校验单地址，不校验多寄存器字段是否完整。 */
#define HOLDREG_VALID(addr)  ((addr) >= HOLDREG_START_ADDR && (addr) <= HOLDREG_END_ADDR)
/* 把合法 Wartsila 协议地址 addr 换算为 g_holding_regs 的零基数组下标；调用前必须先通过 HOLDREG_VALID。 */
#define HOLDREG_OFFSET(addr) ((addr) - HOLDREG_START_ADDR)

/* 在地址有效时向 Wartsila 保持寄存器镜像写入一个 16 位值；无效地址静默忽略，宏参数不得带有依赖单次求值的副作用。 */
#define HR_SET(addr, val) \
    do{ if(HOLDREG_VALID(addr)) g_holding_regs[HOLDREG_OFFSET(addr)] = (uint16_t)(val); }while(0)

/* 从 Wartsila 保持寄存器镜像读取一个 16 位值；地址越界时返回 0，因此调用方不能仅凭返回 0 区分合法零值与非法地址。 */
#define HR_GET(addr) \
    ( HOLDREG_VALID(addr) ? g_holding_regs[HOLDREG_OFFSET(addr)] : 0 )

#endif /* WARTSILA_MODBUS_WARTSILA_REGISTER_MAP_H_ */
