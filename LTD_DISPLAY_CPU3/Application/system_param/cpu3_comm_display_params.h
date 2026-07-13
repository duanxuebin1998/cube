#ifndef __CPU3_COMM_DISPLAY_PARAMS_H__
#define __CPU3_COMM_DISPLAY_PARAMS_H__

#include <stdint.h>
#include "com_port_config.h"
#include <stdbool.h>
#include "display_tankopera.h"
#ifdef __cplusplus
extern "C" {
#endif

#define FRAM_CPU3_PARAM_ADDRESS   0x00001000U   /* CPU3参数存储地址 */


/* ==================== Cpu3 通讯 + 显示参数（含每口配置） ==================== */

typedef struct
{
    /* ---------- 屏幕 / 基本信息 ---------- */
    uint32_t local_led_version;      /* CPU3程序版本 */
    uint8_t  language;               /* 语言：0=中文 1=英文 */


    /* ---------- 数据源选择 ---------- */
    uint8_t screen_source_oil;
    uint8_t screen_source_water;
    uint8_t screen_source_d;
    uint8_t screen_source_t;

    /* ---------- 手工输入 ---------- */
    /* 手输值按菜单原始倍率保存，不能用 uint8_t 截断。 */
    int32_t screen_input_oil;
    int32_t screen_input_water;
    int32_t screen_input_d;
    uint8_t screen_input_d_switch;
    int32_t screen_input_t;

    /* ---------- 显示类参数 ---------- */
    uint8_t  screen_decimal;
    uint16_t screen_password;
    uint8_t  screen_off_time;
    uint8_t  screen_brightness;

    /* ---------- SI 专用参数 ---------- */
    uint16_t si_auto_profile_interval;
    uint8_t  si_auto_profile_enable;
    uint8_t  si_auto_profile_hour;
    uint8_t  si_auto_profile_minute;
    uint16_t si_low_density_setpoint;
    uint16_t si_high_density_setpoint;
    int16_t  si_low_temperature_setpoint;
    int16_t  si_high_temperature_setpoint;
    uint16_t si_ll_level_setpoint;
    uint16_t si_hh_level_setpoint;
    uint16_t si_low_level_setpoint;
    uint16_t si_high_level_setpoint;
    uint16_t si_temp_deviation_setpoint;
    uint16_t si_density_deviation_setpoint;

    /* ---------- 串口配置（每口一个结构体） ---------- */
    ComPortConfig com1;   /* COM1 = USART6 */
    ComPortConfig com2;   /* COM2 = USART2 */
    ComPortConfig com3;   /* COM3 = USART3 */

} Cpu3CommAndDisplayParams;

/* 全局实例 */
extern Cpu3CommAndDisplayParams g_cpu3_comm_display_params;
/**
 * @brief 判断操作号是否属于 CPU3 本地参数。
 * @param opera 参数操作号。
 * @return true 表示 CPU3 本地参数，false 表示需要转发 CPU2。
 */
bool Cpu3Local_IsParam(OperatingNumber opera); /* 判断当前参数是否为CPU3参数 */
/**
 * @brief 读取 CPU3 本地参数当前值。
 * @param opera 参数操作号。
 * @return 参数值。
 */
int32_t Cpu3Local_ReadValue(OperatingNumber opera);
/**
 * @brief 写入 CPU3 本地参数并触发必要的运行态刷新。
 * @param opera 参数操作号。
 * @param v 待写入值。
 */
void    Cpu3Local_WriteValue(OperatingNumber opera, int32_t v);
/**
 * @brief 写入 CPU3 本地参数并返回 FRAM 写后读回校验结果。
 * @param opera 参数操作号。
 * @param v 待写入值。
 * @return true 表示参数已持久化，false 表示 FRAM 写后读回不一致。
 */
bool    Cpu3Local_WriteValueChecked(OperatingNumber opera, int32_t v);
/**
 * @brief 显示或打印参数存储中的 Cpu3Local_ApplyDisplayRuntimeParams 逻辑。
 */
void    Cpu3Local_ApplyDisplayRuntimeParams(void);

/* 可选：用于判断写入后是否需要重配串口 */
bool Cpu3Local_IsUartParam(OperatingNumber opera);

/* 初始化默认值（上电调用一次，或 FRAM 无效时用） */
void Cpu3_Params_InitDefaults(void);

/* 根据 g_cpu3_comm_display_params.com1/com2/com3 重配置 3 个串口 */
void Cpu3_ReinitAllUarts(void);

/*
 * @brief 根据已保存的端口配置只重初始化一个外部 COM 口并恢复 DMA 接收。
 * @param port_idx 外部端口编号，1=COM1，2=COM2，3=COM3。
 * @return true 表示 UART 初始化和 DMA 接收启动成功，false 表示端口号无效或启动失败。
 */
bool Cpu3_ReinitPortUart(uint8_t port_idx);

/* ========= FRAM 存储接口 ========= */

/**
 * @brief 从 FRAM 加载 Cpu3 通讯+显示参数到 g_cpu3_comm_display_params
 *        - CRC 或 magic/version 不对时，会调用 Cpu3_Params_InitDefaults() 并写回 FRAM
 */
void Cpu3_Params_LoadFromFRAM(void);

/**
 * @brief 将当前 g_cpu3_comm_display_params 保存到 FRAM（带 CRC）
 * @return true 表示现有镜像一致或写后读回校验通过，false 表示持久化失败。
 */
bool Cpu3_Params_SaveToFRAM(void);


#ifdef __cplusplus
}
#endif

#endif /* __CPU3_COMM_DISPLAY_PARAMS_H__ */
