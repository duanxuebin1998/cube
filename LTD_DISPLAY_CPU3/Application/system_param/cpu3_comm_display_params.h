#ifndef __CPU3_COMM_DISPLAY_PARAMS_H__
#define __CPU3_COMM_DISPLAY_PARAMS_H__

#include <stdint.h>
#include "com_port_config.h"
#include <stdbool.h>
#include "display_tankopera.h"
#ifdef __cplusplus
extern "C" {
#endif

#define FRAM_CPU3_PARAM_ADDRESS   0x00001000U   // CPU3参数存储地址


/* ==================== Cpu3 通讯 + 显示参数（含每口配置） ==================== */

typedef struct
{
    /* ---------- 屏幕 / 基本信息 ---------- */
    uint16_t local_led_version;      // 屏幕程序版本
    uint8_t  language;               // 语言：0=中文 1=英文


    /* ---------- 数据源选择 ---------- */
    uint8_t screen_source_oil;
    uint8_t screen_source_water;
    uint8_t screen_source_d;
    uint8_t screen_source_t;

    /* ---------- 手工输入 ---------- */
    uint8_t screen_input_oil;
    uint8_t screen_input_water;
    uint8_t screen_input_d;
    uint8_t screen_input_d_switch;
    uint8_t screen_input_t;

    /* ---------- 显示类参数 ---------- */
    uint8_t  screen_decimal;
    uint16_t screen_password;
    uint8_t  screen_off_time;

    /* ---------- 串口配置（每口一个结构体） ---------- */
    ComPortConfig com1;   // COM1 = USART6
    ComPortConfig com2;   // COM2 = USART2
    ComPortConfig com3;   // COM3 = USART3

} Cpu3CommAndDisplayParams;

/* 全局实例 */
extern Cpu3CommAndDisplayParams g_cpu3_comm_display_params;
bool Cpu3Local_IsParam(OperatingNumber opera);//判断当前参数是否为CPU3参数
int32_t Cpu3Local_ReadValue(OperatingNumber opera);
void    Cpu3Local_WriteValue(OperatingNumber opera, int32_t v);

/* 可选：用于判断写入后是否需要重配串口 */
bool Cpu3Local_IsUartParam(OperatingNumber opera);

/* 初始化默认值（上电调用一次，或 FRAM 无效时用） */
void Cpu3_Params_InitDefaults(void);

/* 根据 g_cpu3_comm_display_params.com1/com2/com3 重配置 3 个串口 */
void Cpu3_ReinitAllUarts(void);

/* ========= FRAM 存储接口 ========= */

/**
 * @brief 从 FRAM 加载 Cpu3 通讯+显示参数到 g_cpu3_comm_display_params
 *        - CRC 或 magic/version 不对时，会调用 Cpu3_Params_InitDefaults() 并写回 FRAM
 */
void Cpu3_Params_LoadFromFRAM(void);

/**
 * @brief 将当前 g_cpu3_comm_display_params 保存到 FRAM（带 CRC）
 */
void Cpu3_Params_SaveToFRAM(void);


#ifdef __cplusplus
}
#endif

#endif /* __CPU3_COMM_DISPLAY_PARAMS_H__ */
