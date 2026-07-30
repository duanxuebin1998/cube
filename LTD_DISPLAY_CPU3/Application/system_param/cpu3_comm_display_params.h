#ifndef __CPU3_COMM_DISPLAY_PARAMS_H__
/* __CPU3_COMM_DISPLAY_PARAMS_H__ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define __CPU3_COMM_DISPLAY_PARAMS_H__

#include <stdint.h>
#include "com_port_config.h"
#include <stdbool.h>
#include "display_tankopera.h"
#ifdef __cplusplus
extern "C" {
#endif

#define FRAM_CPU3_PARAM_ADDRESS   0x00001000U   /* CPU3参数存储地址 */
/* CPU3 为旧 SI 保持寄存器接口保留的兼容字段数量 6；默认值表、持久化结构和 SI 映射必须使用同一计数。 */
#define CPU3_SI_COMPAT_HOLDING_COUNT 6U


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

    /* ---------- SI 40004～40009原始兼容槽 ---------- */
    uint16_t si_compat_holding[CPU3_SI_COMPAT_HOLDING_COUNT];

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
 * @brief 事务式写入 CPU3 本地参数并返回 FRAM 写后读回校验结果。
 * @param opera 参数操作号。
 * @param v 待写入值。
 * @return true 表示参数已持久化；false 表示 FRAM 校验失败且运行态已回滚。
 */
bool    Cpu3Local_WriteValueChecked(OperatingNumber opera, int32_t v);
/**
 * @brief 事务式写入远程切换目标协议并保持当前串口物理参数。
 *
 * @details 调用场景：0x46 管理帧 ACK 发送完成后的主循环安全点。
 * @note 关键约束：调用方只能传入 COM1、COM2 或 COM3 的协议操作号。
 */
bool    Cpu3Local_WriteProtocolPreserveSerialChecked(OperatingNumber opera, int32_t v);
/**
 * @brief 读取SI 40004～40009原始兼容槽。
 *
 * @details 调用场景：SI Modbus FC03刷新保持寄存器快照。
 * @note 关键约束：字段只做原值保存，不参与测量、报警或控制。
 */
uint16_t Cpu3Local_ReadSiCompatHolding(uint8_t index);
/**
 * @brief 写入SI原始兼容槽并校验FRAM持久化结果。
 *
 * @details 调用场景：SI Modbus FC06写40004～40009。
 * @note 关键约束：持久化失败时恢复运行态旧值，禁止回显伪成功。
 */
bool Cpu3Local_WriteSiCompatHoldingChecked(uint8_t index, uint16_t value);
/**
 * @brief 将 CPU3 本地显示参数同步到运行期全局变量，并立即应用 OLED 亮度。
 *
 * 上电时 FRAM 加载发生在 OLED 默认初始化之后，所以这里必须再应用一次保存的挡位。
 */
void    Cpu3Local_ApplyDisplayRuntimeParams(void);

/**
 * @brief 判断指定操作号是否属于 CPU3 本地串口参数。
 *
 * 可选：用于判断写入后是否需要重配串口。
 *
 * @param opera 参数操作号。
 * @return true 表示串口参数，false 表示其他本地参数。
 */
bool Cpu3Local_IsUartParam(OperatingNumber opera);

/**
 * @brief 恢复 CPU3 显示参数以及三路外部 COM 口的编译期默认配置。
 *
 * 初始化默认值（上电调用一次，或 FRAM 无效时用）。
 */
void Cpu3_Params_InitDefaults(void);

/**
 * @brief 依次按当前 COM1、COM2、COM3 配置重新初始化三路外部 UART。
 *
 * 根据 g_cpu3_comm_display_params.com1/com2/com3 重配置3个串口并启动接收。
 *
 * @return true 表示 COM1、COM2、COM3 均按当前配置重初始化成功；任一端口失败时返回 false。
 */
bool Cpu3_ReinitAllUarts(void);

/**
 * @brief 按当前 CPU3 本机参数只重初始化指定的外部 COM 口。
 *
 * @details 调用场景：协议切换应答发送完成后，由主循环保存新协议并调用。
 *
 * 根据已保存的端口配置只重初始化一个外部 COM 口并恢复 DMA 接收。
 * @param port_idx 外部端口编号，1=COM1，2=COM2，3=COM3。
 * @return true 表示 UART 初始化和 DMA 接收启动成功，false 表示端口号无效或启动失败。
 *
 * @note 关键约束：不影响其它外部 COM 口；调用前应确保目标端口发送已经完成。
 *
 * @param port_idx 零基外部串口索引。
 * @return true 表示端口 1～3 对应 UART 已停止旧 DMA、按当前配置重新初始化并恢复空闲接收；false 表示端口编号无效，或 HAL 初始化或接收启动失败。
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
