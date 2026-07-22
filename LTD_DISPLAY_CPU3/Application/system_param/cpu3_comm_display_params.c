/*
 * cpu3_comm_display_params.c
 *
 *  Created on: 2025年12月10日
 *      Author: Duan Xuebin
 */


#include "cpu3_comm_display_params.h"
#include "usart.h"
#include <stddef.h>
#include <string.h>
#include "mb85rs2m.h"     /* WriteMultiData / ReadMultiData */
#include "my_crc.h"
#include "display_tankopera.h"
#include "app_version.h"
#include "display.h"
#include "hgs.h"
#include "system_parameter.h"
/* 这些在 usart.c 里定义 */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern uint8_t UART2_RX_BUF[UART2_RX_BUF_SIZE];
extern uint8_t UART3_RX_BUF[UART3_RX_BUF_SIZE];
extern uint8_t UART6_RX_BUF[UART6_RX_BUF_SIZE];

/* 全局实例。DisplayInit 早于 FRAM 加载，先给显示参数一个安全默认值。 */
Cpu3CommAndDisplayParams g_cpu3_comm_display_params = {
    .local_led_version = CPU3_APP_VERSION_U32,
    .screen_decimal = 2U,
    .screen_brightness = OLED_BRIGHTNESS_LEVEL_LOW,
};

static const uint16_t s_cpu3_si_compat_holding_defaults[CPU3_SI_COMPAT_HOLDING_COUNT] = {
    0U, 50U, 1000U, 0U, 5U, 1U
};

/* ================= CPU3 本机参数描述表 ================= */




#define CPU3_LOCAL_PARAM_COUNT (sizeof(cpu3_local_param_table) / sizeof(cpu3_local_param_table[0]))

/* CPU3版本跟随当前显示板固件，避免被FRAM旧参数覆盖。 */
static bool Cpu3_ApplyFirmwareVersionRuntime(void)
{
    if (g_cpu3_comm_display_params.local_led_version == CPU3_APP_VERSION_U32) {
        return false;
    }

    g_cpu3_comm_display_params.local_led_version = CPU3_APP_VERSION_U32;
    return true;
}


/* 判断当前操作是否 CPU3 本机参数 */
bool Cpu3Local_IsParam(OperatingNumber opera)
{
    return (opera >= COM_NUM_PARA_LOCAL_START);
}

/* ========== CPU3 通信/显示参数 <-> 菜单值 的转换 ========== */

/* 波特率索引 -> 实际波特率 */
static uint32_t Cpu3_BaudIndexToValue(int idx)
{
	switch (idx) {
	case 0:	return 1200;
	case 1:	return 2400;
	case 2:	return 4800;
	case 3:	return 9600;
	case 4:	return 19200;
	case 5:	return 38400;
	case 6:	return 57600;
	case 7:	return 115200;
	default:	return 9600;
	}
}

/* 实际波特率 -> 索引 */
static int Cpu3_BaudValueToIndex(uint32_t baud)
{
	switch (baud) {
	case 1200:	return 0;
	case 2400:	return 1;
	case 4800:	return 2;
	case 9600:	return 3;
	case 19200:	return 4;
	case 38400:	return 5;
	case 57600:	return 6;
	case 115200:	return 7;
	default:	return 3;	/* 默认 9600 */
	}
}

/*
 * 归一化协议枚举值。
 * FRAM 旧值或菜单异常值可能落入未定义范围，必须先收敛再作为协议分发表索引。
 */
static ComProtocolType Cpu3_NormalizeProtocol(int32_t protocol)
{
    /*
     * 只允许现场菜单确认过的协议值。
     * 预留协议值 4 收敛回 DSM，避免选择后保留不明确的串口组合。
     */
    switch (protocol) {
    case COM_PROTO_DSM:
    case COM_PROTO_WARTSILA:
    case COM_PROTO_LTD:
    case COM_PROTO_LH:
    case COM_PROTO_SI:
        return (ComProtocolType)protocol;
    default:
        return COM_PROTO_DSM;
    }
}

/*
 * 根据协议生成默认串口模板。
 * 只有写入协议字段时才应用该模板，普通串口字段允许现场单独覆盖。
 */
static void Cpu3_FillProtocolSerialProfile(ComProtocolType protocol, ComPortConfig *profile)
{
    if (profile == NULL) {
        return;
    }

    profile->protocol = Cpu3_NormalizeProtocol(protocol);
    profile->baudrate = 4800U;
    profile->databits = 8U;
    profile->parity = COM_PARITY_NONE;
    profile->stopbits = COM_STOPBITS_1;

    switch (profile->protocol) {
    case COM_PROTO_DSM:
        /* 计量仪/DSM 按 4800 8N1。 */
        profile->baudrate = 4800U;
        profile->databits = 8U;
        profile->parity = COM_PARITY_NONE;
        profile->stopbits = COM_STOPBITS_1;
        break;

    case COM_PROTO_WARTSILA:
        /* 瓦锡兰通信按现场要求使用 2400 8E1。 */
        profile->baudrate = 2400U;
        profile->databits = 8U;
        profile->parity = COM_PARITY_EVEN;
        profile->stopbits = COM_STOPBITS_1;
        break;

    case COM_PROTO_LTD:
        /* LTD 对外串口默认值与计量仪/DSM 保持一致，不影响板间 UART5。 */
        profile->baudrate = 4800U;
        profile->databits = 8U;
        profile->parity = COM_PARITY_NONE;
        profile->stopbits = COM_STOPBITS_1;
        break;

    case COM_PROTO_LH:
        /* LH 现场协议固定带出 9600 8N1，协议切换不影响其它外部端口。 */
        profile->baudrate = 9600U;
        profile->databits = 8U;
        profile->parity = COM_PARITY_NONE;
        profile->stopbits = COM_STOPBITS_1;
        break;

    case COM_PROTO_SI:
        /* SI 官方 Modbus 资料要求 9600 8O1。 */
        profile->baudrate = 9600U;
        profile->databits = 8U;
        profile->parity = COM_PARITY_ODD;
        profile->stopbits = COM_STOPBITS_1;
        break;

    default:
        break;
    }
}

/*
 * 将单个串口配置切换到当前协议的默认参数。
 * 返回值表示配置是否被修正，调用方据此决定是否回写 FRAM。
 */
static uint8_t Cpu3_ApplyProtocolSerialProfile(ComPortConfig *cfg)
{
    ComPortConfig profile;
    uint8_t changed = 0U;

    if (cfg == NULL) {
        return 0U;
    }

    Cpu3_FillProtocolSerialProfile(cfg->protocol, &profile);

    if (cfg->protocol != profile.protocol) {
        cfg->protocol = profile.protocol;
        changed = 1U;
    }
    if (cfg->baudrate != profile.baudrate) {
        cfg->baudrate = profile.baudrate;
        changed = 1U;
    }
    if (cfg->databits != profile.databits) {
        cfg->databits = profile.databits;
        changed = 1U;
    }
    if (cfg->parity != profile.parity) {
        cfg->parity = profile.parity;
        changed = 1U;
    }
    if (cfg->stopbits != profile.stopbits) {
        cfg->stopbits = profile.stopbits;
        changed = 1U;
    }

    return changed;
}

/*
 * 旧版本把瓦锡兰默认值写成 4800 无校验；加载 FRAM 时只迁移这一类旧默认值。
 * 其它合法人工配置仍保留，避免升级后覆盖现场已确认的串口参数。
 */
static uint8_t Cpu3_MigrateLegacyWartsilaDefault(ComPortConfig *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }

    if ((cfg->protocol == COM_PROTO_WARTSILA)
        && (cfg->baudrate == 4800U)
        && (cfg->databits == 8U)
        && (cfg->parity == COM_PARITY_NONE)
        && ((cfg->stopbits == COM_STOPBITS_1) || (cfg->stopbits == COM_STOPBITS_2)))
    {
        return Cpu3_ApplyProtocolSerialProfile(cfg);
    }

    return 0U;
}

/*
 * 对三路外部串口执行瓦锡兰旧默认值迁移。
 */
static uint8_t Cpu3_MigrateLegacyWartsilaDefaults(void)
{
    uint8_t changed = 0U;

    changed |= Cpu3_MigrateLegacyWartsilaDefault(&g_cpu3_comm_display_params.com1);
    changed |= Cpu3_MigrateLegacyWartsilaDefault(&g_cpu3_comm_display_params.com2);
    changed |= Cpu3_MigrateLegacyWartsilaDefault(&g_cpu3_comm_display_params.com3);

    return changed;
}

/*
 * 判断 FRAM 或外部写入的波特率是否在菜单支持范围内。
 */
static uint8_t Cpu3_IsSupportedBaudrate(uint32_t baudrate)
{
    switch (baudrate) {
    case 1200U:
    case 2400U:
    case 4800U:
    case 9600U:
    case 19200U:
    case 38400U:
    case 57600U:
    case 115200U:
        return 1U;
    default:
        return 0U;
    }
}

/*
 * 只修正非法串口字段，不按协议覆盖用户已经保存的合法物理参数。
 * 该函数用于 FRAM 加载或写入兜底，避免旧值越界导致 UART 初始化异常。
 */
static uint8_t Cpu3_SanitizePortConfig(ComPortConfig *cfg)
{
    ComPortConfig profile;
    uint8_t changed = 0U;

    if (cfg == NULL) {
        return 0U;
    }

    Cpu3_FillProtocolSerialProfile(cfg->protocol, &profile);

    if (cfg->protocol != profile.protocol) {
        cfg->protocol = profile.protocol;
        changed = 1U;
    }
    if (Cpu3_IsSupportedBaudrate(cfg->baudrate) == 0U) {
        cfg->baudrate = profile.baudrate;
        changed = 1U;
    }
    if ((cfg->databits != 8U) && (cfg->databits != 9U)) {
        cfg->databits = profile.databits;
        changed = 1U;
    }
    if ((cfg->parity != COM_PARITY_NONE)
        && (cfg->parity != COM_PARITY_EVEN)
        && (cfg->parity != COM_PARITY_ODD))
    {
        cfg->parity = profile.parity;
        changed = 1U;
    }
    if ((cfg->stopbits != COM_STOPBITS_1) && (cfg->stopbits != COM_STOPBITS_2)) {
        cfg->stopbits = profile.stopbits;
        changed = 1U;
    }

    return changed;
}

/*
 * 对三个外部串口统一执行非法值修正。
 * 该函数不直接重启 UART，只修正参数结构，避免和通信收发并发。
 */
static uint8_t Cpu3_SanitizeAllPortConfigs(void)
{
    uint8_t changed = 0U;

    /* 三个外部口共用同一套合法性规则，后续新增端口时只在这里补入口。 */
    changed |= Cpu3_SanitizePortConfig(&g_cpu3_comm_display_params.com1);
    changed |= Cpu3_SanitizePortConfig(&g_cpu3_comm_display_params.com2);
    changed |= Cpu3_SanitizePortConfig(&g_cpu3_comm_display_params.com3);

    return changed;
}

/*
 * 函数用途：恢复SI 40004～40009原始兼容槽的现场默认值。
 * 调用场景：恢复出厂、旧FRAM迁移和V6到V7升级。
 * 关键约束：这些值没有业务含义，只允许原样读写和持久化。
 */
static void Cpu3_InitSiCompatHolding(void)
{
    memcpy(g_cpu3_comm_display_params.si_compat_holding,
           s_cpu3_si_compat_holding_defaults,
           sizeof(g_cpu3_comm_display_params.si_compat_holding));
}

/*
 * SI 自动 profile 和报警限值是 CPU3 本机协议参数。
 * 这里统一补默认值，供恢复出厂和旧 FRAM 迁移复用。
 */
static void Cpu3_InitSiParams(void)
{
    g_cpu3_comm_display_params.si_auto_profile_interval = 60U;
    g_cpu3_comm_display_params.si_auto_profile_enable = 0U;
    g_cpu3_comm_display_params.si_auto_profile_hour = 0U;
    g_cpu3_comm_display_params.si_auto_profile_minute = 0U;
    g_cpu3_comm_display_params.si_low_density_setpoint = 0U;
    g_cpu3_comm_display_params.si_high_density_setpoint = 0U;
    g_cpu3_comm_display_params.si_low_temperature_setpoint = 0U;
    g_cpu3_comm_display_params.si_high_temperature_setpoint = 0U;
    g_cpu3_comm_display_params.si_ll_level_setpoint = 0U;
    g_cpu3_comm_display_params.si_hh_level_setpoint = 0U;
    g_cpu3_comm_display_params.si_low_level_setpoint = 0U;
    g_cpu3_comm_display_params.si_high_level_setpoint = 0U;
    g_cpu3_comm_display_params.si_temp_deviation_setpoint = 0U;
    g_cpu3_comm_display_params.si_density_deviation_setpoint = 0U;
    Cpu3_InitSiCompatHolding();
}

static uint16_t Cpu3_ClampU16Param(int32_t value)
{
    if (value <= 0) {
        return 0U;
    }
    if (value > 65535) {
        return 65535U;
    }
    return (uint16_t)value;
}

static int16_t Cpu3_ClampS16Param(int32_t value)
{
    if (value < -32768) {
        return (int16_t)-32768;
    }
    if (value > 32767) {
        return (int16_t)32767;
    }
    return (int16_t)value;
}

/* 读取 CPU3 本机参数当前值（统一入口） */
int32_t Cpu3Local_ReadValue(OperatingNumber opera)
{
    switch (opera)
    {
    /* 界面显示类 / 基本信息 */
    case COM_NUM_PARA_LOCAL_LEDVERSION:
        return (int32_t)g_cpu3_comm_display_params.local_led_version;

    case COM_NUM_PARA_LANG:
        return g_cpu3_comm_display_params.language;

    case COM_NUM_SCREEN_DECIMAL:
        return g_cpu3_comm_display_params.screen_decimal;

    case COM_NUM_SCREEN_PASSWARD:
        return g_cpu3_comm_display_params.screen_password;

    case COM_NUM_SCREEN_OFF:
        return g_cpu3_comm_display_params.screen_off_time;

    case COM_NUM_SCREEN_BRIGHTNESS:
        return g_cpu3_comm_display_params.screen_brightness;

    /* 数据源选择 */
    case COM_NUM_SCREEN_SOURCE_OIL:
        return g_cpu3_comm_display_params.screen_source_oil;

    case COM_NUM_SCREEN_SOURCE_WATER:
        return g_cpu3_comm_display_params.screen_source_water;

    case COM_NUM_SCREEN_SOURCE_D:
        return g_cpu3_comm_display_params.screen_source_d;

    case COM_NUM_SCREEN_SOURCE_T:
        return g_cpu3_comm_display_params.screen_source_t;

    /* 手工输入 */
    case COM_NUM_SCREEN_INPUT_OIL:
        return g_cpu3_comm_display_params.screen_input_oil;

    case COM_NUM_SCREEN_INPUT_WATER:
        return g_cpu3_comm_display_params.screen_input_water;

    case COM_NUM_SCREEN_INPUT_D:
        return g_cpu3_comm_display_params.screen_input_d;

    case COM_NUM_SCREEN_INPUT_D_SWITCH:
        return g_cpu3_comm_display_params.screen_input_d_switch;

    case COM_NUM_SCREEN_INPUT_T:
        return g_cpu3_comm_display_params.screen_input_t;

    /* COM1 */
    case COM_NUM_CPU3_COM1_BAUDRATE:
        return Cpu3_BaudValueToIndex(g_cpu3_comm_display_params.com1.baudrate);
    case COM_NUM_CPU3_COM1_DATABITS:
        return (g_cpu3_comm_display_params.com1.databits == 9) ? 1 : 0;
    case COM_NUM_CPU3_COM1_PARITY:
        return (int32_t)g_cpu3_comm_display_params.com1.parity;
    case COM_NUM_CPU3_COM1_STOPBITS:
        return (int32_t)g_cpu3_comm_display_params.com1.stopbits;
    case COM_NUM_CPU3_COM1_PROTOCOL:
        return (int32_t)g_cpu3_comm_display_params.com1.protocol;

    /* COM2 */
    case COM_NUM_CPU3_COM2_BAUDRATE:
        return Cpu3_BaudValueToIndex(g_cpu3_comm_display_params.com2.baudrate);
    case COM_NUM_CPU3_COM2_DATABITS:
        return (g_cpu3_comm_display_params.com2.databits == 9) ? 1 : 0;
    case COM_NUM_CPU3_COM2_PARITY:
        return (int32_t)g_cpu3_comm_display_params.com2.parity;
    case COM_NUM_CPU3_COM2_STOPBITS:
        return (int32_t)g_cpu3_comm_display_params.com2.stopbits;
    case COM_NUM_CPU3_COM2_PROTOCOL:
        return (int32_t)g_cpu3_comm_display_params.com2.protocol;

    /* COM3 */
    case COM_NUM_CPU3_COM3_BAUDRATE:
        return Cpu3_BaudValueToIndex(g_cpu3_comm_display_params.com3.baudrate);
    case COM_NUM_CPU3_COM3_DATABITS:
        return (g_cpu3_comm_display_params.com3.databits == 9) ? 1 : 0;
    case COM_NUM_CPU3_COM3_PARITY:
        return (int32_t)g_cpu3_comm_display_params.com3.parity;
    case COM_NUM_CPU3_COM3_STOPBITS:
        return (int32_t)g_cpu3_comm_display_params.com3.stopbits;
    case COM_NUM_CPU3_COM3_PROTOCOL:
        return (int32_t)g_cpu3_comm_display_params.com3.protocol;

    /* SI */
    case COM_NUM_CPU3_SI_AUTO_PROFILE_INTERVAL:
        return g_cpu3_comm_display_params.si_auto_profile_interval;
    case COM_NUM_CPU3_SI_AUTO_PROFILE_ENABLE:
        return g_cpu3_comm_display_params.si_auto_profile_enable;
    case COM_NUM_CPU3_SI_AUTO_PROFILE_HOUR:
        return g_cpu3_comm_display_params.si_auto_profile_hour;
    case COM_NUM_CPU3_SI_AUTO_PROFILE_MINUTE:
        return g_cpu3_comm_display_params.si_auto_profile_minute;
    case COM_NUM_CPU3_SI_LOW_DENSITY_SETPOINT:
        return g_cpu3_comm_display_params.si_low_density_setpoint;
    case COM_NUM_CPU3_SI_HIGH_DENSITY_SETPOINT:
        return g_cpu3_comm_display_params.si_high_density_setpoint;
    case COM_NUM_CPU3_SI_LOW_TEMPERATURE_SETPOINT:
        return g_cpu3_comm_display_params.si_low_temperature_setpoint;
    case COM_NUM_CPU3_SI_HIGH_TEMPERATURE_SETPOINT:
        return g_cpu3_comm_display_params.si_high_temperature_setpoint;
    case COM_NUM_CPU3_SI_LL_LEVEL_SETPOINT:
        return g_cpu3_comm_display_params.si_ll_level_setpoint;
    case COM_NUM_CPU3_SI_HH_LEVEL_SETPOINT:
        return g_cpu3_comm_display_params.si_hh_level_setpoint;
    case COM_NUM_CPU3_SI_LOW_LEVEL_SETPOINT:
        return g_cpu3_comm_display_params.si_low_level_setpoint;
    case COM_NUM_CPU3_SI_HIGH_LEVEL_SETPOINT:
        return g_cpu3_comm_display_params.si_high_level_setpoint;
    case COM_NUM_CPU3_SI_TEMP_DEVIATION_SETPOINT:
        return g_cpu3_comm_display_params.si_temp_deviation_setpoint;
    case COM_NUM_CPU3_SI_DENSITY_DEVIATION_SETPOINT:
        return g_cpu3_comm_display_params.si_density_deviation_setpoint;

    default:
        return 0;
    }
}

uint16_t Cpu3Local_ReadSiCompatHolding(uint8_t index)
{
    if (index >= CPU3_SI_COMPAT_HOLDING_COUNT) {
        return 0U;
    }

    return g_cpu3_comm_display_params.si_compat_holding[index];
}

/*
 * 函数用途：事务式写入 SI 兼容保持寄存器并校验 FRAM 持久化结果。
 * 调用场景：SI FC06 写入 40004～40009 时同步更新运行态和掉电参数。
 * 关键约束：即使写入值未变化也执行持久化；失败时恢复整份旧镜像并尽力修复 FRAM。
 */
bool Cpu3Local_WriteSiCompatHoldingChecked(uint8_t index, uint16_t value)
{
    Cpu3CommAndDisplayParams old_params = g_cpu3_comm_display_params;

    if (index >= CPU3_SI_COMPAT_HOLDING_COUNT) {
        return false;
    }

    g_cpu3_comm_display_params.si_compat_holding[index] = value;
    if (!Cpu3_Params_SaveToFRAM()) {
        g_cpu3_comm_display_params = old_params;
        /* 首次校验失败时 FRAM 可能已部分改变，尽力恢复完整旧镜像。 */
        (void)Cpu3_Params_SaveToFRAM();
        return false;
    }

    return true;
}

/*
 * 函数用途：事务式写入 CPU3 本机参数并返回 FRAM 持久化校验结果。
 * 调用场景：屏幕菜单、本机参数写入、远程协议切换和 SI FC06 共用本事务内核。
 * 关键约束：FRAM 写后读回失败时恢复整份旧运行态，不允许伪成功或保留未持久化的新值。
 */
static bool Cpu3Local_WriteValueCheckedInternal(OperatingNumber opera,
                                                int32_t v,
                                                bool apply_protocol_serial_profile)
{
    Cpu3CommAndDisplayParams old_params = g_cpu3_comm_display_params;
    bool display_runtime_changed = false;

    switch (opera)
    {
    /* 界面显示类 */
    case COM_NUM_PARA_LANG:
        g_cpu3_comm_display_params.language = (uint8_t)v;
        display_runtime_changed = true;
        break;

    case COM_NUM_SCREEN_DECIMAL:
        g_cpu3_comm_display_params.screen_decimal = (uint8_t)v;
        display_runtime_changed = true;
        break;

    case COM_NUM_SCREEN_PASSWARD:
        g_cpu3_comm_display_params.screen_password = (uint16_t)v;
        display_runtime_changed = true;
        break;

    case COM_NUM_SCREEN_OFF:
        g_cpu3_comm_display_params.screen_off_time = (uint8_t)v;
        display_runtime_changed = true;
        break;

    case COM_NUM_SCREEN_BRIGHTNESS:
        g_cpu3_comm_display_params.screen_brightness = (uint8_t)v;
        display_runtime_changed = true;
        break;

    case COM_NUM_SCREEN_SOURCE_OIL:
        g_cpu3_comm_display_params.screen_source_oil = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_SOURCE_WATER:
        g_cpu3_comm_display_params.screen_source_water = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_SOURCE_D:
        g_cpu3_comm_display_params.screen_source_d = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_SOURCE_T:
        g_cpu3_comm_display_params.screen_source_t  = (uint8_t)v;
        break;

    case COM_NUM_SCREEN_INPUT_OIL:
        g_cpu3_comm_display_params.screen_input_oil = v;
        break;
    case COM_NUM_SCREEN_INPUT_WATER:
        g_cpu3_comm_display_params.screen_input_water = v;
        break;
    case COM_NUM_SCREEN_INPUT_D:
        g_cpu3_comm_display_params.screen_input_d = v;
        break;
    case COM_NUM_SCREEN_INPUT_D_SWITCH:
        g_cpu3_comm_display_params.screen_input_d_switch = (uint8_t)v;
        break;
    case COM_NUM_SCREEN_INPUT_T:
        g_cpu3_comm_display_params.screen_input_t = v;
        break;

    /* COM1 */
    case COM_NUM_CPU3_COM1_BAUDRATE:
        g_cpu3_comm_display_params.com1.baudrate = Cpu3_BaudIndexToValue(v);
        break;
    case COM_NUM_CPU3_COM1_DATABITS:
        g_cpu3_comm_display_params.com1.databits = (v == 1) ? 9 : 8;
        break;
    case COM_NUM_CPU3_COM1_PARITY:
        g_cpu3_comm_display_params.com1.parity = (ComParityType)v;
        break;
    case COM_NUM_CPU3_COM1_STOPBITS:
        g_cpu3_comm_display_params.com1.stopbits = (ComStopBitsType)v;
        break;
    case COM_NUM_CPU3_COM1_PROTOCOL:
        g_cpu3_comm_display_params.com1.protocol = Cpu3_NormalizeProtocol(v);
        if (apply_protocol_serial_profile) {
            (void)Cpu3_ApplyProtocolSerialProfile(&g_cpu3_comm_display_params.com1);
        }
        break;

    /* COM2 */
    case COM_NUM_CPU3_COM2_BAUDRATE:
        g_cpu3_comm_display_params.com2.baudrate = Cpu3_BaudIndexToValue(v);
        break;
    case COM_NUM_CPU3_COM2_DATABITS:
        g_cpu3_comm_display_params.com2.databits = (v == 1) ? 9 : 8;
        break;
    case COM_NUM_CPU3_COM2_PARITY:
        g_cpu3_comm_display_params.com2.parity = (ComParityType)v;
        break;
    case COM_NUM_CPU3_COM2_STOPBITS:
        g_cpu3_comm_display_params.com2.stopbits = (ComStopBitsType)v;
        break;
    case COM_NUM_CPU3_COM2_PROTOCOL:
        g_cpu3_comm_display_params.com2.protocol = Cpu3_NormalizeProtocol(v);
        if (apply_protocol_serial_profile) {
            (void)Cpu3_ApplyProtocolSerialProfile(&g_cpu3_comm_display_params.com2);
        }
        break;

    /* COM3 */
    case COM_NUM_CPU3_COM3_BAUDRATE:
        g_cpu3_comm_display_params.com3.baudrate = Cpu3_BaudIndexToValue(v);
        break;
    case COM_NUM_CPU3_COM3_DATABITS:
        g_cpu3_comm_display_params.com3.databits = (v == 1) ? 9 : 8;
        break;
    case COM_NUM_CPU3_COM3_PARITY:
        g_cpu3_comm_display_params.com3.parity = (ComParityType)v;
        break;
    case COM_NUM_CPU3_COM3_STOPBITS:
        g_cpu3_comm_display_params.com3.stopbits = (ComStopBitsType)v;
        break;
    case COM_NUM_CPU3_COM3_PROTOCOL:
        g_cpu3_comm_display_params.com3.protocol = Cpu3_NormalizeProtocol(v);
        if (apply_protocol_serial_profile) {
            (void)Cpu3_ApplyProtocolSerialProfile(&g_cpu3_comm_display_params.com3);
        }
        break;

    /* SI */
    case COM_NUM_CPU3_SI_AUTO_PROFILE_INTERVAL:
        g_cpu3_comm_display_params.si_auto_profile_interval = (v <= 0) ? 1U : Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_AUTO_PROFILE_ENABLE:
        g_cpu3_comm_display_params.si_auto_profile_enable = (v != 0) ? 1U : 0U;
        break;
    case COM_NUM_CPU3_SI_AUTO_PROFILE_HOUR:
        g_cpu3_comm_display_params.si_auto_profile_hour = (uint8_t)((v < 0) ? 0 : ((v > 23) ? 23 : v));
        break;
    case COM_NUM_CPU3_SI_AUTO_PROFILE_MINUTE:
        g_cpu3_comm_display_params.si_auto_profile_minute = (uint8_t)((v < 0) ? 0 : ((v > 59) ? 59 : v));
        break;
    case COM_NUM_CPU3_SI_LOW_DENSITY_SETPOINT:
        g_cpu3_comm_display_params.si_low_density_setpoint = Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_HIGH_DENSITY_SETPOINT:
        g_cpu3_comm_display_params.si_high_density_setpoint = Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_LOW_TEMPERATURE_SETPOINT:
        g_cpu3_comm_display_params.si_low_temperature_setpoint = Cpu3_ClampS16Param(v);
        break;
    case COM_NUM_CPU3_SI_HIGH_TEMPERATURE_SETPOINT:
        g_cpu3_comm_display_params.si_high_temperature_setpoint = Cpu3_ClampS16Param(v);
        break;
    case COM_NUM_CPU3_SI_LL_LEVEL_SETPOINT:
        g_cpu3_comm_display_params.si_ll_level_setpoint = Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_HH_LEVEL_SETPOINT:
        g_cpu3_comm_display_params.si_hh_level_setpoint = Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_LOW_LEVEL_SETPOINT:
        g_cpu3_comm_display_params.si_low_level_setpoint = Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_HIGH_LEVEL_SETPOINT:
        g_cpu3_comm_display_params.si_high_level_setpoint = Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_TEMP_DEVIATION_SETPOINT:
        g_cpu3_comm_display_params.si_temp_deviation_setpoint = Cpu3_ClampU16Param(v);
        break;
    case COM_NUM_CPU3_SI_DENSITY_DEVIATION_SETPOINT:
        g_cpu3_comm_display_params.si_density_deviation_setpoint = Cpu3_ClampU16Param(v);
        break;

    default:
        break;
    }

    if (apply_protocol_serial_profile && Cpu3Local_IsUartParam(opera)) {
        /* 非协议字段允许现场覆盖；这里只兜底修正越界值，避免 UART 初始化异常。 */
        (void)Cpu3_SanitizeAllPortConfigs();
    }

    if (display_runtime_changed) {
        Cpu3Local_ApplyDisplayRuntimeParams();
    }

    if (!Cpu3_Params_SaveToFRAM()) {
        g_cpu3_comm_display_params = old_params;
        if (display_runtime_changed) {
            Cpu3Local_ApplyDisplayRuntimeParams();
        }
        /* 首次校验失败时 FRAM 可能已部分改变，尽力恢复完整旧镜像。 */
        (void)Cpu3_Params_SaveToFRAM();
        return false;
    }

    return true;
}

/*
 * 函数用途：按屏幕配置语义事务式写入本机参数。
 * 调用场景：屏幕菜单修改协议时同步套用目标协议默认串口参数，其它参数沿用原写入行为。
 * 关键约束：屏幕与远程协议切换均在旧参数应答完成后调用；修改协议时同步应用默认串口参数。
 */
bool Cpu3Local_WriteValueChecked(OperatingNumber opera, int32_t v)
{
    return Cpu3Local_WriteValueCheckedInternal(opera, v, true);
}

/*
 * 函数用途：保持现有菜单和参数写入口的无返回值接口。
 * 调用场景：不需要同步处理 FRAM 写入结果的既有调用方。
 * 关键约束：需要确认持久化结果时必须调用 Cpu3Local_WriteValueChecked。
 */
void Cpu3Local_WriteValue(OperatingNumber opera, int32_t v)
{
    (void)Cpu3Local_WriteValueChecked(opera, v);
}

/**
 * @brief 判断指定操作号是否属于 CPU3 本地串口参数。
 * @param opera 参数操作号。
 * @return true 表示串口参数，false 表示其他本地参数。
 */
bool Cpu3Local_IsUartParam(OperatingNumber opera)
{
    return (opera >= COM_NUM_CPU3_COM1_BAUDRATE && opera <= COM_NUM_CPU3_COM3_PROTOCOL);
}

/*
 * 将 CPU3 本地显示参数同步到运行期全局变量，并立即应用 OLED 亮度。
 * 上电时 FRAM 加载发生在 OLED 默认初始化之后，所以这里必须再应用一次保存的挡位。
 */
void Cpu3Local_ApplyDisplayRuntimeParams(void)
{
    screen_parameter.decimalplaces = g_cpu3_comm_display_params.screen_decimal;
    screen_parameter.passward = g_cpu3_comm_display_params.screen_password;
    screen_parameter.language = g_cpu3_comm_display_params.language;
    screen_parameter.screenoff = g_cpu3_comm_display_params.screen_off_time;
    screen_parameter.brightness = g_cpu3_comm_display_params.screen_brightness;
    OLED_SetBrightnessLevel(g_cpu3_comm_display_params.screen_brightness);
}

/* ================ 内部小工具：用一个 ComPortConfig 初始化一个 UART ================ */
static bool Cpu3_ReinitOneUart(UART_HandleTypeDef *huart, const ComPortConfig *cfg)
{
    if ((huart == NULL) || (cfg == NULL)) {
        return false;
    }

    if (HAL_UART_DeInit(huart) != HAL_OK) {
        return false;
    }

    huart->Init.BaudRate = cfg->baudrate;
    /*
     * STM32 HAL 的有校验 8 数据位需要配置 9B，最高位由硬件作为校验位发送。
     * 如果仍配置 8B，SI 的 8O1 会实际变成 7O1。
     */
    huart->Init.WordLength =
        ((cfg->databits == 9) || (cfg->parity != COM_PARITY_NONE)) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;

    /* 校验 */
    switch (cfg->parity) {
    case COM_PARITY_EVEN: huart->Init.Parity = UART_PARITY_EVEN; break;
    case COM_PARITY_ODD:  huart->Init.Parity = UART_PARITY_ODD;  break;
    case COM_PARITY_NONE:
    default:              huart->Init.Parity = UART_PARITY_NONE; break;
    }

    /* 停止位 */
    switch (cfg->stopbits) {
    case COM_STOPBITS_2: huart->Init.StopBits = UART_STOPBITS_2; break;
    case COM_STOPBITS_1:
    default:             huart->Init.StopBits = UART_STOPBITS_1; break;
    }

    huart->Init.Mode       = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK) {
        return false;
    }
    return true;
}

/* ========================== 初始化默认值 ========================== */
void Cpu3_Params_InitDefaults(void)
{
    memset(&g_cpu3_comm_display_params, 0, sizeof(g_cpu3_comm_display_params));

    /* 屏幕基本信息 */
    g_cpu3_comm_display_params.local_led_version = CPU3_APP_VERSION_U32;
    g_cpu3_comm_display_params.language          = 0;

    /* 数据源默认 */
    g_cpu3_comm_display_params.screen_source_oil   = 0;
    g_cpu3_comm_display_params.screen_source_water = 0;
    g_cpu3_comm_display_params.screen_source_d     = 0;
    g_cpu3_comm_display_params.screen_source_t     = 0;

    /* 显示类默认 */
    g_cpu3_comm_display_params.screen_decimal  = 2;
    g_cpu3_comm_display_params.screen_brightness = OLED_BRIGHTNESS_LEVEL_LOW;
    Cpu3_InitSiParams();

    /* ========== 串口默认：保持和你现在 usart.c 一致 ========== */

    /* COM1 = USART6: 4800 8N1 DSM */
    g_cpu3_comm_display_params.com1.baudrate = 4800;
    g_cpu3_comm_display_params.com1.databits = 8;
    g_cpu3_comm_display_params.com1.parity   = COM_PARITY_NONE;
    g_cpu3_comm_display_params.com1.stopbits = COM_STOPBITS_1;
    g_cpu3_comm_display_params.com1.protocol = COM_PROTO_DSM;

    /* COM2 = USART2: 2400 8E1 Wartsila */
    g_cpu3_comm_display_params.com2.baudrate = 2400;
    g_cpu3_comm_display_params.com2.databits = 8;
    g_cpu3_comm_display_params.com2.parity   = COM_PARITY_EVEN;
    g_cpu3_comm_display_params.com2.stopbits = COM_STOPBITS_1;
    g_cpu3_comm_display_params.com2.protocol = COM_PROTO_WARTSILA;

    /* COM3 = USART3: 2400 8E1 Wartsila */
    g_cpu3_comm_display_params.com3.baudrate = 2400;
    g_cpu3_comm_display_params.com3.databits = 8;
    g_cpu3_comm_display_params.com3.parity   = COM_PARITY_EVEN;
    g_cpu3_comm_display_params.com3.stopbits = COM_STOPBITS_1;
    g_cpu3_comm_display_params.com3.protocol = COM_PROTO_WARTSILA;
}

/* ========================== 重配全部串口 ========================== */
bool Cpu3_ReinitAllUarts(void)
{
    bool com1_ok = Cpu3_ReinitPortUart(1U);
    bool com2_ok = Cpu3_ReinitPortUart(2U);
    bool com3_ok = Cpu3_ReinitPortUart(3U);

    return com1_ok && com2_ok && com3_ok;
}

/*
 * 函数用途：按当前 CPU3 本机参数只重初始化指定的外部 COM 口。
 * 调用场景：协议切换应答发送完成后，由主循环保存新协议并调用。
 * 关键约束：不影响其它外部 COM 口；调用前应确保目标端口发送已经完成。
 */
bool Cpu3_ReinitPortUart(uint8_t port_idx)
{
    UART_HandleTypeDef *huart;
    const ComPortConfig *cfg;
    uint8_t *rx_buf;
    uint16_t rx_buf_size;

    switch (port_idx)
    {
    case 1U:
        huart = &huart6;
        cfg = &g_cpu3_comm_display_params.com1;
        rx_buf = UART6_RX_BUF;
        rx_buf_size = UART6_RX_BUF_SIZE;
        break;

    case 2U:
        huart = &huart2;
        cfg = &g_cpu3_comm_display_params.com2;
        rx_buf = UART2_RX_BUF;
        rx_buf_size = UART2_RX_BUF_SIZE;
        break;

    case 3U:
        huart = &huart3;
        cfg = &g_cpu3_comm_display_params.com3;
        rx_buf = UART3_RX_BUF;
        rx_buf_size = UART3_RX_BUF_SIZE;
        break;

    default:
        return false;
    }

    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    (void)HAL_UART_DMAStop(huart);
    if (!Cpu3_ReinitOneUart(huart, cfg)) {
        return false;
    }
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (HAL_UART_Receive_DMA(huart, rx_buf, rx_buf_size) != HAL_OK) {
        return false;
    }
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    return true;
}

/* ==================== FRAM 存储结构定义 ==================== */
/*
 * 为了兼容后续版本，我们在 FRAM 里存储的是一个带头的结构：
 *  - magic   : 固定魔数，用来判断这个区域是不是 Cpu3 的参数
 *  - version : 版本号，后续结构体变大时可以用来做兼容
 *  - params  : 当前的 Cpu3CommAndDisplayParams
 *  - crc     : 对上述 (magic, version, params) 计算 CRC32
 */

#define CPU3_PARAM_MAGIC   0x43505533UL   /* 'CPU3' */
#define CPU3_PARAM_VERSION_V3 0x0003U
#define CPU3_PARAM_VERSION_V4 0x0004U
#define CPU3_PARAM_VERSION_V5 0x0005U
#define CPU3_PARAM_VERSION_V6 0x0006U
#define CPU3_PARAM_VERSION 0x0007U

typedef struct
{
    uint32_t local_led_version;
    uint8_t  language;
    uint8_t  screen_source_oil;
    uint8_t  screen_source_water;
    uint8_t  screen_source_d;
    uint8_t  screen_source_t;
    int32_t  screen_input_oil;
    int32_t  screen_input_water;
    int32_t  screen_input_d;
    uint8_t  screen_input_d_switch;
    int32_t  screen_input_t;
    uint8_t  screen_decimal;
    uint16_t screen_password;
    uint8_t  screen_off_time;
    ComPortConfig com1;
    ComPortConfig com2;
    ComPortConfig com3;
} Cpu3CommAndDisplayParamsV3;

typedef struct
{
    uint32_t                    magic;
    uint16_t                    version;
    uint16_t                    reserved;
    Cpu3CommAndDisplayParamsV3  params;
    uint32_t                    crc;
} Cpu3ParamStorageV3;

typedef struct
{
    uint32_t local_led_version;
    uint8_t  language;
    uint8_t screen_source_oil;
    uint8_t screen_source_water;
    uint8_t screen_source_d;
    uint8_t screen_source_t;
    int32_t screen_input_oil;
    int32_t screen_input_water;
    int32_t screen_input_d;
    uint8_t screen_input_d_switch;
    int32_t screen_input_t;
    uint8_t  screen_decimal;
    uint16_t screen_password;
    uint8_t  screen_off_time;
    uint8_t  screen_brightness;
    ComPortConfig com1;
    ComPortConfig com2;
    ComPortConfig com3;
} Cpu3CommAndDisplayParamsV5;

typedef struct
{
    uint32_t                    magic;
    uint16_t                    version;
    uint16_t                    reserved;
    Cpu3CommAndDisplayParamsV5  params;
    uint32_t                    crc;
} Cpu3ParamStorageV5;

/*
 * V6结构必须保持升级前的精确字段顺序和类型。
 * 当前V7只在该结构末尾追加六个uint16_t兼容槽。
 */
typedef struct
{
    uint32_t local_led_version;
    uint8_t  language;
    uint8_t  screen_source_oil;
    uint8_t  screen_source_water;
    uint8_t  screen_source_d;
    uint8_t  screen_source_t;
    int32_t  screen_input_oil;
    int32_t  screen_input_water;
    int32_t  screen_input_d;
    uint8_t  screen_input_d_switch;
    int32_t  screen_input_t;
    uint8_t  screen_decimal;
    uint16_t screen_password;
    uint8_t  screen_off_time;
    uint8_t  screen_brightness;
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
    ComPortConfig com1;
    ComPortConfig com2;
    ComPortConfig com3;
} Cpu3CommAndDisplayParamsV6;

/* 编译期逐字段确认V6布局就是V7兼容槽之前的完整前缀。 */
#define CPU3_ASSERT_V6_FIELD_OFFSET(field) \
    _Static_assert(offsetof(Cpu3CommAndDisplayParamsV6, field) == \
                   offsetof(Cpu3CommAndDisplayParams, field), \
                   "CPU3 V6参数字段偏移不兼容")
CPU3_ASSERT_V6_FIELD_OFFSET(local_led_version);
CPU3_ASSERT_V6_FIELD_OFFSET(language);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_source_oil);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_source_water);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_source_d);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_source_t);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_input_oil);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_input_water);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_input_d);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_input_d_switch);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_input_t);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_decimal);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_password);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_off_time);
CPU3_ASSERT_V6_FIELD_OFFSET(screen_brightness);
CPU3_ASSERT_V6_FIELD_OFFSET(si_auto_profile_interval);
CPU3_ASSERT_V6_FIELD_OFFSET(si_auto_profile_enable);
CPU3_ASSERT_V6_FIELD_OFFSET(si_auto_profile_hour);
CPU3_ASSERT_V6_FIELD_OFFSET(si_auto_profile_minute);
CPU3_ASSERT_V6_FIELD_OFFSET(si_low_density_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_high_density_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_low_temperature_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_high_temperature_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_ll_level_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_hh_level_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_low_level_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_high_level_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_temp_deviation_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(si_density_deviation_setpoint);
CPU3_ASSERT_V6_FIELD_OFFSET(com1);
CPU3_ASSERT_V6_FIELD_OFFSET(com2);
CPU3_ASSERT_V6_FIELD_OFFSET(com3);
#undef CPU3_ASSERT_V6_FIELD_OFFSET
_Static_assert(sizeof(Cpu3CommAndDisplayParamsV6) ==
               offsetof(Cpu3CommAndDisplayParams, si_compat_holding),
               "CPU3 V6参数前缀长度不兼容");

typedef struct
{
    uint32_t                    magic;
    uint16_t                    version;
    uint16_t                    reserved;
    Cpu3CommAndDisplayParamsV6  params;
    uint32_t                    crc;
} Cpu3ParamStorageV6;

typedef struct
{
    uint32_t                 magic;
    uint16_t                 version;
    uint16_t                 reserved;  /* 对齐/预留 */
    Cpu3CommAndDisplayParams params;
    uint32_t                 crc;
} Cpu3ParamStorage;

/* 把当前 CPU3 本机参数组织成可直接落 FRAM 的镜像结构。
 * 这样保存前后的比较、CRC 计算、真正写入，三者都使用同一份数据组织方式，
 * 避免“比较的是一套数据、写入的是另一套数据”导致的误判。 */
static void Cpu3_Params_BuildStorage(Cpu3ParamStorage *stor)
{
    stor->magic    = CPU3_PARAM_MAGIC;
    stor->version  = CPU3_PARAM_VERSION;
    stor->reserved = 0;
    Cpu3_ApplyFirmwareVersionRuntime();
    stor->params   = g_cpu3_comm_display_params;

    /* 计算 CRC：不包含最后的 crc 字段本身 */
    {
        uint32_t crc_len = sizeof(Cpu3ParamStorage) - sizeof(stor->crc);
        stor->crc = CRC32_HAL((uint8_t*)stor, crc_len);
    }
}

/* 判断 FRAM 里现有的 CPU3 参数镜像是否有效。
 * 只有 magic/version/CRC 同时成立，才允许把它当成“可信旧值”参与判重；
 * 否则宁可重写一次，也不能基于脏数据跳过保存。 */
static bool Cpu3_Params_StorageValid(const Cpu3ParamStorage *stor)
{
    uint32_t crc_len;
    uint32_t crc_calc;

    if ((stor->magic != CPU3_PARAM_MAGIC) || (stor->version != CPU3_PARAM_VERSION)) {
        return false;
    }

    crc_len = sizeof(Cpu3ParamStorage) - sizeof(stor->crc);
    crc_calc = CRC32_HAL((uint8_t*)stor, crc_len);
    return crc_calc == stor->crc;
}

/**
 * @brief 执行参数存储中的 Cpu3_Params_StorageV3Valid 逻辑。
 *
 * @param stor 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
static bool Cpu3_Params_StorageV3Valid(const Cpu3ParamStorageV3 *stor)
{
    uint32_t crc_len;
    uint32_t crc_calc;

    if ((stor->magic != CPU3_PARAM_MAGIC) || (stor->version != CPU3_PARAM_VERSION_V3)) {
        return false;
    }

    crc_len = sizeof(Cpu3ParamStorageV3) - sizeof(stor->crc);
    crc_calc = CRC32_HAL((uint8_t*)stor, crc_len);
    return crc_calc == stor->crc;
}

static bool Cpu3_Params_StorageV5Valid(const Cpu3ParamStorageV5 *stor)
{
    uint32_t crc_len;
    uint32_t crc_calc;

    if ((stor->magic != CPU3_PARAM_MAGIC) ||
        ((stor->version != CPU3_PARAM_VERSION_V4) && (stor->version != CPU3_PARAM_VERSION_V5)))
    {
        return false;
    }

    crc_len = sizeof(Cpu3ParamStorageV5) - sizeof(stor->crc);
    crc_calc = CRC32_HAL((uint8_t*)stor, crc_len);
    return crc_calc == stor->crc;
}

static bool Cpu3_Params_StorageV6Valid(const Cpu3ParamStorageV6 *stor)
{
    uint32_t crc_len;
    uint32_t crc_calc;

    if ((stor->magic != CPU3_PARAM_MAGIC) || (stor->version != CPU3_PARAM_VERSION_V6)) {
        return false;
    }

    crc_len = sizeof(Cpu3ParamStorageV6) - sizeof(stor->crc);
    crc_calc = CRC32_HAL((uint8_t*)stor, crc_len);
    return crc_calc == stor->crc;
}

/**
 * @brief 执行参数存储中的 Cpu3_Params_MigrateFromV3 逻辑。
 *
 * @param stor 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void Cpu3_Params_MigrateFromV3(const Cpu3ParamStorageV3 *stor)
{
    memset(&g_cpu3_comm_display_params, 0, sizeof(g_cpu3_comm_display_params));

    g_cpu3_comm_display_params.local_led_version = stor->params.local_led_version;
    g_cpu3_comm_display_params.language = stor->params.language;
    g_cpu3_comm_display_params.screen_source_oil = stor->params.screen_source_oil;
    g_cpu3_comm_display_params.screen_source_water = stor->params.screen_source_water;
    g_cpu3_comm_display_params.screen_source_d = stor->params.screen_source_d;
    g_cpu3_comm_display_params.screen_source_t = stor->params.screen_source_t;
    g_cpu3_comm_display_params.screen_input_oil = stor->params.screen_input_oil;
    g_cpu3_comm_display_params.screen_input_water = stor->params.screen_input_water;
    g_cpu3_comm_display_params.screen_input_d = stor->params.screen_input_d;
    g_cpu3_comm_display_params.screen_input_d_switch = stor->params.screen_input_d_switch;
    g_cpu3_comm_display_params.screen_input_t = stor->params.screen_input_t;
    g_cpu3_comm_display_params.screen_decimal = stor->params.screen_decimal;
    g_cpu3_comm_display_params.screen_password = stor->params.screen_password;
    g_cpu3_comm_display_params.screen_off_time = stor->params.screen_off_time;
    g_cpu3_comm_display_params.screen_brightness = OLED_BRIGHTNESS_LEVEL_LOW;
    g_cpu3_comm_display_params.com1 = stor->params.com1;
    g_cpu3_comm_display_params.com2 = stor->params.com2;
    g_cpu3_comm_display_params.com3 = stor->params.com3;
    Cpu3_InitSiParams();
}

static void Cpu3_Params_MigrateFromV5(const Cpu3ParamStorageV5 *stor)
{
    memset(&g_cpu3_comm_display_params, 0, sizeof(g_cpu3_comm_display_params));

    g_cpu3_comm_display_params.local_led_version = stor->params.local_led_version;
    g_cpu3_comm_display_params.language = stor->params.language;
    g_cpu3_comm_display_params.screen_source_oil = stor->params.screen_source_oil;
    g_cpu3_comm_display_params.screen_source_water = stor->params.screen_source_water;
    g_cpu3_comm_display_params.screen_source_d = stor->params.screen_source_d;
    g_cpu3_comm_display_params.screen_source_t = stor->params.screen_source_t;
    g_cpu3_comm_display_params.screen_input_oil = stor->params.screen_input_oil;
    g_cpu3_comm_display_params.screen_input_water = stor->params.screen_input_water;
    g_cpu3_comm_display_params.screen_input_d = stor->params.screen_input_d;
    g_cpu3_comm_display_params.screen_input_d_switch = stor->params.screen_input_d_switch;
    g_cpu3_comm_display_params.screen_input_t = stor->params.screen_input_t;
    g_cpu3_comm_display_params.screen_decimal = stor->params.screen_decimal;
    g_cpu3_comm_display_params.screen_password = stor->params.screen_password;
    g_cpu3_comm_display_params.screen_off_time = stor->params.screen_off_time;
    g_cpu3_comm_display_params.screen_brightness = stor->params.screen_brightness;
    g_cpu3_comm_display_params.com1 = stor->params.com1;
    g_cpu3_comm_display_params.com2 = stor->params.com2;
    g_cpu3_comm_display_params.com3 = stor->params.com3;
    Cpu3_InitSiParams();
}

/*
 * 函数用途：把完整V6参数前缀迁移到V7并补入六个SI原始兼容槽。
 * 调用场景：上电发现FRAM版本为V6且CRC有效。
 * 关键约束：V6已有SI参数和三路合法串口配置保持不变；非法串口字段仍按既有加载规则归一化。
 */
static void Cpu3_Params_MigrateFromV6(const Cpu3ParamStorageV6 *stor)
{
    memset(&g_cpu3_comm_display_params, 0, sizeof(g_cpu3_comm_display_params));
    memcpy(&g_cpu3_comm_display_params, &stor->params, sizeof(stor->params));
    Cpu3_InitSiCompatHolding();
}

static int32_t Cpu3_MigrateDensityInputX10ToX100(int32_t raw)
{
    if (raw > (INT32_MAX / (int32_t)DENSITY_PARAM_MIGRATE_FACTOR)) {
        return INT32_MAX;
    }
    if (raw < (INT32_MIN / (int32_t)DENSITY_PARAM_MIGRATE_FACTOR)) {
        return INT32_MIN;
    }
    return raw * (int32_t)DENSITY_PARAM_MIGRATE_FACTOR;
}

/**
 * @brief 将当前 CPU3 本机参数保存到 FRAM，并执行写后读回校验。
 * @return true 表示现有镜像一致或写后读回校验通过，false 表示持久化失败。
 */
bool Cpu3_Params_SaveToFRAM(void)
{
    Cpu3ParamStorage stor;
    Cpu3ParamStorage current;
    Cpu3ParamStorage verify;

    /* 先构造“准备写入 FRAM 的目标镜像”，后续判重和真实写入都基于它。 */
    Cpu3_Params_BuildStorage(&stor);

    /* 读出现有 FRAM 内容用于判重。
     * 如果现有内容有效且参数区完全一致，则说明这次只是重复保存同样的值，
     * 直接跳过写入，减少 FRAM 总线访问和不必要的持久化动作。 */
    memset(&current, 0, sizeof(current));
    ReadMultiData((uint8_t*)&current, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorage));

    if (Cpu3_Params_StorageValid(&current)
        && memcmp(&current.params, &stor.params, sizeof(stor.params)) == 0)
    {
        printf("CPU3参数未变化，跳过保存。\r\n");
        return true;
    }

    WriteMultiData((uint8_t*)&stor, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorage));

    memset(&verify, 0, sizeof(verify));
    ReadMultiData((uint8_t*)&verify, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorage));
    if ((!Cpu3_Params_StorageValid(&verify))
        || (memcmp(&verify.params, &stor.params, sizeof(stor.params)) != 0))
    {
        printf("CPU3参数写入FRAM后读回校验失败。\r\n");
        return false;
    }

    printf("CPU3参数已保存到FRAM，CRC=0x%08lX\r\n", (unsigned long)stor.crc);
    return true;
}

/**
 * @brief 加载或恢复参数存储中的 Cpu3_Params_LoadFromFRAM 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void Cpu3_Params_LoadFromFRAM(void)
{
    Cpu3ParamStorage stor;
    memset(&stor, 0, sizeof(stor));

    ReadMultiData((uint8_t*)&stor, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorage));

    uint8_t use_default = 0;

    /* 检查 magic & version */
    if ((stor.magic == CPU3_PARAM_MAGIC) && (stor.version == CPU3_PARAM_VERSION_V3)) {
        Cpu3ParamStorageV3 legacy;

        memset(&legacy, 0, sizeof(legacy));
        ReadMultiData((uint8_t*)&legacy, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorageV3));
        if (Cpu3_Params_StorageV3Valid(&legacy)) {
            Cpu3_Params_MigrateFromV3(&legacy);
            g_cpu3_comm_display_params.screen_input_d =
                Cpu3_MigrateDensityInputX10ToX100(g_cpu3_comm_display_params.screen_input_d);
            (void)Cpu3_ApplyFirmwareVersionRuntime();
            (void)Cpu3_SanitizeAllPortConfigs();
            (void)Cpu3_MigrateLegacyWartsilaDefaults();
            Cpu3Local_ApplyDisplayRuntimeParams();
            Cpu3_Params_SaveToFRAM();
            printf("CPU3 FRAM参数已从V3升级到V7，亮度、SI参数与兼容槽使用默认值。\r\n");
            return;
        }

        printf("CPU3 FRAM V3参数CRC无效，使用默认值。\r\n");
        use_default = 1;
    } else if ((stor.magic == CPU3_PARAM_MAGIC) &&
               ((stor.version == CPU3_PARAM_VERSION_V4) || (stor.version == CPU3_PARAM_VERSION_V5))) {
        Cpu3ParamStorageV5 legacy;

        memset(&legacy, 0, sizeof(legacy));
        ReadMultiData((uint8_t*)&legacy, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorageV5));
        if (Cpu3_Params_StorageV5Valid(&legacy)) {
            Cpu3_Params_MigrateFromV5(&legacy);
            if (legacy.version == CPU3_PARAM_VERSION_V4) {
                g_cpu3_comm_display_params.screen_input_d =
                    Cpu3_MigrateDensityInputX10ToX100(g_cpu3_comm_display_params.screen_input_d);
            }
            (void)Cpu3_ApplyFirmwareVersionRuntime();
            (void)Cpu3_SanitizeAllPortConfigs();
            (void)Cpu3_MigrateLegacyWartsilaDefaults();
            Cpu3Local_ApplyDisplayRuntimeParams();
            Cpu3_Params_SaveToFRAM();
            printf("CPU3 FRAM参数已从V%u升级到V7，补入SI参数与兼容槽默认值。\r\n",
                   (unsigned)legacy.version);
            return;
        }

        printf("CPU3 FRAM V%u参数CRC无效，使用默认值。\r\n", (unsigned)stor.version);
        use_default = 1;
    } else if ((stor.magic == CPU3_PARAM_MAGIC) && (stor.version == CPU3_PARAM_VERSION_V6)) {
        Cpu3ParamStorageV6 legacy;

        memset(&legacy, 0, sizeof(legacy));
        ReadMultiData((uint8_t*)&legacy, FRAM_CPU3_PARAM_ADDRESS, sizeof(Cpu3ParamStorageV6));
        if (Cpu3_Params_StorageV6Valid(&legacy)) {
            Cpu3_Params_MigrateFromV6(&legacy);
            (void)Cpu3_ApplyFirmwareVersionRuntime();
            (void)Cpu3_SanitizeAllPortConfigs();
            /*
             * V6已经是当前发布基线，不能再按“旧默认值”猜测并改写合法串口参数。
             * 尤其要保留现场可能主动配置的Wartsila 4800 8N1/8N2组合。
             */
            Cpu3Local_ApplyDisplayRuntimeParams();
            if (!Cpu3_Params_SaveToFRAM()) {
                /* 运行态继续使用已迁移参数，但不得把未通过校验的FRAM误报为升级成功。 */
                printf("CPU3 FRAM参数从V6迁移到V7后保存校验失败，本次不报告升级成功，下次上电将按FRAM实际镜像重新判定。\r\n");
                return;
            }
            printf("CPU3 FRAM参数已从V6升级到V7，旧SI参数与串口参数已保留。\r\n");
            return;
        }

        printf("CPU3 FRAM V6参数CRC无效，使用默认值。\r\n");
        use_default = 1;
    } else if ((stor.magic != CPU3_PARAM_MAGIC) ||
               (stor.version != CPU3_PARAM_VERSION)) {
        printf("CPU3 FRAM参数魔术字/版本无效，使用默认值。\r\n");
        use_default = 1;
    } else {
        /* 校验 CRC */
        uint32_t crc_len = sizeof(Cpu3ParamStorage) - sizeof(stor.crc);
        uint32_t crc_calc = CRC32_HAL((uint8_t*)&stor, crc_len);

        if (crc_calc != stor.crc) {
            printf("CPU3 FRAM参数CRC不匹配，FRAM=0x%08lX，计算=0x%08lX\r\n",
                   (unsigned long)stor.crc, (unsigned long)crc_calc);
            use_default = 1;
        }
    }

    /* 先处理异常边界，避免参数存储状态机带故障继续运行。 */
    if (use_default) {
        /* 使用默认值并立刻写回 FRAM */
        Cpu3_Params_InitDefaults();
        Cpu3Local_ApplyDisplayRuntimeParams();
        Cpu3_Params_SaveToFRAM();
    } else {
        /* 正常加载 */
        uint8_t need_save = 0U;

        g_cpu3_comm_display_params = stor.params;
        if (Cpu3_ApplyFirmwareVersionRuntime()) {
            need_save = 1U;
        }
        if (Cpu3_SanitizeAllPortConfigs() != 0U) {
            /* FRAM 参数加载后只修正非法串口字段，合法的人工串口配置必须原样保留。 */
            need_save = 1U;
        }
        /* V7不再依据合法物理参数猜测历史默认值，避免改写现场串口配置。 */
        Cpu3Local_ApplyDisplayRuntimeParams();
        if (need_save != 0U) {
            Cpu3_Params_SaveToFRAM();
        }
        printf("CPU3参数已从FRAM加载，CRC=0x%08lX\r\n", (unsigned long)stor.crc);
    }
}
